#!/usr/bin/env python3
"""
Listens for newline-delimited JSON scene updates on a TCP port and mirrors them into a Meshcat viewer.
Expected message format (all fields optional):
{
  "action": "update" | "delete_all",
  "timestamp": 123.4,
  "robot_spheres": [
    {"robot": "arm0", "robot_id": 0, "link": "link_3", "sphere_index": 12,
     "center": [x,y,z], "radius": r, "rgba": [r,g,b,a]}
  ],
  "objects": [
    {"name": "box", "type": "box", "size": [l,w,h],
     "position": [x,y,z], "quaternion": [qx,qy,qz,qw], "rgba": [r,g,b,a]}
  ]
}
"""

import argparse
import asyncio
import json
import math
import sys
import time
from typing import Dict, List

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import numpy as np


def rgba_to_int(rgba: List[float]) -> int:
    r, g, b = (max(0, min(255, int(c * 255))) for c in rgba[:3])
    return (r << 16) + (g << 8) + b


def log(msg: str):
    print(msg, file=sys.stderr, flush=True)


def material_from_rgba(rgba: List[float]) -> g.MeshLambertMaterial:
    color = rgba_to_int(rgba)
    opacity = rgba[3] if len(rgba) > 3 else 1.0
    return g.MeshLambertMaterial(
        color=color, transparent=opacity < 0.999, opacity=opacity
    )


def pose_to_matrix(position: List[float], quaternion: List[float]):
    """Meshcat expects a 4x4 transform; quaternion order is [x, y, z, w] here."""
    tx = tf.translation_matrix(position)
    qx, qy, qz, qw = quaternion
    tq = tf.quaternion_matrix([qw, qx, qy, qz])
    return tf.concatenate_matrices(tx, tq)


class MeshcatBridge:
    def __init__(
        self,
        port: int,
        *,
        host: str = "127.0.0.1",
        meshcat_http_port: int = 7000,
        robot_mode: str = "pointcloud",
        verbose: bool = False,
    ):
        self._meshcat_http_port = meshcat_http_port  # stored for logging, not currently configurable
        self._viz = meshcat.Visualizer()
        self._verbose = verbose
        self._robot_mode = robot_mode
        log(f"[bridge] Meshcat server started at {self._viz.url()} (robot_mode={robot_mode})")
        self._host = host
        self._port = port
        self._geom_cache = {}
        self._material_cache = {}
        self._points_material_cache = {}
        self._last_geom_key = {}
        self._last_material_key = {}
        self._last_pose_key = {}
        self._last_instanced_paths = set()
        self._last_cloud_paths = set()
        self._meshcat_set_object_calls = 0
        self._meshcat_set_transform_calls = 0
        self._meshcat_delete_calls = 0
        # When rendering spheres, we can update per-link transforms (fast) instead of per-sphere transforms (slow).
        # These caches store the reference sphere centers (first received frame) for each robot+link.
        self._link_ref_points = {}  # link_path -> (N,3) ndarray
        self._link_sphere_order = {}  # link_path -> [sphere_index...]
        self._set_floor()

    def _viz_set_object(self, path: str, geom, material) -> None:
        self._meshcat_set_object_calls += 1
        self._viz[path].set_object(geom, material)

    def _viz_set_transform(self, path: str, matrix) -> None:
        self._meshcat_set_transform_calls += 1
        self._viz[path].set_transform(matrix)

    def _viz_delete(self, path: str) -> None:
        self._meshcat_delete_calls += 1
        self._viz[path].delete()

    def _set_floor(self):
        material = g.MeshLambertMaterial(color=0x777777, transparent=True, opacity=0.2)
        floor = g.Box([2.5, 2.5, 0.001])
        self._viz_set_object("/Floor", floor, material)
        self._viz_set_transform("/Floor", tf.translation_matrix([0, 0, -0.0005]))

    def clear(self):
        self._viz.delete()
        self._set_floor()
        self._last_geom_key.clear()
        self._last_material_key.clear()
        self._last_pose_key.clear()
        self._last_instanced_paths.clear()
        self._last_cloud_paths.clear()
        self._link_ref_points.clear()
        self._link_sphere_order.clear()

    def _get_material(self, rgba: List[float]):
        key = (rgba_to_int(rgba), rgba[3] if len(rgba) > 3 else 1.0)
        if key not in self._material_cache:
            self._material_cache[key] = material_from_rgba(rgba)
        return key, self._material_cache[key]

    @staticmethod
    def _pose_changed(path: str, pose_key: tuple, cache: Dict[str, tuple], tol: float = 1e-6) -> bool:
        last = cache.get(path)
        if last is None:
            cache[path] = pose_key
            return True
        if len(last) != len(pose_key):
            cache[path] = pose_key
            return True
        for a, b in zip(last, pose_key):
            if abs(a - b) > tol:
                cache[path] = pose_key
                return True
        return False

    def _maybe_set_object(self, path: str, geom_key, geom, material_key, material) -> bool:
        if self._last_geom_key.get(path) != geom_key or self._last_material_key.get(path) != material_key:
            self._viz_set_object(path, geom, material)
            self._last_geom_key[path] = geom_key
            self._last_material_key[path] = material_key
            return True
        return False

    def _set_robot_spheres(self, spheres: List[Dict]):
        if self._robot_mode == "pointcloud":
            self._set_robot_spheres_pointcloud(spheres)
            return

        if self._robot_mode == "spheres_naive":
            self._set_robot_spheres_naive(spheres)
            return

        # Default: spheres rendered with per-link rigid transforms (much faster than per-sphere updates).
        self._set_robot_spheres_by_link(spheres)

    @staticmethod
    def _rigid_transform(ref_points: np.ndarray, cur_points: np.ndarray) -> np.ndarray:
        """Return a 4x4 rigid transform T such that cur ~= (T @ [ref;1])."""
        if ref_points.shape != cur_points.shape:
            raise ValueError("ref_points and cur_points must have the same shape")
        if ref_points.ndim != 2 or ref_points.shape[1] != 3:
            raise ValueError("points must be shaped (N,3)")
        n = ref_points.shape[0]
        if n == 0:
            return np.eye(4)
        if n == 1:
            t = cur_points[0] - ref_points[0]
            out = np.eye(4)
            out[:3, 3] = t
            return out

        c_ref = ref_points.mean(axis=0)
        c_cur = cur_points.mean(axis=0)
        x = ref_points - c_ref
        y = cur_points - c_cur
        h = x.T @ y
        u, _, vt = np.linalg.svd(h)
        r = vt.T @ u.T
        # Enforce a proper rotation (det=+1).
        if np.linalg.det(r) < 0:
            vt[-1, :] *= -1
            r = vt.T @ u.T
        t = c_cur - r @ c_ref

        out = np.eye(4)
        out[:3, :3] = r
        out[:3, 3] = t
        return out

    def _set_robot_spheres_naive(self, spheres: List[Dict]):
        for s in spheres:
            center = s.get("center", [0.0, 0.0, 0.0])
            radius = s.get("radius", 0.05)
            robot_name = s.get("robot") or f"robot_{s.get('robot_id', 0)}"
            link = s.get("link", "link")
            sphere_idx = s.get("sphere_index", 0)
            rgba = s.get("rgba", [0.1, 0.6, 1.0, 0.8])

            path = f"robots/{robot_name}/{link}/sphere_{sphere_idx}"
            geom_key = ("sphere", radius)
            if geom_key not in self._geom_cache:
                self._geom_cache[geom_key] = g.Sphere(radius)
            material_key, material = self._get_material(rgba)
            self._maybe_set_object(path, geom_key, self._geom_cache[geom_key], material_key, material)

            pose_key = tuple(center)
            if self._pose_changed(path, pose_key, self._last_pose_key):
                self._viz_set_transform(path, tf.translation_matrix(center))

    def _set_robot_spheres_by_link(self, spheres: List[Dict]):
        by_link: Dict[str, Dict[int, Dict]] = {}
        for s in spheres:
            robot_name = s.get("robot") or f"robot_{s.get('robot_id', 0)}"
            link = s.get("link", "link")
            sphere_idx = int(s.get("sphere_index", 0))
            link_path = f"robots/{robot_name}/{link}"
            by_link.setdefault(link_path, {})[sphere_idx] = s

        for link_path, spheres_by_idx in by_link.items():
            order = self._link_sphere_order.get(link_path)
            ref_points = self._link_ref_points.get(link_path)

            if order is None or ref_points is None:
                order = sorted(spheres_by_idx.keys())
                if not order:
                    continue
                ref_pts = np.zeros((len(order), 3), dtype=np.float64)
                for i, sphere_idx in enumerate(order):
                    s = spheres_by_idx[sphere_idx]
                    center = s.get("center", [0.0, 0.0, 0.0])
                    ref_pts[i, :] = np.asarray(center, dtype=np.float64)

                    radius = s.get("radius", 0.05)
                    rgba = s.get("rgba", [0.1, 0.6, 1.0, 0.8])
                    sphere_path = f"{link_path}/sphere_{sphere_idx}"
                    geom_key = ("sphere", radius)
                    if geom_key not in self._geom_cache:
                        self._geom_cache[geom_key] = g.Sphere(radius)
                    material_key, material = self._get_material(rgba)
                    self._maybe_set_object(
                        sphere_path, geom_key, self._geom_cache[geom_key], material_key, material
                    )
                    # Store the reference center as the fixed local transform under the link.
                    self._viz_set_transform(sphere_path, tf.translation_matrix(center))

                # Link transform is identity in the reference frame.
                self._viz_set_transform(link_path, np.eye(4))
                self._link_sphere_order[link_path] = order
                self._link_ref_points[link_path] = ref_pts
                continue

            cur_pts = np.zeros((len(order), 3), dtype=np.float64)
            missing = False
            for i, sphere_idx in enumerate(order):
                s = spheres_by_idx.get(sphere_idx)
                if s is None:
                    missing = True
                    break
                center = s.get("center", [0.0, 0.0, 0.0])
                cur_pts[i, :] = np.asarray(center, dtype=np.float64)
            if missing:
                continue

            try:
                tform = self._rigid_transform(ref_points, cur_pts)
            except Exception:
                continue
            self._viz_set_transform(link_path, tform)

    def _set_robot_spheres_pointcloud(self, spheres: List[Dict]):
        by_robot: Dict[str, List[Dict]] = {}
        for s in spheres:
            robot_name = s.get("robot") or f"robot_{s.get('robot_id', 0)}"
            by_robot.setdefault(robot_name, []).append(s)

        for robot_name, rspheres in by_robot.items():
            points = np.zeros((3, len(rspheres)), dtype=np.float32)
            colors = np.zeros((3, len(rspheres)), dtype=np.float32)
            sizes = []
            for i, s in enumerate(rspheres):
                c = s.get("center", [0.0, 0.0, 0.0])
                rgba = s.get("rgba", [0.1, 0.6, 1.0, 0.8])
                points[:, i] = np.asarray(c, dtype=np.float32)
                colors[:, i] = np.asarray(rgba[:3], dtype=np.float32)
                sizes.append(float(s.get("radius", 0.01)))

            # Meshcat's PointsMaterial supports a single constant point size; use an average sphere radius.
            point_size = float(np.clip(np.mean(sizes) if sizes else 0.01, 0.001, 0.05))
            point_size_key = round(point_size, 6)
            path = f"robots/{robot_name}/spheres_cloud"
            geom = g.PointsGeometry(points, colors)
            material = self._points_material_cache.get(point_size_key)
            if material is None:
                material = g.PointsMaterial(size=point_size, color=0xFFFFFF)
                self._points_material_cache[point_size_key] = material
            self._viz[path].set_object(geom, material)

    def _set_objects(self, objects: List[Dict]):
        for obj in objects:
            name = obj.get("name", "object")
            obj_type = obj.get("type", "box")
            position = obj.get("position", [0.0, 0.0, 0.0])
            quat = obj.get("quaternion", [0.0, 0.0, 0.0, 1.0])
            rgba = obj.get("rgba", [0.8, 0.8, 0.8, 1.0])

            if len(quat) != 4 or not all(math.isfinite(x) for x in quat):
                quat = [0.0, 0.0, 0.0, 1.0]
            if len(position) != 3 or not all(math.isfinite(x) for x in position):
                position = [0.0, 0.0, 0.0]

            geom = None
            if obj_type == "box":
                size = obj.get("size", [0.05, 0.05, 0.05])
                geom_key = ("box", tuple(size))
                if geom_key not in self._geom_cache:
                    self._geom_cache[geom_key] = g.Box(size)
            elif obj_type == "sphere":
                radius = obj.get("radius", 0.02)
                geom_key = ("sphere", radius)
                if geom_key not in self._geom_cache:
                    self._geom_cache[geom_key] = g.Sphere(radius)
            elif obj_type == "cylinder":
                length = obj.get("length", 0.05)
                radius = obj.get("radius", 0.02)
                geom_key = ("cylinder", length, radius)
                if geom_key not in self._geom_cache:
                    self._geom_cache[geom_key] = g.Cylinder(length, radius)
            else:
                log(f"[bridge] Skipping unsupported object type '{obj_type}' for {name}")
                continue

            path = f"objects/{name}"
            material_key, material = self._get_material(rgba)
            geom = self._geom_cache[geom_key]
            created = self._maybe_set_object(path, geom_key, geom, material_key, material)

            pose_key = tuple(position + quat)
            if self._pose_changed(path, pose_key, self._last_pose_key):
                self._viz_set_transform(path, pose_to_matrix(position, quat))
            if created:
                log(f"[bridge] Created object {name} type={obj_type}")

    def handle_message(self, msg: Dict):
        t_start = time.perf_counter()
        obj0 = self._meshcat_set_object_calls
        xf0 = self._meshcat_set_transform_calls
        del0 = self._meshcat_delete_calls
        action = msg.get("action", "update")
        if action == "delete_all":
            self.clear()
            return

        if deleted := msg.get("deleted_objects"):
            for name in deleted:
                self._viz_delete(f"objects/{name}")
            if self._verbose:
                log(f"[bridge] Deleted {len(deleted)} objects")

        if robot_spheres := msg.get("robot_spheres"):
            t0 = time.perf_counter()
            self._set_robot_spheres(robot_spheres)
            if self._verbose:
                dt_ms = (time.perf_counter() - t0) * 1000.0
                log(f"[bridge] Rendered {len(robot_spheres)} robot spheres in {dt_ms:.1f} ms")
        if objects := msg.get("objects"):
            t0 = time.perf_counter()
            self._set_objects(objects)
            if self._verbose:
                dt_ms = (time.perf_counter() - t0) * 1000.0
                log(f"[bridge] Rendered {len(objects)} objects in {dt_ms:.1f} ms")
        if self._verbose:
            total_ms = (time.perf_counter() - t_start) * 1000.0
            log(f"[bridge] Message handled in {total_ms:.1f} ms")
            dobj = self._meshcat_set_object_calls - obj0
            dxf = self._meshcat_set_transform_calls - xf0
            ddel = self._meshcat_delete_calls - del0
            log(f"[bridge] Meshcat cmds: set_transform={dxf} set_object={dobj} delete={ddel}")


async def main():
    parser = argparse.ArgumentParser(description="Meshcat JSON bridge")
    parser.add_argument("--host", default="127.0.0.1", help="Listen host")
    parser.add_argument("--port", type=int, default=7600, help="Listen port for JSON socket")
    parser.add_argument("--meshcat-http-port", type=int, default=7000, help="Port to serve the Meshcat viewer")
    parser.add_argument(
        "--robot-mode",
        choices=["pointcloud", "spheres", "spheres_naive"],
        default="spheres",
        help="Robot rendering mode (spheres uses fast per-link transforms; spheres_naive updates each sphere individually).",
    )
    parser.add_argument(
        "--latest-only",
        action="store_true",
        help="When backlogged, drop queued messages and render only the latest update.",
    )
    parser.add_argument("--verbose", action="store_true", help="Print per-message timing logs")
    args = parser.parse_args()

    bridge = MeshcatBridge(
        port=args.port,
        host=args.host,
        meshcat_http_port=args.meshcat_http_port,
        robot_mode=args.robot_mode,
        verbose=args.verbose,
    )

    async def handle(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        peer = writer.get_extra_info("peername")
        log(f"[bridge] Connection from {peer}")
        last_rx = None
        try:
            while True:
                t0 = time.perf_counter()
                line = await reader.readline()
                t1 = time.perf_counter()
                if not line:
                    break
                dropped = 0
                if args.latest_only:
                    while True:
                        try:
                            line_next = await asyncio.wait_for(reader.readline(), timeout=0.0)
                        except asyncio.TimeoutError:
                            break
                        if not line_next:
                            break
                        line = line_next
                        dropped += 1
                    if dropped and bridge._verbose:
                        log(f"[bridge] Dropped {dropped} queued messages")
                    t1 = time.perf_counter()
                try:
                    if bridge._verbose:
                        dt_ms = None if last_rx is None else (t1 - last_rx) * 1000.0
                        last_rx = t1
                        t2 = time.perf_counter()
                        text = line.decode("utf-8")
                        t3 = time.perf_counter()
                        msg = json.loads(text)
                        t4 = time.perf_counter()
                        dt_str = "n/a" if dt_ms is None else f"{dt_ms:.1f}"
                        log(
                            f"[bridge] Received message (wait={((t1 - t0) * 1000.0):.1f} ms,"
                            f" decode={((t3 - t2) * 1000.0):.1f} ms,"
                            f" json={((t4 - t3) * 1000.0):.1f} ms,"
                            f" dt={dt_str} ms,"
                            f" bytes={len(line)})"
                        )
                    else:
                        msg = json.loads(line.decode("utf-8"))
                except Exception as exc:
                    log(f"[bridge] JSON decode failed: {exc}; line={line!r}")
                    continue
                bridge.handle_message(msg)
        finally:
            writer.close()
            await writer.wait_closed()
            log(f"[bridge] Connection closed {peer}")

    # Allow large scene updates (many objects) by raising the reader limit from the default 64 KiB.
    server = await asyncio.start_server(handle, args.host, args.port, limit=10 * 1024 * 1024)
    addrs = ", ".join(str(sock.getsockname()) for sock in server.sockets)
    log(f"[bridge] Listening on {addrs}")

    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(main())
