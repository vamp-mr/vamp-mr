#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
import sys
from typing import List
import time

import numpy as np


def _sphere_points(center: np.ndarray, *, n: int, radius: float, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    dirs = rng.normal(size=(n, 3)).astype(np.float64)
    norms = np.linalg.norm(dirs, axis=1, keepdims=True)
    dirs = dirs / np.clip(norms, 1e-12, None)
    u = rng.random(size=(n, 1)).astype(np.float64)
    r = (u ** (1.0 / 3.0)) * float(radius)
    pts = center.reshape(1, 3) + dirs * r
    return pts.astype(np.float32)


def _maybe_override_two_robot_bases(env, *, offset_x: float) -> None:
    info = env.info()
    if int(info.get("num_robots", 0)) < 2:
        return

    t0 = np.eye(4, dtype=float)
    t1 = np.eye(4, dtype=float)
    t1[:3, :3] = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    t1[0, 3] = float(offset_x)
    env.set_robot_base_transforms([t0.tolist(), t1.tolist()])


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Sanity test for VAMP multi-robot pointcloud collision checking (optional Meshcat viz)."
    )
    parser.add_argument("--vamp-environment", default="dual_gp4")
    parser.add_argument("--robot-id", type=int, default=1)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--sample-attempts", type=int, default=200)
    parser.add_argument("--override-bases", action="store_true")
    parser.add_argument("--base-offset-x", type=float, default=1.0)

    parser.add_argument("--n-points", type=int, default=5000)
    parser.add_argument("--cloud-radius", type=float, default=0.05)
    parser.add_argument("--filter-min-dist", type=float, default=0.015)
    parser.add_argument("--filter-max-range", type=float, default=2.0)
    parser.add_argument("--workspace-min", type=float, nargs=3, default=[-1.5, -1.5, -0.2])
    parser.add_argument("--workspace-max", type=float, nargs=3, default=[1.5, 1.5, 1.8])

    parser.add_argument("--self-padding", type=float, default=0.01)
    parser.add_argument("--r-min", type=float, default=0.01)
    parser.add_argument("--r-max", type=float, default=0.25)
    parser.add_argument("--r-point", type=float, default=0.0025)

    parser.add_argument("--meshcat", action="store_true")
    parser.add_argument("--meshcat-host", default="127.0.0.1")
    parser.add_argument("--meshcat-port", type=int, default=7600)
    args = parser.parse_args()

    # Prefer the in-tree build of the Python bindings when running from a source checkout.
    # This avoids accidentally picking up an older system-installed mr_planner_core.
    build_python = Path(__file__).resolve().parents[2] / "build" / "python"
    if build_python.exists():
        sys.path.insert(0, str(build_python))

    import mr_planner_core

    env = mr_planner_core.VampEnvironment(args.vamp_environment, vmax=1.0, seed=args.seed)
    if args.override_bases:
        _maybe_override_two_robot_bases(env, offset_x=args.base_offset_x)
    if args.meshcat:
        env.enable_meshcat(args.meshcat_host, args.meshcat_port)

    info = env.info()
    num_robots = int(info["num_robots"])
    if num_robots < 1:
        raise SystemExit("environment has no robots")
    if args.robot_id < 0 or args.robot_id >= num_robots:
        raise SystemExit(f"--robot-id out of range: {args.robot_id} (num_robots={num_robots})")

    joints: List[List[float]] = []
    for rid in range(num_robots):
        q = env.sample_collision_free_pose(rid, max_attempts=args.sample_attempts, self_only=False)
        joints.append(list(q))

    baseline_collision = bool(env.in_collision(joints, self_only=False))
    if baseline_collision:
        raise SystemExit("baseline pose is already in collision; try a different seed/environment")

    # Publish the sampled robot configuration so Meshcat shows the robots.
    if args.meshcat:
        env.set_joint_positions(joints, update_scene=True)

    ee_tf = np.asarray(env.end_effector_transform(args.robot_id, joints[args.robot_id]), dtype=np.float64)
    ee_pos = ee_tf[:3, 3].copy()

    raw_cloud = _sphere_points(ee_pos, n=args.n_points, radius=args.cloud_radius, seed=args.seed)

    filtered = mr_planner_core.filter_pointcloud(
        raw_cloud,
        min_dist=args.filter_min_dist,
        max_range=args.filter_max_range,
        origin=tuple(ee_pos.tolist()),
        workspace_min=tuple(args.workspace_min),
        workspace_max=tuple(args.workspace_max),
        cull=True,
    )

    # First validate that pointcloud collision checking is active by feeding the (unfiltered)
    # cloud around the end-effector, which should collide with the robot.
    env.set_pointcloud(filtered, r_min=args.r_min, r_max=args.r_max, r_point=args.r_point)
    if args.meshcat:
        env.update_scene()

    collision_with_pc = bool(env.in_collision(joints, self_only=False))
    if not collision_with_pc:
        raise SystemExit(
            "expected collision with pointcloud but got collision-free; try increasing --cloud-radius or --r-point"
        )

    # Now self-filter using the current pose; the remaining points should no longer collide.
    filtered_self = env.filter_self_from_pointcloud(filtered, joints, padding=args.self_padding)
    env.set_pointcloud(filtered_self, r_min=args.r_min, r_max=args.r_max, r_point=args.r_point)
    if args.meshcat:
        env.update_scene()

    collision_after_self_filter = bool(env.in_collision(joints, self_only=False))
    if collision_after_self_filter:
        raise SystemExit(
            "expected collision-free after self-filtering but got collision; try increasing --self-padding"
        )

    env.clear_pointcloud()
    if args.meshcat:
        time.sleep(30.0)
        env.update_scene()

    collision_cleared = bool(env.in_collision(joints, self_only=False))
    if collision_cleared:
        raise SystemExit("collision remained after clearing pointcloud")

    print(
        json.dumps(
            {
                "environment": args.vamp_environment,
                "num_robots": num_robots,
                "robot_id": args.robot_id,
                "n_points_raw": int(raw_cloud.shape[0]),
                "n_points_filtered": int(np.asarray(filtered).shape[0]),
                "n_points_filtered_self": int(np.asarray(filtered_self).shape[0]),
                "collision_with_pointcloud": collision_with_pc,
                "collision_after_self_filter": collision_after_self_filter,
                "ok": True,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
