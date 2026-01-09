#!/usr/bin/env python3

"""
RoboBallet dataset tools (VAMP, ROS-free).

Dataset format (v1)
-------------------
Output directory:
  <out>/
    meta.json
    shards/
      shard_000000.tar
      shard_000001.tar
      ...

Each sample is stored in its shard as:
  <key>.json         # metadata (schema, env, sampling cfg, metrics)
  <key>.npz          # numpy arrays (scene + trajectories) via np.savez_compressed

Keys are zero-padded decimal integers, e.g. "000000123".

The .npz contains (at minimum):
  - base_transforms: (R,4,4) float32
  - start: (R,D) float32
  - goal: (R,D) float32
  - task_transforms: (R,4,4) float32
  - obstacles: (N,10) float32 where columns are [x,y,z,qx,qy,qz,qw,l,w,h]
Optionally on success:
  - traj_raw: (R,T0,D) float32
  - times_raw: (T0,) float32
  - traj: (R,T1,D) float32
  - times: (T1,) float32
"""

from __future__ import annotations

import argparse
import io
import json
import os
import random
import shutil
import sys
import tarfile
import time
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Optional, Tuple

import numpy as np


SCHEMA_NAME = "mr_planner_core.roboballet_dataset"
SCHEMA_VERSION = 1


def _json_bytes(obj: Any) -> bytes:
    return (json.dumps(obj, separators=(",", ":"), sort_keys=True) + "\n").encode("utf-8")


def _npz_bytes(arrays: Dict[str, np.ndarray], *, compressed: bool) -> bytes:
    buf = io.BytesIO()
    if compressed:
        np.savez_compressed(buf, **arrays)
    else:
        np.savez(buf, **arrays)
    return buf.getvalue()


def _tar_add_bytes(tar: tarfile.TarFile, name: str, data: bytes) -> None:
    info = tarfile.TarInfo(name=name)
    info.size = len(data)
    info.mtime = int(time.time())
    tar.addfile(info, io.BytesIO(data))


class ShardedTarWriter:
    def __init__(
        self,
        out_dir: Path,
        *,
        samples_per_shard: int,
        shard_index_start: int = 0,
        tar_mode: str = "w",
        npz_compressed: bool = True,
    ):
        if samples_per_shard <= 0:
            raise ValueError("samples_per_shard must be > 0")
        self._out_dir = out_dir
        self._shards_dir = out_dir / "shards"
        self._shards_dir.mkdir(parents=True, exist_ok=True)
        self._samples_per_shard = int(samples_per_shard)
        self._tar_mode = tar_mode
        self._npz_compressed = bool(npz_compressed)
        self._shard_idx = int(shard_index_start)
        self._in_shard = 0
        self._tar: Optional[tarfile.TarFile] = None
        self._open_new_shard()

    def _open_new_shard(self) -> None:
        if self._tar is not None:
            self._tar.close()
        shard_path = self._shards_dir / f"shard_{self._shard_idx:06d}.tar"
        self._tar = tarfile.open(shard_path, mode=self._tar_mode)
        self._in_shard = 0
        self._shard_idx += 1

    def write(self, *, key: str, record: Dict[str, Any], arrays: Dict[str, np.ndarray]) -> None:
        if self._tar is None:
            raise RuntimeError("writer is closed")
        if self._in_shard >= self._samples_per_shard:
            self._open_new_shard()
        _tar_add_bytes(self._tar, f"{key}.json", _json_bytes(record))
        _tar_add_bytes(self._tar, f"{key}.npz", _npz_bytes(arrays, compressed=self._npz_compressed))
        self._in_shard += 1

    def close(self) -> None:
        if self._tar is not None:
            self._tar.close()
            self._tar = None


def iter_tar_samples(dataset_dir: Path) -> Iterator[Tuple[str, Dict[str, Any], Dict[str, np.ndarray]]]:
    shards_dir = dataset_dir / "shards"
    if not shards_dir.is_dir():
        raise RuntimeError(f"Missing shards dir: {shards_dir}")

    for shard in sorted(shards_dir.glob("shard_*.tar")):
        with tarfile.open(shard, mode="r:*") as tar:
            members = [m for m in tar.getmembers() if m.isfile()]
            by_key: Dict[str, Dict[str, tarfile.TarInfo]] = {}
            for m in members:
                stem, dot, ext = m.name.rpartition(".")
                if not dot or not ext:
                    continue
                by_key.setdefault(stem, {})[ext] = m

            for key, exts in sorted(by_key.items()):
                if "json" not in exts or "npz" not in exts:
                    continue
                jf = tar.extractfile(exts["json"])
                nf = tar.extractfile(exts["npz"])
                if jf is None or nf is None:
                    continue
                record = json.loads(jf.read().decode("utf-8"))
                with np.load(io.BytesIO(nf.read())) as data:
                    arrays = {k: data[k] for k in data.files}
                yield key, record, arrays


def load_tar_sample(dataset_dir: Path, key: str) -> Tuple[Dict[str, Any], Dict[str, np.ndarray]]:
    shards_dir = dataset_dir / "shards"
    if not shards_dir.is_dir():
        raise RuntimeError(f"Missing shards dir: {shards_dir}")

    json_name = f"{key}.json"
    npz_name = f"{key}.npz"
    for shard in sorted(shards_dir.glob("shard_*.tar")):
        with tarfile.open(shard, mode="r:*") as tar:
            try:
                jm = tar.getmember(json_name)
                nm = tar.getmember(npz_name)
            except KeyError:
                continue
            jf = tar.extractfile(jm)
            nf = tar.extractfile(nm)
            if jf is None or nf is None:
                raise RuntimeError(f"Failed to extract {key} from {shard}")
            record = json.loads(jf.read().decode("utf-8"))
            with np.load(io.BytesIO(nf.read())) as data:
                arrays = {k: data[k] for k in data.files}
            return record, arrays

    raise KeyError(f"Sample not found: {key}")


def write_meta(out_dir: Path, meta: Dict[str, Any]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    meta_path = out_dir / "meta.json"
    meta_payload = dict(meta)
    meta_payload.setdefault("schema", SCHEMA_NAME)
    meta_payload.setdefault("schema_version", SCHEMA_VERSION)
    meta_path.write_text(json.dumps(meta_payload, indent=2, sort_keys=True) + "\n")


def _mat4_list_to_array(mats: Optional[List[List[List[float]]]], *, num_robots: int) -> np.ndarray:
    if mats is None:
        out = np.zeros((num_robots, 4, 4), dtype=np.float32)
        for i in range(num_robots):
            out[i] = np.eye(4, dtype=np.float32)
        return out
    arr = np.asarray(mats, dtype=np.float32)
    if arr.shape != (num_robots, 4, 4):
        raise ValueError(f"expected transforms shape ({num_robots},4,4), got {arr.shape}")
    return arr


def _obstacles_to_array(obstacles: List[Dict[str, float]]) -> np.ndarray:
    # Boxes only: [x,y,z,qx,qy,qz,qw,l,w,h]
    out = np.zeros((len(obstacles), 10), dtype=np.float32)
    for i, o in enumerate(obstacles):
        out[i, 0] = float(o.get("x", 0.0))
        out[i, 1] = float(o.get("y", 0.0))
        out[i, 2] = float(o.get("z", 0.0))
        out[i, 3] = float(o.get("qx", 0.0))
        out[i, 4] = float(o.get("qy", 0.0))
        out[i, 5] = float(o.get("qz", 0.0))
        out[i, 6] = float(o.get("qw", 1.0))
        out[i, 7] = float(o.get("length", 0.0))
        out[i, 8] = float(o.get("width", 0.0))
        out[i, 9] = float(o.get("height", 0.0))
    return out


def _load_solution_csv(csv_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    import csv
    import re

    def nonempty(tokens):
        return [t.strip() for t in tokens if t is not None and t.strip() != ""]

    with csv_path.open("r", newline="") as f:
        reader = csv.reader(f)
        try:
            header_raw = next(reader)
        except StopIteration as exc:
            raise RuntimeError(f"Empty CSV: {csv_path}") from exc
        header = nonempty(header_raw)
        if not header or header[0] != "time":
            raise RuntimeError(f"Invalid CSV header: {header[:5]}")

        spec: List[Tuple[int, int]] = []
        for col in header[1:]:
            m = re.match(r"^q(\d+)_(\d+)$", col)
            if not m:
                raise RuntimeError(f"Invalid joint column name: {col!r}")
            spec.append((int(m.group(1)), int(m.group(2))))
        if not spec:
            raise RuntimeError("No joint columns found in CSV")

        num_robots = max(r for r, _ in spec) + 1
        dof_by_robot = [0 for _ in range(num_robots)]
        for r, j in spec:
            dof_by_robot[r] = max(dof_by_robot[r], j + 1)

        dof = dof_by_robot[0]
        if any(d != dof for d in dof_by_robot):
            raise RuntimeError(f"Mixed-DOF CSV not supported: dof_by_robot={dof_by_robot}")

        times: List[float] = []
        traj: List[List[List[float]]] = [[] for _ in range(num_robots)]
        for row_raw in reader:
            row = nonempty(row_raw)
            if not row:
                continue
            if len(row) != 1 + len(spec):
                raise RuntimeError(f"Row has {len(row)} columns, expected {1 + len(spec)}")
            t = float(row[0])
            times.append(t)
            idx = 1
            for r in range(num_robots):
                q = [float(row[idx + k]) for k in range(dof)]
                idx += dof
                traj[r].append(q)

    if not times:
        raise RuntimeError(f"CSV contains no trajectory rows: {csv_path}")

    times_arr = np.asarray(times, dtype=np.float32)
    traj_arr = np.asarray(traj, dtype=np.float32)
    return times_arr, traj_arr


def _metrics_from_traj(times: np.ndarray, traj: np.ndarray, *, move_tol: float = 1e-5) -> Tuple[float, float]:
    if times.ndim != 1:
        raise ValueError("times must be 1D")
    if traj.ndim != 3:
        raise ValueError("traj must be (R,T,D)")
    if traj.shape[1] != times.shape[0]:
        raise ValueError("traj timestep count mismatch")
    makespan = float(times[-1])
    tol_sq = float(move_tol * move_tol)
    flowtime = 0.0
    for r in range(traj.shape[0]):
        last: Optional[float] = None
        for i in range(1, traj.shape[1]):
            d = traj[r, i, :] - traj[r, i - 1, :]
            if float(np.dot(d, d)) > tol_sq:
                last = float(times[i])
        if last is not None:
            flowtime += last
    return makespan, flowtime


def main() -> int:
    parser = argparse.ArgumentParser(description="RoboBallet dataset tools (VAMP, ROS-free).")
    sub = parser.add_subparsers(dest="cmd", required=True)

    init = sub.add_parser("init", help="Initialize dataset directory with meta.json (no samples generated).")
    init.add_argument("--output-dir", required=True)

    gen = sub.add_parser("generate", help="Generate planning dataset shards.")
    gen.add_argument("--output-dir", required=True)
    gen.add_argument("--vamp-environment", default="roboballet_panda4")
    gen.add_argument("--planner", default="composite_rrt", choices=["composite_rrt", "cbs_prm"])
    gen.add_argument("--planning-time", type=float, default=8.0)
    gen.add_argument("--shortcut-time", type=float, default=0.2)
    gen.add_argument("--dt", type=float, default=0.1)
    gen.add_argument("--vmax", type=float, default=1.0)
    gen.add_argument("--seed", type=int, default=1)
    gen.add_argument(
        "--num-samples",
        type=int,
        default=1000,
        help="Number of samples to write (successes unless --keep-failures).",
    )
    gen.add_argument("--max-attempts", type=int, default=0, help="Stop after this many attempts (0=unlimited).")
    gen.add_argument("--keep-failures", action="store_true", help="Store failed attempts (no trajectories).")
    gen.add_argument("--samples-per-shard", type=int, default=1000)
    gen.add_argument("--shard-index-start", type=int, default=0)
    gen.add_argument("--sample-index-start", type=int, default=0)
    gen.add_argument("--work-dir", default="", help="Work directory for env.plan outputs (defaults inside output-dir).")
    gen.add_argument(
        "--write-plan-files",
        action="store_true",
        help="Keep env.plan outputs on disk under --work-dir (useful for debugging; slower).",
    )
    gen.add_argument(
        "--no-npz-compress",
        action="store_true",
        help="Disable np.savez_compressed (faster, larger shards).",
    )
    gen.add_argument("--profile", action="store_true", help="Print per-attempt timing breakdowns to stderr.")
    gen.add_argument("--profile-out", default="", help="Write JSONL timing to this file path (append).")
    gen.add_argument(
        "--profile-every",
        type=int,
        default=25,
        help="If --profile, print a summary every N attempts (0 disables periodic summaries).",
    )

    gen.add_argument("--base-mode", choices=["fixed", "rails"], default="rails")
    gen.add_argument("--rail-length", type=float, default=1.6)
    gen.add_argument("--rail-offset", type=float, default=0.55)
    gen.add_argument("--rail-z-table", type=float, default=0.1)
    gen.add_argument("--rail-z-ceiling", type=float, default=1.3)
    gen.add_argument("--base-attempts", type=int, default=50)

    gen.add_argument("--obstacle-mode", choices=["struts", "random"], default="random")
    gen.add_argument("--num-obstacles", type=int, default=30)
    gen.add_argument("--add-table", action="store_true")
    gen.add_argument("--table-xy", type=float, default=1.4)
    gen.add_argument("--no-welding-sticks", action="store_true")

    gen.add_argument("--task-offset", type=float, default=0.02)
    gen.add_argument("--task-max-dir-deg", type=float, default=22.5)
    gen.add_argument("--tol-pos", type=float, default=0.025)
    gen.add_argument("--tol-ang-deg", type=float, default=15.0)
    gen.add_argument("--ik-restarts", type=int, default=25)
    gen.add_argument("--ik-iters", type=int, default=120)
    gen.add_argument("--plan-attempts", type=int, default=5, help="Resample goal sets up to this many times per attempt.")
    gen.add_argument("--require-linear-path", action="store_true")
    gen.add_argument(
        "--goal-mode",
        choices=["ik_obstacle", "joint"],
        default="ik_obstacle",
        help="Goal sampling mode. ik_obstacle matches RoboBallet-style end-effector tasks; joint is much faster.",
    )
    gen.add_argument(
        "--min-goal-joint-dist",
        type=float,
        default=0.5,
        help="Minimum joint-space distance (L2) between start and goal per robot (goal_mode=joint).",
    )

    gen.add_argument("--roadmap-samples", type=int, default=600)
    gen.add_argument("--roadmap-max-dist", type=float, default=2.0)

    view = sub.add_parser("view", help="Visualize one or more dataset samples in Meshcat.")
    view.add_argument("--dataset-dir", required=True)
    view.add_argument("--key", action="append", default=[], help="Sample key to play (repeatable). If omitted, plays the first sample.")
    view.add_argument("--meshcat-host", default="127.0.0.1")
    view.add_argument("--meshcat-port", type=int, default=7600)
    view.add_argument("--meshcat-rate", type=float, default=1.0)
    view.add_argument("--play", choices=["none", "raw", "shortcutted", "both"], default="shortcutted")

    ls = sub.add_parser("ls", help="List samples in an existing dataset (keys only).")
    ls.add_argument("--dataset-dir", required=True)
    ls.add_argument("--limit", type=int, default=10)

    args = parser.parse_args()

    if args.cmd == "init":
        out_dir = Path(args.output_dir).expanduser().resolve()
        write_meta(out_dir, meta={"created_at": time.time()})
        return 0

    if args.cmd == "generate":
        import mr_planner_core
        import plan_roboballet_reaching as reach

        out_dir = Path(args.output_dir).expanduser().resolve()
        work_dir = Path(args.work_dir).expanduser().resolve() if args.work_dir else (out_dir / f"_work_{os.getpid()}")
        work_dir.mkdir(parents=True, exist_ok=True)

        env = mr_planner_core.VampEnvironment(args.vamp_environment, vmax=args.vmax, seed=args.seed)
        info = env.info()
        num_robots = int(info["num_robots"])

        write_meta(
            out_dir,
            meta={
                "created_at": time.time(),
                "vamp_environment": args.vamp_environment,
                "num_robots": num_robots,
                "planner": args.planner,
                "planning_time": args.planning_time,
                "shortcut_time": args.shortcut_time,
                "dt": args.dt,
                "vmax": args.vmax,
                "seed": args.seed,
                "base_mode": args.base_mode,
                "obstacle_mode": args.obstacle_mode,
                "goal_mode": args.goal_mode,
                "write_plan_files": bool(args.write_plan_files),
                "npz_compressed": not bool(args.no_npz_compress),
            },
        )

        writer = ShardedTarWriter(
            out_dir,
            samples_per_shard=args.samples_per_shard,
            shard_index_start=args.shard_index_start,
            npz_compressed=not bool(args.no_npz_compress),
        )

        written = 0
        attempts = 0
        attempt_idx = int(args.sample_index_start)
        max_dir_angle = float(args.task_max_dir_deg) * np.pi / 180.0
        tol_ang = float(args.tol_ang_deg) * np.pi / 180.0

        profile_file = None
        if str(args.profile_out).strip() != "":
            profile_path = Path(str(args.profile_out)).expanduser().resolve()
            profile_path.parent.mkdir(parents=True, exist_ok=True)
            profile_file = profile_path.open("a", encoding="utf-8")

        def write_profile(entry: Dict[str, Any]) -> None:
            if profile_file is None:
                return
            profile_file.write(json.dumps(entry, sort_keys=True) + "\n")
            profile_file.flush()

        sums_attempt: Dict[str, float] = {}
        sums_success: Dict[str, float] = {}
        successes = 0

        while written < int(args.num_samples):
            if args.max_attempts > 0 and attempts >= int(args.max_attempts):
                break
            attempts += 1

            key = f"{attempt_idx:09d}"
            seed = int(args.seed) + attempt_idx
            rng = random.Random(seed)

            timing: Dict[str, float] = {}
            counts: Dict[str, int] = {
                "attempt_idx": int(attempt_idx),
                "ik_calls": 0,
                "ik_success": 0,
                "goal_sets": 0,
                "base_tries": 0,
                "obstacle_rejects": 0,
            }
            t_attempt0 = time.perf_counter()

            try:
                env.set_random_seed(seed)
            except Exception:
                pass

            t0 = time.perf_counter()
            env.reset_scene(reset_sim=True)
            timing["reset_scene_sec"] = time.perf_counter() - t0

            def finalize_attempt(*, success: bool, error: str = "") -> None:
                nonlocal successes
                timing["total_sec"] = time.perf_counter() - t_attempt0
                for k, v in timing.items():
                    sums_attempt[k] = sums_attempt.get(k, 0.0) + float(v)
                if success:
                    successes += 1
                    for k, v in timing.items():
                        sums_success[k] = sums_success.get(k, 0.0) + float(v)

                profile_entry = {
                    "key": key,
                    "seed": seed,
                    "success": bool(success),
                    "error": str(error) if error else "",
                    "timing": timing,
                    "counts": counts,
                }
                write_profile(profile_entry)

                if args.profile:
                    msg = f"[profile] key={key} success={int(success)} total={timing['total_sec']:.3f}s"
                    if error:
                        msg += f" error={error}"
                    print(msg, file=sys.stderr, flush=True)

                    every = int(args.profile_every)
                    if every > 0 and (attempts % every) == 0:
                        def mean(sums: Dict[str, float], n: int, k: str) -> float:
                            if n <= 0:
                                return 0.0
                            return float(sums.get(k, 0.0)) / float(n)

                        sr = (float(successes) / float(attempts)) if attempts > 0 else 0.0
                        print(
                            "[profile] summary"
                            f" attempts={attempts}"
                            f" successes={successes}"
                            f" success_rate={sr:.3f}"
                            f" mean_total={mean(sums_attempt, attempts, 'total_sec'):.3f}s"
                            f" mean_goal={mean(sums_attempt, attempts, 'sample_goal_sec'):.3f}s"
                            f" mean_ik={mean(sums_attempt, attempts, 'ik_sec'):.3f}s"
                            f" mean_plan={mean(sums_attempt, attempts, 'plan_sec'):.3f}s"
                            f" mean_write={mean(sums_attempt, attempts, 'write_shard_sec'):.3f}s",
                            file=sys.stderr,
                            flush=True,
                        )

            scene_boxes: List[Dict[str, float]] = []
            if args.add_table:
                t0 = time.perf_counter()
                table = reach.make_table(size_xy=args.table_xy)
                env.add_object(reach.to_object(table, shape="Box", state="Static"))
                scene_boxes.append(table)
                timing["add_table_sec"] = time.perf_counter() - t0

            if not args.no_welding_sticks:
                t0 = time.perf_counter()
                reach.add_welding_sticks(env, num_robots)
                timing["add_welding_sticks_sec"] = time.perf_counter() - t0

            base_transforms: Optional[List[List[List[float]]]] = None
            q_start: Optional[List[List[float]]] = None
            t0 = time.perf_counter()
            if args.base_mode == "rails":
                for _ in range(max(1, int(args.base_attempts))):
                    counts["base_tries"] += 1
                    base_transforms = reach.sample_rail_base_transforms(
                        num_robots=num_robots,
                        rail_length=args.rail_length,
                        rail_offset=args.rail_offset,
                        z_table=args.rail_z_table,
                        z_ceiling=args.rail_z_ceiling,
                        rng=rng,
                    )
                    env.set_robot_base_transforms(base_transforms)
                    try:
                        q_start = reach.sample_collision_free_joint_matrix(env, num_robots, max_attempts=4000)
                    except Exception:
                        continue
                    if not env.in_collision(q_start, self_only=False):
                        break
                if q_start is None:
                    timing["sample_start_sec"] = time.perf_counter() - t0
                    finalize_attempt(success=False, error="Failed to sample collision-free start/base")
                    attempt_idx += 1
                    continue
            else:
                q_start = reach.sample_collision_free_joint_matrix(env, num_robots, max_attempts=4000)
            timing["sample_start_sec"] = time.perf_counter() - t0

            obstacles: List[Dict[str, float]] = []
            t0 = time.perf_counter()
            if args.obstacle_mode == "struts":
                obstacles = reach.make_strut_obstacles()
                for obs in obstacles:
                    qx, qy, qz, qw = reach.yaw_to_quaternion(float(obs.get("yaw", 0.0)))
                    obs["qx"], obs["qy"], obs["qz"], obs["qw"] = qx, qy, qz, qw
                    env.add_object(reach.to_object(obs, shape="Box", state="Static"))
                scene_boxes.extend(obstacles)

                if env.in_collision(q_start, self_only=False):
                    try:
                        q_start = reach.sample_collision_free_joint_matrix(env, num_robots, max_attempts=8000)
                    except Exception:
                        timing["add_obstacles_sec"] = time.perf_counter() - t0
                        finalize_attempt(success=False, error="Failed to resample collision-free start after struts")
                        attempt_idx += 1
                        continue
                    if env.in_collision(q_start, self_only=False):
                        timing["add_obstacles_sec"] = time.perf_counter() - t0
                        finalize_attempt(success=False, error="Start collides after struts")
                        attempt_idx += 1
                        continue
            else:
                obstacles_ok = True
                for idx in range(int(args.num_obstacles)):
                    sampled = False
                    for _ in range(5000):
                        obs = reach.make_random_cuboid(name=f"obs_{idx:02d}", table_xy=args.table_xy, rng=rng)
                        env.add_object(reach.to_object(obs, shape="Box", state="Static"))
                        if env.in_collision(q_start, self_only=False):
                            env.remove_object(obs["name"])
                            counts["obstacle_rejects"] += 1
                            continue
                        obstacles.append(obs)
                        sampled = True
                        break
                    if not sampled:
                        obstacles_ok = False
                        break
                if not obstacles_ok:
                    timing["add_obstacles_sec"] = time.perf_counter() - t0
                    finalize_attempt(success=False, error="Failed to sample obstacles")
                    attempt_idx += 1
                    continue
                scene_boxes.extend(obstacles)
            timing["add_obstacles_sec"] = time.perf_counter() - t0

            t0 = time.perf_counter()
            env.update_scene()
            timing["update_scene_sec"] = time.perf_counter() - t0

            task_poses: List[List[List[float]]] = []
            q_goal: Optional[List[List[float]]] = None
            ik_time_sec = 0.0

            t0 = time.perf_counter()
            if args.goal_mode == "joint":
                min_dist = float(args.min_goal_joint_dist)
                min_dist_sq = min_dist * min_dist
                for _ in range(max(1, int(args.plan_attempts))):
                    counts["goal_sets"] += 1
                    try:
                        goals = reach.sample_collision_free_joint_matrix(env, num_robots, max_attempts=4000)
                    except Exception:
                        continue
                    too_close = False
                    for rid in range(num_robots):
                        d = np.asarray(goals[rid], dtype=np.float64) - np.asarray(q_start[rid], dtype=np.float64)
                        if float(np.dot(d, d)) < min_dist_sq:
                            too_close = True
                            break
                    if too_close:
                        continue
                    if args.require_linear_path and env.motion_in_collision(q_start, goals, step_size=0.1, self_only=False):
                        continue
                    q_goal = goals
                    task_poses = [env.end_effector_transform(rid, q_goal[rid]) for rid in range(num_robots)]
                    break
            else:
                for _ in range(max(1, int(args.plan_attempts))):
                    counts["goal_sets"] += 1
                    goals = []
                    poses = []
                    ok = True
                    for rid in range(num_robots):
                        found = False
                        for _ in range(500):
                            obs = rng.choice(obstacles)
                            T = reach.sample_task_on_obstacle(
                                obstacle=obs,
                                offset=args.task_offset,
                                max_dir_angle_rad=max_dir_angle,
                                rng=rng,
                            )
                            counts["ik_calls"] += 1
                            t_ik0 = time.perf_counter()
                            q_sol = env.inverse_kinematics(
                                rid,
                                T,
                                seed=q_start[rid],
                                fixed_joints=q_start,
                                max_restarts=args.ik_restarts,
                                max_iters=args.ik_iters,
                                tol_pos=args.tol_pos,
                                tol_ang=tol_ang,
                                step_scale=1.0,
                                damping=1e-3,
                                self_only=False,
                            )
                            ik_time_sec += time.perf_counter() - t_ik0
                            if q_sol is None:
                                continue
                            counts["ik_success"] += 1
                            goals.append(q_sol)
                            poses.append(T)
                            found = True
                            break
                        if not found:
                            ok = False
                            break
                    if not ok:
                        continue
                    if env.in_collision(goals, self_only=False):
                        continue
                    if args.require_linear_path and env.motion_in_collision(q_start, goals, step_size=0.1, self_only=False):
                        continue
                    task_poses = poses
                    q_goal = goals
                    break

            timing["sample_goal_sec"] = time.perf_counter() - t0
            if ik_time_sec > 0.0:
                timing["ik_sec"] = ik_time_sec

            dof = len(q_start[0])

            record: Dict[str, Any] = {
                "schema": SCHEMA_NAME,
                "schema_version": SCHEMA_VERSION,
                "key": key,
                "seed": seed,
                "vamp_environment": args.vamp_environment,
                "num_robots": num_robots,
                "dof": dof,
                "base_mode": args.base_mode,
                "obstacle_mode": args.obstacle_mode,
                "goal_mode": args.goal_mode,
                "planner": {
                    "name": args.planner,
                    "planning_time": args.planning_time,
                    "shortcut_time": args.shortcut_time,
                    "dt": args.dt,
                    "vmax": args.vmax,
                    "roadmap_samples": args.roadmap_samples,
                    "roadmap_max_dist": args.roadmap_max_dist,
                },
                "sampling": {
                    "add_table": bool(args.add_table),
                    "table_xy": float(args.table_xy),
                    "welding_sticks": not bool(args.no_welding_sticks),
                    "task_offset": float(args.task_offset),
                    "task_max_dir_deg": float(args.task_max_dir_deg),
                    "tol_pos": float(args.tol_pos),
                    "tol_ang_deg": float(args.tol_ang_deg),
                    "ik_restarts": int(args.ik_restarts),
                    "ik_iters": int(args.ik_iters),
                    "require_linear_path": bool(args.require_linear_path),
                    "min_goal_joint_dist": float(args.min_goal_joint_dist),
                },
                "success": False,
            }

            arrays: Dict[str, np.ndarray] = {
                "base_transforms": _mat4_list_to_array(base_transforms, num_robots=num_robots),
                "start": np.asarray(q_start, dtype=np.float32),
                "goal": np.asarray(q_goal, dtype=np.float32) if q_goal is not None else np.zeros((num_robots, dof), dtype=np.float32),
                "task_transforms": _mat4_list_to_array(task_poses if task_poses else None, num_robots=num_robots),
                "obstacles": _obstacles_to_array(scene_boxes),
            }

            if q_goal is None:
                record["error"] = "Failed to sample a collision-free multi-robot IK goal set"
                if args.keep_failures:
                    t0 = time.perf_counter()
                    writer.write(key=key, record=record, arrays=arrays)
                    timing["write_shard_sec"] = time.perf_counter() - t0
                    written += 1
                finalize_attempt(success=False, error=record["error"])
                attempt_idx += 1
                continue

            t0 = time.perf_counter()
            try:
                if args.write_plan_files:
                    plan_dir = work_dir / "plan" / key
                    plan_dir.mkdir(parents=True, exist_ok=True)
                    res = env.plan(
                        planner=args.planner,
                        planning_time=args.planning_time,
                        shortcut_time=args.shortcut_time,
                        seed=seed,
                        dt=args.dt,
                        vmax=args.vmax,
                        start=q_start,
                        goal=q_goal,
                        output_dir=str(plan_dir),
                        write_tpg=False,
                        roadmap_samples=args.roadmap_samples,
                        roadmap_max_dist=args.roadmap_max_dist,
                        write_files=True,
                        return_trajectories=False,
                    )
                else:
                    res = env.plan(
                        planner=args.planner,
                        planning_time=args.planning_time,
                        shortcut_time=args.shortcut_time,
                        seed=seed,
                        dt=args.dt,
                        vmax=args.vmax,
                        start=q_start,
                        goal=q_goal,
                        output_dir="",
                        write_tpg=False,
                        roadmap_samples=args.roadmap_samples,
                        roadmap_max_dist=args.roadmap_max_dist,
                        write_files=False,
                        return_trajectories=True,
                    )
            except TypeError:
                # Backwards-compatible fallback (older bindings without write_files/return_trajectories).
                plan_dir = work_dir / "plan"
                shutil.rmtree(plan_dir, ignore_errors=True)
                plan_dir.mkdir(parents=True, exist_ok=True)
                res = env.plan(
                    planner=args.planner,
                    planning_time=args.planning_time,
                    shortcut_time=args.shortcut_time,
                    seed=seed,
                    dt=args.dt,
                    vmax=args.vmax,
                    start=q_start,
                    goal=q_goal,
                    output_dir=str(plan_dir),
                    write_tpg=False,
                    roadmap_samples=args.roadmap_samples,
                    roadmap_max_dist=args.roadmap_max_dist,
                )
            except Exception as exc:
                timing["plan_sec"] = time.perf_counter() - t0
                record["error"] = f"plan failed: {exc}"
                if args.keep_failures:
                    t1 = time.perf_counter()
                    writer.write(key=key, record=record, arrays=arrays)
                    timing["write_shard_sec"] = timing.get("write_shard_sec", 0.0) + (time.perf_counter() - t1)
                    written += 1
                finalize_attempt(success=False, error=record["error"])
                attempt_idx += 1
                continue
            timing["plan_sec"] = time.perf_counter() - t0

            record["success"] = True
            record["planner_time_sec"] = float(res.get("planner_time_sec", 0.0))
            record["init_time_sec"] = float(res.get("init_time_sec", 0.0))
            record["n_node_expanded"] = int(res.get("n_node_expanded", 0))
            record["n_col_checks"] = int(res.get("n_col_checks", 0))

            t0 = time.perf_counter()
            try:
                if "traj_raw" in res and "times_raw" in res:
                    times_raw = np.asarray(res["times_raw"], dtype=np.float32)
                    traj_raw = np.asarray(res["traj_raw"], dtype=np.float32)
                    times = np.asarray(res["times"], dtype=np.float32)
                    traj = np.asarray(res["traj"], dtype=np.float32)
                else:
                    times_raw, traj_raw = _load_solution_csv(Path(res["solution_raw_csv"]))
                    times, traj = _load_solution_csv(Path(res["solution_csv"]))

                makespan_pre, flowtime_pre = _metrics_from_traj(times_raw, traj_raw)
                makespan_post, flowtime_post = _metrics_from_traj(times, traj)
                record["pre_shortcut_makespan"] = makespan_pre
                record["pre_shortcut_flowtime"] = flowtime_pre
                record["post_shortcut_makespan"] = makespan_post
                record["post_shortcut_flowtime"] = flowtime_post
                arrays["times_raw"] = times_raw
                arrays["traj_raw"] = traj_raw
                arrays["times"] = times
                arrays["traj"] = traj
            except Exception as exc:
                record["traj_error"] = str(exc)
            timing["load_traj_sec"] = time.perf_counter() - t0

            t0 = time.perf_counter()
            writer.write(key=key, record=record, arrays=arrays)
            timing["write_shard_sec"] = timing.get("write_shard_sec", 0.0) + (time.perf_counter() - t0)
            written += 1
            finalize_attempt(success=True)
            attempt_idx += 1

        writer.close()
        if profile_file is not None:
            profile_file.close()

        if args.profile:
            def mean(sums: Dict[str, float], n: int, k: str) -> float:
                if n <= 0:
                    return 0.0
                return float(sums.get(k, 0.0)) / float(n)

            sr = (float(successes) / float(attempts)) if attempts > 0 else 0.0
            print(
                "[profile] done"
                f" attempts={attempts}"
                f" successes={successes}"
                f" success_rate={sr:.3f}"
                f" mean_total={mean(sums_attempt, attempts, 'total_sec'):.3f}s"
                f" mean_total_success={mean(sums_success, successes, 'total_sec'):.3f}s"
                f" mean_goal={mean(sums_attempt, attempts, 'sample_goal_sec'):.3f}s"
                f" mean_ik={mean(sums_attempt, attempts, 'ik_sec'):.3f}s"
                f" mean_plan={mean(sums_attempt, attempts, 'plan_sec'):.3f}s"
                f" mean_write={mean(sums_attempt, attempts, 'write_shard_sec'):.3f}s",
                file=sys.stderr,
                flush=True,
            )
        return 0

    if args.cmd == "view":
        import mr_planner_core
        import plan_roboballet_reaching as reach

        dataset_dir = Path(args.dataset_dir).expanduser().resolve()
        keys: List[str] = [str(k) for k in (args.key or []) if str(k).strip() != ""]
        if not keys:
            for k, _, _ in iter_tar_samples(dataset_dir):
                keys = [k]
                break
        if not keys:
            raise RuntimeError("No samples found")

        for key in keys:
            record, arrays = load_tar_sample(dataset_dir, key)
            vamp_environment = str(record.get("vamp_environment", "roboballet_panda4"))
            vmax = float(record.get("planner", {}).get("vmax", 1.0))
            seed = int(record.get("seed", 1))

            env = mr_planner_core.VampEnvironment(vamp_environment, vmax=vmax, seed=seed)
            env.enable_meshcat(host=args.meshcat_host, port=args.meshcat_port)
            env.reset_scene(reset_sim=True)

            info = env.info()
            num_robots = int(info["num_robots"])

            base_mode = str(record.get("base_mode", "fixed"))
            if base_mode == "rails" and "base_transforms" in arrays:
                env.set_robot_base_transforms(arrays["base_transforms"].tolist())

            sampling = record.get("sampling", {}) or {}
            if bool(sampling.get("welding_sticks", False)):
                reach.add_welding_sticks(env, num_robots)

            if "obstacles" in arrays:
                obs = arrays["obstacles"]
                if obs.ndim != 2 or obs.shape[1] != 10:
                    raise RuntimeError(f"Invalid obstacles array shape: {obs.shape}")
                for i in range(obs.shape[0]):
                    obj = mr_planner_core.Object()
                    obj.name = f"dataset_obs_{i:03d}"
                    obj.parent_link = "world"
                    obj.robot_id = -1
                    obj.state = mr_planner_core.Object.State.Static
                    obj.shape = mr_planner_core.Object.Shape.Box
                    obj.x = float(obs[i, 0])
                    obj.y = float(obs[i, 1])
                    obj.z = float(obs[i, 2])
                    obj.qx = float(obs[i, 3])
                    obj.qy = float(obs[i, 4])
                    obj.qz = float(obs[i, 5])
                    obj.qw = float(obs[i, 6])
                    obj.length = float(obs[i, 7])
                    obj.width = float(obs[i, 8])
                    obj.height = float(obs[i, 9])
                    env.add_object(obj)

            env.update_scene()

            if args.play == "none":
                continue

            if not bool(record.get("success", False)):
                continue

            dt = float(record.get("planner", {}).get("dt", 0.1))
            if args.play in ("raw", "both") and "traj_raw" in arrays:
                env.play_trajectory(arrays["traj_raw"].tolist(), dt=dt, rate=args.meshcat_rate)
            if args.play in ("shortcutted", "both") and "traj" in arrays:
                env.play_trajectory(arrays["traj"].tolist(), dt=dt, rate=args.meshcat_rate)

        return 0

    if args.cmd == "ls":
        dataset_dir = Path(args.dataset_dir).expanduser().resolve()
        n = 0
        for key, _, _ in iter_tar_samples(dataset_dir):
            print(key)
            n += 1
            if args.limit > 0 and n >= args.limit:
                break
        return 0

    raise RuntimeError(f"Unknown cmd: {args.cmd}")


if __name__ == "__main__":
    raise SystemExit(main())
