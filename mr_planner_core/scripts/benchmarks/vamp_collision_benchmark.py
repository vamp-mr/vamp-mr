#!/usr/bin/env python3

import argparse
import json
import time
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple


def sample_multi_robot_pose(
    env,
    *,
    num_robots: int,
    collision_free: bool,
    max_attempts: int,
    self_only: bool,
) -> List[List[float]]:
    q: List[List[float]] = []
    for rid in range(num_robots):
        if collision_free:
            q.append(env.sample_collision_free_pose(rid, max_attempts=max_attempts, self_only=self_only))
        else:
            q.append(env.sample_pose(rid))
    return q


def bench_pose_collision(
    env,
    *,
    num_robots: int,
    queries: int,
    collision_free: bool,
    max_attempts: int,
    self_only: bool,
    warmup: int,
) -> Dict[str, Any]:
    t0 = time.perf_counter()
    samples = [
        sample_multi_robot_pose(
            env,
            num_robots=num_robots,
            collision_free=collision_free,
            max_attempts=max_attempts,
            self_only=self_only,
        )
        for _ in range(max(0, queries))
    ]
    sample_sec = time.perf_counter() - t0

    warmup_n = min(len(samples), max(0, warmup))
    for i in range(warmup_n):
        env.in_collision(samples[i], self_only=self_only)

    t1 = time.perf_counter()
    collisions = 0
    checked = 0
    for i in range(warmup_n, len(samples)):
        checked += 1
        if env.in_collision(samples[i], self_only=self_only):
            collisions += 1
    check_sec = time.perf_counter() - t1

    return {
        "queries": queries,
        "warmup": warmup_n,
        "checked": checked,
        "collisions": collisions,
        "collision_rate": (collisions / checked) if checked else 0.0,
        "sample_time_sec": sample_sec,
        "check_time_sec": check_sec,
        "avg_check_time_usec": (check_sec * 1e6 / checked) if checked else 0.0,
        "checks_per_sec": (checked / check_sec) if check_sec > 0.0 else 0.0,
        "sampling_mode": "collision_free" if collision_free else "random",
    }


def bench_motion_collision(
    env,
    *,
    num_robots: int,
    queries: int,
    collision_free: bool,
    max_attempts: int,
    self_only: bool,
    step_size: float,
    warmup: int,
) -> Dict[str, Any]:
    t0 = time.perf_counter()
    samples: List[Tuple[List[List[float]], List[List[float]]]] = []
    for _ in range(max(0, queries)):
        start = sample_multi_robot_pose(
            env,
            num_robots=num_robots,
            collision_free=collision_free,
            max_attempts=max_attempts,
            self_only=self_only,
        )
        goal = sample_multi_robot_pose(
            env,
            num_robots=num_robots,
            collision_free=collision_free,
            max_attempts=max_attempts,
            self_only=self_only,
        )
        samples.append((start, goal))
    sample_sec = time.perf_counter() - t0

    warmup_n = min(len(samples), max(0, warmup))
    for i in range(warmup_n):
        s, g = samples[i]
        env.motion_in_collision(s, g, step_size=step_size, self_only=self_only)

    t1 = time.perf_counter()
    collisions = 0
    checked = 0
    for i in range(warmup_n, len(samples)):
        s, g = samples[i]
        checked += 1
        if env.motion_in_collision(s, g, step_size=step_size, self_only=self_only):
            collisions += 1
    check_sec = time.perf_counter() - t1

    return {
        "queries": queries,
        "warmup": warmup_n,
        "checked": checked,
        "collisions": collisions,
        "collision_rate": (collisions / checked) if checked else 0.0,
        "sample_time_sec": sample_sec,
        "check_time_sec": check_sec,
        "avg_check_time_usec": (check_sec * 1e6 / checked) if checked else 0.0,
        "checks_per_sec": (checked / check_sec) if check_sec > 0.0 else 0.0,
        "step_size": step_size,
        "sampling_mode": "collision_free" if collision_free else "random",
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Benchmark VAMP collision checking from Python (pose + motion collision queries)."
    )
    parser.add_argument("--vamp-environment", default="dual_gp4")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--vmax", type=float, default=1.0)

    parser.add_argument("--pose-queries", type=int, default=5000)
    parser.add_argument("--motion-queries", type=int, default=2000)
    parser.add_argument("--warmup", type=int, default=50)
    parser.add_argument("--self-only", action="store_true")
    parser.add_argument("--collision-free", action="store_true", help="Sample per-robot collision-free poses before timing")
    parser.add_argument("--max-sample-attempts", type=int, default=200)

    parser.add_argument("--motion-step-size", type=float, default=0.1)

    parser.add_argument("--json-out", default="", help="Optional path to write JSON results")
    args = parser.parse_args()

    import mr_planner_core

    env = mr_planner_core.VampEnvironment(args.vamp_environment, vmax=args.vmax, seed=args.seed)
    info = env.info()
    num_robots = int(info["num_robots"])

    t0 = time.perf_counter()
    pose_stats = bench_pose_collision(
        env,
        num_robots=num_robots,
        queries=args.pose_queries,
        collision_free=args.collision_free,
        max_attempts=args.max_sample_attempts,
        self_only=args.self_only,
        warmup=args.warmup,
    )
    motion_stats = bench_motion_collision(
        env,
        num_robots=num_robots,
        queries=args.motion_queries,
        collision_free=args.collision_free,
        max_attempts=args.max_sample_attempts,
        self_only=args.self_only,
        step_size=args.motion_step_size,
        warmup=args.warmup,
    )
    total_sec = time.perf_counter() - t0

    out: Dict[str, Any] = {
        "vamp_environment": args.vamp_environment,
        "seed": args.seed,
        "vmax": args.vmax,
        "self_only": bool(args.self_only),
        "environment_info": info,
        "pose_collision": pose_stats,
        "motion_collision": motion_stats,
        "total_time_sec": total_sec,
    }

    if args.json_out:
        out_path = Path(args.json_out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(out, indent=2))

    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

