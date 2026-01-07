#!/usr/bin/env python3

import argparse
import csv
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence


SUMMARY_COLUMNS = [
    "task",
    "seed",
    "vamp_environment",
    "assign_time_sec",
    "plan_time_sec",
    "total_time_sec",
    "adg_dt",
    "makespan_pre",
    "makespan_post",
    "flowtime_pre",
    "flowtime_post",
    "num_activities",
    "type2_edges",
]


def find_repo_root() -> Path:
    lego_root = Path(__file__).resolve().parents[2]
    if lego_root.name == "mr_planner_lego":
        maybe_monorepo = lego_root.parent
        if (maybe_monorepo / "config" / "lego_tasks").is_dir():
            return maybe_monorepo
    return lego_root


def default_lego_bin_dir(repo_root: Path) -> Path:
    candidates = [
        repo_root / "mr_planner_lego" / "build",  # monorepo
        repo_root / "build",  # lego-only checkout
    ]
    for c in candidates:
        if (c / "mr_planner_lego_assign").is_file() and (c / "mr_planner_lego_plan").is_file():
            return c
    return candidates[0]


def default_core_build_dir(repo_root: Path) -> Optional[Path]:
    candidates = [
        repo_root / "mr_planner_core" / "build",  # monorepo
        repo_root / "build",  # core-only checkout
    ]
    for c in candidates:
        if (c / "libmr_planner_core.so").is_file() or any(c.glob("libmr_planner_core.*")):
            return c
    return None


def find_graph_stats_script(repo_root: Path) -> Path:
    candidates = [
        repo_root / "mr_planner_core" / "scripts" / "io" / "graph_stats.py",  # monorepo
        repo_root / "scripts" / "io" / "graph_stats.py",  # core-only checkout
    ]
    for c in candidates:
        if c.is_file():
            return c
    raise FileNotFoundError("Could not locate mr_planner_core/scripts/io/graph_stats.py")


def ensure_csv_header(path: Path, columns: Sequence[str]) -> None:
    if path.exists() and path.stat().st_size > 0:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(list(columns))


def run(cmd: List[str], *, env: Dict[str, str]) -> None:
    subprocess.check_call(cmd, env=env)


def graph_stats(graph_stats_py: Path, pb_path: Path, *, env: Dict[str, str]) -> dict:
    out = subprocess.check_output([sys.executable, str(graph_stats_py), "--input", str(pb_path)], env=env, text=True)
    return json.loads(out)


def main() -> int:
    parser = argparse.ArgumentParser(description="ROS-free LEGO benchmark using mr_planner_lego CLIs.")
    parser.add_argument(
        "--task",
        action="append",
        default=[],
        help="LEGO task (repeatable). Default: cliff, vessel, big_chair, rss",
    )
    parser.add_argument(
        "--seed",
        action="append",
        type=int,
        default=[],
        help="Seed (repeatable). Default: 1",
    )
    parser.add_argument("--vamp-environment", default="dual_gp4")
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument(
        "--shortcut-time",
        type=float,
        default=None,
        help="Override shortcut time for all tasks (default: 1s for cliff/vessel, 5s for rss/big_chair).",
    )

    parser.add_argument("--root", default="", help="Repo root (defaults to auto-detected)")
    parser.add_argument("--lego-bin-dir", default="", help="Directory containing mr_planner_lego_assign/plan")
    parser.add_argument("--output-dir", default="outputs/reproduce/lego", help="Output directory for steps/out + summary CSV")
    parser.add_argument(
        "--append",
        action="store_true",
        help="Append to an existing lego_benchmark.csv instead of overwriting it",
    )
    args = parser.parse_args()

    repo_root = Path(args.root) if args.root else find_repo_root()
    if not (repo_root / "config" / "lego_tasks").is_dir():
        raise SystemExit(f"[error] missing config/lego_tasks under --root {repo_root}")

    tasks = args.task or ["cliff", "vessel", "big_chair", "rss"]
    seeds = args.seed or [1]
    default_shortcut_time = {
        "cliff": 1.0,
        "vessel": 1.0,
        "rss": 5.0,
        "big_chair": 5.0,
    }

    lego_bin_dir = Path(args.lego_bin_dir) if args.lego_bin_dir else default_lego_bin_dir(repo_root)
    lego_assign = lego_bin_dir / "mr_planner_lego_assign"
    lego_plan = lego_bin_dir / "mr_planner_lego_plan"
    if not lego_assign.is_file() or not lego_plan.is_file():
        raise SystemExit(f"[error] could not find mr_planner_lego CLIs under {lego_bin_dir}")

    core_build = default_core_build_dir(repo_root)
    graph_stats_py = find_graph_stats_script(repo_root)

    out_root = Path(args.output_dir)
    steps_root = out_root / "steps"
    plan_root = out_root / "out"
    summary_csv = out_root / "lego_benchmark.csv"

    env = dict(os.environ)
    ld_paths: List[str] = []
    if core_build is not None:
        ld_paths.append(str(core_build))
    ld_paths.append(str(lego_bin_dir))
    old_ld = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = ":".join(ld_paths + ([old_ld] if old_ld else []))

    if args.append:
        ensure_csv_header(summary_csv, SUMMARY_COLUMNS)
    else:
        summary_csv.parent.mkdir(parents=True, exist_ok=True)
        with summary_csv.open("w", newline="") as f:
            writer = csv.writer(f, lineterminator="\n")
            writer.writerow(SUMMARY_COLUMNS)

    for task in tasks:
        for seed in seeds:
            shortcut_time = (
                float(args.shortcut_time)
                if args.shortcut_time is not None
                else float(default_shortcut_time.get(task, 0.0))
            )
            steps_dir = steps_root / task / f"seed_{seed}"
            task_out = plan_root / task / f"seed_{seed}"
            steps_dir.mkdir(parents=True, exist_ok=True)
            task_out.mkdir(parents=True, exist_ok=True)

            t0 = time.perf_counter()
            run(
                [
                    str(lego_assign),
                    "--task",
                    task,
                    "--root",
                    str(repo_root),
                    "--output-dir",
                    str(steps_dir),
                    "--vamp-environment",
                    args.vamp_environment,
                ],
                env=env,
            )
            t_assign = time.perf_counter() - t0

            t1 = time.perf_counter()
            run(
                [
                    str(lego_plan),
                    "--task",
                    task,
                    "--root",
                    str(repo_root),
                    "--steps-dir",
                    str(steps_dir),
                    "--output-dir",
                    str(task_out),
                    "--vamp-environment",
                    args.vamp_environment,
                    "--planning-time",
                    str(args.planning_time),
                    "--shortcut-time",
                    str(shortcut_time),
                    "--seed",
                    str(seed),
                ],
                env=env,
            )
            t_plan = time.perf_counter() - t1

            adg_pb = task_out / "adg.pb"
            if not adg_pb.is_file():
                raise SystemExit(f"[error] missing {adg_pb} (task={task} seed={seed})")

            stats = graph_stats(graph_stats_py, adg_pb, env=env)
            row = [
                task,
                str(seed),
                args.vamp_environment,
                f"{t_assign:.6g}",
                f"{t_plan:.6g}",
                f"{(t_assign + t_plan):.6g}",
                f"{float(stats.get('dt', 0.0)):.6g}",
                f"{float(stats.get('pre_shortcut_makespan', 0.0)):.6g}",
                f"{float(stats.get('post_shortcut_makespan', 0.0)):.6g}",
                f"{float(stats.get('pre_shortcut_flowtime', 0.0)):.6g}",
                f"{float(stats.get('post_shortcut_flowtime', 0.0)):.6g}",
                str(int(stats.get('num_activities', 0))),
                str(int(stats.get('type2_edges', 0))),
            ]

            with summary_csv.open("a", newline="") as f:
                writer = csv.writer(f, lineterminator="\n")
                writer.writerow(row)

            print(
                f"[bench] task={task} seed={seed}: assign {t_assign:.2f}s, plan {t_plan:.2f}s, "
                f"makespan {stats.get('pre_shortcut_makespan', 0.0):.2f}->{stats.get('post_shortcut_makespan', 0.0):.2f}"
            )

    print(f"[bench] wrote {summary_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
