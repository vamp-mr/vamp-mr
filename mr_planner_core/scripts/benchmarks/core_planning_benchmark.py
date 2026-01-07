#!/usr/bin/env python3

import argparse
import csv
import statistics
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
from xml.etree import ElementTree


PLANNING_BENCH_COLUMNS = [
    "start_pose",
    "goal_pose",
    "flowtime_pre",
    "makespan_pre",
    "flowtime_post",
    "makespan_post",
    "t_plan",
    "n_node_expanded",
    "t_init",
    "t_shortcut",
    "t_mcp",
    "t_check",
    "n_check",
    "n_valid",
    "n_type2_pre",
    "n_colcheck_pre",
    "n_type2_post",
    "n_colcheck_post",
    "pathlen",
    "t_wait",
    "norm_isj",
    "dir_consistency",
    "n_comp",
    "n_pp",
    "n_path",
    "n_v_comp",
    "n_v_pp",
    "n_v_path",
]


def _dedup(seq: Iterable[str]) -> List[str]:
    out: List[str] = []
    seen = set()
    for s in seq:
        if s in seen:
            continue
        seen.add(s)
        out.append(s)
    return out


def find_repo_root() -> Path:
    core_root = Path(__file__).resolve().parents[2]
    if core_root.name == "mr_planner_core":
        maybe_monorepo = core_root.parent
        if (maybe_monorepo / "mr_planner_lego").is_dir() and (maybe_monorepo / "launch").is_dir():
            return maybe_monorepo
    return core_root


def find_srdf(repo_root: Path, env: str) -> Path:
    candidates = [
        repo_root / "mr_planner_core" / "config" / "srdf" / f"{env}.srdf",  # monorepo
        repo_root / "config" / "srdf" / f"{env}.srdf",  # core-only checkout
    ]
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError(f"Could not locate SRDF for env={env!r} under {repo_root}")


def srdf_group_state_names(srdf_path: Path, move_group: str) -> List[str]:
    root = ElementTree.parse(str(srdf_path)).getroot()
    names = []
    for node in root.iter():
        if node.tag != "group_state":
            continue
        if node.get("group") != move_group:
            continue
        name = node.get("name", "")
        if name:
            names.append(name)
    return _dedup(names)


def preferred_pose_names(env: str) -> List[str]:
    preferred_by_env: Dict[str, List[str]] = {
        "dual_gp4": [
            "random1",
            "random2",
            "random3",
            "random4",
            "random5",
            "random6",
            "left_push_up",
            "right_push_up",
            "left_rotated",
            "right_rotated",
            "left_push",
            "right_push",
            "ready_pose",
        ],
        "panda_two_rod": [
            "ready_pose",
            "twist_left",
            "twist_right",
            "mirror",
            "mirror_down",
            "left_push",
            "right_push",
            "coop_up",
            "coop_down",
            "tiled_up",
            "tiled_down",
            "left_up",
        ],
        "panda_four": [
            "ready",
            "down",
            "front",
            "back",
            "up_and_down",
            "front_and_back",
            "left_and_right",
            "center",
            "tiled_mess",
            "tiled_clean",
            "up",
        ],
        "panda_four_bins": [
            "up",
            "four_bins_1",
            "four_bins_2",
            "three_bins_1",
            "three_bins_2",
            "three_bins_3",
            "purple_yellow_1",
            "purple_yellow_2",
            "red_green",
            "blue",
            "red",
            "ready_pose",
        ],
    }
    return preferred_by_env.get(env, [])


def infer_pose_pairs(pose_names: Sequence[str], mode: str) -> List[Tuple[str, str]]:
    if len(pose_names) < 2:
        return []
    if mode == "sequential":
        return list(zip(pose_names[:-1], pose_names[1:]))
    if mode == "all_pairs":
        return [(a, b) for a in pose_names for b in pose_names if a != b]
    raise ValueError(f"Invalid pose mode: {mode!r}")


def read_last_time(csv_path: Path) -> float:
    last_t: Optional[float] = None
    with csv_path.open("r", newline="") as f:
        reader = csv.reader(f)
        _ = next(reader, None)  # header
        for row in reader:
            if not row:
                continue
            try:
                t = float(row[0])
            except Exception:  # noqa: BLE001
                continue
            last_t = t
    if last_t is None:
        raise RuntimeError(f"Could not read makespan from {csv_path}")
    return float(last_t)


def ensure_csv_header(path: Path, columns: Sequence[str]) -> None:
    if path.exists() and path.stat().st_size > 0:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(list(columns))


def append_planning_row(
    *,
    out_csv: Path,
    start_pose: str,
    goal_pose: str,
    makespan_post: float,
    t_plan: float,
    n_node_expanded: int,
    t_init: float,
) -> None:
    ensure_csv_header(out_csv, PLANNING_BENCH_COLUMNS)
    row = [""] * len(PLANNING_BENCH_COLUMNS)
    row[0] = start_pose
    row[1] = goal_pose
    row[3] = f"{makespan_post:.6g}"
    row[5] = f"{makespan_post:.6g}"
    row[6] = f"{t_plan:.6g}"
    row[7] = str(int(n_node_expanded))
    row[8] = f"{t_init:.6g}"
    with out_csv.open("a", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="ROS-free planning benchmark (RRT and CBS) using mr_planner_core Python bindings."
    )
    parser.add_argument(
        "--env",
        action="append",
        default=[],
        help="VAMP environment (repeatable). Default: panda_two_rod, panda_four, panda_four_bins",
    )
    parser.add_argument(
        "--planner",
        action="append",
        default=[],
        choices=["composite_rrt", "cbs_prm"],
        help="Planner backend (repeatable). Default: composite_rrt, cbs_prm",
    )
    parser.add_argument("--pose-mode", choices=["sequential", "all_pairs"], default="all_pairs")
    parser.add_argument("--max-poses", type=int, default=99)

    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--vmax", type=float, default=1.0)
    parser.add_argument("--roadmap-samples", type=int, default=5000)
    parser.add_argument("--roadmap-max-dist", type=float, default=2.0)
    parser.add_argument(
        "--min-success-rate",
        type=float,
        default=0.0,
        help="Fail (exit nonzero) if any env/planner success rate is below this threshold (0 disables).",
    )

    parser.add_argument(
        "--output-dir",
        default="outputs/reproduce/planning",
        help="Output directory for benchmark CSVs and per-case outputs",
    )
    parser.add_argument(
        "--append",
        action="store_true",
        help="Append to existing <env>_benchmark.csv files instead of overwriting them",
    )
    args = parser.parse_args()

    import mr_planner_core

    repo_root = find_repo_root()
    out_root = Path(args.output_dir)
    case_root = out_root / "cases"

    envs = args.env or ["panda_two_rod", "panda_four", "panda_four_bins"]
    planners = args.planner or ["composite_rrt", "cbs_prm"]

    overall_failures: List[str] = []
    rate_failures: List[str] = []
    for env_name in envs:
        env_info = mr_planner_core.vamp_environment_info(env_name)
        move_group = env_info["move_group"]
        robot_groups = env_info["robot_groups"]
        srdf_path = find_srdf(repo_root, env_name)

        srdf_poses = srdf_group_state_names(srdf_path, move_group)
        preferred = [p for p in preferred_pose_names(env_name) if p in set(srdf_poses)]
        pose_candidates = preferred + [p for p in srdf_poses if p not in set(preferred)]

        env_instance = mr_planner_core.VampEnvironment(env_name, vmax=args.vmax, seed=args.seed)

        pose_matrix: Dict[str, List[List[float]]] = {}
        valid_pose_names: List[str] = []
        for name in pose_candidates:
            if args.max_poses > 0 and len(valid_pose_names) >= args.max_poses:
                break
            try:
                mat = mr_planner_core.pose_matrix_from_named_pose(
                    srdf_path, move_group=move_group, robot_groups=robot_groups, pose_name=name
                )
                if env_instance.in_collision(mat, self_only=False):
                    continue
                pose_matrix[name] = mat
                valid_pose_names.append(name)
            except Exception as exc:  # noqa: BLE001
                overall_failures.append(f"{env_name}: invalid pose {name!r}: {exc}")

        pose_pairs = infer_pose_pairs(valid_pose_names, args.pose_mode)
        if not pose_pairs:
            raise RuntimeError(f"{env_name}: no pose pairs (valid poses={valid_pose_names})")

        print(f"[bench] env={env_name}: {len(valid_pose_names)} valid poses, {len(pose_pairs)} pose pairs")

        for planner in planners:
            bench_csv = out_root / planner / f"{env_name}_benchmark.csv"
            if args.append:
                ensure_csv_header(bench_csv, PLANNING_BENCH_COLUMNS)
            else:
                bench_csv.parent.mkdir(parents=True, exist_ok=True)
                with bench_csv.open("w", newline="") as f:
                    writer = csv.writer(f, lineterminator="\n")
                    writer.writerow(PLANNING_BENCH_COLUMNS)
            successes = 0
            attempted = 0
            plan_times: List[float] = []

            for start_pose, goal_pose in pose_pairs:
                attempted += 1
                out_dir = case_root / env_name / planner / f"{start_pose}_to_{goal_pose}"
                out_dir.mkdir(parents=True, exist_ok=True)

                try:
                    t0 = time.perf_counter()
                    res = env_instance.plan(
                        planner=planner,
                        planning_time=args.planning_time,
                        shortcut_time=0.0,
                        seed=args.seed,
                        dt=args.dt,
                        vmax=args.vmax,
                        start=pose_matrix[start_pose],
                        goal=pose_matrix[goal_pose],
                        output_dir=str(out_dir),
                        write_tpg=False,
                        roadmap_samples=args.roadmap_samples,
                        roadmap_max_dist=args.roadmap_max_dist,
                    )
                    wall_dt = time.perf_counter() - t0
                    t_plan = float(res.get("planner_time_sec", wall_dt))
                    t_init = float(res.get("init_time_sec", 0.0))
                    n_node_expanded = int(res.get("n_node_expanded", 0))

                    sol_csv = Path(res["solution_csv"])
                    makespan_post = read_last_time(sol_csv)
                    append_planning_row(
                        out_csv=bench_csv,
                        start_pose=start_pose,
                        goal_pose=goal_pose,
                        makespan_post=makespan_post,
                        t_plan=t_plan,
                        n_node_expanded=n_node_expanded,
                        t_init=t_init,
                    )
                    successes += 1
                    plan_times.append(t_plan)
                except Exception as exc:  # noqa: BLE001
                    overall_failures.append(f"{env_name}/{planner}: {start_pose}->{goal_pose} failed: {exc}")

            rate = (successes / attempted) if attempted else 0.0
            med = statistics.median(plan_times) if plan_times else float("nan")
            print(f"[bench] env={env_name} planner={planner}: {successes}/{attempted} ({rate*100:.1f}%), median t_plan={med:.3f}s")
            if args.min_success_rate > 0.0 and attempted > 0 and rate < args.min_success_rate:
                rate_failures.append(
                    f"{env_name}/{planner}: {successes}/{attempted} ({rate*100:.1f}%) < {args.min_success_rate*100:.1f}%"
                )

    if overall_failures:
        print(f"[bench] completed with {len(overall_failures)} failures (showing up to 20):")
        for line in overall_failures[:20]:
            print("  " + line)

    if rate_failures:
        print("[bench] success-rate failures:")
        for line in rate_failures:
            print("  " + line)
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
