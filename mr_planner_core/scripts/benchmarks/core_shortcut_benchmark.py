#!/usr/bin/env python3

import argparse
import csv
import statistics
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


BENCH_COLUMNS = [
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


def _nonempty(tokens: Sequence[str]) -> List[str]:
    return [t.strip() for t in tokens if t is not None and t.strip() != ""]


def load_solution_csv(path: Path) -> Tuple[List[float], List[List[List[float]]]]:
    import re

    with path.open("r", newline="") as f:
        reader = csv.reader(f)
        try:
            header_raw = next(reader)
        except StopIteration as exc:
            raise ValueError(f"Empty CSV: {path}") from exc

        header = _nonempty(header_raw)
        if not header or header[0] != "time":
            raise ValueError(f"Invalid CSV header (expected leading 'time'): {header[:5]}")

        spec: List[Tuple[int, int]] = []
        for col in header[1:]:
            m = re.match(r"^q(\d+)_(\d+)$", col)
            if not m:
                raise ValueError(f"Invalid joint column name: {col!r}")
            spec.append((int(m.group(1)), int(m.group(2))))

        if not spec:
            raise ValueError("No joint columns found in CSV")

        num_robots = max(r for r, _ in spec) + 1
        dof_by_robot = [0 for _ in range(num_robots)]
        for r, j in spec:
            dof_by_robot[r] = max(dof_by_robot[r], j + 1)

        expected: List[Tuple[int, int]] = []
        for r in range(num_robots):
            for j in range(dof_by_robot[r]):
                expected.append((r, j))
        if spec != expected:
            raise ValueError(
                "CSV joint columns are not in expected order; "
                f"expected {expected[:6]}..., got {spec[:6]}..."
            )

        times: List[float] = []
        traj: List[List[List[float]]] = [[] for _ in range(num_robots)]

        for row_raw in reader:
            row = _nonempty(row_raw)
            if not row:
                continue
            if len(row) != 1 + len(spec):
                raise ValueError(f"Row has {len(row)} columns, expected {1 + len(spec)}: {row[:6]}")
            t = float(row[0])
            joints = [[0.0 for _ in range(dof_by_robot[r])] for r in range(num_robots)]
            for i, (r, j) in enumerate(spec):
                joints[r][j] = float(row[1 + i])
            times.append(t)
            for r in range(num_robots):
                traj[r].append(joints[r])

        if not times:
            raise ValueError("CSV contains no trajectory rows")

        return times, traj


def infer_dt(times: Sequence[float], default: float = 0.1) -> float:
    if len(times) < 2:
        return default
    diffs = [times[i] - times[i - 1] for i in range(1, len(times)) if (times[i] - times[i - 1]) > 0.0]
    if not diffs:
        return default
    diffs_sorted = sorted(diffs)
    return diffs_sorted[len(diffs_sorted) // 2]


def write_solution_csv(path: Path, traj: Sequence[Sequence[Sequence[float]]], dt: float) -> None:
    if dt <= 0.0:
        raise ValueError(f"dt must be > 0 (got {dt})")
    num_robots = len(traj)
    if num_robots <= 0:
        raise ValueError("trajectory has 0 robots")

    dof_by_robot = [len(traj[r][0]) if traj[r] else 0 for r in range(num_robots)]
    if any(d <= 0 for d in dof_by_robot):
        raise ValueError(f"invalid dof_by_robot={dof_by_robot}")

    num_steps = len(traj[0])
    if any(len(traj[r]) != num_steps for r in range(num_robots)):
        raise ValueError("trajectory robot timelines have different lengths")

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        header = ["time"] + [f"q{r}_{j}" for r in range(num_robots) for j in range(dof_by_robot[r])]
        writer.writerow(header + [""])  # preserve trailing comma format

        for i in range(num_steps):
            t = i * dt
            row = [t]
            for r in range(num_robots):
                row.extend(traj[r][i])
            writer.writerow(row + [""])  # preserve trailing comma format


def ensure_csv_header(path: Path, columns: Sequence[str]) -> None:
    if path.exists() and path.stat().st_size > 0:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(list(columns))


def append_shortcut_row(
    *,
    out_csv: Path,
    start_pose: str,
    goal_pose: str,
    makespan_pre: float,
    makespan_post: float,
    t_shortcut: float,
) -> None:
    ensure_csv_header(out_csv, BENCH_COLUMNS)
    row = [""] * len(BENCH_COLUMNS)
    row[0] = start_pose
    row[1] = goal_pose
    row[3] = f"{makespan_pre:.6g}"
    row[5] = f"{makespan_post:.6g}"
    row[9] = f"{t_shortcut:.6g}"
    with out_csv.open("a", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(row)


def parse_case_name(case_dir: Path) -> Tuple[str, str]:
    name = case_dir.name
    if "_to_" not in name:
        return name, ""
    start, goal = name.split("_to_", 1)
    return start, goal


def _dedup(seq: Iterable[str]) -> List[str]:
    out: List[str] = []
    seen = set()
    for s in seq:
        if s in seen:
            continue
        seen.add(s)
        out.append(s)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(
        description="ROS-free shortcut benchmark (Thompson selector) that reuses trajectories saved by core_planning_benchmark.py."
    )
    parser.add_argument(
        "--planning-dir",
        default="outputs/reproduce/planning",
        help="Output directory previously created by core_planning_benchmark.py (must contain cases/<env>/<planner>/...).",
    )
    parser.add_argument("--env", action="append", default=[], help="Environment filter (repeatable). Defaults to all found.")
    parser.add_argument(
        "--planner",
        action="append",
        default=[],
        help="Planner filter (repeatable). Defaults to all found under cases/<env>/.",
    )
    parser.add_argument("--shortcut-time", type=float, default=1.0)
    parser.add_argument(
        "--method",
        default="thompson",
        choices=["thompson", "auto", "round_robin", "comp", "prioritized", "path"],
        help="Shortcutting method (passed to mr_planner_core.VampEnvironment.shortcut_trajectory).",
    )
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--vmax", type=float, default=1.0)
    parser.add_argument("--dt", type=float, default=0.0, help="Optional dt override (default: infer from input CSV).")
    parser.add_argument("--output-dir", default="outputs/reproduce/shortcut", help="Output directory for new CSVs + summary.")
    parser.add_argument("--append", action="store_true", help="Append to existing <env>_benchmark.csv files.")
    args = parser.parse_args()

    if args.shortcut_time <= 0.0:
        raise SystemExit("--shortcut-time must be > 0 for a shortcut benchmark")

    import mr_planner_core

    planning_root = Path(args.planning_dir)
    planning_cases = planning_root / "cases"
    if not planning_cases.is_dir():
        raise SystemExit(f"[error] missing {planning_cases} (did you pass the planning benchmark --output-dir?)")

    out_root = Path(args.output_dir)
    out_cases = out_root / "cases"

    envs: List[str]
    if args.env:
        envs = _dedup(args.env)
    else:
        envs = sorted([p.name for p in planning_cases.iterdir() if p.is_dir()])
    if not envs:
        raise SystemExit(f"[error] no environments found under {planning_cases}")

    overall_failures: List[str] = []

    for env_name in envs:
        env_case_root = planning_cases / env_name
        if not env_case_root.is_dir():
            overall_failures.append(f"{env_name}: missing {env_case_root}")
            continue

        planners: List[str]
        if args.planner:
            planners = _dedup(args.planner)
        else:
            planners = sorted([p.name for p in env_case_root.iterdir() if p.is_dir()])
        if not planners:
            overall_failures.append(f"{env_name}: no planners found under {env_case_root}")
            continue

        env_instance = mr_planner_core.VampEnvironment(env_name, vmax=args.vmax, seed=args.seed)

        for planner in planners:
            planner_root = env_case_root / planner
            if not planner_root.is_dir():
                overall_failures.append(f"{env_name}/{planner}: missing {planner_root}")
                continue

            bench_csv = out_root / planner / f"{env_name}_benchmark.csv"
            if args.append:
                ensure_csv_header(bench_csv, BENCH_COLUMNS)
            else:
                bench_csv.parent.mkdir(parents=True, exist_ok=True)
                with bench_csv.open("w", newline="") as f:
                    writer = csv.writer(f, lineterminator="\n")
                    writer.writerow(BENCH_COLUMNS)

            attempted = 0
            succeeded = 0
            shortcut_times: List[float] = []
            makespan_improv: List[float] = []

            for case_dir in sorted([p for p in planner_root.iterdir() if p.is_dir()]):
                input_csv = case_dir / "solution.csv"
                if not input_csv.is_file():
                    overall_failures.append(f"{env_name}/{planner}/{case_dir.name}: missing {input_csv}")
                    continue

                start_pose, goal_pose = parse_case_name(case_dir)
                attempted += 1

                try:
                    times, traj = load_solution_csv(input_csv)
                    dt = args.dt if args.dt > 0.0 else infer_dt(times, default=0.1)

                    t0 = time.perf_counter()
                    res = env_instance.shortcut_trajectory(
                        traj, shortcut_time=args.shortcut_time, dt=dt, seed=args.seed, method=args.method
                    )
                    t_sc = time.perf_counter() - t0
                    if not res.get("success", False):
                        raise RuntimeError("shortcut failed")

                    out_case = out_cases / env_name / planner / case_dir.name
                    out_case.mkdir(parents=True, exist_ok=True)
                    out_csv = out_case / "solution_shortcut.csv"
                    out_traj = res["trajectory"]
                    write_solution_csv(out_csv, out_traj, dt)

                    makespan_pre = float(times[-1])
                    makespan_post = (len(out_traj[0]) - 1) * dt if out_traj and out_traj[0] else float("nan")
                    append_shortcut_row(
                        out_csv=bench_csv,
                        start_pose=start_pose,
                        goal_pose=goal_pose,
                        makespan_pre=makespan_pre,
                        makespan_post=makespan_post,
                        t_shortcut=t_sc,
                    )

                    succeeded += 1
                    shortcut_times.append(t_sc)
                    if makespan_pre > 0 and makespan_post == makespan_post:
                        makespan_improv.append((makespan_pre - makespan_post) / makespan_pre * 100.0)
                except Exception as exc:  # noqa: BLE001
                    overall_failures.append(f"{env_name}/{planner}/{case_dir.name}: {exc}")

            rate = (succeeded / attempted) if attempted else 0.0
            med_t = statistics.median(shortcut_times) if shortcut_times else float("nan")
            med_improv = statistics.median(makespan_improv) if makespan_improv else float("nan")
            print(
                f"[bench] env={env_name} planner={planner}: {succeeded}/{attempted} ({rate*100:.1f}%), "
                f"median t_shortcut={med_t:.3f}s, median makespan_improv={med_improv:.1f}%"
            )

    if overall_failures:
        print(f"[bench] completed with {len(overall_failures)} failures (showing up to 20):")
        for line in overall_failures[:20]:
            print("  " + line)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
