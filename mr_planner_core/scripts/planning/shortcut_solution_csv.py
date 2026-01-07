#!/usr/bin/env python3

import argparse
import csv
import json
import re
from pathlib import Path
from typing import List, Sequence, Tuple


def _nonempty(tokens: Sequence[str]) -> List[str]:
    return [t.strip() for t in tokens if t is not None and t.strip() != ""]


def load_solution_csv(path: Path) -> Tuple[List[float], List[List[List[float]]]]:
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


def infer_environment_from_skillplan(skillplan_path: Path) -> str:
    try:
        data = json.loads(skillplan_path.read_text())
    except Exception:  # noqa: BLE001
        return ""
    env = data.get("environment", {})
    if isinstance(env, dict):
        name = env.get("name", "")
        if isinstance(name, str):
            return name
    return ""


def default_output_path(input_csv: Path) -> Path:
    if input_csv.name == "solution.csv":
        return input_csv.with_name("solution_shortcut.csv")
    return input_csv.with_name(f"{input_csv.stem}_shortcut{input_csv.suffix or '.csv'}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Shortcut an existing solution.csv and write a new CSV.")
    parser.add_argument("--input", required=True, help="Path to solution.csv")
    parser.add_argument("--output", default="", help="Output CSV path (default: beside input)")

    parser.add_argument("--vamp-environment", default="", help="VAMP environment name (default: infer from skillplan.json)")
    parser.add_argument("--vmax", type=float, default=1.0)

    parser.add_argument("--shortcut-time", type=float, default=0.5)
    parser.add_argument("--dt", type=float, default=0.0, help="Optional dt override (default: infer from CSV time column)")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--method", default="thompson", choices=["thompson", "auto", "round_robin", "comp", "prioritized", "path"])

    parser.add_argument("--meshcat", action="store_true")
    parser.add_argument("--meshcat-host", default="127.0.0.1")
    parser.add_argument("--meshcat-port", type=int, default=7600)
    parser.add_argument("--meshcat-rate", type=float, default=1.0)
    args = parser.parse_args()

    import mr_planner_core

    input_csv = Path(args.input)
    if not input_csv.is_file():
        raise FileNotFoundError(input_csv)

    times, traj = load_solution_csv(input_csv)
    dt = args.dt if args.dt > 0.0 else infer_dt(times, default=0.1)

    env_name = args.vamp_environment
    if not env_name:
        inferred = infer_environment_from_skillplan(input_csv.parent / "skillplan.json")
        env_name = inferred or "dual_gp4"

    env = mr_planner_core.VampEnvironment(env_name, vmax=args.vmax, seed=args.seed)
    res = env.shortcut_trajectory(traj, shortcut_time=args.shortcut_time, dt=dt, seed=args.seed, method=args.method)
    if not res.get("success", False):
        raise RuntimeError("Shortcutting failed")

    out_csv = Path(args.output) if args.output else default_output_path(input_csv)
    write_solution_csv(out_csv, res["trajectory"], dt)

    if args.meshcat:
        env.enable_meshcat(args.meshcat_host, args.meshcat_port)
        env.play_solution_csv(str(out_csv), rate=args.meshcat_rate)

    print(
        json.dumps(
            {
                "input_csv": str(input_csv),
                "output_csv": str(out_csv),
                "vamp_environment": env_name,
                "dt": dt,
                "shortcut_time": args.shortcut_time,
                "method": args.method,
                "seed": args.seed,
                "num_steps_in": len(times),
                "num_steps_out": len(res["trajectory"][0]) if res["trajectory"] and res["trajectory"][0] else 0,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
