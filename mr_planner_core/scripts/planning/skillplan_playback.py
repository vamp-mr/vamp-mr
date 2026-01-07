#!/usr/bin/env python3

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build an execution graph (TPG/ADG) from skillplan.json and optionally play via Meshcat."
    )
    parser.add_argument("--skillplan", required=True, help="Path to skillplan.json")
    parser.add_argument("--graph-type", choices=["tpg", "adg"], default="adg")
    parser.add_argument("--vamp-environment", default="", help="Override skillplan.environment.name")
    parser.add_argument("--output-dir", default="", help="Output directory for protobuf (defaults to /tmp)")
    parser.add_argument("--pb-file", default="", help="Optional output protobuf path")
    parser.add_argument("--dot-file", default="", help="Optional output dot path")
    parser.add_argument("--dt", type=float, default=0.0, help="Discretization timestep (0 uses 0.05/vmax)")
    parser.add_argument("--vmax", type=float, default=1.0)

    parser.add_argument("--meshcat", action="store_true")
    parser.add_argument("--meshcat-host", default="127.0.0.1")
    parser.add_argument("--meshcat-port", type=int, default=7600)
    parser.add_argument("--meshcat-rate", type=float, default=1.0)
    args = parser.parse_args()

    import mr_planner_core

    output_dir = args.output_dir
    if not output_dir:
        output_dir = str(Path("/tmp") / "mr_planner_py_skillplan_playback")
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    res = mr_planner_core.skillplan_to_execution_graph(
        args.skillplan,
        graph_type=args.graph_type,
        vamp_environment=args.vamp_environment,
        output_dir=output_dir,
        pb_file=args.pb_file,
        dot_file=args.dot_file,
        dt=args.dt,
        vmax=args.vmax,
    )

    if args.meshcat:
        env_name = res["environment"] if not args.vamp_environment else args.vamp_environment
        env = mr_planner_core.VampEnvironment(env_name, vmax=args.vmax, seed=1)
        env.enable_meshcat(args.meshcat_host, args.meshcat_port)
        env.reset_scene(True)
        env.seed_initial_scene_from_skillplan(args.skillplan)
        env.update_scene()
        env.play_execution_graph(res["pb_path"], rate=args.meshcat_rate)

    print(json.dumps(res, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
