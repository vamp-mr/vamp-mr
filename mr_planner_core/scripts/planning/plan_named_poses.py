#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
from typing import List, Sequence, Tuple


def find_srdf(catkin_src: Path, env: str) -> Path:
    candidates = list(catkin_src.glob(f"**/{env}_moveit_config/config/*.srdf"))
    if candidates:
        return sorted(candidates)[0]
    candidates = list(catkin_src.glob(f"**/{env}.srdf"))
    if candidates:
        return sorted(candidates)[0]
    raise FileNotFoundError(f"Could not locate SRDF for env={env!r} under {catkin_src}")


def plan_once(
    *,
    vamp_environment: str,
    planner: str,
    planning_time: float,
    shortcut_time: float,
    seed: int,
    dt: float,
    vmax: float,
    roadmap_samples: int,
    roadmap_max_dist: float,
    srdf_path: Path,
    move_group: str,
    robot_groups: Sequence[str],
    start_pose: str,
    goal_pose: str,
    output_dir: Path,
    write_tpg: bool,
    meshcat: bool,
    meshcat_host: str,
    meshcat_port: int,
    meshcat_rate: float,
) -> dict:
    import mr_planner_core

    start = mr_planner_core.pose_matrix_from_named_pose(
        srdf_path, move_group=move_group, robot_groups=robot_groups, pose_name=start_pose
    )
    goal = mr_planner_core.pose_matrix_from_named_pose(
        srdf_path, move_group=move_group, robot_groups=robot_groups, pose_name=goal_pose
    )

    output_dir.mkdir(parents=True, exist_ok=True)
    env = mr_planner_core.VampEnvironment(vamp_environment, vmax=vmax, seed=seed)
    res = env.plan(
        planner=planner,
        planning_time=planning_time,
        shortcut_time=shortcut_time,
        seed=seed,
        dt=dt,
        vmax=vmax,
        start=start,
        goal=goal,
        output_dir=str(output_dir),
        write_tpg=write_tpg,
        roadmap_samples=roadmap_samples,
        roadmap_max_dist=roadmap_max_dist,
    )

    if meshcat:
        env.enable_meshcat(meshcat_host, meshcat_port)
        env.play_solution_csv(res["solution_csv"], rate=meshcat_rate)

    return res


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Plan between SRDF named poses and optionally play in Meshcat (requires meshcat_bridge.py)."
    )
    parser.add_argument("--vamp-environment", default="dual_gp4")
    parser.add_argument("--planner", choices=["composite_rrt", "cbs_prm"], default="composite_rrt")
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--shortcut-time", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--vmax", type=float, default=1.0)
    parser.add_argument("--roadmap-samples", type=int, default=300)
    parser.add_argument("--roadmap-max-dist", type=float, default=2.0)

    parser.add_argument("--srdf", default="", help="Path to .srdf (auto-detected if empty)")
    parser.add_argument(
        "--catkin-src",
        default="",
        help="catkin_ws/src path (used for SRDF auto-detection; defaults to repo_root/..)",
    )

    parser.add_argument("--move-group", default="", help="SRDF group that defines the named poses (default: env config)")
    parser.add_argument(
        "--robot-groups",
        default="",
        help="Comma-separated SRDF robot groups (default: env config)",
    )

    parser.add_argument("--start", default="", help="SRDF group_state name (e.g. ready_pose)")
    parser.add_argument("--goal", default="", help="SRDF group_state name")
    parser.add_argument("--poses", nargs="*", default=[], help="If provided, plan sequentially across this pose list.")

    parser.add_argument("--output-dir", default="", help="Output directory (defaults under /tmp)")
    parser.add_argument("--no-tpg", action="store_true", help="Skip building/writing tpg.pb")

    parser.add_argument("--meshcat", action="store_true")
    parser.add_argument("--meshcat-host", default="127.0.0.1")
    parser.add_argument("--meshcat-port", type=int, default=7600)
    parser.add_argument("--meshcat-rate", type=float, default=1.0)
    args = parser.parse_args()

    import mr_planner_core

    env_info = mr_planner_core.vamp_environment_info(args.vamp_environment)
    move_group = args.move_group or env_info["move_group"]
    robot_groups: List[str]
    if args.robot_groups:
        robot_groups = [x.strip() for x in args.robot_groups.split(",") if x.strip()]
    else:
        robot_groups = list(env_info["robot_groups"])

    core_root = Path(__file__).resolve().parents[2]
    default_catkin_src = core_root.parent
    if core_root.name == "mr_planner_core" and (core_root.parent / "mr_planner_lego").is_dir():
        default_catkin_src = core_root.parent.parent
    catkin_src = Path(args.catkin_src) if args.catkin_src else default_catkin_src
    srdf_path = Path(args.srdf) if args.srdf else find_srdf(catkin_src, args.vamp_environment)

    if args.poses:
        if len(args.poses) < 2:
            raise SystemExit("--poses requires at least 2 pose names")
        pose_pairs: Sequence[Tuple[str, str]] = list(zip(args.poses[:-1], args.poses[1:]))
    else:
        if not args.start or not args.goal:
            raise SystemExit("Provide --start and --goal (or use --poses ...)")
        pose_pairs = [(args.start, args.goal)]

    output_root = Path(args.output_dir) if args.output_dir else Path("/tmp") / "mr_planner_py_meshcat"

    results = []
    for idx, (start_pose, goal_pose) in enumerate(pose_pairs):
        out_dir = output_root / f"{idx:02d}_{args.vamp_environment}_{args.planner}_{start_pose}_to_{goal_pose}"
        res = plan_once(
            vamp_environment=args.vamp_environment,
            planner=args.planner,
            planning_time=args.planning_time,
            shortcut_time=args.shortcut_time,
            seed=args.seed,
            dt=args.dt,
            vmax=args.vmax,
            roadmap_samples=args.roadmap_samples,
            roadmap_max_dist=args.roadmap_max_dist,
            srdf_path=srdf_path,
            move_group=move_group,
            robot_groups=robot_groups,
            start_pose=start_pose,
            goal_pose=goal_pose,
            output_dir=out_dir,
            write_tpg=(not args.no_tpg),
            meshcat=args.meshcat,
            meshcat_host=args.meshcat_host,
            meshcat_port=args.meshcat_port,
            meshcat_rate=args.meshcat_rate,
        )
        results.append(res)

    print(json.dumps({"srdf": str(srdf_path), "move_group": move_group, "robot_groups": robot_groups, "runs": results}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
