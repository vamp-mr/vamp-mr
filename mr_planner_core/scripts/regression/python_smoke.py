#!/usr/bin/env python3
import argparse
import json
import os
import subprocess
import sys
import time
import statistics
from pathlib import Path
from typing import Dict, List, Sequence, Tuple
from xml.etree import ElementTree


def run(cmd, *, cwd=None):
    subprocess.check_call(cmd, cwd=cwd)


def graph_stats(repo_root: Path, pb_path: Path) -> dict:
    out = subprocess.check_output(
        [sys.executable, str(repo_root / "mr_planner_core/scripts/io/graph_stats.py"), "--input", str(pb_path)],
        text=True,
    )
    return json.loads(out)


def find_workspace_root(repo_root: Path) -> Path:
    # Allow running from the mr_planner monorepo root or from the mr_planner_core subdir.
    if (repo_root / "launch").is_dir() and (repo_root / "mr_planner_core").is_dir():
        return repo_root
    if repo_root.name == "mr_planner_core" and (repo_root.parent / "launch").is_dir():
        return repo_root.parent
    return repo_root


def default_catkin_src(workspace_root: Path) -> Path:
    # In the monorepo layout, the ROS workspace is one level up from the repo root.
    return workspace_root.parent


def parse_launch_pose_names(path: Path) -> List[str]:
    if not path.is_file():
        raise FileNotFoundError(path)

    root = ElementTree.parse(str(path)).getroot()
    found: List[Tuple[int, str]] = []
    for node in root.iter():
        if node.tag != "param":
            continue
        name = node.get("name", "")
        if not name.startswith("pose_name"):
            continue
        val = node.get("value")
        if not val:
            continue
        suffix = name[len("pose_name") :]
        idx = 0 if suffix == "" else int(suffix) if suffix.isdigit() else 0
        found.append((idx, val))

    found_sorted = [v for _, v in sorted(found, key=lambda x: x[0])]
    # Deduplicate while preserving order.
    out: List[str] = []
    seen = set()
    for v in found_sorted:
        if v in seen:
            continue
        seen.add(v)
        out.append(v)
    return out



def find_srdf(repo_root: Path, env: str, *, catkin_src: Path) -> Path:
    # Preferred: vendored SRDFs under mr_planner_core (avoid depending on external catkin packages).
    candidates = [
        repo_root / "mr_planner_core" / "config" / "srdf" / f"{env}.srdf",  # monorepo
        repo_root / "config" / "srdf" / f"{env}.srdf",  # core-only checkout
    ]
    for path in candidates:
        if path.is_file():
            return path

    # Backward-compatible fallback: locate SRDF in a catkin workspace (moveit_config packages).
    candidates = list(catkin_src.glob(f"**/{env}_moveit_config/config/*.srdf"))
    if candidates:
        return sorted(candidates)[0]

    candidates = list(catkin_src.glob(f"**/{env}.srdf"))
    if candidates:
        return sorted(candidates)[0]

    raise FileNotFoundError(f"Could not locate SRDF for env={env!r} under {catkin_src}")


def fmt_seconds(values: Sequence[float]) -> str:
    if not values:
        return "n/a"
    med = statistics.median(values)
    std = statistics.stdev(values) if len(values) >= 2 else 0.0
    return f"{med:.3f}s (std {std:.3f}s)"


def run_planning_suite(
    *,
    repo_root: Path,
    work_dir: Path,
    envs: Sequence[str],
    planners: Sequence[str],
    pose_mode: str,
    max_poses: int,
    planning_time: float,
    roadmap_samples: int,
    roadmap_max_dist: float,
    allow_failures: bool,
    min_success_rate: float,
) -> None:
    import mr_planner_core

    workspace_root = find_workspace_root(repo_root)
    catkin_src = default_catkin_src(workspace_root)
    failures: List[str] = []
    successes: List[str] = []
    failure_counts: Dict[str, Dict[str, int]] = {}
    rate_failures: List[str] = []

    preferred_by_env = {
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
        "panda_two": [
            "ready",
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
            "red"
        ],
    }

    for env in envs:
        print("==========================================================")
        print("Planning suite for environment:", env)
        env_info = mr_planner_core.vamp_environment_info(env)
        move_group = env_info["move_group"]
        robot_groups = env_info["robot_groups"]
        srdf_path = find_srdf(repo_root, env, catkin_src=catkin_src)
        env_instance = mr_planner_core.VampEnvironment(env, vmax=1.0, seed=1)

        launch_file = workspace_root / "launch" / f"{env}.launch"
        pose_names = parse_launch_pose_names(launch_file)

        preferred = preferred_by_env.get(env, [])
        preferred_in_launch = [p for p in preferred if p in set(pose_names)]
        pose_candidates = preferred_in_launch + [p for p in pose_names if p not in set(preferred_in_launch)]

        pose_matrix: Dict[str, List[List[float]]] = {}
        valid_pose_names: List[str] = []
        invalid_pose_names: List[str] = []
        for name in pose_candidates:
            if max_poses > 0 and len(valid_pose_names) >= max_poses:
                break
            try:
                mat = mr_planner_core.pose_matrix_from_named_pose(
                    srdf_path, move_group=move_group, robot_groups=robot_groups, pose_name=name
                )
                pose_matrix[name] = mat
                print(name, mat)
                if env_instance.in_collision(mat, self_only=False):
                    invalid_pose_names.append(name)
                    continue
                valid_pose_names.append(name)
            except Exception as exc:  # noqa: BLE001
                invalid_pose_names.append(name)
                failures.append(f"[fail] {env}: pose {name!r} invalid: {exc}")
                failure_counts.setdefault(env, {}).setdefault("__pose__", 0)
                failure_counts[env]["__pose__"] += 1

        if invalid_pose_names:
            print(f"[bench] filtered {len(invalid_pose_names)} invalid/colliding poses:", ", ".join(invalid_pose_names))

        pose_names = valid_pose_names

        if len(pose_names) < 2:
            raise RuntimeError(f"{env}: need at least 2 poses (got {pose_names})")

        if pose_mode == "sequential":
            pose_pairs = list(zip(pose_names[:-1], pose_names[1:]))
        elif pose_mode == "all_pairs":
            pose_pairs = [(a, b) for a in pose_names for b in pose_names if a != b]
        else:
            raise ValueError(f"invalid pose_mode={pose_mode!r}")

        for planner in planners:
            print("----------------------------------------")
            print("Benchmarking planner:", planner)
            attempted = 0
            succeeded = 0
            wall_times_sec: List[float] = []
            planner_times_sec: List[float] = []
            for start_name, goal_name in pose_pairs:
                case = f"{env}:{planner}:{start_name}->{goal_name}"
                out_dir = work_dir / "bench" / env / planner / f"{start_name}_to_{goal_name}"
                out_dir.mkdir(parents=True, exist_ok=True)

                try:
                    case_planning_time = planning_time * (2.0 if planner == "cbs_prm" else 1.0)
                    start = pose_matrix[start_name]
                    goal = pose_matrix[goal_name]

                    print(f"[bench] {case} (srdf={srdf_path.name}, t={case_planning_time:g}s)")
                    attempted += 1
                    t0 = time.perf_counter()
                    res = env_instance.plan(
                        planner=planner,
                        planning_time=case_planning_time,
                        shortcut_time=0.0,
                        seed=1,
                        start=start,
                        goal=goal,
                        output_dir=str(out_dir),
                        write_tpg=False,
                        roadmap_samples=roadmap_samples,
                        roadmap_max_dist=roadmap_max_dist,
                    )
                    dt = time.perf_counter() - t0

                    sol = Path(res["solution_csv"])
                    if not sol.is_file():
                        raise RuntimeError(f"missing solution.csv at {sol}")
                    successes.append(case)
                    succeeded += 1
                    wall_times_sec.append(dt)
                    planner_times_sec.append(float(res.get("planner_time_sec", dt)))
                except Exception as exc:  # noqa: BLE001
                    msg = f"[fail] {case}: {exc}"
                    failures.append(msg)
                    failure_counts.setdefault(env, {}).setdefault(planner, 0)
                    failure_counts[env][planner] += 1
                    if not allow_failures:
                        continue
            
            rate = (succeeded / attempted) if attempted else 0.0
            print(
                f"[bench] {env}/{planner}: success {succeeded}/{attempted} ({rate*100:.1f}%), "
                f"wall {fmt_seconds(wall_times_sec)}, planner {fmt_seconds(planner_times_sec)}"
            )
            if min_success_rate > 0.0 and attempted > 0 and rate < min_success_rate:
                rate_failures.append(
                    f"[fail] {env}/{planner}: success rate {rate*100:.1f}% < {min_success_rate*100:.1f}% "
                    f"({succeeded}/{attempted})"
                )

    if failures and not allow_failures:
        summary = "\n".join(failures[:50])
        raise RuntimeError(f"planning suite had {len(failures)} failures; first failures:\n{summary}")

    if rate_failures:
        summary = "\n".join(rate_failures[:50])
        raise RuntimeError(f"planning suite success-rate failures:\n{summary}")

    print(f"[bench] completed with {len(failures)} failures and {len(successes)} successes")

    if failures:
        for line in failures[:20]:
            print(line)
        print("[bench] failure counts by environment/planner:")
        for env_name in sorted(failure_counts):
            planners_for_env = failure_counts[env_name]
            for planner_name in sorted(planners_for_env):
                count = planners_for_env[planner_name]
                print(f"  {env_name}/{planner_name}: {count} failures")

    if not successes:
        raise RuntimeError("planning suite had 0 successes")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--work-dir", required=True)
    parser.add_argument("--suite", choices=["smoke", "benchmark"], default="smoke")
    parser.add_argument("--bench-env", action="append", default=[])
    parser.add_argument("--bench-planner", action="append", default=[])
    parser.add_argument("--bench-pose-mode", choices=["sequential", "all_pairs"], default="all_pairs")
    parser.add_argument("--bench-max-poses", type=int, default=99)
    parser.add_argument("--bench-planning-time", type=float, default=5.0)
    parser.add_argument("--bench-roadmap-samples", type=int, default=5000)
    parser.add_argument("--bench-roadmap-max-dist", type=float, default=2.0)
    parser.add_argument("--bench-allow-failures", action="store_true")
    parser.add_argument("--bench-strict", action="store_true", help="Fail if any planning query fails")
    parser.add_argument("--bench-min-success-rate", type=float, default=0.0, help="Fail if any env/planner success rate is below this threshold (0 disables)")
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve()
    workspace_root = find_workspace_root(repo_root)
    prefix = Path(args.prefix)
    work_dir = Path(args.work_dir)

    import mr_planner_core

    if args.suite == "benchmark":
        envs = args.bench_env or ["panda_two_rod", "panda_four", "panda_four_bins"]
        planners = args.bench_planner or ["composite_rrt", "cbs_prm"]
        min_rate = float(args.bench_min_success_rate)
        if args.bench_strict and min_rate <= 0.0:
            min_rate = 1.0

        bench_py = repo_root / "mr_planner_core/scripts/benchmarks/core_planning_benchmark.py"
        bench_out = work_dir / "bench_planning"

        cmd = [
            sys.executable,
            str(bench_py),
            "--output-dir",
            str(bench_out),
            "--pose-mode",
            args.bench_pose_mode,
            "--max-poses",
            str(args.bench_max_poses),
            "--planning-time",
            str(args.bench_planning_time),
            "--roadmap-samples",
            str(args.bench_roadmap_samples),
            "--roadmap-max-dist",
            str(args.bench_roadmap_max_dist),
            "--min-success-rate",
            str(min_rate),
        ]
        for env_name in envs:
            cmd += ["--env", env_name]
        for planner_name in planners:
            cmd += ["--planner", planner_name]
        subprocess.check_call(cmd)
        return 0

    out_dir = work_dir / "py_plan_out"
    out_dir.mkdir(parents=True, exist_ok=True)

    env = mr_planner_core.VampEnvironment("dual_gp4", vmax=1.0, seed=1)
    res = env.plan(
        planner="composite_rrt",
        planning_time=3.0,
        shortcut_time=0.1,
        seed=1,
        output_dir=str(out_dir),
        write_tpg=True,
    )

    skillplan_path = Path(res["skillplan_json"])
    tpg_pb_path = Path(res["tpg_pb"])
    assert skillplan_path.is_file(), f"missing {skillplan_path}"
    assert tpg_pb_path.is_file(), f"missing {tpg_pb_path}"

    json.load(skillplan_path.open("rb"))
    tpg_json = mr_planner_core.graphfile_to_json(str(tpg_pb_path))
    assert '"tpg"' in tpg_json, "expected GraphFile.tpg payload in JSON"

    # Backward-compat: allow nonstandard environment names (infer from robots[]).
    bad_env_skillplan = work_dir / "skillplan_bad_env.json"
    bad = json.load(skillplan_path.open("rb"))
    bad.setdefault("environment", {})["name"] = "mr_planner_lego"
    bad_env_skillplan.write_text(json.dumps(bad, indent=2))
    inferred = mr_planner_core.skillplan_to_execution_graph(
        str(bad_env_skillplan),
        graph_type="tpg",
        vamp_environment="",
        output_dir=str(work_dir / "py_skillplan_bad_env_graphs"),
    )
    assert inferred["environment"] == "dual_gp4"
    assert Path(inferred["pb_path"]).is_file()

    # Round-trip protobuf JSON I/O.
    rt_pb = work_dir / "roundtrip_tpg.pb"
    mr_planner_core.graphfile_from_json(tpg_json, str(rt_pb))
    assert rt_pb.is_file(), f"missing {rt_pb}"

    # Build graphs from a (simple) skillplan.
    graph_out = work_dir / "py_skillplan_graphs"
    graph_out.mkdir(parents=True, exist_ok=True)

    tpg_res = mr_planner_core.skillplan_to_execution_graph(
        str(skillplan_path),
        graph_type="tpg",
        vamp_environment="dual_gp4",
        output_dir=str(graph_out),
    )
    assert Path(tpg_res["pb_path"]).is_file()

    # Collision checks and object API smoke.
    assert env.in_collision(res["start"], self_only=True) is False
    assert (
        env.trajectory_in_collision(
            [[res["start"][0], res["goal"][0]], [res["start"][1], res["goal"][1]]], self_only=True
        )
        is False
    )
    box = mr_planner_core.Object()
    box.name = "py_smoke_box"
    box.state = mr_planner_core.Object.State.Static
    box.shape = mr_planner_core.Object.Shape.Box
    box.length = 0.1
    box.width = 0.1
    box.height = 0.1
    box.x = 10.0
    box.y = 10.0
    box.z = 10.0
    env.add_object(box)
    assert env.has_object(box.name)
    box.x = 9.5
    env.move_object(box)
    env.remove_object(box.name)
    assert not env.has_object(box.name)

    # Build a graph from a lego-generated skillplan.
    steps_dir = work_dir / "py_lego_steps"
    lego_out = work_dir / "py_lego_out"
    steps_dir.mkdir(parents=True, exist_ok=True)
    lego_out.mkdir(parents=True, exist_ok=True)

    lego_assign = prefix / "bin/mr_planner_lego_assign"
    lego_plan = prefix / "bin/mr_planner_lego_plan"
    have_lego = lego_assign.is_file() and lego_plan.is_file()
    have_lego_config = (workspace_root / "config/lego_tasks").is_dir()

    if not have_lego or not have_lego_config:
        print("[info] skipping lego CLI checks (mr_planner_lego not available)")
        return 0

    lego_graph_out = work_dir / "py_lego_skillplan_graphs"
    lego_graph_out.mkdir(parents=True, exist_ok=True)

    tasks = ["test", "cliff", "vessel"]
    for task in tasks:
        task_steps = steps_dir / task
        task_out = lego_out / task
        task_steps.mkdir(parents=True, exist_ok=True)
        task_out.mkdir(parents=True, exist_ok=True)

        run(
            [
                str(lego_assign),
                "--task",
                task,
                "--root",
                str(workspace_root),
                "--output-dir",
                str(task_steps),
                "--vamp-environment",
                "dual_gp4",
            ]
        )

        run(
            [
                str(lego_plan),
                "--task",
                task,
                "--root",
                str(workspace_root),
                "--steps-dir",
                str(task_steps),
                "--output-dir",
                str(task_out),
                "--vamp-environment",
                "dual_gp4",
                "--planning-time",
                "5",
                "--shortcut-time",
                "0",
                "--seed",
                "1",
            ]
        )

        task_skillplan = task_out / "skillplan.json"
        task_adg_pb = task_out / "adg.pb"
        assert task_skillplan.is_file(), f"missing {task_skillplan}"
        assert task_adg_pb.is_file(), f"missing {task_adg_pb}"
        json.load(task_skillplan.open("rb"))

        task_graph_out = lego_graph_out / task
        task_graph_out.mkdir(parents=True, exist_ok=True)
        task_adg = mr_planner_core.skillplan_to_execution_graph(
            str(task_skillplan),
            graph_type="adg",
            vamp_environment="dual_gp4",
            output_dir=str(task_graph_out),
            shortcut_time=0.0,
        )
        task_adg_from_skillplan_pb = Path(task_adg["pb_path"])
        assert task_adg_from_skillplan_pb.is_file(), f"missing {task_adg_from_skillplan_pb}"

        direct = graph_stats(workspace_root, task_adg_pb)
        derived = graph_stats(workspace_root, task_adg_from_skillplan_pb)

        required_equal = [
            "type",
            "robots",
            "dt",
            "nodes_per_robot",
            "type2_edges",
            "num_activities",
            "activities_per_robot",
            "num_objects",
            "exec_start_act",
        ]
        for key in required_equal:
            assert direct.get(key) == derived.get(key), f"{task}: mismatch {key}: direct={direct.get(key)} derived={derived.get(key)}"

        tol = 1e-9
        float_keys = [
            "pre_shortcut_flowtime",
            "pre_shortcut_makespan",
            "post_shortcut_flowtime",
            "post_shortcut_makespan",
        ]
        for key in float_keys:
            a = float(direct.get(key, 0.0))
            b = float(derived.get(key, 0.0))
            assert abs(a - b) <= tol, f"{task}: mismatch {key}: direct={a} derived={b}"

        # Quick JSON I/O sanity on the derived graph.
        task_adg_json = mr_planner_core.graphfile_to_json(str(task_adg_from_skillplan_pb))
        assert '"adg"' in task_adg_json, "expected GraphFile.adg payload in JSON"

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
