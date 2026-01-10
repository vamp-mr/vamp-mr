#!/usr/bin/env python3
import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import List


def run(cmd, *, cwd=None):
    subprocess.check_call(cmd, cwd=cwd)

def ok(msg: str) -> None:
    print(f"[ok] {msg}", file=sys.stderr)


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
    ok(f"env.plan composite_rrt produced outputs in {out_dir}")

    skillplan_path = Path(res["skillplan_json"])
    tpg_pb_path = Path(res["tpg_pb"])
    assert skillplan_path.is_file(), f"missing {skillplan_path}"
    assert tpg_pb_path.is_file(), f"missing {tpg_pb_path}"

    json.load(skillplan_path.open("rb"))
    ok("skillplan.json parses as JSON")
    tpg_json = mr_planner_core.graphfile_to_json(str(tpg_pb_path))
    assert '"tpg"' in tpg_json, "expected GraphFile.tpg payload in JSON"
    ok("tpg.pb converts to GraphFile JSON")

    # Backward-compat: allow nonstandard environment names (infer from robots[]).
    bad_env_skillplan = work_dir / "skillplan_bad_env.json"
    bad = json.load(skillplan_path.open("rb"))
    # Use a non-existent name (avoid accidentally resolving to a repo folder like ./mr_planner_lego).
    bad.setdefault("environment", {})["name"] = "mr_planner_invalid_env_name"
    bad_env_skillplan.write_text(json.dumps(bad, indent=2))
    inferred = mr_planner_core.skillplan_to_execution_graph(
        str(bad_env_skillplan),
        graph_type="tpg",
        vamp_environment="",
        output_dir=str(work_dir / "py_skillplan_bad_env_graphs"),
    )
    assert inferred["environment"] == "dual_gp4"
    assert Path(inferred["pb_path"]).is_file()
    ok("skillplan_to_execution_graph infers env from robots[]")

    # Round-trip protobuf JSON I/O.
    rt_pb = work_dir / "roundtrip_tpg.pb"
    mr_planner_core.graphfile_from_json(tpg_json, str(rt_pb))
    assert rt_pb.is_file(), f"missing {rt_pb}"
    ok("GraphFile JSON -> protobuf roundtrip works")

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
    ok("skillplan_to_execution_graph produces a TPG")

    # Collision checks and object API smoke.
    assert env.in_collision(res["start"], self_only=True) is False
    ok("in_collision(self_only=True) passes on start")
    assert (
        env.trajectory_in_collision(
            [[res["start"][0], res["goal"][0]], [res["start"][1], res["goal"][1]]], self_only=True
        )
        is False
    )
    ok("trajectory_in_collision(self_only=True) passes on start->goal")
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
    ok("add_object + has_object works")
    box.x = 9.5
    env.move_object(box)
    ok("move_object works")
    env.remove_object(box.name)
    assert not env.has_object(box.name)
    ok("remove_object works")

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
        ok(f"lego_assign task={task}")

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
        ok(f"lego_plan task={task}")

        task_skillplan = task_out / "skillplan.json"
        task_adg_pb = task_out / "adg.pb"
        assert task_skillplan.is_file(), f"missing {task_skillplan}"
        assert task_adg_pb.is_file(), f"missing {task_adg_pb}"
        json.load(task_skillplan.open("rb"))
        ok(f"lego outputs exist + skillplan parses (task={task})")

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
        ok(f"skillplan_to_execution_graph ADG built (task={task})")

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
        ok(f"ADG parity matched for task={task}")

        # Quick JSON I/O sanity on the derived graph.
        task_adg_json = mr_planner_core.graphfile_to_json(str(task_adg_from_skillplan_pb))
        assert '"adg"' in task_adg_json, "expected GraphFile.adg payload in JSON"
        ok(f"derived ADG protobuf converts to JSON (task={task})")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
