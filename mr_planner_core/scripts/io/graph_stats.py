#!/usr/bin/env python3

import argparse
import json
import pathlib
from collections import defaultdict
from typing import Any, Dict, List

import mr_planner_graph_pb2 as pb2


def _read_graph_file(path: pathlib.Path) -> pb2.GraphFile:
    data = path.read_bytes()
    graph = pb2.GraphFile()
    graph.ParseFromString(data)
    return graph


def _tpg_stats(tpg: pb2.TPGGraph) -> Dict[str, Any]:
    robots = list(tpg.robots)
    return {
        "type": "TPG",
        "dt": float(tpg.dt),
        "robots": len(robots),
        "nodes_per_robot": [len(r.nodes) for r in robots],
        "type2_edges": len(tpg.type2_edges),
        "pre_shortcut_flowtime": float(tpg.pre_shortcut_flowtime),
        "pre_shortcut_makespan": float(tpg.pre_shortcut_makespan),
        "post_shortcut_flowtime": float(tpg.post_shortcut_flowtime),
        "post_shortcut_makespan": float(tpg.post_shortcut_makespan),
    }


def _adg_stats(adg: pb2.ADGGraph) -> Dict[str, Any]:
    ag = adg.activity_graph
    per_robot: Dict[int, int] = defaultdict(int)
    for a in ag.activities:
        per_robot[int(a.key.robot_id)] += 1

    return {
        "type": "ADG",
        "robots": int(ag.num_robots),
        "num_activities": len(ag.activities),
        "activities_per_robot": [per_robot.get(rid, 0) for rid in range(int(ag.num_robots))],
        "num_objects": len(ag.objects),
        "exec_start_act": list(adg.exec_start_act),
        **{k: v for k, v in _tpg_stats(adg.base).items() if k != "type"},
    }


def main(argv: List[str]) -> int:
    ap = argparse.ArgumentParser(description="Machine-readable summary of mr_planner portable TPG/ADG protobuf graphs.")
    ap.add_argument("--input", required=True, help="Path to .pb file (mr_planner_graph.proto GraphFile).")
    ap.add_argument("--pretty", action="store_true", help="Pretty-print JSON.")
    args = ap.parse_args(argv)

    graph = _read_graph_file(pathlib.Path(args.input))
    which = graph.WhichOneof("graph")
    if which == "tpg":
        stats = _tpg_stats(graph.tpg)
    elif which == "adg":
        stats = _adg_stats(graph.adg)
    else:
        raise SystemExit("error: protobuf file has no graph set (expected oneof graph {tpg|adg})")

    if args.pretty:
        print(json.dumps(stats, indent=2, sort_keys=True))
    else:
        print(json.dumps(stats, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(argv=list(__import__("sys").argv[1:])))
