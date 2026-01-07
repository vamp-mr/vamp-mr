#!/usr/bin/env python3

import argparse
import heapq
import pathlib
import shutil
import subprocess
import sys
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import mr_planner_graph_pb2 as pb2


NodeKey = Tuple[int, int]  # (robot_id, time_step)
ActKey = Tuple[int, int]  # (robot_id, act_id)


def _dot_escape(text: str) -> str:
    return text.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", "\\n")


def _node_id(key: NodeKey) -> str:
    rid, step = key
    return f"r{rid}_t{step}"


def _act_id(key: ActKey) -> str:
    rid, act = key
    return f"a_r{rid}_act{act}"


def _read_graph_file(path: pathlib.Path) -> pb2.GraphFile:
    data = path.read_bytes()
    graph = pb2.GraphFile()
    graph.ParseFromString(data)
    return graph


def _type2_from(edge: pb2.Type2Edge) -> pb2.NodeKey:
    # "from" is a Python keyword, so protobuf generates an attribute named "from"
    # which cannot be accessed via dot syntax (edge.from is a SyntaxError).
    return getattr(edge, "from")


@dataclass(frozen=True)
class TopoResult:
    order: List[str]
    has_cycle: bool
    levels: Dict[str, int]


def _topo_sort(nodes: Iterable[str], edges: Iterable[Tuple[str, str]]) -> TopoResult:
    adj: Dict[str, List[str]] = defaultdict(list)
    indeg: Dict[str, int] = {}
    levels: Dict[str, int] = {}

    for n in nodes:
        indeg.setdefault(n, 0)
        levels.setdefault(n, 0)

    for u, v in edges:
        if u == v:
            indeg.setdefault(u, 0)
            levels.setdefault(u, 0)
            indeg[u] += 1
            continue
        indeg.setdefault(u, 0)
        indeg.setdefault(v, 0)
        levels.setdefault(u, 0)
        levels.setdefault(v, 0)
        adj[u].append(v)
        indeg[v] += 1

    ready = [n for n, d in indeg.items() if d == 0]
    heapq.heapify(ready)
    order: List[str] = []
    while ready:
        u = heapq.heappop(ready)
        order.append(u)
        for v in adj.get(u, []):
            if levels[v] < levels[u] + 1:
                levels[v] = levels[u] + 1
            indeg[v] -= 1
            if indeg[v] == 0:
                heapq.heappush(ready, v)

    has_cycle = len(order) != len(indeg)
    return TopoResult(order=order, has_cycle=has_cycle, levels=levels)


def _summarize_tpg(tpg: pb2.TPGGraph) -> str:
    robots = tpg.robots
    node_counts = [len(r.nodes) for r in robots]
    lines = [
        f"type: TPG",
        f"dt: {tpg.dt:g}",
        f"robots: {len(robots)}",
        f"nodes_per_robot: {node_counts}",
        f"type2_edges: {len(tpg.type2_edges)}",
        f"pre_shortcut: flowtime={tpg.pre_shortcut_flowtime:g}, makespan={tpg.pre_shortcut_makespan:g}",
        f"post_shortcut: flowtime={tpg.post_shortcut_flowtime:g}, makespan={tpg.post_shortcut_makespan:g}",
    ]
    return "\n".join(lines)


def _summarize_adg(adg: pb2.ADGGraph) -> str:
    ag = adg.activity_graph
    per_robot_acts: Dict[int, int] = defaultdict(int)
    for a in ag.activities:
        per_robot_acts[int(a.key.robot_id)] += 1
    act_counts = [per_robot_acts.get(rid, 0) for rid in range(int(ag.num_robots))]
    lines = [
        f"type: ADG",
        f"robots: {ag.num_robots}",
        f"activities_per_robot: {act_counts}",
        f"objects: {len(ag.objects)}",
        f"exec_start_act: {list(adg.exec_start_act)}",
        "",
        _summarize_tpg(adg.base).replace("type: TPG", "base_graph: TPG"),
    ]
    return "\n".join(lines)


def _collect_tpg_graph(tpg: pb2.TPGGraph) -> Tuple[Dict[NodeKey, pb2.Node], List[Tuple[NodeKey, NodeKey]], List[pb2.Type2Edge]]:
    nodes: Dict[NodeKey, pb2.Node] = {}
    type1_edges: List[Tuple[NodeKey, NodeKey]] = []

    for r in tpg.robots:
        rid = int(r.robot_id)
        prev_key: Optional[NodeKey] = None
        for n in r.nodes:
            key = (rid, int(n.time_step))
            nodes[key] = n
            if prev_key is not None:
                type1_edges.append((prev_key, key))
            prev_key = key

    return nodes, type1_edges, list(tpg.type2_edges)


def _tpg_to_dot(tpg: pb2.TPGGraph) -> str:
    nodes, type1_edges, type2_edges = _collect_tpg_graph(tpg)

    lines: List[str] = []
    lines.append("digraph TPG {")
    lines.append("  rankdir=LR;")
    lines.append("  node [shape=box, fontsize=10];")
    lines.append("  edge [fontsize=9];")

    for r in tpg.robots:
        rid = int(r.robot_id)
        rlabel = r.robot_name if r.robot_name else f"robot_{rid}"
        lines.append(f"  subgraph cluster_r{rid} {{")
        lines.append(f"    label=\"{_dot_escape(rlabel)}\";")
        lines.append("    color=\"#cccccc\";")
        for n in r.nodes:
            key = (rid, int(n.time_step))
            nid = _node_id(key)
            label = f"t={n.time_step}\\nact={n.act_id}"
            lines.append(f"    \"{nid}\" [label=\"{_dot_escape(label)}\"];")
        lines.append("  }")

    for u, v in type1_edges:
        lines.append(f"  \"{_node_id(u)}\" -> \"{_node_id(v)}\" [color=\"#333333\"];")

    for e in type2_edges:
        frm = _type2_from(e)
        u = (int(frm.robot_id), int(frm.time_step))
        v = (int(e.to.robot_id), int(e.to.time_step))
        attrs = [
            "color=\"#d62728\"",
            f"label=\"{e.edge_id}\"",
            "style=dashed" if e.switchable else "style=solid",
        ]
        if e.tight:
            attrs.append("penwidth=2")
        lines.append(f"  \"{_node_id(u)}\" -> \"{_node_id(v)}\" [{', '.join(attrs)}];")

    lines.append("}")
    return "\n".join(lines) + "\n"


def _collect_adg_activity_edges(adg: pb2.ADGGraph) -> Tuple[Dict[ActKey, pb2.Activity], List[Tuple[ActKey, ActKey]]]:
    ag = adg.activity_graph
    acts: Dict[ActKey, pb2.Activity] = {}
    for a in ag.activities:
        key = (int(a.key.robot_id), int(a.key.act_id))
        acts[key] = a

    edges: List[Tuple[ActKey, ActKey]] = []

    # Type1 edges implied by per-robot act_id ordering.
    per_robot: Dict[int, List[ActKey]] = defaultdict(list)
    for key in acts.keys():
        per_robot[key[0]].append(key)
    for rid, keys in per_robot.items():
        keys_sorted = sorted(keys, key=lambda k: k[1])
        for i in range(len(keys_sorted) - 1):
            edges.append((keys_sorted[i], keys_sorted[i + 1]))

    # Type2 edges from explicit dependencies.
    for akey, a in acts.items():
        for dep in a.type2_prev:
            dkey = (int(dep.robot_id), int(dep.act_id))
            edges.append((dkey, akey))

    return acts, edges


def _adg_activities_to_dot(adg: pb2.ADGGraph, include_objects: bool) -> str:
    ag = adg.activity_graph
    acts, edges = _collect_adg_activity_edges(adg)

    exec_starts: Dict[int, int] = {}
    if len(adg.exec_start_act) == int(ag.num_robots):
        exec_starts = {rid: int(adg.exec_start_act[rid]) for rid in range(int(ag.num_robots))}

    lines: List[str] = []
    lines.append("digraph ADG {")
    lines.append("  rankdir=LR;")
    lines.append("  node [shape=box, fontsize=10];")
    lines.append("  edge [fontsize=9];")

    per_robot: Dict[int, List[ActKey]] = defaultdict(list)
    for key in acts.keys():
        per_robot[key[0]].append(key)

    for rid, keys in sorted(per_robot.items(), key=lambda kv: kv[0]):
        lines.append(f"  subgraph cluster_a_r{rid} {{")
        lines.append(f"    label=\"robot_{rid}\";")
        lines.append("    color=\"#cccccc\";")
        for key in sorted(keys, key=lambda k: k[1]):
            act = acts[key]
            nid = _act_id(key)
            type_name = act.type_name if act.type_name else str(act.type)
            start = int(act.start_time_step)
            end = int(act.end_time_step)
            if start >= 0 or end >= 0:
                span = f"[{start},{end}]"
            else:
                span = ""
            label = f"{key[1]} {type_name}{span}"
            attrs = []
            if exec_starts.get(rid) == key[1]:
                attrs.append("peripheries=2")
            lines.append(f"    \"{nid}\" [label=\"{_dot_escape(label)}\"{', ' if attrs else ''}{', '.join(attrs)}];")
        lines.append("  }")

    # Optional object nodes
    if include_objects:
        lines.append("  node [shape=ellipse, fontsize=10];")
        for o in ag.objects:
            name = o.object.name if o.HasField("object") else f"obj_{o.obj_node_id}"
            onid = f"obj_{o.obj_node_id}"
            lines.append(f"  \"{onid}\" [label=\"{_dot_escape(name)}\"];")

        for o in ag.objects:
            onid = f"obj_{o.obj_node_id}"
            if o.HasField("prev_detach"):
                src = _act_id((int(o.prev_detach.robot_id), int(o.prev_detach.act_id)))
                lines.append(f"  \"{src}\" -> \"{onid}\" [color=\"#1f77b4\", label=\"detach\"];")
            if o.HasField("next_attach"):
                dst = _act_id((int(o.next_attach.robot_id), int(o.next_attach.act_id)))
                lines.append(f"  \"{onid}\" -> \"{dst}\" [color=\"#1f77b4\", label=\"attach\"];")
        lines.append("  node [shape=box, fontsize=10];")

    for u, v in edges:
        lines.append(f"  \"{_act_id(u)}\" -> \"{_act_id(v)}\" [color=\"#d62728\"];")

    lines.append("}")
    return "\n".join(lines) + "\n"


def _write_text(path: pathlib.Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _render_dot(dot_exe: str, dot_path: pathlib.Path, out_png: pathlib.Path) -> None:
    subprocess.run([dot_exe, "-Tpng", str(dot_path), "-o", str(out_png)], check=True)


def main(argv: Sequence[str]) -> int:
    ap = argparse.ArgumentParser(description="Inspect/plot mr_planner portable TPG/ADG protobuf graphs.")
    ap.add_argument("--input", required=True, help="Path to .pb file (mr_planner_graph.proto GraphFile).")
    ap.add_argument("--dot", default="", help="Write node-level DOT (TPG/base graph) to this path.")
    ap.add_argument("--adg-dot", default="", help="Write activity-level DOT (ADG only) to this path.")
    ap.add_argument("--no-render", action="store_false", dest="render",
                    help="Disable rendering DOT outputs to PNG using graphviz 'dot'.")
    ap.add_argument("--no-include-objects", action="store_false", dest="include_objects",
                    help="Exclude object nodes from --adg-dot output.")
    ap.add_argument("--topo", choices=["none", "nodes", "activities", "both"], default="none",
                    help="Compute a topological ordering for selected graph(s).")
    ap.add_argument("--max-print", type=int, default=25, help="Max entries to print for topo order previews.")
    ap.set_defaults(render=True, include_objects=True)
    args = ap.parse_args(list(argv))

    in_path = pathlib.Path(args.input)
    default_dot_path = in_path.with_suffix(".dot")
    dot_exe = shutil.which("dot")
    render_enabled = args.render and dot_exe is not None
    if args.render and dot_exe is None:
        print("warning: graphviz 'dot' not found; skipping PNG render.", file=sys.stderr)

    graph = _read_graph_file(in_path)
    which = graph.WhichOneof("graph")
    if not which:
        print("error: protobuf file has no graph set (expected oneof graph {tpg|adg})", file=sys.stderr)
        return 2

    if which == "tpg":
        if not args.dot:
            args.dot = str(default_dot_path)
        print(_summarize_tpg(graph.tpg))
        if args.dot:
            dot_path = pathlib.Path(args.dot)
            _write_text(dot_path, _tpg_to_dot(graph.tpg))
            print(f"\nwrote DOT: {dot_path}")
            if render_enabled:
                png_path = dot_path.with_suffix(".png")
                _render_dot(dot_exe, dot_path, png_path)
                print(f"wrote PNG: {png_path}")

        if args.topo in ("nodes", "both"):
            nodes, type1_edges, type2_edges = _collect_tpg_graph(graph.tpg)
            edge_pairs: List[Tuple[str, str]] = [(_node_id(u), _node_id(v)) for (u, v) in type1_edges]
            for e in type2_edges:
                frm = _type2_from(e)
                u = (int(frm.robot_id), int(frm.time_step))
                v = (int(e.to.robot_id), int(e.to.time_step))
                edge_pairs.append((_node_id(u), _node_id(v)))
            topo = _topo_sort([_node_id(k) for k in nodes.keys()], edge_pairs)
            print(f"\nnode topo: count={len(topo.order)}, cycle={topo.has_cycle}")
            if topo.order:
                preview = topo.order[: args.max_print]
                print("  head:", " ".join(preview))
    elif which == "adg":
        if not args.adg_dot:
            args.adg_dot = str(default_dot_path)
        print(_summarize_adg(graph.adg))
        if args.dot:
            dot_path = pathlib.Path(args.dot)
            _write_text(dot_path, _tpg_to_dot(graph.adg.base))
            print(f"\nwrote DOT: {dot_path}")
            if render_enabled:
                png_path = dot_path.with_suffix(".png")
                _render_dot(dot_exe, dot_path, png_path)
                print(f"wrote PNG: {png_path}")

        if args.adg_dot:
            dot_path = pathlib.Path(args.adg_dot)
            _write_text(dot_path, _adg_activities_to_dot(graph.adg, include_objects=args.include_objects))
            print(f"wrote ADG DOT: {dot_path}")
            if render_enabled:
                png_path = dot_path.with_suffix(".png")
                _render_dot(dot_exe, dot_path, png_path)
                print(f"wrote ADG PNG: {png_path}")

        if args.topo in ("nodes", "both"):
            nodes, type1_edges, type2_edges = _collect_tpg_graph(graph.adg.base)
            edge_pairs = [(_node_id(u), _node_id(v)) for (u, v) in type1_edges]
            for e in type2_edges:
                frm = _type2_from(e)
                u = (int(frm.robot_id), int(frm.time_step))
                v = (int(e.to.robot_id), int(e.to.time_step))
                edge_pairs.append((_node_id(u), _node_id(v)))
            topo = _topo_sort([_node_id(k) for k in nodes.keys()], edge_pairs)
            print(f"\nnode topo: count={len(topo.order)}, cycle={topo.has_cycle}")
            if topo.order:
                preview = topo.order[: args.max_print]
                print("  head:", " ".join(preview))

        if args.topo in ("activities", "both"):
            acts, edges = _collect_adg_activity_edges(graph.adg)
            topo = _topo_sort([_act_id(k) for k in acts.keys()], [(_act_id(u), _act_id(v)) for (u, v) in edges])
            print(f"\nactivity topo: count={len(topo.order)}, cycle={topo.has_cycle}")
            if topo.order:
                preview = topo.order[: args.max_print]
                print("  head:", " ".join(preview))
    else:
        print(f"error: unknown graph type '{which}'", file=sys.stderr)
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
