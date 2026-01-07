from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Set, Tuple, Union
from xml.etree import ElementTree


@dataclass(frozen=True)
class SrdfGroup:
    joints: Tuple[str, ...]
    subgroups: Tuple[str, ...]


@dataclass(frozen=True)
class SrdfModel:
    groups: Dict[str, SrdfGroup]
    group_states: Dict[Tuple[str, str], Tuple[Tuple[str, float], ...]]  # (group, state) -> ordered joints


def _as_path(path: Union[str, Path]) -> Path:
    return path if isinstance(path, Path) else Path(path)


def load_srdf(srdf_path: Union[str, Path]) -> SrdfModel:
    srdf_path = _as_path(srdf_path)
    root = ElementTree.parse(str(srdf_path)).getroot()

    groups: Dict[str, SrdfGroup] = {}
    for group in root.findall("group"):
        name = group.get("name")
        if not name:
            continue
        joints: List[str] = []
        subgroups: List[str] = []
        for child in list(group):
            if child.tag == "joint":
                jname = child.get("name")
                if jname:
                    joints.append(jname)
            elif child.tag == "group":
                gname = child.get("name")
                if gname:
                    subgroups.append(gname)
        groups[name] = SrdfGroup(joints=tuple(joints), subgroups=tuple(subgroups))

    group_states: Dict[Tuple[str, str], Tuple[Tuple[str, float], ...]] = {}
    for state in root.findall("group_state"):
        state_name = state.get("name")
        group_name = state.get("group")
        if not state_name or not group_name:
            continue
        ordered: List[Tuple[str, float]] = []
        for j in state.findall("joint"):
            jname = j.get("name")
            v = j.get("value")
            if not jname or v is None:
                continue
            ordered.append((jname, float(v)))
        group_states[(group_name, state_name)] = tuple(ordered)

    return SrdfModel(groups=groups, group_states=group_states)


def resolve_group_joints(model: SrdfModel, group_name: str) -> Tuple[str, ...]:
    resolved: List[str] = []
    stack: List[str] = [group_name]
    visiting: Set[str] = set()

    def visit(name: str) -> None:
        if name in visiting:
            raise ValueError(f"SRDF group recursion detected: {name}")
        visiting.add(name)
        group = model.groups.get(name)
        if not group:
            raise KeyError(f"SRDF group not found: {name}")
        resolved.extend(group.joints)
        for sub in group.subgroups:
            visit(sub)
        visiting.remove(name)

    while stack:
        visit(stack.pop())

    return tuple(resolved)


def group_state(model: SrdfModel, group_name: str, state_name: str) -> Tuple[Tuple[str, float], ...]:
    try:
        return model.group_states[(group_name, state_name)]
    except KeyError as exc:
        available = sorted({k[1] for k in model.group_states.keys() if k[0] == group_name})
        raise KeyError(
            f"SRDF group_state not found: group={group_name!r} state={state_name!r} "
            f"(available: {available})"
        ) from exc


def pose_matrix_from_group_state(
    model: SrdfModel,
    *,
    move_group: str,
    pose_name: str,
    robot_groups: Sequence[str],
) -> List[List[float]]:
    joints_and_values = list(group_state(model, move_group, pose_name))
    if not joints_and_values:
        raise ValueError(f"Empty group_state for group={move_group!r}, pose={pose_name!r}")

    joint_map: Dict[str, float] = {j: float(v) for j, v in joints_and_values}

    out: List[List[float]] = []
    for g in robot_groups:
        ordered_joints = resolve_group_joints(model, g)
        values = [joint_map[j] for j in ordered_joints if j in joint_map]
        if not values:
            raise ValueError(
                f"Pose {pose_name!r} for move_group {move_group!r} had no joints for robot_group {g!r}"
            )
        out.append(values)
    return out


def pose_matrix_from_named_pose(
    srdf_path: Union[str, Path],
    *,
    move_group: str,
    robot_groups: Sequence[str],
    pose_name: str,
) -> List[List[float]]:
    model = load_srdf(srdf_path)

    if (move_group, pose_name) in model.group_states:
        return pose_matrix_from_group_state(model, move_group=move_group, pose_name=pose_name, robot_groups=robot_groups)

    # Fallback: allow per-robot group_state definitions (same name across robot groups).
    out: List[List[float]] = []
    missing: List[str] = []
    for g in robot_groups:
        key = (g, pose_name)
        if key not in model.group_states:
            missing.append(g)
            continue
        joint_map: Dict[str, float] = {j: float(v) for j, v in model.group_states[key]}
        ordered_joints = resolve_group_joints(model, g)
        values = [joint_map[j] for j in ordered_joints if j in joint_map]
        if not values:
            raise ValueError(f"Pose {pose_name!r} for robot_group {g!r} had no joints")
        out.append(values)
    if missing:
        raise KeyError(
            f"Pose {pose_name!r} not defined for move_group {move_group!r} "
            f"and missing per-robot group_state for: {missing}"
        )
    return out
