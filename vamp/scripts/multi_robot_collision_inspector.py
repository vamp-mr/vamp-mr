#!/usr/bin/env python3
"""Interactive multi-robot collision inspector using ``vamp.fkcc_multi_all``.

This tool spawns one or more robots in PyBullet, lets you adjust their joint
values live, and repeatedly evaluates ``vamp.fkcc_multi_all`` against the
current configurations.  The collision status is displayed in the PyBullet GUI
so you can verify whether a configuration is collision free while you tweak it.

Robot specifications are supplied with ``--robot``.  Each spec accepts an
optional base-frame translation and Euler XYZ rotation using ``@`` separators:

    panda                      # Panda at the origin (identity base transform)
    panda@0.75,0,0             # Panda translated +0.75 m along X
    gp4@1,0,0@0,0,1.5708       # GP4 translated and rotated 90 degrees about Z

You can pass ``--robot`` multiple times to mix different robots or duplicate
the same one at different base transforms.

Each robot can be assigned a friendly label by prefixing the spec with
``label=``—for example ``left=panda`` or ``right=gp4@0.75,0,0``.  Labels show up
in the PyBullet sliders and are accepted by ``--state`` overrides so you can
copy/paste joint dumps directly from logs:

    ./multi_robot_collision_inspector.py \
        --robot left=panda \
        --robot right=panda@0.75,0,0 \
        --state "left=-1.63 1.93 -0.03 -2.55 0.55 -2.97 0" \
        --state "right=1.45 -1.77 -0.86 2.64 -1.62 2.01 0"
"""

from __future__ import annotations

import argparse
import math
import random
import re
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np
import pybullet as pb
from pybullet_utils.bullet_client import BulletClient

import vamp
from vamp.transformations import euler_matrix, quaternion_from_matrix


@dataclass
class RobotSpec:
    name: str
    label: str
    transform: np.ndarray  # 4x4 float32


@dataclass
class RobotRuntime:
    spec: RobotSpec
    module: object
    body_id: int
    joint_names: Sequence[str]
    joint_indices: Sequence[int]
    link_names: Dict[int, str]
    slider_ids: Sequence[int]
    configuration: np.ndarray  # working buffer
    joint_limits: Sequence[Tuple[float, float]]
    attachment: Optional[vamp.Attachment] = None
    attachment_body: Optional[int] = None
    attachment_transform: Optional[np.ndarray] = None  # 4x4 relative TF
    attachment_color: Optional[Sequence[float]] = None


@dataclass
class ObstacleSpec:
    kind: str
    position: np.ndarray
    size: np.ndarray
    color: Optional[Sequence[float]] = None
    name: Optional[str] = None


@dataclass
class AttachmentSpec:
    kind: str
    translation: Sequence[float]
    rotation_rpy: Sequence[float]
    length: float
    radius: float
    color: Optional[Sequence[float]] = None
    sphere_radius: Optional[float] = None
    min_spheres: int = 0

    def relative_transform(self) -> np.ndarray:
        tf = np.eye(4, dtype=np.float32)
        tf[:3, :3] = euler_matrix(*self.rotation_rpy)[:3, :3].astype(np.float32)
        tf[:3, 3] = np.asarray(self.translation, dtype=np.float32)
        return tf

    def build_attachment(self) -> vamp.Attachment:
        relative = self.relative_transform()
        attachment = vamp.Attachment(relative)
        if self.kind == "cylinder":
            cylinder = vamp.Cylinder([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], float(self.radius), float(self.length))
            attachment.add_cylinder(cylinder, self.sphere_radius or -1.0, self.min_spheres)
        elif self.kind == "cuboid":
            half_extents = [float(self.radius), float(self.radius), float(self.length) / 2.0]
            cuboid = vamp.Cuboid([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], half_extents)
            attachment.add_cuboid(cuboid, self.sphere_radius or -1.0, self.min_spheres)
        else:
            raise ValueError(f"Unsupported attachment kind '{self.kind}'")
        return attachment


def _hollow_box_obstacles(
    center: Sequence[float],
    size: Sequence[float],
    color: Optional[Sequence[float]] = None,
    thickness: float = 0.02,
) -> List[ObstacleSpec]:
    cx, cy, cz = (float(v) for v in center)
    length, width, height = (float(v) for v in size)
    color_arr = list(color) if color is not None else None

    obstacles: List[ObstacleSpec] = []

    bottom_size = np.asarray([length, width, thickness], dtype=np.float32)
    bottom_pos = np.asarray([cx, cy, cz], dtype=np.float32)
    obstacles.append(ObstacleSpec("cube", bottom_pos, bottom_size, color_arr))

    side_z = cz + thickness * 0.5 + height * 0.5
    front_size = np.asarray([length, thickness, height], dtype=np.float32)
    front_pos = np.asarray([cx, cy + width / 2.0, side_z], dtype=np.float32)
    back_pos = np.asarray([cx, cy - width / 2.0, side_z], dtype=np.float32)
    obstacles.append(ObstacleSpec("cube", front_pos, front_size, color_arr))
    obstacles.append(ObstacleSpec("cube", back_pos, front_size, color_arr))

    side_size = np.asarray([thickness, width, height], dtype=np.float32)
    left_pos = np.asarray([cx - length / 2.0, cy, side_z], dtype=np.float32)
    right_pos = np.asarray([cx + length / 2.0, cy, side_z], dtype=np.float32)
    obstacles.append(ObstacleSpec("cube", left_pos, side_size, color_arr))
    obstacles.append(ObstacleSpec("cube", right_pos, side_size, color_arr))

    return obstacles

@dataclass
class PresetConfig:
    robot_specs: List[str]
    attachments: Dict[str, AttachmentSpec]
    obstacles: List[ObstacleSpec]

PRESETS: Dict[str, PresetConfig] = {
    "dual_gp4": PresetConfig(
        robot_specs=[
            "left_arm=gp4@0,0.0,0.0@0.0,0.0,0.0",
            "right_arm=gp4@0.88128092,-0.01226491,0.0@0.0,0.0,3.13792336",
        ],
        attachments={},
        obstacles=[],
    ), 
    "panda_two_rod": PresetConfig(
        robot_specs=[
            "panda0=panda@0.4,0.0,0.1@0.0,0.0,-3.141592653589793",
            "panda1=panda@-0.4,0.0,0.1@0.0,0.0,0.0",
        ],
        attachments={
            "panda0": AttachmentSpec(
                kind="cylinder",
                translation=[0.0, 0.0, -0.005],
                rotation_rpy=[0.0, 1.5707963267948966, 0.0],
                length=0.5 * 1.01,
                radius=0.015,
                color=[0.0, 1.0, 0.0, 0.8],
                min_spheres=5,
            ),
            "panda1": AttachmentSpec(
                kind="cylinder",
                translation=[0.0, 0.0, -0.005],
                rotation_rpy=[0.0, 1.5707963267948966, 0.0],
                length=0.5 * 1.01,
                radius=0.015,
                color=[0.0, 0.0, 1.0, 0.8],
                min_spheres=5,
            ),
        },
        obstacles=[
            ObstacleSpec(
                kind="cube",
                position=np.asarray([0.0, 0.0, 0.05], dtype=np.float32),
                size=np.asarray([0.6, 2.0, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor1",
            ),
            ObstacleSpec(
                kind="cube",
                position=np.asarray([0.75, 0.0, 0.05], dtype=np.float32),
                size=np.asarray([0.5, 2.0, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor2",
            ),
            ObstacleSpec(
                kind="cube",
                position=np.asarray([-0.75, 0.0, 0.05], dtype=np.float32),
                size=np.asarray([0.5, 2.0, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor3",
            ),
            ObstacleSpec(
                kind="cube",
                position=np.asarray([0.4, 0.55, 0.05], dtype=np.float32),
                size=np.asarray([0.2, 0.9, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor4",
            ),
            ObstacleSpec(
                kind="cube",
                position=np.asarray([0.4, -0.55, 0.05], dtype=np.float32),
                size=np.asarray([0.2, 0.9, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor5",
            ),
            ObstacleSpec(
                kind="cube",
                position=np.asarray([-0.4, 0.55, 0.05], dtype=np.float32),
                size=np.asarray([0.2, 0.9, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor6",
            ),
            ObstacleSpec(
                kind="cube",
                position=np.asarray([-0.4, -0.55, 0.05], dtype=np.float32),
                size=np.asarray([0.2, 0.9, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor7",
            ),
        ],
    ),
    "panda_two": PresetConfig(
        robot_specs=[
            "panda0=panda@0.4,0.0,0.1@0.0,0.0,-3.141592653589793",
            "panda1=panda@-0.4,0.0,0.1@0.0,0.0,0.0",
        ],
        attachments={},
        obstacles=[
            ObstacleSpec(
                kind="cube",
                position=np.asarray([0.0, 0.0, 0.05], dtype=np.float32),
                size=np.asarray([0.6, 2.0, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor1",
            ),
            ObstacleSpec(
                kind="cube",
                position=np.asarray([0.75, 0.0, 0.05], dtype=np.float32),
                size=np.asarray([0.5, 2.0, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor2",
            ),
            ObstacleSpec(
                kind="cube",
                position=np.asarray([-0.75, 0.0, 0.05], dtype=np.float32),
                size=np.asarray([0.5, 2.0, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor3",
            ),
            ObstacleSpec(
                kind="cube",
                position=np.asarray([0.4, 0.55, 0.05], dtype=np.float32),
                size=np.asarray([0.2, 0.9, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor4",
            ),
            ObstacleSpec(
                kind="cube",
                position=np.asarray([0.4, -0.55, 0.05], dtype=np.float32),
                size=np.asarray([0.2, 0.9, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor5",
            ),
            ObstacleSpec(
                kind="cube",
                position=np.asarray([-0.4, 0.55, 0.05], dtype=np.float32),
                size=np.asarray([0.2, 0.9, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor6",
            ),
            ObstacleSpec(
                kind="cube",
                position=np.asarray([-0.4, -0.55, 0.05], dtype=np.float32),
                size=np.asarray([0.2, 0.9, 0.1], dtype=np.float32),
                color=[0.7, 0.7, 0.7, 0.2],
                name="floor7",
            ),
        ],
    ),
    "panda_four": PresetConfig(
        robot_specs=[
            "panda0=panda@0.4,-0.4,0.1@0.0,0.0,2.28",
            "panda1=panda@-0.4,-0.4,0.1@0.0,0.0,0.66",
            "panda2=panda@-0.4,0.4,0.1@0.0,0.0,-0.66",
            "panda3=panda@0.4,0.4,0.1@0.0,0.0,-2.28",
        ],
        attachments={},
        obstacles=[],
    ),
    "panda_four_binpick": PresetConfig(
        robot_specs=[
            "panda0=panda@0.4,-0.4,0.1@0.0,0.0,2.28",
            "panda1=panda@-0.4,-0.4,0.1@0.0,0.0,0.66",
            "panda2=panda@-0.4,0.4,0.1@0.0,0.0,-0.66",
            "panda3=panda@0.4,0.4,0.1@0.0,0.0,-2.28",
        ],
        attachments={
            "panda0": AttachmentSpec(
                kind="cylinder",
                translation=[0.0, 0.0, -0.005],
                rotation_rpy=[0.0, 1.5707963267948966, 0.0],
                length=0.3,
                radius=0.01,
                color=[0.0, 1.0, 0.0, 0.8],
                min_spheres=3,
            ),
            "panda1": AttachmentSpec(
                kind="cylinder",
                translation=[0.0, 0.0, -0.005],
                rotation_rpy=[0.0, 1.5707963267948966, 0.0],
                length=0.3,
                radius=0.01,
                color=[1.0, 0.0, 0.0, 0.8],
                min_spheres=3,
            ),
            "panda2": AttachmentSpec(
                kind="cylinder",
                translation=[0.0, 0.0, -0.005],
                rotation_rpy=[0.0, 1.5707963267948966, 0.0],
                length=0.3,
                radius=0.01,
                color=[0.0, 0.0, 1.0, 0.8],
                min_spheres=3,
            ),
            "panda3": AttachmentSpec(
                kind="cylinder",
                translation=[0.0, 0.0, -0.005],
                rotation_rpy=[0.0, 1.5707963267948966, 0.0],
                length=0.3,
                radius=0.01,
                color=[1.0, 1.0, 0.0, 0.8],
                min_spheres=3,
            ),
        },
        obstacles=[
            *_hollow_box_obstacles(
                center=[0.0, 0.0, 0.0],
                size=[0.5, 0.5, 0.5],
                color=[0.0, 0.0, 1.0, 0.4],
            ),
            *_hollow_box_obstacles(
                center=[0.5, 0.0, 0.0],
                size=[0.5, 0.5, 0.5],
                color=[0.0, 1.0, 0.0, 0.4],
            ),
            *_hollow_box_obstacles(
                center=[-0.5, 0.0, 0.0],
                size=[0.5, 0.5, 0.5],
                color=[1.0, 0.0, 0.0, 0.4],
            ),
            *_hollow_box_obstacles(
                center=[0.0, 0.5, 0.0],
                size=[0.5, 0.5, 0.5],
                color=[1.0, 1.0, 0.0, 0.4],
            ),
            *_hollow_box_obstacles(
                center=[0.0, -0.5, 0.0],
                size=[0.5, 0.5, 0.5],
                color=[0.5, 0.0, 0.5, 0.4],
            ),
        ],
    ),
}

def _create_attachment_body(
    client: BulletClient,
    spec: AttachmentSpec,
    attachment: vamp.Attachment,
) -> int:
    identity = np.eye(4, dtype=np.float32)
    attachment.set_ee_pose(identity)

    rgba = list(spec.color) if spec.color is not None else [0.2, 0.6, 0.8, 0.6]

    if spec.kind == "cylinder":
        collision = client.createCollisionShape(
            pb.GEOM_CYLINDER,
            radius=float(spec.radius),
            height=float(spec.length),
        )
        visual = client.createVisualShape(
            pb.GEOM_CYLINDER,
            radius=float(spec.radius),
            length=float(spec.length),
            rgbaColor=rgba,
        )
    elif spec.kind == "cuboid":
        half_extents = [float(spec.radius), float(spec.radius), float(spec.length) * 0.5]
        collision = client.createCollisionShape(pb.GEOM_BOX, halfExtents=half_extents)
        visual = client.createVisualShape(
            pb.GEOM_BOX,
            halfExtents=half_extents,
            rgbaColor=rgba,
        )
    else:
        raise ValueError(f"Unsupported attachment kind '{spec.kind}'")

    body_id = client.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[0.0, 0.0, 0.0],
        baseOrientation=[0.0, 0.0, 0.0, 1.0],
    )
    return body_id


def _update_attachment_pose(
    client: BulletClient,
    runtime: RobotRuntime,
    ee_transform: np.ndarray,
) -> None:
    if runtime.attachment is None or runtime.attachment_transform is None:
        return

    base_tf = runtime.spec.transform
    world_ee_tf = (base_tf @ ee_transform).astype(np.float32, copy=False)
    runtime.attachment.set_ee_pose(world_ee_tf)
    world_tf = world_ee_tf @ runtime.attachment_transform
    position = world_tf[:3, 3]
    quat_xyzw = quaternion_from_matrix(world_tf)
    client.resetBasePositionAndOrientation(
        runtime.attachment_body,
        position.tolist(),
        (float(quat_xyzw[0]), float(quat_xyzw[1]), float(quat_xyzw[2]), float(quat_xyzw[3])),
    )



def _build_state(runtime: RobotRuntime, client: BulletClient) -> Dict[str, object]:
    state = {
        "robot": runtime.spec.name,
        "configuration": runtime.configuration.tolist(),
        "transform": runtime.spec.transform.astype(np.float32),
        "label": runtime.spec.label,
    }

    if runtime.attachment is not None and runtime.attachment_transform is not None:
        ee_tf = runtime.module.eefk(runtime.configuration.tolist())
        _update_attachment_pose(client, runtime, ee_tf)
        state["attachment"] = runtime.attachment

    return state


def _parse_float_list(text: str) -> List[float]:
    tokens = [token for token in re.split(r"[\s,]+", text.strip()) if token]
    if not tokens:
        raise ValueError("No numeric values provided")
    return [float(token) for token in tokens]


def _parse_state_override(raw: str) -> Tuple[str, List[float]]:
    if "=" not in raw:
        raise ValueError(f"State override '{raw}' must use the form label=values")
    key, values = raw.split("=", 1)
    key = key.strip()
    if not key:
        raise ValueError(f"State override '{raw}' is missing a label")
    return key, _parse_float_list(values)


def _normalize_state_key(key: str) -> str:
    key = key.strip()
    if key.startswith("#"):
        key = key[1:]
    return key.lower()


def _normalize_link_name(name: str) -> str:
    return name.replace("::", "/").strip().lower()


def _normalize_link_pairs(raw: Dict[str, Set[Tuple[str, str]]]) -> Dict[str, Set[Tuple[str, str]]]:
    normalized: Dict[str, Set[Tuple[str, str]]] = {}
    for robot, pairs in raw.items():
        norm_pairs: Set[Tuple[str, str]] = set()
        for first, second in pairs:
            norm_pairs.add(tuple(sorted((_normalize_link_name(first), _normalize_link_name(second)))))
        normalized[robot] = norm_pairs
    return normalized


IGNORED_LINK_COLLISIONS: Dict[str, Set[Tuple[str, str]]] = _normalize_link_pairs(
    {
        "gp4": {
            ("fts", "link_4"),
            ("fts", "link_5"),
            ("fts", "link_6"),
            ("link_4", "link_6"),
        },
        "panda": [
            ("panda_link7", "panda_hand"),
        ]
    }
)


ATTACHMENT_COLLISION_EXCLUDES: Dict[str, Set[str]] = {
    "panda": {
        "panda_grasptarget",
        "panda_hand",
        "panda_leftfinger",
        "panda_rightfinger",
    }
}


def _should_ignore_link_collision(robot_name: str, link_a: str, link_b: str) -> bool:
    normalized_pairs = IGNORED_LINK_COLLISIONS.get(robot_name)
    if not normalized_pairs:
        return False
    normalized = tuple(sorted((_normalize_link_name(link_a), _normalize_link_name(link_b))))
    return normalized in normalized_pairs


def _sanitize_configuration(values: Sequence[float], dimension: int, label: str) -> np.ndarray:
    sanitized = list(values)
    if len(sanitized) < dimension:
        missing = dimension - len(sanitized)
        print(
            f"[multi_robot_collision_inspector] '{label}' provided {len(sanitized)} joint values; padding "
            f"{missing} trailing value(s) with 0.0",
            file=sys.stderr,
        )
        sanitized.extend([0.0] * missing)
    elif len(sanitized) > dimension:
        print(
            f"[multi_robot_collision_inspector] '{label}' provided {len(sanitized)} joint values; truncating "
            f"to the first {dimension}",
            file=sys.stderr,
        )
        sanitized = sanitized[:dimension]
    return np.asarray(sanitized, dtype=np.float32)


def _parse_obstacle(raw: str) -> ObstacleSpec:
    parts = raw.split("@")
    if not parts:
        raise ValueError("Obstacle specification cannot be empty")

    kind = parts[0].strip().lower()
    if kind != "cube":
        raise ValueError(f"Unsupported obstacle kind '{parts[0]}' (only 'cube' is supported)")

    if len(parts) != 3:
        raise ValueError("Cube obstacle must be specified as cube@x,y,z@lx,ly,lz")

    try:
        position = np.asarray([float(v) for v in parts[1].split(",")], dtype=np.float32)
        size = np.asarray([float(v) for v in parts[2].split(",")], dtype=np.float32)
    except ValueError as exc:
        raise ValueError(f"Failed to parse obstacle floats: {exc}") from exc

    if position.shape != (3,) or size.shape != (3,):
        raise ValueError("Cube obstacle position and size must each provide exactly three floats")

    if np.any(size <= 0.0):
        raise ValueError("Cube obstacle sizes must be strictly positive")

    return ObstacleSpec(kind="cube", position=position, size=size)


def _build_initial_overrides(specs: Sequence[RobotSpec], raw_states: Optional[Sequence[str]]) -> Dict[int, np.ndarray]:
    if not raw_states:
        return {}

    parsed: Dict[str, List[float]] = {}
    for entry in raw_states:
        key, values = _parse_state_override(entry)
        parsed[_normalize_state_key(key)] = values

    overrides: Dict[int, np.ndarray] = {}
    used_keys: Set[str] = set()
    for index, spec in enumerate(specs):
        module = getattr(vamp, spec.name)
        dimension = int(module.dimension()) if hasattr(module, "dimension") else len(module.joint_names())

        candidates = {
            _normalize_state_key(spec.label),
            _normalize_state_key(spec.name),
            _normalize_state_key(str(index)),
            _normalize_state_key(f"{spec.label}_{index}"),
            _normalize_state_key(f"{spec.name}_{index}"),
        }

        matched_values: Optional[List[float]] = None
        matched_key: Optional[str] = None
        for candidate in candidates:
            if candidate in parsed:
                matched_values = parsed[candidate]
                matched_key = candidate
                break

        if matched_values is None:
            continue

        overrides[index] = _sanitize_configuration(matched_values, dimension, spec.label)
        if matched_key:
            used_keys.add(matched_key)

    unused = set(parsed) - used_keys
    if unused:
        print(
            "[multi_robot_collision_inspector] No robot matched state override(s): "
            + ", ".join(sorted(unused)),
            file=sys.stderr,
        )

    return overrides


def _compute_vamp_contacts(
    runtimes: Sequence[RobotRuntime],
    env: vamp.Environment,
    states: Sequence[Dict[str, object]],
) -> Tuple[Set[str], List[str]]:
    summaries: Set[str] = set()
    lines: List[str] = []

    contact_report = dict(vamp.compute_contacts(states, env))
    per_state_raw = list(contact_report.get("states", []))

    for runtime, state_report_raw in zip(runtimes, per_state_raw):
        state_report = dict(state_report_raw)
        label = runtime.spec.label

        env_hits = state_report.get("environment", [])
        for sphere_index, hits in enumerate(env_hits):
            if not hits:
                continue
            unique_hits = sorted({str(name) for name in hits})
            key = f"env|{label}|{sphere_index}|{','.join(unique_hits)}"
            if key in summaries:
                continue
            summaries.add(key)
            lines.append(
                f"{label}: sphere {sphere_index} vs environment ({', '.join(unique_hits)})"
            )

        for self_entry_raw in state_report.get("self", []):
            self_entry = dict(self_entry_raw)
            idx_a = int(self_entry["a"])
            idx_b = int(self_entry["b"])
            if idx_a > idx_b:
                idx_a, idx_b = idx_b, idx_a
            key = f"self|{label}|{idx_a}|{idx_b}"
            if key in summaries:
                continue
            summaries.add(key)
            penetration = float(self_entry["penetration"])
            lines.append(
                f"{label}: sphere {idx_a} <> sphere {idx_b} (penetration {penetration:.4f} m)"
            )

        attachments_env = state_report.get("attachments_environment", [])
        for attachment_index, hits in enumerate(attachments_env):
            if not hits:
                continue
            unique_hits = sorted({str(name) for name in hits})
            key = f"attach_env|{label}|{attachment_index}|{','.join(unique_hits)}"
            if key in summaries:
                continue
            summaries.add(key)
            lines.append(
                f"{label}: attachment {attachment_index} vs environment ({', '.join(unique_hits)})"
            )

        for entry_raw in state_report.get("attachment_self", []):
            entry = dict(entry_raw)
            attachment_index = int(entry["attachment_index"])
            sphere_index = int(entry["sphere_index"])
            key = f"attach_self|{label}|{attachment_index}|{sphere_index}"
            if key in summaries:
                continue
            summaries.add(key)
            penetration = float(entry["penetration"])
            lines.append(
                f"{label}: attachment {attachment_index} <> sphere {sphere_index} "
                f"(penetration {penetration:.4f} m)"
            )

    cross_sphere = list(contact_report.get("cross_sphere", []))
    for entry_raw in cross_sphere:
        entry = dict(entry_raw)
        first_state = int(entry["first_state"])
        second_state = int(entry["second_state"])
        label_a = runtimes[first_state].spec.label
        label_b = runtimes[second_state].spec.label
        sphere_a = int(entry["first_sphere"])
        sphere_b = int(entry["second_sphere"])
        key = f"cross|{label_a}|{sphere_a}|{label_b}|{sphere_b}"
        if key in summaries:
            continue
        summaries.add(key)
        penetration = float(entry["penetration"])
        lines.append(
            f"{label_a} sphere {sphere_a} <> {label_b} sphere {sphere_b} "
            f"(penetration {penetration:.4f} m)"
        )

    cross_attach_sphere = list(contact_report.get("cross_attachment_sphere", []))
    for entry_raw in cross_attach_sphere:
        entry = dict(entry_raw)
        attachment_state = int(entry["attachment_state"])
        sphere_state = int(entry["sphere_state"])
        attachment_idx = int(entry["attachment_index"])
        sphere_idx = int(entry["sphere_index"])
        label_a = runtimes[attachment_state].spec.label
        label_b = runtimes[sphere_state].spec.label
        key = f"attach_cross|{label_a}|{attachment_idx}|{label_b}|{sphere_idx}"
        if key in summaries:
            continue
        summaries.add(key)
        penetration = float(entry["penetration"])
        lines.append(
            f"{label_a} attachment {attachment_idx} <> {label_b} sphere {sphere_idx} "
            f"(penetration {penetration:.4f} m)"
        )

    cross_attach_attach = list(contact_report.get("cross_attachment_attachment", []))
    for entry_raw in cross_attach_attach:
        entry = dict(entry_raw)
        first_state = int(entry["first_state"])
        second_state = int(entry["second_state"])
        attach_a = int(entry["first_attachment"])
        attach_b = int(entry["second_attachment"])
        label_a = runtimes[first_state].spec.label
        label_b = runtimes[second_state].spec.label
        key = f"attach_attach|{label_a}|{attach_a}|{label_b}|{attach_b}"
        if key in summaries:
            continue
        summaries.add(key)
        penetration = float(entry["penetration"])
        lines.append(
            f"{label_a} attachment {attach_a} <> {label_b} attachment {attach_b} "
            f"(penetration {penetration:.4f} m)"
        )

    return summaries, lines


def _collect_pybullet_contacts(
    client: BulletClient,
    runtimes: Sequence[RobotRuntime],
) -> Tuple[Set[str], List[str]]:
    summaries: Set[str] = set()
    lines: List[str] = []

    for runtime in runtimes:
        label = runtime.spec.label
        for contact in client.getContactPoints(bodyA=runtime.body_id, bodyB=runtime.body_id):
            link_a = int(contact[3])
            link_b = int(contact[4])
            if link_a == link_b:
                continue
            name_a_raw = runtime.link_names.get(link_a, f"link{link_a}")
            name_b_raw = runtime.link_names.get(link_b, f"link{link_b}")
            if _should_ignore_link_collision(runtime.spec.name, name_a_raw, name_b_raw):
                continue
            ordered = tuple(sorted((link_a, link_b)))
            key = f"self|{label}|{ordered[0]}|{ordered[1]}"
            if key in summaries:
                continue
            summaries.add(key)
            name_a = runtime.link_names.get(ordered[0], f"link{ordered[0]}")
            name_b = runtime.link_names.get(ordered[1], f"link{ordered[1]}")
            distance = float(contact[8])
            lines.append(f"{label}: {name_a} <> {name_b} (distance {distance:.4f} m)")

    for first in range(len(runtimes)):
        for second in range(first + 1, len(runtimes)):
            runtime_a = runtimes[first]
            runtime_b = runtimes[second]
            label_a = runtime_a.spec.label
            label_b = runtime_b.spec.label
            for contact in client.getContactPoints(bodyA=runtime_a.body_id, bodyB=runtime_b.body_id):
                link_a = int(contact[3])
                link_b = int(contact[4])
                key = f"cross|{label_a}|{link_a}|{label_b}|{link_b}"
                if key in summaries:
                    continue
                summaries.add(key)
                name_a = runtime_a.link_names.get(link_a, f"link{link_a}")
                name_b = runtime_b.link_names.get(link_b, f"link{link_b}")
                distance = float(contact[8])
                lines.append(
                    f"{label_a} {name_a} <> {label_b} {name_b} (distance {distance:.4f} m)"
                )

    body_to_runtime = {runtime.body_id: runtime for runtime in runtimes}
    attachment_to_runtime = {
        runtime.attachment_body: runtime for runtime in runtimes if runtime.attachment_body is not None
    }

    for runtime in runtimes:
        if runtime.attachment_body is None:
            continue

        attachment_label = f"{runtime.spec.label} attachment"
        for contact in client.getContactPoints(bodyA=runtime.attachment_body):
            target_body = int(contact[1])
            if target_body == runtime.attachment_body:
                continue

            distance = float(contact[8])
            link_b = int(contact[4])

            if target_body in body_to_runtime:
                other_runtime = body_to_runtime[target_body]
                key = f"attach|{runtime.spec.label}|{other_runtime.spec.label}|{link_b}"
                if key in summaries:
                    continue
                summaries.add(key)
                name_b = other_runtime.link_names.get(link_b, f"link{link_b}")
                lines.append(
                    f"{attachment_label} <> {other_runtime.spec.label} {name_b} (distance {distance:.4f} m)"
                )
            elif target_body in attachment_to_runtime:
                other_runtime = attachment_to_runtime[target_body]
                if other_runtime is runtime:
                    continue
                key = f"attach_attach|{runtime.spec.label}|{other_runtime.spec.label}"
                if key in summaries:
                    continue
                summaries.add(key)
                lines.append(
                    f"{attachment_label} <> {other_runtime.spec.label} attachment (distance {distance:.4f} m)"
                )
            else:
                continue

    return summaries, lines


def _randomize_and_compare(
    client: BulletClient,
    runtimes: Sequence[RobotRuntime],
    env: vamp.Environment,
    iterations: int,
    seed: Optional[int],
    base_configurations: Sequence[np.ndarray],
) -> None:
    if iterations <= 0 or not runtimes:
        return

    rng = random.Random(seed)
    print(
        f"[random] Evaluating {iterations} random sample(s) (seed={seed if seed is not None else 'auto'})"
    )

    mismatches = 0
    errors = 0

    for sample_idx in range(iterations):
        states = []
        for runtime in runtimes:
            draws: List[float] = []
            for lower, upper in runtime.joint_limits:
                lo = lower if math.isfinite(lower) else -math.pi
                hi = upper if math.isfinite(upper) else math.pi
                if not (lo < hi):
                    lo, hi = -math.pi, math.pi
                draws.append(rng.uniform(lo, hi))

            runtime.configuration[:] = draws
            for joint_index, value in zip(runtime.joint_indices, draws):
                client.resetJointState(runtime.body_id, joint_index, value)

            state = _build_state(runtime, client)
            states.append(state)

        try:
            vamp_result = vamp.fkcc_multi_all(states, env)
        except ValueError as exc:
            errors += 1
            vamp_result = False
            print(
                f"[random {sample_idx + 1}/{iterations}] fkcc_multi_all error: {exc}"
            )

        client.performCollisionDetection()
        pb_summary, pb_lines = _collect_pybullet_contacts(client, runtimes)
        vamp_summary, vamp_lines = _compute_vamp_contacts(runtimes, env, states)

        pb_collision = len(pb_summary) > 0
        summary = (
            f"[random {sample_idx + 1}/{iterations}] VAMP={'collision' if not vamp_result else 'free'} | "
            f"PyBullet={'collision' if pb_collision else 'free'}"
        )

        if (not vamp_result) != pb_collision:
            mismatches += 1
            print(summary + " -> mismatch")
            if pb_lines:
                print("  PyBullet contacts:")
                for line in pb_lines:
                    print(f"    {line}")
            else:
                print("  PyBullet contacts: none")

            if vamp_lines:
                print("  VAMP collisions:")
                for line in vamp_lines:
                    print(f"    {line}")
            else:
                print("  VAMP collisions: none")

            print("  Joint samples:")
            for runtime in runtimes:
                formatted = ", ".join(f"{value:.4f}" for value in runtime.configuration)
                print(f"    {runtime.spec.label}: [{formatted}]")
            print("---")
        else:
            print(summary)

    print(
        f"[random] Completed {iterations} sample(s): {mismatches} mismatch(es), {errors} error(s)."
    )

    for runtime, baseline in zip(runtimes, base_configurations):
        runtime.configuration[:] = baseline
        for joint_index, value in zip(runtime.joint_indices, baseline):
            client.resetJointState(runtime.body_id, joint_index, float(value))
        if runtime.attachment is not None and runtime.attachment_transform is not None:
            ee_tf = runtime.module.eefk(runtime.configuration.tolist())
            _update_attachment_pose(client, runtime, ee_tf)

def _place_obstacles(
    client: BulletClient,
    env: vamp.Environment,
    obstacles: Sequence[ObstacleSpec],
) -> List[int]:
    body_ids: List[int] = []
    for obstacle in obstacles:
        if obstacle.kind != "cube":
            continue

        half_extents = (obstacle.size * 0.5).astype(np.float32)
        collision = client.createCollisionShape(
            pb.GEOM_BOX, halfExtents=half_extents.tolist()
        )
        rgba = list(obstacle.color) if obstacle.color is not None else [0.7, 0.7, 0.7, 0.4]
        visual = client.createVisualShape(
            pb.GEOM_BOX,
            halfExtents=half_extents.tolist(),
            rgbaColor=rgba,
        )
        body_id = client.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=obstacle.position.tolist(),
        )
        body_ids.append(body_id)

        cube = vamp.Cuboid(
                [float(obstacle.position[0]),
                float(obstacle.position[1]),
                float(obstacle.position[2])],
                [0.0, 0.0, 0.0], # euler xyz
                [float(half_extents[0]),
                float(half_extents[1]),
                float(half_extents[2])],
            )
        if obstacle.name:
            cube.name = obstacle.name
        env.add_cuboid(cube)

    return body_ids


def _unique_paths(paths: Iterable[Path]) -> List[str]:
    seen: List[str] = []
    for path in paths:
        try:
            resolved = str(path.resolve())
        except FileNotFoundError:
            continue
        if resolved not in seen:
            seen.append(resolved)
    return seen


def _default_urdf(name: str) -> Path:
    repo_root = Path(__file__).resolve().parent.parent
    candidates = [
        repo_root / "resources" / name / f"{name}_spherized.urdf",
        repo_root / "resources" / name / f"{name}_spherized_visual.urdf",
        repo_root / "resources" / name / f"{name}.urdf",
        repo_root.parent / "foam" / "assets" / f"{name}_lego" / f"{name}_spherized.urdf",
        repo_root.parent / "foam" / "assets" / f"{name}_lego" / f"{name}_spherized_visual.urdf",
        repo_root.parent / "foam" / "assets" / name / f"{name}_spherized_visual.urdf",
        repo_root.parent / "foam" / "assets" / name / f"{name}.urdf",
    ]

    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()

    raise FileNotFoundError(f"Could not locate a URDF for robot '{name}'. Tried: {', '.join(str(c) for c in candidates)}")


def parse_robot_spec(raw: str) -> RobotSpec:
    segments = raw.split("@")
    head = segments[0]

    label: Optional[str] = None
    name_part = head
    if "=" in head:
        alias, name_token = head.split("=", 1)
        alias = alias.strip()
        name_part = name_token.strip()
        if not name_part:
            raise ValueError(f"Invalid robot spec '{raw}': missing robot name after '='")
        if not alias:
            raise ValueError(f"Invalid robot spec '{raw}': alias cannot be empty")
        label = alias

    name = name_part.strip()
    if not name:
        raise ValueError(f"Invalid robot spec '{raw}': missing name")

    if label is None:
        label = name

    translation = np.zeros(3, dtype=np.float32)
    rotation = np.zeros(3, dtype=np.float32)

    if len(segments) >= 2 and segments[1]:
        values = [float(v) for v in segments[1].split(",") if v.strip()]
        if len(values) != 3:
            raise ValueError(f"Robot spec '{raw}' translation must provide 3 comma-separated floats")
        translation = np.asarray(values, dtype=np.float32)

    if len(segments) >= 3 and segments[2]:
        values = [float(v) for v in segments[2].split(",") if v.strip()]
        if len(values) != 3:
            raise ValueError(f"Robot spec '{raw}' rotation must provide 3 comma-separated floats (radians)")
        rotation = np.asarray(values, dtype=np.float32)

    transform = np.eye(4, dtype=np.float32)
    transform[:3, :3] = euler_matrix(*rotation)[:3, :3].astype(np.float32)
    transform[:3, 3] = translation

    return RobotSpec(name=name, label=label, transform=transform)


def _load_robot(
    client: BulletClient,
    urdf_path: Path,
    spec: RobotSpec,
    search_paths: Sequence[str],
    ignore_visuals: bool,
    initial_configuration: Optional[Sequence[float]] = None,
    attachment_spec: Optional[AttachmentSpec] = None,
) -> RobotRuntime:
    for path in search_paths:
        client.setAdditionalSearchPath(path)

    quat_xyzw = quaternion_from_matrix(spec.transform)
    base_quat = (quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3])
    base_pos = spec.transform[:3, 3].tolist()

    flags = pb.URDF_MAINTAIN_LINK_ORDER | pb.URDF_USE_SELF_COLLISION
    if ignore_visuals:
        flags |= pb.URDF_IGNORE_VISUAL_SHAPES

    body_id = client.loadURDF(
        str(urdf_path),
        basePosition=base_pos,
        baseOrientation=base_quat,
        useFixedBase=True,
        flags=flags,
    )

    joint_map = {}
    link_names: Dict[int, str] = {-1: "base"}
    joint_indices: List[int] = []
    joint_names = getattr(vamp, spec.name).joint_names()
    for joint_index in range(client.getNumJoints(body_id)):
        info = client.getJointInfo(body_id, joint_index)
        joint_map[info[1].decode()] = joint_index
        link_name = info[12].decode() if isinstance(info[12], (bytes, bytearray)) else str(info[12])
        link_names[joint_index] = link_name

    sliders: List[int] = []
    configuration = np.zeros(len(joint_names), dtype=np.float32)
    limits: List[Tuple[float, float]] = []
    for idx, joint_name in enumerate(joint_names):
        bullet_index = joint_map.get(joint_name)
        if bullet_index is None:
            raise RuntimeError(f"Joint '{joint_name}' not found in URDF '{urdf_path}' for robot '{spec.name}'")
        info = client.getJointInfo(body_id, bullet_index)
        lower, upper = info[8], info[9]
        if not (lower < upper) or math.isinf(lower) or math.isinf(upper):
            lower, upper = -math.pi, math.pi
        default_value = 0.0
        if initial_configuration is not None and idx < len(initial_configuration):
            default_value = float(initial_configuration[idx])
        slider_id = client.addUserDebugParameter(
            f"{spec.label}[{idx}] {joint_name}", lower, upper, default_value
        )
        sliders.append(slider_id)
        joint_indices.append(bullet_index)
        configuration[idx] = default_value
        limits.append((float(lower), float(upper)))
        client.resetJointState(body_id, bullet_index, default_value)

    module = getattr(vamp, spec.name)

    runtime = RobotRuntime(
        spec=spec,
        module=module,
        body_id=body_id,
        joint_names=joint_names,
        joint_indices=joint_indices,
        link_names=link_names,
        slider_ids=sliders,
        configuration=configuration,
        joint_limits=limits,
    )

    if attachment_spec is not None:
        attachment = attachment_spec.build_attachment()
        relative_tf = attachment_spec.relative_transform()
        body = _create_attachment_body(client, attachment_spec, attachment)

        runtime.attachment = attachment
        runtime.attachment_transform = relative_tf
        runtime.attachment_body = body
        runtime.attachment_color = attachment_spec.color

        ee_tf = module.eefk(runtime.configuration.tolist())
        _update_attachment_pose(client, runtime, ee_tf)

        disabled_links: Set[int] = set()
        disabled_names = set(ATTACHMENT_COLLISION_EXCLUDES.get(spec.name, set()))

        ee_name: Optional[str] = None
        if hasattr(module, "end_effector"):
            try:
                ee_name = module.end_effector()
            except TypeError:
                ee_name = getattr(module, "end_effector", None)

        if ee_name:
            disabled_names.add(str(ee_name))

        for link_index, link_name in link_names.items():
            if link_name in disabled_names:
                disabled_links.add(link_index)

        num_joints = client.getNumJoints(runtime.body_id)
        for link_index in range(-1, num_joints):
            enable = 0 if link_index in disabled_links else 1
            client.setCollisionFilterPair(runtime.body_id, body, link_index, -1, enableCollision=enable)

    return runtime


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Interactive multi-robot collision inspector")
    parser.add_argument(
        "--robot",
        action="append",
        dest="robots",
        help="Robot spec name[@tx,ty,tz[@roll,pitch,yaw]] (can be repeated).",
    )
    parser.add_argument(
        "--urdf",
        action="append",
        dest="urdfs",
        help="Optional URDF override path for the corresponding --robot entry.",
    )
    parser.add_argument(
        "--state",
        action="append",
        dest="states",
        metavar="LABEL=VALUES",
        help=(
            "Initial joint values for a robot. Use the robot alias, name, or index (0-based), and supply "
            "comma or space separated numbers. Example: --state \"left_arm=-1.63 1.93 -0.03 -2.55 0.55 -2.97 0\""
        ),
    )
    parser.add_argument("--sleep", type=float, default=1.0 / 120.0, help="Loop sleep in seconds (default: 1/120)")
    parser.add_argument("--ignore-visuals", action="store_true", help="Disable URDF visual meshes (avoids missing texture warnings)")
    parser.add_argument("--no-gui", action="store_true", help="Run without the GUI (sliders disabled; not recommended)")
    parser.add_argument("--print", action="store_true", help="Print collision status changes to stdout")
    parser.add_argument(
        "--show-collisions",
        action="store_true",
        help="Log PyBullet and VAMP collision pairs when they change",
    )
    parser.add_argument(
        "--randomize",
        type=int,
        default=0,
        metavar="N",
        help="Generate N random joint samples per run to compare VAMP vs PyBullet before launching the GUI",
    )
    parser.add_argument(
        "--random-seed",
        type=int,
        default=None,
        help="Seed for the random sampler used with --randomize",
    )
    parser.add_argument(
        "--obstacle",
        action="append",
        dest="obstacles",
        metavar="cube@x,y,z@lx,ly,lz",
        help="Add an axis-aligned cuboid obstacle at the given center with edge lengths (meters).",
    )
    parser.add_argument(
        "--preset",
        choices=sorted(PRESETS.keys()),
        help="Load a predefined multi-robot scenario (robots, attachments, obstacles).",
    )
    return parser


def _status_color(collision_free: bool) -> List[float]:
    return [0.0, 1.0, 0.0] if collision_free else [1.0, 0.1, 0.1]


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    specs_raw = list(args.robots) if args.robots else []
    attachment_map: Dict[str, AttachmentSpec] = {}
    obstacle_specs: List[ObstacleSpec] = []

    if args.preset:
        preset = PRESETS[args.preset]
        specs_raw = preset.robot_specs + specs_raw
        attachment_map.update(preset.attachments)
        for obstacle in preset.obstacles:
            obstacle_specs.append(
                ObstacleSpec(
                    kind=obstacle.kind,
                    position=obstacle.position.copy(),
                    size=obstacle.size.copy(),
                    color=list(obstacle.color) if obstacle.color is not None else None,
                )
            )

    if not specs_raw:
        specs_raw = ["panda", "panda@0.75,0,0"]
    specs = [parse_robot_spec(raw) for raw in specs_raw]

    for spec in specs:
        if spec.name not in vamp.robots:
            raise SystemExit(f"Robot '{spec.name}' is not available. Known robots: {', '.join(vamp.robots)}")

    if args.obstacles:
        for raw in args.obstacles:
            try:
                obstacle_specs.append(_parse_obstacle(raw))
            except ValueError as exc:
                raise SystemExit(f"Failed to parse --obstacle '{raw}': {exc}") from exc

    initial_overrides = _build_initial_overrides(specs, args.states)

    urdf_overrides: List[Optional[Path]] = []
    overrides = args.urdfs or []
    if overrides and len(overrides) != len(specs):
        raise SystemExit("If --urdf is provided it must be supplied once per --robot entry.")
    for item in overrides:
        urdf_overrides.append(Path(item))
    while len(urdf_overrides) < len(specs):
        urdf_overrides.append(None)

    if args.no_gui:
        raise SystemExit("--no-gui is not supported for the interactive inspector; omit the flag to launch the GUI.")

    connection_mode = pb.GUI

    client = BulletClient(connection_mode=connection_mode)
    client.configureDebugVisualizer(pb.COV_ENABLE_GUI, 1)
    client.configureDebugVisualizer(pb.COV_ENABLE_MOUSE_PICKING, 0)
    client.resetDebugVisualizerCamera(
        cameraDistance=2.4,
        cameraYaw=135,
        cameraPitch=-35,
        cameraTargetPosition=[0.5, 0.0, 0.6],
    )

    repo_root = Path(__file__).resolve().parent.parent
    search_roots = _unique_paths([
        repo_root,
        repo_root / "resources",
        repo_root.parent / "foam" / "assets",
    ])

    runtimes: List[RobotRuntime] = []
    for index, (spec, override) in enumerate(zip(specs, urdf_overrides)):
        urdf_path = override.resolve() if override else _default_urdf(spec.name)
        per_robot_paths = search_roots + _unique_paths([urdf_path.parent, repo_root / "resources" / spec.name])
        initial_state = initial_overrides.get(index)
        runtime = _load_robot(
            client,
            urdf_path,
            spec,
            per_robot_paths,
            args.ignore_visuals,
            initial_configuration=initial_state,
            attachment_spec=attachment_map.get(spec.label),
        )
        runtimes.append(runtime)
        if initial_state is not None:
            formatted = ", ".join(f"{value:.4f}" for value in initial_state)
            print(f"Applied initial state to {spec.label}: [{formatted}]")

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    status_id: Optional[int] = None
    last_status: Optional[bool] = None
    env = vamp.Environment()
    _place_obstacles(client, env, obstacle_specs)
    baseline_configs = [runtime.configuration.copy() for runtime in runtimes]
    last_pb_summary: Set[str] = set()
    last_vamp_summary: Set[str] = set()

    if args.randomize:
        _randomize_and_compare(
            client,
            runtimes,
            env,
            args.randomize,
            args.random_seed,
            baseline_configs,
        )
        last_pb_summary.clear()
        last_vamp_summary.clear()
    else:
        try:
            while True:
                states = []
                for runtime in runtimes:
                    for idx, (slider_id, joint_index) in enumerate(zip(runtime.slider_ids, runtime.joint_indices)):
                        value = client.readUserDebugParameter(slider_id)
                        runtime.configuration[idx] = value
                        client.resetJointState(runtime.body_id, joint_index, value)

                    states.append(_build_state(runtime, client))

                try:
                    ts = time.time()
                    collision_free = vamp.fkcc_multi_all(states, env)
                    te = time.time()
                except ValueError as exc:
                    text = f"fkcc_multi_all error: {exc}"
                    color = [1.0, 0.5, 0.0]
                    collision_free = False
                else:
                    text = "Collision Free" if collision_free else "In Collision"
                    color = _status_color(collision_free)

                client.performCollisionDetection()

                pb_summary: Optional[Set[str]] = None
                vamp_summary: Optional[Set[str]] = None
                if args.show_collisions:
                    pb_summary, pb_lines = _collect_pybullet_contacts(client, runtimes)
                    vamp_summary, vamp_lines = _compute_vamp_contacts(runtimes, env, states)

                    if pb_summary != last_pb_summary or vamp_summary != last_vamp_summary:
                        if pb_summary:
                            print("PyBullet contacts:")
                            for line in pb_lines:
                                print(f"  {line}")
                        elif last_pb_summary:
                            print("PyBullet contacts: none")

                        if vamp_summary:
                            print("VAMP collisions:")
                            for line in vamp_lines:
                                print(f"  {line}")
                        elif last_vamp_summary:
                            print("VAMP collisions: none")

                        print("---")

                    last_pb_summary = pb_summary or set()
                    last_vamp_summary = vamp_summary or set()

                    text = (
                        f"{text} | VAMP:{len(last_vamp_summary)} PB:{len(last_pb_summary)}"
                    )

                if status_id is not None:
                    client.removeUserDebugItem(status_id)

                anchor = [0.0, 0.0, 0.0]
                parent = -1
                if runtimes:
                    parent = runtimes[0].body_id
                    anchor = [0.0, 0.0, 1.2]

                status_id = client.addUserDebugText(
                    text,
                    textPosition=anchor,
                    textColorRGB=color,
                    textSize=2.0,
                    lifeTime=0,
                    parentObjectUniqueId=parent,
                )

                if args.print and collision_free != last_status:
                    t_cost = (te - ts) * 1000.0
                    print(f"[{time.strftime('%H:%M:%S')}] fkcc_multi_all: {t_cost:.2f} ms")
                    print(f"Collision status changed -> {'free' if collision_free else 'collision'}")
                last_status = collision_free

                client.stepSimulation()
                time.sleep(args.sleep)
        finally:
            client.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
