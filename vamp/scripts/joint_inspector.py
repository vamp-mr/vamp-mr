#!/usr/bin/env python3
"""Interactive joint inspector with PyBullet visualization."""

import argparse
import signal
import sys
import time
from dataclasses import dataclass
from math import pi
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
from collections import OrderedDict

import numpy as np
import pybullet as pb
from pybullet_utils.bullet_client import BulletClient

import vamp


@dataclass
class LoadedRobot:
    body_id: int
    joint_map: Dict[str, int]


@dataclass
class JointSlider:
    name: str
    indices: List[int]
    slider_id: int
    original_joint_index: int
    spherized_joint_index: Optional[int]


def _resolve_path(path: Path) -> Path:
    if not path.exists():
        raise FileNotFoundError(f"URDF not found: {path}")
    return path.resolve()


def _additional_search_paths(paths: Iterable[Optional[Path]]) -> List[str]:
    unique: List[str] = []
    for p in paths:
        if p is None:
            continue
        try:
            resolved = str(p.resolve())
        except FileNotFoundError:
            continue
        if resolved not in unique:
            unique.append(resolved)
    return unique


def _load_robot(
    client: BulletClient,
    urdf_path: Path,
    base_pose: Tuple[float, float, float],
    extra_search_paths: Sequence[Optional[Path]] = (),
) -> LoadedRobot:
    for extra in _additional_search_paths([urdf_path.parent, *extra_search_paths]):
        client.setAdditionalSearchPath(extra)

    body_id = client.loadURDF(
        str(urdf_path),
        basePosition=base_pose,
        baseOrientation=(0, 0, 0, 1),
        useFixedBase=True,
    )

    joint_map: Dict[str, int] = {}
    for joint_index in range(client.getNumJoints(body_id)):
        info = client.getJointInfo(body_id, joint_index)
        joint_name_bytes = info[1]
        joint_name = joint_name_bytes.decode()
        joint_map.setdefault(joint_name, joint_index)

    return LoadedRobot(body_id=body_id, joint_map=joint_map)


def _joint_limits(client: BulletClient, body_id: int, joint_index: int) -> Tuple[float, float]:
    info = client.getJointInfo(body_id, joint_index)
    lower, upper = info[8], info[9]
    if lower < upper and not (np.isinf(lower) or np.isinf(upper)):
        return lower, upper
    return -pi, pi


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect robot joint configurations with PyBullet.")
    parser.add_argument("robot", nargs="?", default="gp4", help="Robot name registered in vamp (default: gp4)")
    parser.add_argument("--original", type=Path, help="Path to the mesh URDF (defaults to resources/<robot>/<robot>.urdf)")
    parser.add_argument("--spherized", type=Path, help="Path to the spherized URDF (defaults to *_spherized_visual.urdf)")
    parser.add_argument("--offset", type=float, default=0.8, help="X offset for the spherized clone (default: 2.0)")
    parser.add_argument(
        "--extra-path",
        action="append",
        dest="extra_paths",
        default=[],
        help="Additional directory to add to Bullet search paths (can be used multiple times)",
    )
    parser.add_argument(
        "--hide-ui",
        action="store_true",
        help="Disable the PyBullet debug GUI overlay (hides sliders/text).",
    )
    parser.add_argument("--no-gui", action="store_true", help="Run without GUI (loads DIRECT mode)")
    parser.add_argument("--check-bounds", action="store_true", help="Request vamp.validate to check joint bounds")
    parser.add_argument("--sleep", type=float, default=1.0 / 120.0, help="Loop sleep in seconds (default: 1/120)")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _build_parser().parse_args(argv)

    if args.no_gui:
        raise SystemExit("--no-gui is not supported for interactive inspection; omit the flag to launch the GUI.")

    if args.robot not in vamp.robots:
        raise SystemExit(f"Robot '{args.robot}' is not available. Known robots: {', '.join(vamp.robots)}")

    repo_root = Path(__file__).resolve().parent.parent
    resource_dir = repo_root / "resources" / args.robot

    original_candidates = [
        repo_root.parent / "foam" / "assets" / f"{args.robot}_lego" / f"{args.robot}.urdf",
        resource_dir / f"{args.robot}.urdf",
        repo_root.parent / "foam" / "assets" / args.robot / f"{args.robot}.urdf",
    ]
    for candidate in original_candidates:
        if candidate.exists():
            default_original = candidate
            break
    else:
        raise FileNotFoundError(f"Could not locate a mesh URDF for {args.robot}. Tried: {', '.join(str(c) for c in original_candidates)}")

    default_spherized = resource_dir / f"{args.robot}_spherized_visual.urdf"
    if not default_spherized.exists():
        fallback = resource_dir / f"{args.robot}_spherized.urdf"
        if fallback.exists():
            default_spherized = fallback

    original_path = _resolve_path(args.original) if args.original else _resolve_path(default_original)
    spherized_path = _resolve_path(args.spherized) if args.spherized else _resolve_path(default_spherized)

    extra_search_dirs: List[Path] = []

    def _add_candidate(path_like: Optional[Path]):
        if path_like is None:
            return
        candidate = Path(path_like).expanduser()
        if candidate.exists() and candidate not in extra_search_dirs:
            extra_search_dirs.append(candidate)

    candidate_paths = [
        resource_dir,
        resource_dir / "meshes",
        original_path.parent,
        spherized_path.parent,
        repo_root / "assets" / args.robot,
        repo_root.parent / "foam" / "assets" / args.robot,
    ]
    for extra in candidate_paths:
        _add_candidate(extra)
    for supplied in args.extra_paths or []:
        _add_candidate(Path(supplied))

    robot_module = getattr(vamp, args.robot)
    joint_names = robot_module.joint_names()
    dimension = robot_module.dimension()

    if len(joint_names) != dimension:
        print("Warning: joint name count does not match configuration dimension.", file=sys.stderr)

    mode = pb.GUI if not args.no_gui else pb.DIRECT
    client = BulletClient(connection_mode=mode)
    client.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0 if args.hide_ui else 1)
    client.configureDebugVisualizer(pb.COV_ENABLE_MOUSE_PICKING, 0)
    client.resetDebugVisualizerCamera(cameraDistance=2.0, cameraYaw=135, cameraPitch=-30, cameraTargetPosition=[0.4, 0.2, 0.6])

    original = _load_robot(client, original_path, (0.0, 0.0, 0.0), extra_search_dirs)
    spherized = _load_robot(client, spherized_path, (args.offset, 0.0, 0.0), extra_search_dirs)

    index_map: Dict[str, List[int]] = OrderedDict()
    for idx, name in enumerate(joint_names):
        index_map.setdefault(name, []).append(idx)

    sliders: List[JointSlider] = []
    for name, indices in index_map.items():
        original_index = original.joint_map.get(name)
        if original_index is None:
            raise SystemExit(f"Joint '{name}' not found in URDF '{original_path}'.")
        lower, upper = _joint_limits(client, original.body_id, original_index)
        label = name if len(indices) == 1 else f"{name} [{', '.join(str(i) for i in indices)}]"
        slider_id = client.addUserDebugParameter(label, lower, upper, 0.0)
        sliders.append(
            JointSlider(
                name=name,
                indices=indices,
                slider_id=slider_id,
                original_joint_index=original_index,
                spherized_joint_index=spherized.joint_map.get(name),
            )
        )

    covered = sorted(idx for slider in sliders for idx in slider.indices)
    if covered != list(range(dimension)):
        print("Warning: slider coverage does not match configuration dimension.", file=sys.stderr)

    text_uid = client.addUserDebugText("Initializing...", [0, 0, 1.4], textColorRGB=[1, 1, 1])
    environment = vamp.Environment()

    run = True

    def _handle_sigint(_sig, _frame):
        nonlocal run
        run = False

    signal.signal(signal.SIGINT, _handle_sigint)

    previous_config: Optional[np.ndarray] = None

    while run:
        slider_values = np.array([client.readUserDebugParameter(slider.slider_id) for slider in sliders], dtype=np.float32)
        full_config = np.zeros(dimension, dtype=np.float32)

        for slider_value, slider in zip(slider_values, sliders):
            for idx in slider.indices:
                full_config[idx] = np.float32(slider_value)
            client.resetJointState(original.body_id, slider.original_joint_index, float(slider_value))
            if slider.spherized_joint_index is not None:
                client.resetJointState(spherized.body_id, slider.spherized_joint_index, float(slider_value))

        ts = time.time()
        is_valid = bool(robot_module.validate(full_config, environment, args.check_bounds))
        te = time.time()
        print(f"Validation took {(te - ts) * 1000.0:.2f} ms")
        if not is_valid:
            env_hits, self_hits = robot_module.debug(full_config, environment)
            offending_links = [
                (i, hits) for i, hits in enumerate(env_hits)
            ]
            if offending_links:
                print("[debug] env collisions:")
                for idx, hits in offending_links:
                    if len(hits) > 0:
                        print(f"  sphere {idx}: {hits}")
            if self_hits:
                print("[debug] self collisions (sphere indices):", self_hits)

        summary = "VALID" if is_valid else "INVALID"
        client.addUserDebugText(
            f"Config: {summary}",
            [0, 0, 1.4],
            textColorRGB=[0, 1, 0] if is_valid else [1, 0, 0],
            replaceItemUniqueId=text_uid,
        )

        if previous_config is None or not np.allclose(full_config, previous_config):
            readable = "; ".join(
                f"{slider.name}={float(value):+.4f}" for slider, value in zip(sliders, slider_values)
            )
            print(f"[joint_inspector] {args.robot} -> {readable}; valid={is_valid}")
            expanded = ", ".join(f"{name}[{idx}]={full_config[idx]:+.4f}" for idx, name in enumerate(joint_names))
            print(f"[joint_inspector] raw vector: {np.array2string(full_config, precision=4)}")
            print(f"[joint_inspector] expanded: {expanded}")
            previous_config = full_config.copy()

        if mode == pb.DIRECT:
            client.stepSimulation()
        time.sleep(args.sleep)

    client.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
