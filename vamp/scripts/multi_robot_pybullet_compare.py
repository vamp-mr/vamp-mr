"""Compare VAMP multi-robot collision checking against PyBullet contacts.

This script places two Panda arms in identical configurations with different
base transforms, evaluates collision status using the new
``vamp.fkcc_multi_all`` binding, and verifies the result with PyBullet's
contact test.  The example runs twice: once with overlapping bases (expected
collision) and once with the second arm shifted away (expected no collision).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence

import numpy as np
import pybullet as pb
from pybullet_utils.bullet_client import BulletClient

import vamp
import time


# PyBullet's Panda URDF references a texture that isn't shipped with the repo, which causes
# a flood of warnings when loading the robot.  A tiny placeholder texture silences those
# warnings and keeps the comparison output readable.
def ensure_placeholder_texture() -> None:
    texture_path = Path(__file__).parents[1] / "resources" / "panda" / "meshes" / "visual" / "colors.png"
    if texture_path.exists():
        return

    texture_path.parent.mkdir(parents=True, exist_ok=True)
    import base64

    png_bytes = base64.b64decode(
        "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR4nGMAAQAABQABDQottAAAAABJRU5ErkJggg=="
    )
    texture_path.write_bytes(png_bytes)


@dataclass
class RobotInstance:
    body_id: int
    joint_indices: List[int]


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> Sequence[float]:
    """Convert a rotation matrix to a quaternion (x, y, z, w)."""

    m = matrix
    trace = np.trace(m)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s

    return [qx, qy, qz, qw]


def load_robot(client: BulletClient, urdf_path: Path, transform: np.ndarray) -> RobotInstance:
    position = transform[:3, 3]
    orientation = rotation_matrix_to_quaternion(transform[:3, :3])

    body_id = client.loadURDF(
        str(urdf_path),
        basePosition=position,
        baseOrientation=orientation,
        useFixedBase=True,
        flags=pb.URDF_MAINTAIN_LINK_ORDER | pb.URDF_USE_SELF_COLLISION | pb.URDF_IGNORE_VISUAL_SHAPES,
    )

    joint_names = vamp.panda.joint_names()
    joint_map = {}
    for idx in range(client.getNumJoints(body_id)):
        info = client.getJointInfo(body_id, idx)
        name = info[1].decode() if isinstance(info[1], bytes) else info[1]
        joint_map[name] = idx

    joint_indices = [joint_map[name] for name in joint_names]
    return RobotInstance(body_id=body_id, joint_indices=joint_indices)


def set_joint_positions(client: BulletClient, robot: RobotInstance, positions: Sequence[float]) -> None:
    for joint, value in zip(robot.joint_indices, positions):
        client.resetJointState(robot.body_id, joint, value, targetVelocity=0.0)


def pybullet_collision(configs: Iterable[Sequence[float]], transforms: Iterable[np.ndarray]) -> bool:
    client = BulletClient(connection_mode=pb.DIRECT)
    client.setGravity(0, 0, -9.81)

    robot_dir = Path(__file__).parents[1] / "resources" / "panda"
    urdf_path = robot_dir / "panda_spherized.urdf"

    ensure_placeholder_texture()

    robots = [
        load_robot(client, urdf_path, transform)
        for transform in transforms
    ]

    for robot, configuration in zip(robots, configs):
        set_joint_positions(client, robot, configuration)

    client.performCollisionDetection()
    contacts = client.getContactPoints(robots[0].body_id, robots[1].body_id)
    client.disconnect()
    return len(contacts) == 0


def vamp_collision(configs: Iterable[Sequence[float]], transforms: Iterable[np.ndarray]) -> bool:
    env = vamp.Environment()
    states = []
    for configuration, transform in zip(configs, transforms):
        states.append(
            {
                "robot": "panda",
                "configuration": list(configuration),
                "transform": np.asarray(transform, dtype=np.float32),
            }
        )

    return vamp.fkcc_multi_all(states, env)


def main() -> None:
    nominal = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    def run_case(offset: float) -> None:
        configs = [nominal, nominal]
        transforms = [np.eye(4, dtype=np.float32), np.eye(4, dtype=np.float32)]
        transforms[1][0, 3] = offset

        ts = time.time()
        vamp_result = vamp_collision(configs, transforms)
        te = time.time()
        bullet_result = pybullet_collision(configs, transforms)
        tb = time.time()

        print(f"Offset {offset:.3f}m -> VAMP: {vamp_result}, PyBullet: {bullet_result}")
        print(f"VAMP collision check took {te - ts:.6f}s, PyBullet took {tb - te:.6f}s")

        assert vamp_result == bullet_result

    print("Testing multi-robot collision agreement")
    run_case(0.0)
    run_case(0.5)
    run_case(0.75)


if __name__ == "__main__":
    main()
