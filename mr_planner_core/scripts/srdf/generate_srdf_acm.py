#!/usr/bin/env python3

"""
Generate / update an SRDF Allowed Collision Matrix (ACM) by random sampling.

This is a thin wrapper around MoveIt's `collisions_updater` tool (from
`moveit_setup_assistant`), which:
  - samples random joint configurations,
  - identifies link pairs that are *never* in collision (safe to disable),
  - optionally identifies link pairs that are *always* in collision (often due
    to overlapping/approximate collision geometry),
  - writes updated `<disable_collisions .../>` entries into an SRDF.

Notes:
  - This script does NOT require a running ROS master, but it DOES require a
    local MoveIt installation that provides `collisions_updater`.
  - For spherized URDFs (foam/cricket), this can help auto-generate the SRDF
    collision disables expected by cricket / MoveIt-style workflows.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


def _read_robot_name_from_xml(path: Path) -> str:
    tree = ET.parse(str(path))
    root = tree.getroot()
    if root.tag != "robot":
        raise RuntimeError(f"Expected root <robot> in {path}, got <{root.tag}>")
    name = root.get("name", "").strip()
    if not name:
        raise RuntimeError(f"Missing robot name attribute in {path}")
    return name


def _write_stub_srdf(path: Path, robot_name: str, group_name: str, chain_base: str, chain_tip: str) -> None:
    root = ET.Element("robot", attrib={"name": robot_name})
    if group_name and chain_base and chain_tip:
        group = ET.SubElement(root, "group", attrib={"name": group_name})
        ET.SubElement(group, "chain", attrib={"base_link": chain_base, "tip_link": chain_tip})
    tree = ET.ElementTree(root)
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def _rewrite_srdf_robot_name(srdf_in: Path, srdf_out: Path, robot_name: str) -> None:
    tree = ET.parse(str(srdf_in))
    root = tree.getroot()
    if root.tag != "robot":
        raise RuntimeError(f"Expected root <robot> in {srdf_in}, got <{root.tag}>")
    root.set("name", robot_name)
    tree.write(str(srdf_out), encoding="utf-8", xml_declaration=True)


def _locate_collisions_updater(arg: str) -> Path:
    if arg:
        p = Path(arg).expanduser().resolve()
        if not p.is_file():
            raise FileNotFoundError(p)
        return p

    # Prefer the standard ROS Noetic location if present.
    candidate = Path("/opt/ros/noetic/lib/moveit_setup_assistant/collisions_updater")
    if candidate.is_file():
        return candidate

    which = shutil.which("collisions_updater")
    if which:
        return Path(which).resolve()

    raise FileNotFoundError(
        "Could not find MoveIt's collisions_updater. Install moveit_setup_assistant or pass --collisions-updater."
    )


def shlex_quote(s: str) -> str:
    # Minimal single-quote shell escaping.
    return "'" + s.replace("'", "'\"'\"'") + "'"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True, help="Path to URDF (or xacro).")
    parser.add_argument(
        "--srdf-in",
        type=Path,
        default=None,
        help="Optional SRDF template to update (keeps groups / named poses). If omitted, a minimal stub SRDF is used.",
    )
    parser.add_argument("--srdf-out", type=Path, required=True, help="Output SRDF path.")

    parser.add_argument(
        "--collisions-updater",
        default="",
        help="Path to MoveIt's collisions_updater (defaults to /opt/ros/noetic/... if present).",
    )
    parser.add_argument(
        "--ros-setup",
        default="/opt/ros/noetic/setup.bash",
        help="ROS setup.bash to source before running collisions_updater (needed for LD_LIBRARY_PATH).",
    )

    parser.add_argument("--xacro-args", default="", help="Optional arguments forwarded to xacro.")

    parser.add_argument("--trials", type=int, default=100000, help="Number of random samples (can be large, e.g. 1e6).")
    parser.add_argument(
        "--min-collision-fraction",
        type=float,
        default=1.0,
        help="Fraction of a small sample size to label pairs as always colliding.",
    )
    parser.add_argument("--verbose", action="store_true")

    parser.add_argument("--default", dest="do_default", action="store_true", default=True)
    parser.add_argument("--no-default", dest="do_default", action="store_false")
    parser.add_argument("--always", dest="do_always", action="store_true", default=True)
    parser.add_argument("--no-always", dest="do_always", action="store_false")
    parser.add_argument("--keep", dest="do_keep", action="store_true", default=True)
    parser.add_argument("--no-keep", dest="do_keep", action="store_false")

    parser.add_argument(
        "--group-name",
        default="",
        help="Optional group name for stub SRDF (only used when --srdf-in is omitted).",
    )
    parser.add_argument(
        "--chain-base",
        default="",
        help="Optional chain base_link for stub SRDF (only used when --srdf-in is omitted).",
    )
    parser.add_argument(
        "--chain-tip",
        default="",
        help="Optional chain tip_link for stub SRDF (only used when --srdf-in is omitted).",
    )
    args = parser.parse_args()

    urdf = args.urdf.expanduser().resolve()
    if not urdf.is_file():
        raise FileNotFoundError(urdf)

    srdf_out = args.srdf_out.expanduser().resolve()
    srdf_out.parent.mkdir(parents=True, exist_ok=True)

    collisions_updater = _locate_collisions_updater(args.collisions_updater)

    robot_name = _read_robot_name_from_xml(urdf)

    with tempfile.TemporaryDirectory(prefix="mr_planner_srdf_acm.") as td:
        tmp = Path(td)
        srdf_in: Path
        if args.srdf_in:
            srdf_template = args.srdf_in.expanduser().resolve()
            if not srdf_template.is_file():
                raise FileNotFoundError(srdf_template)
            # collisions_updater is strict about SRDF robot name matching URDF; patch if needed.
            srdf_in = tmp / "srdf_in_patched.srdf"
            _rewrite_srdf_robot_name(srdf_template, srdf_in, robot_name)
        else:
            srdf_in = tmp / "srdf_in_stub.srdf"
            _write_stub_srdf(srdf_in, robot_name, args.group_name, args.chain_base, args.chain_tip)

        cmd = [
            str(collisions_updater),
            "--urdf",
            str(urdf),
            "--srdf",
            str(srdf_in),
            "--output",
            str(srdf_out),
            "--trials",
            str(int(args.trials)),
            "--min-collision-fraction",
            str(float(args.min_collision_fraction)),
        ]
        if args.xacro_args.strip():
            cmd += ["--xacro-args", args.xacro_args.strip()]
        if args.do_default:
            cmd.append("--default")
        if args.do_always:
            cmd.append("--always")
        if args.do_keep and args.srdf_in:
            cmd.append("--keep")
        if args.verbose:
            cmd.append("--verbose")

        # collisions_updater is a ROS/MoveIt binary: source the setup.bash for runtime deps.
        setup_bash = args.ros_setup.strip()
        if setup_bash:
            setup_path = Path(setup_bash).expanduser()
            if not setup_path.is_file():
                raise FileNotFoundError(setup_path)
            shell_cmd = f"source {shlex_quote(str(setup_path))} && " + " ".join(shlex_quote(x) for x in cmd)
            proc = subprocess.run(["bash", "-lc", shell_cmd], check=False)
        else:
            proc = subprocess.run(cmd, check=False)

        if proc.returncode != 0:
            raise RuntimeError(f"collisions_updater failed with exit code {proc.returncode}")

    print(f"Wrote {srdf_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
