#!/usr/bin/env python3

import argparse
import json
import math
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional


def run(cmd: List[str], *, cwd: Optional[Path] = None) -> None:
    subprocess.check_call(cmd, cwd=str(cwd) if cwd else None)


def sanitize_cpp_identifier(name: str) -> str:
    # Keep alnum/underscore, and ensure it doesn't start with a digit.
    out = re.sub(r"[^0-9A-Za-z_]", "_", name)
    if not out:
        raise ValueError("empty robot name")
    if out[0].isdigit():
        out = "_" + out
    return out


def snake(name: str) -> str:
    s1 = re.sub("(.)([A-Z][a-z]+)", r"\\1_\\2", name)
    return re.sub("([a-z0-9])([A-Z])", r"\\1_\\2", s1).lower()


def repeated_template(struct_name: str, count: int) -> str:
    if count <= 0:
        raise ValueError("num_robots must be > 0")
    return ", ".join([f"vamp::robots::{struct_name}"] * count)


def infer_prefix_from_core_dir(core_dir: str) -> Optional[Path]:
    if not core_dir:
        return None
    try:
        p = Path(core_dir).resolve()
    except Exception:
        return None
    # Typical layout: <prefix>/(lib|lib64)/cmake/mr_planner_core
    parts = [x.name for x in p.parents]
    if "cmake" in parts and ("lib" in parts or "lib64" in parts):
        try:
            return p.parents[2]
        except Exception:
            return None
    return None


def locate_fkcc_gen(args) -> str:
    if args.cricket_bin:
        return args.cricket_bin

    # If the user supplied a mr_planner_core prefix, prefer a co-installed cricket binary.
    prefix = Path(args.mr_planner_core_prefix).resolve() if args.mr_planner_core_prefix else infer_prefix_from_core_dir(args.mr_planner_core_dir)
    if prefix:
        candidate = prefix / "bin" / "fkcc_gen"
        if candidate.is_file():
            return str(candidate)

    # Try common locations.
    if args.cricket_dir:
        candidate = Path(args.cricket_dir) / "build" / "fkcc_gen"
        if candidate.is_file():
            return str(candidate)
        candidate = Path(args.cricket_dir) / "build-release" / "fkcc_gen"
        if candidate.is_file():
            return str(candidate)

    which = shutil.which("fkcc_gen")
    if which:
        return which
    raise FileNotFoundError(
        "Could not locate cricket's fkcc_gen. Provide --cricket-bin or build cricket and add fkcc_gen to PATH."
    )


def locate_cricket_templates(args) -> Path:
    if args.cricket_templates:
        p = Path(args.cricket_templates)
        if not p.is_dir():
            raise FileNotFoundError(p)
        return p
    # If the user supplied a mr_planner_core prefix, prefer a co-installed templates directory.
    prefix = Path(args.mr_planner_core_prefix).resolve() if args.mr_planner_core_prefix else infer_prefix_from_core_dir(args.mr_planner_core_dir)
    if prefix:
        candidate = prefix / "share" / "mr_planner_core" / "cricket" / "templates"
        if candidate.is_dir():
            return candidate
    if args.cricket_dir:
        p = Path(args.cricket_dir) / "resources" / "templates"
        if p.is_dir():
            return p
    raise FileNotFoundError("Could not locate cricket templates; provide --cricket-dir or --cricket-templates")


def find_mr_planner_core_cmake_dir(prefix: Path) -> Path:
    for rel in ["lib/cmake/mr_planner_core", "lib64/cmake/mr_planner_core"]:
        p = prefix / rel
        if p.is_dir():
            return p
    raise FileNotFoundError(f"Could not find mr_planner_core cmake package under {prefix}")


def parse_robot_header_metadata(header_path: Path) -> dict:
    txt = header_path.read_text(errors="replace")
    dim = None
    n_spheres = None

    m = re.search(r"static constexpr std::size_t dimension\s*=\s*(\d+)\s*;", txt)
    if m:
        dim = int(m.group(1))

    m = re.search(r"static constexpr std::size_t n_spheres\s*=\s*(\d+)\s*;", txt)
    if m:
        n_spheres = int(m.group(1))

    out = {}
    if dim is not None:
        out["vamp_dimension"] = dim
    if n_spheres is not None:
        out["vamp_n_spheres"] = n_spheres
    return out


def load_base_transforms(path: Path) -> List[Dict[str, Any]]:
    root = json.loads(path.read_text())
    base = root
    if isinstance(root, dict) and "base_transforms" in root:
        base = root["base_transforms"]

    if not isinstance(base, list) or not base:
        raise ValueError(f"Invalid base transforms in {path}: expected a non-empty JSON list")

    out: List[Dict[str, Any]] = []
    for i, item in enumerate(base):
        if not isinstance(item, dict):
            raise ValueError(f"Invalid base transforms in {path}: base_transforms[{i}] must be an object")
        # Minimal validation compatible with mr_planner_core JSON parser:
        # translation: {xyz|translation}: [x,y,z], rotation: {rpy|quaternion}: [...]
        trans = item.get("translation", item.get("xyz"))
        if trans is not None:
            if not (isinstance(trans, list) and len(trans) == 3 and all(isinstance(x, (int, float)) for x in trans)):
                raise ValueError(f"Invalid base transforms in {path}: base_transforms[{i}].translation/xyz must be [x,y,z]")
        rot_rpy = item.get("rpy")
        rot_q = item.get("quaternion")
        if rot_rpy is not None:
            if not (isinstance(rot_rpy, list) and len(rot_rpy) == 3 and all(isinstance(x, (int, float)) for x in rot_rpy)):
                raise ValueError(f"Invalid base transforms in {path}: base_transforms[{i}].rpy must be [r,p,y]")
        if rot_q is not None:
            if not (isinstance(rot_q, list) and len(rot_q) == 4 and all(isinstance(x, (int, float)) for x in rot_q)):
                raise ValueError(f"Invalid base transforms in {path}: base_transforms[{i}].quaternion must be [w,x,y,z]")
        out.append(item)
    return out


def write_plugin_sources(
    *,
    out_dir: Path,
    plugin_target: str,
    robot_struct: str,
    robot_header: Path,
    num_robots: int,
) -> Path:
    src_dir = out_dir / "plugin_src"
    src_dir.mkdir(parents=True, exist_ok=True)

    # Copy the header locally so the plugin build is self-contained.
    local_header = src_dir / robot_header.name
    shutil.copy2(robot_header, local_header)

    plugin_cpp = src_dir / "plugin.cpp"
    plugin_cpp.write_text(
        "\n".join(
            [
                '#include <mr_planner/backends/vamp_plugin_api.h>',
                '#include <mr_planner/backends/vamp_instance.h>',
                f'#include "{local_header.name}"',
                "",
                "#include <new>",
                "",
                f"using PluginInstance = VampInstance<{repeated_template(robot_struct, num_robots)}>;",
                "",
                "namespace {",
                "PlanInstance *create_instance() { return new PluginInstance(); }",
                "void destroy_instance(PlanInstance *ptr) { delete ptr; }",
                "",
                "const mr_planner::vamp_plugin::Api kApi{",
                "    mr_planner::vamp_plugin::kAbiVersion,",
                f'    "{plugin_target}",',
                "    &create_instance,",
                "    &destroy_instance,",
                "};",
                "}  // namespace",
                "",
                'extern "C" const mr_planner::vamp_plugin::Api *mr_planner_vamp_plugin_get_api() { return &kApi; }',
                "",
            ]
        )
        + "\n"
    )

    cmake = src_dir / "CMakeLists.txt"
    cmake.write_text(
        "\n".join(
            [
                "cmake_minimum_required(VERSION 3.16)",
                f"project({plugin_target} LANGUAGES CXX)",
                "",
                "find_package(mr_planner_core CONFIG REQUIRED)",
                "",
                f"add_library({plugin_target} SHARED plugin.cpp)",
                f"target_link_libraries({plugin_target} PRIVATE mr_planner::mr_planner_core)",
                f"target_compile_features({plugin_target} PRIVATE cxx_std_17)",
                "",
                "set_target_properties(",
                f"  {plugin_target}",
                "  PROPERTIES",
                "    LIBRARY_OUTPUT_DIRECTORY \"${CMAKE_BINARY_DIR}/lib\"",
                "    RUNTIME_OUTPUT_DIRECTORY \"${CMAKE_BINARY_DIR}/bin\"",
                ")",
                "",
            ]
        )
        + "\n"
    )
    return src_dir


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate and build a mr_planner VAMP robot plugin from URDF/SRDF.")
    parser.add_argument("--env-name", required=True, help="Environment name (used in environment json)")
    parser.add_argument(
        "--environment-name",
        default="",
        help="Value stored in environment json field `environment_name` (defaults to --env-name). "
        "Use this to reuse built-in obstacle/attachment presets (e.g. panda_two).",
    )
    parser.add_argument("--move-group", default="arm")
    parser.add_argument("--robot-groups", default="", help="Comma-separated robot group names (defaults to arm0,arm1,...)")
    parser.add_argument("--hand-groups", default="", help="Comma-separated hand group names (optional)")
    parser.add_argument("--num-robots", type=int, default=1)
    parser.add_argument(
        "--base-transforms",
        default="",
        help="Path to JSON file containing base transforms (either a list of transforms or {\"base_transforms\": [...]}); "
        "if omitted, defaults to identity (1 robot) or a circle layout (N robots).",
    )

    parser.add_argument("--robot-struct", default="", help="C++ struct name (defaults to sanitized env-name)")
    parser.add_argument("--robot-header", default="", help="Use an existing VAMP robot header (.hh) instead of running cricket")

    parser.add_argument("--spherized-urdf", default="", help="Path to spherized URDF (required when running cricket)")
    parser.add_argument("--srdf", default="", help="Path to SRDF (required when running cricket)")
    parser.add_argument("--end-effector", default="", help="SRDF frame name for attachments (required when running cricket)")
    parser.add_argument("--resolution", type=int, default=32)

    parser.add_argument("--output-dir", default="", help="Output directory (defaults under /tmp)")

    parser.add_argument("--mr-planner-core-prefix", default="", help="Prefix where mr_planner_core is installed")
    parser.add_argument("--mr-planner-core-dir", default="", help="Path to mr_planner_core CMake package directory")

    parser.add_argument("--cricket-dir", default="", help="Path to cricket repo (for templates)")
    parser.add_argument("--cricket-templates", default="", help="Path to cricket resources/templates")
    parser.add_argument("--cricket-bin", default="", help="Path to fkcc_gen executable")
    args = parser.parse_args()

    env_name = args.env_name.strip()
    if not env_name:
        raise SystemExit("--env-name must be non-empty")

    environment_name = args.environment_name.strip() or env_name

    num_robots = int(args.num_robots)
    if num_robots <= 0:
        raise SystemExit("--num-robots must be > 0")

    robot_struct = sanitize_cpp_identifier(args.robot_struct.strip() or env_name)
    plugin_target = f"mr_planner_vamp_plugin_{snake(robot_struct)}_{num_robots}r"

    out_dir = Path(args.output_dir) if args.output_dir else Path("/tmp") / f"mr_planner_plugin_{snake(robot_struct)}"
    out_dir.mkdir(parents=True, exist_ok=True)

    # Resolve mr_planner_core package dir.
    mr_planner_core_dir = args.mr_planner_core_dir.strip()
    if not mr_planner_core_dir and args.mr_planner_core_prefix:
        mr_planner_core_dir = str(find_mr_planner_core_cmake_dir(Path(args.mr_planner_core_prefix)))
    if not mr_planner_core_dir:
        raise SystemExit("Provide --mr-planner-core-dir or --mr-planner-core-prefix so the plugin can find_package(mr_planner_core).")

    # Step 1: Generate robot header via cricket (optional).
    header_path: Path
    if args.robot_header:
        header_path = Path(args.robot_header).resolve()
        if not header_path.is_file():
            raise FileNotFoundError(header_path)
    else:
        if not args.spherized_urdf or not args.srdf or not args.end_effector:
            raise SystemExit("To run cricket, provide --spherized-urdf, --srdf, and --end-effector (or pass --robot-header).")
        urdf = Path(args.spherized_urdf).resolve()
        srdf = Path(args.srdf).resolve()
        if not urdf.is_file():
            raise FileNotFoundError(urdf)
        if not srdf.is_file():
            raise FileNotFoundError(srdf)

        fkcc_gen = locate_fkcc_gen(args)
        templates_dir = locate_cricket_templates(args)

        cricket_cfg = out_dir / "cricket_config.json"
        header_path = out_dir / f"{snake(robot_struct)}_fk.hh"
        cfg = {
            "name": robot_struct,
            "urdf": str(urdf),
            "srdf": str(srdf),
            "end_effector": args.end_effector,
            "resolution": int(args.resolution),
            "template": str((templates_dir / "fk_template.hh").resolve()),
            "subtemplates": [{"name": "ccfk", "template": str((templates_dir / "ccfk_template.hh").resolve())}],
            "output": str(header_path),
        }
        cricket_cfg.write_text(json.dumps(cfg, indent=2))

        run([fkcc_gen, str(cricket_cfg)])

        if not header_path.is_file():
            raise RuntimeError(f"cricket did not produce header at {header_path}")

    # Step 2: Write plugin sources and build.
    src_dir = write_plugin_sources(
        out_dir=out_dir,
        plugin_target=plugin_target,
        robot_struct=robot_struct,
        robot_header=header_path,
        num_robots=num_robots,
    )
    build_dir = out_dir / "build"
    build_dir.mkdir(parents=True, exist_ok=True)

    run(
        [
            "cmake",
            "-S",
            str(src_dir),
            "-B",
            str(build_dir),
            "-DCMAKE_BUILD_TYPE=Release",
            f"-Dmr_planner_core_DIR={mr_planner_core_dir}",
        ]
    )
    run(["cmake", "--build", str(build_dir), "-j"])

    lib_dir = build_dir / "lib"
    candidates = list(lib_dir.glob(f"lib{plugin_target}.*"))
    if not candidates:
        raise RuntimeError(f"Failed to locate built plugin library under {lib_dir}")
    plugin_lib = candidates[0].resolve()

    # Step 3: Write environment JSON that mr_planner_core can load.
    header_meta = parse_robot_header_metadata(header_path)
    robot_groups: List[str]
    if args.robot_groups:
        robot_groups = [x.strip() for x in args.robot_groups.split(",") if x.strip()]
    else:
        if num_robots == 1:
            robot_groups = [args.move_group]
        else:
            robot_groups = [f"arm{i}" for i in range(num_robots)]

    hand_groups: List[str] = [x.strip() for x in args.hand_groups.split(",") if x.strip()] if args.hand_groups else []

    base_transforms: List[Dict[str, Any]] = []
    if args.base_transforms:
        base_path = Path(args.base_transforms).expanduser().resolve()
        if not base_path.is_file():
            raise FileNotFoundError(base_path)
        base_transforms = load_base_transforms(base_path)
        if len(base_transforms) != num_robots:
            raise SystemExit(
                f"--base-transforms provided {len(base_transforms)} transforms but --num-robots is {num_robots}"
            )
    else:
        if num_robots == 1:
            base_transforms = [{"translation": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}]
        else:
            radius = 0.6
            for i in range(num_robots):
                angle = (2.0 * math.pi * float(i)) / float(num_robots)
                # Place on a circle and yaw inward.
                base_transforms.append(
                    {
                        "translation": [radius * math.cos(angle), radius * math.sin(angle), 0.0],
                        "rpy": [0.0, 0.0, angle + math.pi],
                    }
                )

    env_json = out_dir / f"{env_name}.json"
    env_json.write_text(
        json.dumps(
            {
                "name": env_name,
                "environment_name": environment_name,
                "move_group": args.move_group,
                "robot_groups": robot_groups,
                "hand_groups": hand_groups,
                "vamp_plugin": str(plugin_lib),
                "base_transforms": base_transforms,
                **header_meta,
            },
            indent=2,
        )
        + "\n"
    )

    print(
        json.dumps(
            {
                "output_dir": str(out_dir),
                "plugin_library": str(plugin_lib),
                "environment_json": str(env_json),
                "env_name": env_name,
                "robot_struct": robot_struct,
                "num_robots": num_robots,
                **header_meta,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
