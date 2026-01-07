# mr_planner_core

Standalone CMake build of the core planning library (planning, shortcutting, TPG/ADG construction, portable graph I/O) for ``VAMP-MR``. This is intended to be the “engine” that pairs naturally with the SIMD-accelerated VAMP collision backend.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

VAMP support is enabled by default; disable with `-DMR_PLANNER_CORE_ENABLE_VAMP=OFF` (library-only).

If OMPL is only available via ROS Noetic packages, source `/opt/ros/noetic/setup.bash` (or set `ompl_DIR`) before configuring.

## Install + `find_package()`

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j
cmake --install build
```

Consumer CMake:

```cmake
find_package(mr_planner_core CONFIG REQUIRED)
target_link_libraries(my_target PRIVATE mr_planner::mr_planner_core)
```

## Custom robots via runtime VAMP plugins

`mr_planner_core` can load VAMP robot instances from a shared-library plugin at runtime (no recompilation of `mr_planner_core` needed).

The JSON environment format supports:
- `vamp_plugin`: path to the plugin `.so` (built by the generator script below),
- `robot_groups`: robot name list (length must match the plugin robot count),
- `base_transforms`: optional per-robot transforms,
- `MR_PLANNER_VAMP_ENV_PATH`: optional search path for `--vamp-environment <name>`.

### Build a plugin (from an existing VAMP robot header)

This is the fastest path if you already have a `vamp/robots/<robot>.hh` file.

```bash
python3 scripts/plugins/generate_vamp_robot_plugin.py \
  --env-name panda_two \
  --base-transforms "scripts/plugins/config/panda_two_base_transforms.json" \
  --robot-header /usr/local/include/vamp/robots/panda.hh \
  --robot-struct Panda \
  --num-robots 2 \
  --move-group panda_multi_arm \
  --robot-groups panda0_arm,panda1_arm \
  --hand-groups panda0_hand,panda1_hand \
  --mr-planner-core-prefix /usr/local \
  --output-dir /tmp/mr_planner_panda_plugin
```

### Build a plugin (from spherized URDF + SRDF via cricket)

If you have a spherized URDF + SRDF (e.g., generated with `foam`) you can generate the required robot header with cricket (`fkcc_gen`) and compile a plugin:

```bash
python3 scripts/plugins/generate_vamp_robot_plugin.py \
  --env-name my_robot_env \
  --spherized-urdf /path/to/robot_spherized.urdf \
  --srdf /path/to/robot.srdf \
  --end-effector tool_frame \
  --cricket-bin /path/to/cricket/build/fkcc_gen \
  --cricket-templates /path/to/cricket/resources/templates \
  --num-robots 1 \
  --move-group arm \
  --mr-planner-core-prefix /usr/local \
  --output-dir /tmp/mr_planner_my_robot_plugin
```

The generator prints a JSON summary and writes an environment JSON file (e.g. `/tmp/mr_planner_my_robot_plugin/my_robot_env.json`), including convenience fields like `vamp_dimension` and `vamp_n_spheres`.

### Use a plugin environment

Use either a direct JSON path:

```bash
./build/mr_planner_core_plan \
  --vamp-environment /tmp/mr_planner_my_robot_plugin/my_robot_env.json \
  --num-robots 1 \
  --output-dir /tmp/mr_planner_core_demo
```

Or register a directory of env JSON files:

```bash
export MR_PLANNER_VAMP_ENV_PATH=/tmp/mr_planner_my_robot_plugin
./build/mr_planner_core_plan --vamp-environment my_robot_env --num-robots 1 --output-dir /tmp/mr_planner_core_demo
```

### Use a plugin environment (Python)

You can pass a direct environment JSON path (most explicit):

```bash
python3 - <<'PY'
import mr_planner_core

env = mr_planner_core.VampEnvironment("/tmp/mr_planner_my_robot_plugin/my_robot_env.json", seed=1)
res = env.plan(planner="composite_rrt", planning_time=2.0, shortcut_time=0.0, seed=1, write_tpg=False)
print(res["solution_csv"])
PY
```

Or set `MR_PLANNER_VAMP_ENV_PATH` and refer to the environment by name:

```bash
export MR_PLANNER_VAMP_ENV_PATH=/tmp/mr_planner_my_robot_plugin
python3 - <<'PY'
import mr_planner_core

env = mr_planner_core.VampEnvironment("my_robot_env", seed=1)
res = env.plan(planner="composite_rrt", planning_time=2.0, shortcut_time=0.0, seed=1, write_tpg=False)
print(res["solution_csv"])
PY
```

## Python bindings

`mr_planner_core` ships a small pybind11 module that exposes:
- multi-robot planning + shortcutting (VAMP backend),
- `skillplan.json` → portable `tpg.pb` / `adg.pb`,
- `GraphFile` protobuf ↔ JSON conversion helpers.

Build + install (Python bindings are enabled by default when building `mr_planner_core` standalone):

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DMR_PLANNER_CORE_ENABLE_PYTHON=ON
cmake --build build -j
cmake --install build
```

If you install to a non-standard prefix, add the installed `site-packages` directory to `PYTHONPATH`:

```bash
export PYTHONPATH="$(python3 -c \"import sysconfig; p='/usr/local'; print(sysconfig.get_path('platlib', vars={'base': p, 'platbase': p}))\"):${PYTHONPATH}"
python3 - <<'PY'
import mr_planner_core

env = mr_planner_core.VampEnvironment("dual_gp4")
res = env.plan(planner="composite_rrt", output_dir="/tmp/mr_planner_py_out", write_tpg=True)
print(res)
PY
```

Python smoke test (builds + installs into a temp prefix and runs basic planning + skillplan→TPG/ADG checks):

```bash
scripts/regression/run_python_smoke.sh
```

Example scripts (require the Meshcat bridge: `python3 scripts/visualization/meshcat_bridge.py --port 7600`):

```bash
python3 scripts/planning/plan_named_poses.py --vamp-environment dual_gp4 --start ready_pose --goal left_push_up --meshcat
python3 scripts/planning/skillplan_playback.py --skillplan /path/to/skillplan.json --graph-type adg --meshcat
```

## CLI: `mr_planner_core_plan`

`mr_planner_core_plan` is a ROS-free CLI that:
- creates a VAMP environment instance,
- plans a multi-robot joint-space trajectory (`composite_rrt` or `cbs_prm`),
- optionally shortcuts it,
- exports `skillplan.json` + `tpg.pb` (protobuf).

Run (example):

```bash
./build/mr_planner_core_plan \
  --vamp-environment dual_gp4 \
  --planner composite_rrt \
  --planning-time 5 \
  --shortcut-time 1 \
  --output-dir /tmp/mr_planner_core_demo
```

Roadmap-based CBS+PRM (example):

```bash
./build/mr_planner_core_plan \
  --vamp-environment dual_gp4 \
  --planner cbs_prm \
  --roadmap-samples 500 \
  --roadmap-max-dist 2.0 \
  --planning-time 10 \
  --output-dir /tmp/mr_planner_core_demo_prm
```

Outputs:
- `/tmp/mr_planner_core_demo/solution_raw.csv`
- `/tmp/mr_planner_core_demo/solution.csv`
- `/tmp/mr_planner_core_demo/shortcut_progress.csv` (if `--shortcut-time > 0`)
- `/tmp/mr_planner_core_demo/skillplan.json`
- `/tmp/mr_planner_core_demo/tpg.pb` (unless `--no-tpg`)

### Meshcat visualization (optional)

In a separate terminal, start the Meshcat bridge:

```bash
python3 scripts/visualization/meshcat_bridge.py --port 7600
```

Then run any CLI with `--meshcat`:

```bash
./build/mr_planner_core_plan \
  --vamp-environment dual_gp4 \
  --planner composite_rrt \
  --planning-time 5 \
  --output-dir /tmp/mr_planner_core_demo \
  --meshcat --meshcat-port 7600
```

Inspect portable graphs:

```bash
python3 scripts/io/inspect_graph.py --input /tmp/mr_planner_core_demo/tpg.pb --topo nodes --max-print 10
```

## CLI: `mr_planner_core_skillplan_to_tpg`

Build a `tpg.pb` or `adg.pb` from an input `skillplan.json` (and optionally visualize it with Meshcat):

```bash
./build/mr_planner_core_skillplan_to_tpg \
  --skillplan /tmp/mr_planner_core_demo/skillplan.json \
  --graph-type tpg \
  --vamp-environment dual_gp4 \
  --output-dir /tmp/mr_planner_core_from_skillplan
```

Outputs:
- `/tmp/mr_planner_core_from_skillplan/tpg_from_skillplan.pb` (or `adg_from_skillplan.pb`)
