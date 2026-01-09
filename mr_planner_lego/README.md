# mr_planner_lego

Standalone (non-ROS) LEGO “application” built on top of `mr_planner_core`. It provides ROS-free CLI tools for:
- LEGO task assignment (generate steps CSV/JSON)
- LEGO planning (VAMP collision checking + multi-robot planning + ADG construction + portable export)

This build sets `MR_PLANNER_WITH_ROS=0` and does not link against ROS / MoveIt.

## Build

In this mono-repo checkout, `mr_planner_lego` defaults to building against the bundled `mr_planner_core`.

```bash
cmake -S mr_planner_lego -B mr_planner_lego/build -DCMAKE_BUILD_TYPE=Release
cmake --build mr_planner_lego/build -j
```

To build against an installed `mr_planner_core` (recommended for a split repo release), configure with:

```bash
cmake -S mr_planner_lego -B mr_planner_lego/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DMR_PLANNER_LEGO_USE_BUNDLED_CORE=OFF
```

Consumer CMake:

```cmake
find_package(mr_planner_lego CONFIG REQUIRED)
target_link_libraries(my_target PRIVATE mr_planner::mr_planner_lego)
```

## CLI

Both CLIs need access to the repo’s `config/` files. Either run them from the `mr_planner` repo root, or pass `--root /path/to/mr_planner`.

### `mr_planner_lego_assign`

Generates step files from `config/lego_tasks/assembly_tasks/<task>.json`:

```bash
./build/mr_planner_lego/mr_planner_lego_assign \
  --task test \
  --root /path/to/mr_planner \
  --output-dir /tmp/mr_steps \
  --vamp-environment dual_gp4
```

Outputs:
- `/tmp/mr_steps/test_seq.json`
- `/tmp/mr_steps/test_steps.csv`
- `/tmp/mr_steps/robots_manifest.json`

### `mr_planner_lego_plan`

Consumes `<task>_steps.csv` and exports a portable ADG protobuf + `skillplan.json`:

```bash
./build/mr_planner_lego/mr_planner_lego_plan \
  --task test \
  --root /path/to/mr_planner \
  --steps-dir /tmp/mr_steps \
  --output-dir /tmp/mr_lego_out \
  --vamp-environment dual_gp4 \
  --planning-time 5 \
  --seed 1
```

Outputs:
- `/tmp/mr_lego_out/adg.pb`
- `/tmp/mr_lego_out/skillplan.json`

### Meshcat visualization (optional)

In a separate terminal (from the `mr_planner` repo root), start the Meshcat bridge:

```bash
python3 mr_planner_core/scripts/visualization/meshcat_bridge.py --port 7600
```

Then add `--meshcat` to the CLIs:

```bash
./build/mr_planner_lego/mr_planner_lego_plan \
  --task test \
  --root /path/to/mr_planner \
  --steps-dir /tmp/mr_steps \
  --output-dir /tmp/mr_lego_out \
  --vamp-environment dual_gp4 \
  --planning-time 5 \
  --seed 1 \
  --meshcat --meshcat-port 7600
```

Inspect the protobuf (from the `mr_planner` repo root):

```bash
python3 mr_planner_core/scripts/io/inspect_graph.py --input /tmp/mr_lego_out/adg.pb --topo activities --max-print 10
```

## Notes

- The ROS-free build currently disables stability checking (TODO: replace the ROS service with a non-ROS client).
