# VAMP-MR Camera-Ready Release Prep — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make VAMP-MR clean, legally complete, and well-documented for public open-source release with the IROS 2026 camera-ready paper, apply behavior-preserving performance optimizations verified against benchmark parity, and produce a written proposal for upstreaming multi-robot support into VAMP.

**Architecture:** Six sequential phases, each ending in a reviewable commit. Phases 1–3 (legal, cleanup, docs) carry zero behavior risk. Phases 4–5 (performance) are each gated behind a build + benchmark-parity check against a baseline captured in Phase 0. Phase 6 is a standalone documentation deliverable. Work happens on a dedicated branch, never on `main`.

**Tech Stack:** C++17 (mr_planner_core/mr_planner_lego), pybind11, CMake, Eigen, protobuf, TBB, OMPL(optional), Python 3.8; VAMP SIMD collision backend (git submodule).

## Global Constraints

- **License:** Apache-2.0 for VAMP-MR's own code; NOTICE file attributing VAMP (Apache-2.0, modified) and APEX-MR (MIT). Copy license text verbatim from https://www.apache.org/licenses/LICENSE-2.0.txt.
- **Behavior preservation is paramount.** No change in Phases 4–5 may alter planning success, produced paths (modulo RNG with a fixed seed), shortcut quality, or TPG/ADG structure. Every perf change is reverted if the parity gate fails.
- **Do not touch the `vamp/` submodule** except in Phase 6 (and Phase 6 only produces a Markdown doc — no code edits to the submodule in this plan).
- **Never commit to `main`.** All work on branch `release-prep` (create in Phase 0).
- **Never edit files under any `build/` directory.**
- **Preserve app-level (`apps/`) CLI `std::cerr` usage messages and `[error]`/`[warn]` reporting** — those are legitimate user-facing output, not debug breadcrumbs.
- Author breadcrumbs and personal initials in comments (`ruic:`, etc.) must be reworded to neutral technical comments, not merely deleted if they carry real information.

---

## Phase 0: Branch + Working Build Baseline + Benchmark Baseline

**Rationale:** The installed `mr_planner_core` Python module is stale (undefined-symbol ABI mismatch) and no `mr_planner_core/build/` currently exists. Phases 4–5 need a working build and a numeric baseline to compare against. Everything downstream depends on this.

**Files:**
- Create: (none — build artifacts only, all under gitignored `build/`)
- Baseline data: `outputs/baseline/` (gitignored)

**Interfaces:**
- Produces: a working `mr_planner_core/build/` with `_mr_planner_core` importable, and `outputs/baseline/*.csv` benchmark numbers that Phases 4–5 compare against.

- [ ] **Step 1: Create the working branch**

```bash
cd /home/philip/Code/vamp-mr
git checkout -b release-prep
git status   # expect: On branch release-prep, clean
```

- [ ] **Step 2: Configure + build mr_planner_core against the installed VAMP**

VAMP CMake config exists at `/home/philip/.local/lib/python3.8/site-packages/share/cmake/vamp/` and `/home/philip/Code/vamp-mr/build/vamp/`. Point CMake at one of them.

```bash
cmake -S mr_planner_core -B mr_planner_core/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DMR_PLANNER_CORE_ENABLE_PYTHON=ON \
  -DCMAKE_PREFIX_PATH="$HOME/.local/lib/python3.8/site-packages/share;$HOME/Code/vamp-mr/build/install"
cmake --build mr_planner_core/build -j
```
Expected: build completes, produces `mr_planner_core/build/_mr_planner_core*.so`, `mr_planner_core/build/mr_planner_core_plan`.
If OMPL is missing and required, add `-DMR_PLANNER_CORE_ENABLE_VAMP=ON` is already default; OMPL comes via `source /opt/ros/noetic/setup.bash` if the `cbs_prm`/`prm` path needs it. If it still fails, record the exact error and stop for review.

- [ ] **Step 3: Verify the freshly built module imports**

```bash
PYTHONPATH="$PWD/mr_planner_core/build:$PWD/mr_planner_core/python" \
  python3 -c "import mr_planner_core as m; print('OK', m.__file__)"
```
Expected: `OK ...` with no undefined-symbol error. (This resolves the stale-install ABI mismatch by importing the fresh build, not `/usr/local/.../dist-packages`.)

- [ ] **Step 4: Run the python smoke test to confirm end-to-end health**

```bash
bash mr_planner_core/scripts/regression/run_python_smoke.sh 2>&1 | tail -20
```
Expected: smoke test passes (basic planning + skillplan→TPG/ADG checks).

- [ ] **Step 5: Capture the performance baseline (fixed seed) for parity comparison**

Use a short, deterministic configuration so Phases 4–5 can diff. Keep planning-time small but non-trivial; the point is *result parity*, not paper-scale runtime.

```bash
mkdir -p outputs/baseline
# Planning baseline (deterministic seed) — composite_rrt on dual_gp4
PYTHONPATH="$PWD/mr_planner_core/build:$PWD/mr_planner_core/python" \
python3 mr_planner_core/scripts/benchmarks/core_planning_benchmark.py \
  --pose-mode all_pairs --max-poses 4 --planning-time 10 \
  --output-dir outputs/baseline/planning 2>&1 | tail -5
# Shortcut baseline
PYTHONPATH="$PWD/mr_planner_core/build:$PWD/mr_planner_core/python" \
python3 mr_planner_core/scripts/benchmarks/core_shortcut_benchmark.py \
  --planning-dir outputs/baseline/planning --shortcut-time 5 \
  --output-dir outputs/baseline/shortcut 2>&1 | tail -5
# LEGO baseline (one small task, fixed seed) — for TPG/ADG construction parity
PYTHONPATH="$PWD/mr_planner_core/build:$PWD/mr_planner_core/python" \
python3 mr_planner_lego/scripts/benchmarks/lego_cli_benchmark.py \
  --task cliff --seed 1 --planning-time 5 \
  --output-dir outputs/baseline/lego 2>&1 | tail -5
```
Expected: CSVs written under `outputs/baseline/`. Record: per-problem success flags, path costs/lengths, shortcut final costs, LEGO makespan and ADG node/edge counts. These are the parity oracle.

- [ ] **Step 6: Snapshot the baseline numbers into a reference the executor can diff against**

```bash
cp -r outputs/baseline outputs/baseline_frozen
git log --oneline -1   # note the commit; baseline corresponds to pre-change source
```
Note: `outputs/` is gitignored by design — the baseline is a local oracle, not committed. Do NOT commit it.

**Phase 0 checkpoint:** Working build + green smoke test + frozen baseline numbers. STOP for review before Phase 1.

---

## Phase 1: Release Hygiene & Legal (zero behavior risk)

**Files:**
- Create: `LICENSE` (Apache-2.0 text)
- Create: `NOTICE`
- Create: `THIRD_PARTY_LICENSES.md`
- Create: `CITATION.cff`
- Modify: `README.md` (add Dependencies section + License section)
- Modify (headers): the 6 vendored APEX-MR files listed below (add attribution header)
- Delete (untrack): `mr_planner_core/scripts/benchmarks/__pycache__/*.pyc`

**Interfaces:**
- Produces: a legally-complete top-level license posture that Phases 2–6 do not alter.

- [ ] **Step 1: Add the root Apache-2.0 LICENSE**

Download the canonical text and write it verbatim to `LICENSE`:
```bash
curl -fsSL https://www.apache.org/licenses/LICENSE-2.0.txt -o LICENSE
head -3 LICENSE   # expect the Apache License, Version 2.0 header
```
If no network, paste the standard Apache-2.0 text (identical wording) into `LICENSE`.

- [ ] **Step 2: Create NOTICE stating bundled works and modifications**

Create `NOTICE`:
```
VAMP-MR
Copyright 2026 Philip Huang, Chenrui Gao, Jiaoyang Li (Carnegie Mellon University; University of Michigan)

This product includes software developed as part of VAMP-MR, released under the Apache License, Version 2.0.

This product bundles the following third-party software:

1. VAMP (Vector-Accelerated Motion Planning) — https://github.com/KavrakiLab/vamp
   Licensed under the Apache License, Version 2.0.
   VAMP-MR includes a MODIFIED version of VAMP (see the `vamp/` submodule, branch `vamp-mr`),
   adding multi-robot composite collision checking. Modifications are described in the VAMP-MR paper
   and in docs/upstream/vamp-multirobot-proposal.md.

2. APEX-MR — https://github.com/intelligent-control-lab/APEX-MR
   Licensed under the MIT License.
   Portions of `mr_planner_lego/` (the LEGO assembly manipulation code) are derived from APEX-MR.
   See THIRD_PARTY_LICENSES.md for the full MIT license text.
```

- [ ] **Step 3: Create THIRD_PARTY_LICENSES.md with the APEX-MR MIT text**

Create `THIRD_PARTY_LICENSES.md` containing: (a) a short intro, (b) the full APEX-MR MIT license text (fetch from https://raw.githubusercontent.com/intelligent-control-lab/APEX-MR/main/LICENSE — verify the copyright line), (c) a pointer to `vamp/LICENSE.txt` for Apache-2.0. If APEX-MR's own LEGO code chains to a further upstream (check APEX-MR's NOTICE/README), record that chain here too.

- [ ] **Step 4: Add attribution headers to the 6 vendored APEX-MR files**

Prepend a comment header to each of:
- `mr_planner_lego/include/mr_planner/applications/lego/lego/Lego.hpp`
- `mr_planner_lego/include/mr_planner/applications/lego/lego/Utils/Common.hpp`
- `mr_planner_lego/include/mr_planner/applications/lego/lego/Utils/ErrorHandling.hpp`
- `mr_planner_lego/include/mr_planner/applications/lego/lego/Utils/FileIO.hpp`
- `mr_planner_lego/include/mr_planner/applications/lego/lego/Utils/Math.hpp`
- `mr_planner_lego/src/applications/lego/core/Lego.cpp`

Header text (adjust copyright holder to match APEX-MR's actual LICENSE copyright line):
```cpp
// This file is derived from APEX-MR (https://github.com/intelligent-control-lab/APEX-MR),
// licensed under the MIT License. See THIRD_PARTY_LICENSES.md for the full license text.
// Modifications for VAMP-MR: ROS-free integration with the mr_planner_core planning engine.
```

- [ ] **Step 5: Remove tracked Python bytecode**

```bash
git rm --cached mr_planner_core/scripts/benchmarks/__pycache__/core_planning_benchmark.cpython-38.pyc \
                mr_planner_core/scripts/benchmarks/__pycache__/core_shortcut_benchmark.cpython-38.pyc
git ls-files | grep -c '\.pyc$'   # expect 0
```

- [ ] **Step 6: Add a Dependencies section to README.md**

Insert, before "Local Install Script", an apt dependency block mirroring `docker/Dockerfile` (read that file for the exact package set; it currently includes eigen3, boost-all-dev, jsoncpp, ompl, protobuf, tbb, yaml-cpp, ninja-build, protobuf-compiler, python3-dev). Example:
```markdown
### System dependencies (Ubuntu 20.04)

```bash
sudo apt-get update && sudo apt-get install -y \
  build-essential cmake ninja-build git \
  libeigen3-dev libboost-all-dev libjsoncpp-dev libompl-dev \
  libprotobuf-dev protobuf-compiler libtbb-dev libyaml-cpp-dev \
  python3-dev python3-pip
```
```
Verify the list against `docker/Dockerfile` so README and Dockerfile agree.

- [ ] **Step 7: Add a License section to README.md**

Append a short License section: VAMP-MR is Apache-2.0; bundles modified VAMP (Apache-2.0) and APEX-MR-derived LEGO code (MIT); see `LICENSE`, `NOTICE`, `THIRD_PARTY_LICENSES.md`.

- [ ] **Step 8: Add CITATION.cff**

Create `CITATION.cff` mirroring the BibTeX in README.md (title, authors Huang/Gao/Li, IROS 2026, URL to project page). Validate the YAML parses:
```bash
python3 -c "import yaml,sys; yaml.safe_load(open('CITATION.cff')); print('CITATION.cff OK')"
```

- [ ] **Step 9: Commit Phase 1**

```bash
git add LICENSE NOTICE THIRD_PARTY_LICENSES.md CITATION.cff README.md \
        mr_planner_lego/include mr_planner_lego/src
git rm --cached mr_planner_core/scripts/benchmarks/__pycache__/*.pyc 2>/dev/null; true
git commit -m "docs: add Apache-2.0 license, third-party attribution, deps + citation for release"
```

**Phase 1 checkpoint:** Legal posture complete. STOP for review.

---

## Phase 2: Code Cleanup — debug prints, commented-out code, TODOs (behavior-preserving)

**Rationale:** A `Logger`/`log()` facility exists (`mr_planner_core/include/mr_planner/core/logger.h`). Raw `std::cout`/`std::cerr` in library code are breadcrumbs; some are pure noise (delete), some report real conditions (route through `log()`). Commented-out code blocks are dead weight (git preserves history).

**Files (all Modify):** enumerated per step below. **Verification:** rebuild must still compile and the smoke test must still pass — no behavior change.

**Interfaces:**
- Consumes: `log(msg, LogLevel::DEBUG|INFO|WARN|ERROR)` from `core/logger.h`.
- Produces: library code free of raw debug I/O and dead commented blocks.

- [ ] **Step 1: Delete pure debug prints**

Delete these raw print statements (they carry no user value):
- `mr_planner_core/src/planning/roadmap.cpp`: lines 237, 256, 268, 270, 272, 274, 286, 291, 297, 349, 354, 497, 587 (representative/shortcut trace prints incl. `"Yes!\n"`, `"Unfortunately no.\n"`)
- `mr_planner_core/src/execution/adg.cpp`: 212, 216, 582
- `mr_planner_core/src/planning/SingleAgentPlanner.cpp`: 19, 22, 127, 129 (raw joint-value dumps)
- `mr_planner_core/src/planning/resampler.cpp`: 62, 85
- `mr_planner_lego/src/applications/lego/core/Lego.cpp`: the ~25 trace prints incl. lines 56, 76, 579, 628, 629, 633, 638, 643 (DH-matrix dumps, "start vcat"/"finished vcat")

Re-read each line before deleting to confirm it is a bare print with no side effects in its arguments (e.g. `std::cout << x` where evaluating `x` is pure). If a print's argument has a side effect, keep the side effect.

- [ ] **Step 2: Route legitimate condition reports through `log()`**

Convert these to `log(...)` at the indicated level (they report real conditions):
- `mr_planner_core/src/planning/planner.cpp`: 441, 453 (INFO — "Applying dense roadmap"), 1637 (INFO — "Loaded N points")
- `mr_planner_core/src/planning/prm.cpp`: 58, 61, 114 (WARN/ERROR — init failure/collision), 515
- `mr_planner_core/src/planning/rrt.cpp`: 31, 40, 46, 79, 95 (INFO/WARN — init failure, time-limit, solution found)
- `mr_planner_core/include/mr_planner/planning/voxel_grid.h`: 82, 87, 107 (WARN — OOB / not-registered; header-inlined)
- `mr_planner_core/include/mr_planner/backends/vamp_instance.h`: 2054 (WARN — attachment collision blocked), 4258, 4307, 4312, 4435, 4441 (`[meshcat]` connect/send — DEBUG)
- `mr_planner_lego/src/applications/lego/core/Lego.cpp`: 494, 1411, 1547, 1566, 1776, 1786 (WARN/ERROR — IK failure, unknown/no-available brick)

For the `vamp_instance.h` state-dump block at 3866–3970 (`[VAMP] Robots/Movable/Attached`): keep it but gate it behind an existing verbosity/log level rather than unconditional `std::cout` (route through `log(..., LogLevel::DEBUG)`).

- [ ] **Step 3: Delete commented-out code blocks**

Remove the dead commented-out code (not doc comments) at:
- `mr_planner_core/src/planning/planner.cpp`: 319–367, 468–473, 610–648, 725–770
- `mr_planner_core/src/execution/tpg.cpp`: 56–81, 128–129, 737–766, 1226–1250, 1332
- `mr_planner_core/src/planning/prm.cpp`: the ~55 commented code lines
- `mr_planner_core/src/execution/adg.cpp`: the ~22 commented lines
- `mr_planner_core/src/planning/roadmap.cpp`: 56, 132, 135, 139, 318, 580, 583
- `mr_planner_core/src/planning/shortcutter_mt.cpp`: the ~6 lines; `shortcutter.cpp:780`
- `mr_planner_core/include/mr_planner/planning/voxel_grid.h:104`

Read each block first; if any commented block documents *why* the code below exists (rationale), convert it to a concise real comment instead of deleting.

- [ ] **Step 4: Resolve/annotate TODOs and breadcrumbs**

- `mr_planner_core/src/execution/tpg.cpp:1856` — `// TODO: fix when type2 edges are inconsistent`: investigate; if not fixable now, reword to a precise known-limitation comment (no `TODO`).
- `mr_planner_lego/src/applications/lego/lego_primitive.cpp:54,786,965` — the three "ROS-free stability checker" TODOs: since the feature is disabled (line 54 logs it), either (a) document as a known limitation in `mr_planner_lego/README.md` and reword the comments, or (b) remove the dead scaffolding. Prefer (a) to avoid behavior change.
- `mr_planner_lego/include/mr_planner/applications/lego/lego/Utils/Math.hpp:14` — reword `#define N_JOINTS 6 // ruic: for now 6 joints` → `// assumes 6-DOF arms`.
- `mr_planner_core/include/mr_planner/planning/planner.h:32` and `SingleAgentPlanner.h:92` — tidy the identical "For simplicity... for now" design-musing comments.
- `mr_planner_core/src/execution/tpg.cpp:2264` — verify `// Remove this edge temporarily` accurately describes the code; fix or clarify.

- [ ] **Step 5: Rebuild and verify compilation**

```bash
cmake --build mr_planner_core/build -j 2>&1 | tail -15
cmake --build mr_planner_lego/build -j 2>&1 | tail -15
```
Expected: both build clean. If a deleted print removed a variable's only use and triggers `-Wunused`, remove the now-unused variable too (confirm it has no side effect).

- [ ] **Step 6: Verify no behavior change via smoke test**

```bash
bash mr_planner_core/scripts/regression/run_python_smoke.sh 2>&1 | tail -20
```
Expected: still passes.

- [ ] **Step 7: Confirm the cleanup is complete via grep**

```bash
grep -rnE 'std::cout|std::cerr' mr_planner_core/src mr_planner_core/include mr_planner_lego/src mr_planner_lego/include \
  | grep -v 'logger.cpp' | grep -v '/apps/' | wc -l
grep -rnE 'TODO|FIXME|XXX|HACK' mr_planner_core mr_planner_lego --include=*.cpp --include=*.h --include=*.hpp | grep -v '/build/'
```
Expected: the first count is near zero (only intentional keeps); the second lists only resolved/annotated items.

- [ ] **Step 8: Commit Phase 2**

```bash
git add -A
git commit -m "refactor: route library debug output through Logger, remove dead commented code and stale TODOs"
```

**Phase 2 checkpoint:** STOP for review.

---

## Phase 3: Documentation — header purpose lines, API docs, Python docstrings (behavior-preserving)

**Rationale:** 39 of 40 C++ headers have zero doc comments and no top-of-file purpose line. `srdf.py` (an installed module) and `__init__.py` have no docstrings. The pybind file (`mr_planner_core_pybind.cpp`) is well-documented and is the style template.

**Files (all Modify — comments/docstrings only):** the headers and Python modules below.

**Interfaces:** none changed — comments only. Verification: still compiles; Python imports.

- [ ] **Step 1: Add top-of-file purpose comments to every core/lego header**

For each header under `mr_planner_core/include/mr_planner/**` and `mr_planner_lego/include/mr_planner/**`, add a 1–3 line `//` file-purpose banner at the top describing what the file provides. Keep it factual and short.

- [ ] **Step 2: Document the highest-value public headers (class + public method docs)**

Add Doxygen-style (`///` or `/** */`) doc comments to public classes and public methods, in priority order:
1. `mr_planner_core/include/mr_planner/backends/vamp_instance.h` — the central `VampInstance<RobotTs...>` environment/collision class (4505 lines): document the class, the template parameters, and the key public methods (`checkCollision`, `checkMultiRobotMotion`, `connect`, subset-dispatch entry points, attachment/meshcat helpers). This is the biggest surface; do it thoroughly.
2. `mr_planner_core/include/mr_planner/execution/tpg.h` (437) and `core/instance.h` (406).
3. `mr_planner_core/include/mr_planner/planning/planner.h`, `roadmap.h`, `shortcutter.h`, `pose_hash.h`, `nn_kdtree.h`.
4. `mr_planner_core/include/mr_planner/core/graph.h`, `task.h`, `execution/adg.h`, `execution/policy.h`.
5. One-line purpose + key-type docs for `backends/{vamp_plugin_api.h,vamp_plugin_loader.h,vamp_env_factory.h,vamp_presets.h}`, `common/eigen_config.h`, `core/metrics.h`, `io/{graph_proto.h,skillplan.h}`.

Match the description style used in `mr_planner_core_pybind.cpp`.

- [ ] **Step 3: Add Python docstrings**

- `mr_planner_core/python/mr_planner_core/srdf.py` — module docstring + docstrings for all 7 functions and 2 classes (highest priority: it's an installed library module).
- `mr_planner_core/python/mr_planner_core/__init__.py` — module docstring describing the package.
- Add module docstrings (1–3 lines) to the benchmark/tooling scripts that lack them: `scripts/benchmarks/{core_planning_benchmark,core_shortcut_benchmark,vamp_collision_benchmark}.py`, `scripts/io/{inspect_graph,graph_stats}.py`, `scripts/planning/{plan_named_poses,shortcut_solution_csv,skillplan_playback}.py`, `scripts/plugins/generate_vamp_robot_plugin.py`, `scripts/regression/python_smoke.py`, `mr_planner_lego/scripts/task_assignment.py`, `mr_planner_lego/scripts/benchmarks/lego_cli_benchmark.py`.

- [ ] **Step 4: Verify compile + import**

```bash
cmake --build mr_planner_core/build -j 2>&1 | tail -5
python3 -c "import ast; ast.parse(open('mr_planner_core/python/mr_planner_core/srdf.py').read()); print('srdf.py parses')"
```
Expected: clean build; srdf.py parses.

- [ ] **Step 5: Commit Phase 3**

```bash
git add -A
git commit -m "docs: add module/class/method documentation across core + lego headers and Python modules"
```

**Phase 3 checkpoint:** STOP for review.

---

## Phase 4: Performance — Tier-1 SAFE optimizations (behavior-identical, benchmark-gated)

**Rationale:** Behavior-identical wins in genuine hot loops. Each is applied, then the full parity gate (Step 8) runs; any result deviation reverts that change.

**Files (Modify):** `rrt_connect.cpp`, `rrt.cpp`, `include/.../vamp_instance.h`, `sipp_rrt.cpp`, `prm.cpp`, `shortcutter.cpp`.

**Interfaces:** no signature changes; internal-only edits.

- [ ] **Step 1: Remove the vacuous full-tree `std::find` scans in RRT-Connect / RRT**

At `mr_planner_core/src/planning/rrt_connect.cpp:426-427` and `:432-433`, the `validateMotion` collision test is guarded by `std::find(start_tree..., b)==end() && std::find(goal_tree..., b)==end()`. `b` is always freshly `make_shared`'d in `steer()` (call sites 126,168,179,189), so both `std::find`s always return `end()` — pure O(n) waste per iteration that negates the kd-tree. **First confirm** by reading `steer()` and both branches that `b` is never an existing tree node; then delete the two `std::find(...)==end()` conjuncts, keeping the `validateMotion(...)` call. Apply the same removal at `rrt.cpp:179`.

- [ ] **Step 2: Stack-allocate `active` in `checkCollision`**

At `mr_planner_core/include/mr_planner/backends/vamp_instance.h:2295-2303`, replace the per-call heap `std::vector<std::size_t> active; active.reserve(kRobotCount);` with `std::array<std::size_t, kRobotCount> active; std::size_t n_active = 0;` and track count inline. `kRobotCount` is a compile-time constant. Preserve the `size()==kRobotCount` fast-path (compare `n_active`). This is the single hottest op in the system.

- [ ] **Step 3: Reusable 2-element scratch buffer in SIPP**

At `mr_planner_core/src/planning/sipp_rrt.cpp:307,319,377,393`, the code allocates a fresh `{pose, obs}` / `{pose}` vector per obstacle per timestep. Add a member `std::vector<RobotPose> scratch2_` (sized 2) and assign into it (`scratch2_.assign({...})` or index-assign) instead of constructing a new vector each iteration. SIPP is single-threaded, so a single reused member is safe.

- [ ] **Step 4: `const auto&` in `checkConstraint` loops**

At `rrt.cpp:190`, `rrt_connect.cpp:441`, `prm.cpp:787`, change `for (auto constraint : options.constraints)` → `for (const auto &constraint : options.constraints)` (a `Constraint` holds `RobotPose`s with strings/vectors).

- [ ] **Step 5: Stop copying the whole vertex vector in PRM**

At `mr_planner_core/src/planning/prm.cpp:289-290`, replace `neighbors.clear(); neighbors = roadmap_->vertices;` with `const auto &neighbors = roadmap_->vertices;` (iterate directly). Confirm the sample was already added at :288 and `validateMotion(sample, sample)` returns false so behavior is unchanged.

- [ ] **Step 6: Hoist loop-invariant virtual calls in the shortcutter**

At `mr_planner_core/src/planning/shortcutter.cpp` (non-ROS path lines 850,873,1082,1127,1160,1168; ROS-variant 84,88,111,116,141,249,288), hoist `instance_->getNumberOfRobots()` and `getRobotDOF(i)` to locals computed once (member `num_robots_` is already set at :714). Precompute per-robot DOFs into a local array before the loops.

- [ ] **Step 7: Rebuild**

```bash
cmake --build mr_planner_core/build -j 2>&1 | tail -10
```
Expected: clean build.

- [ ] **Step 8: Parity gate — re-run the Phase 0 baseline and diff results**

```bash
mkdir -p outputs/tier1
PYTHONPATH="$PWD/mr_planner_core/build:$PWD/mr_planner_core/python" \
python3 mr_planner_core/scripts/benchmarks/core_planning_benchmark.py \
  --pose-mode all_pairs --max-poses 4 --planning-time 10 --output-dir outputs/tier1/planning 2>&1 | tail -5
PYTHONPATH="$PWD/mr_planner_core/build:$PWD/mr_planner_core/python" \
python3 mr_planner_core/scripts/benchmarks/core_shortcut_benchmark.py \
  --planning-dir outputs/tier1/planning --shortcut-time 5 --output-dir outputs/tier1/shortcut 2>&1 | tail -5
PYTHONPATH="$PWD/mr_planner_core/build:$PWD/mr_planner_core/python" \
python3 mr_planner_lego/scripts/benchmarks/lego_cli_benchmark.py \
  --task cliff --seed 1 --planning-time 5 --output-dir outputs/tier1/lego 2>&1 | tail -5
# Compare success flags, path costs, shortcut final costs, ADG node/edge counts vs outputs/baseline_frozen
```
**Parity criterion:** planning success flags identical; produced path costs/lengths identical for the same seed; shortcut final costs identical; LEGO ADG node/edge counts and makespan identical. Runtime is *expected to improve* — that is the goal and is not a parity failure. If any *result* (not timing) differs, bisect which of Steps 1–6 caused it and revert that one (RNG-consuming code paths must remain byte-identical; the `std::find` removal in Step 1 must not change how many RNG draws occur — verify it doesn't).

- [ ] **Step 9: Commit Phase 4**

```bash
git add mr_planner_core/src mr_planner_core/include
git commit -m "perf: remove vacuous tree scans and hot-loop allocations (Tier-1, behavior-identical, benchmark-verified)"
```

**Phase 4 checkpoint:** report before/after runtimes and confirmed result parity. STOP for review.

---

## Phase 5: Performance — Tier-2 gated optimizations (behavior-preserving, each individually gated)

**Rationale:** Larger structural wins that preserve results but warrant per-change benchmark verification. Apply **one at a time**, gating each behind the parity check before moving on.

**Files (Modify):** `core/instance.cpp`, `include/.../vamp_instance.h`, `execution/tpg.cpp`.

- [ ] **Step 1: Replace `unordered_map` pairing with linear matching in `checkMultiRobotMotion`**

At `mr_planner_core/src/core/instance.cpp:154-158`, the start↔goal pairing builds an `unordered_map<int,RobotPose>` (heap + pose copies) inside every `connect()`. Replace with the same linear `std::find_if` by `robot_id` already used in `computeMotionStepSize` (instance.cpp:122-127). Duplicate `robot_id`s already throw in `gatherPoses`, so no dedup is lost. Rebuild, then run the parity gate (Phase 4 Step 8 commands into `outputs/tier2a/`). Revert if any result differs.

- [ ] **Step 2: Single-robot fast path in `connect`**

At `mr_planner_core/include/mr_planner/backends/vamp_instance.h:2990`, `connect` currently does `!checkMultiRobotMotion({a},{b},step_size,self)`, allocating single-element vectors and deep-copying poses per RRT edge. Add a single-robot fast path that interpolates the one robot and calls `checkCollision` on a reused scratch buffer, reproducing the **exact** step count/interpolation of `checkMultiRobotMotion` (read `computeMotionStepSize` and the interpolation loop to match rounding and endpoint handling precisely). Rebuild, parity gate into `outputs/tier2b/`. This one is the highest behavior-risk item — if step counts differ by even one, collision sampling changes; revert on any parity deviation.

- [ ] **Step 3: Thread-local scratch buffer in TPG `findCollisionDeps`**

At `mr_planner_core/src/execution/tpg.cpp:518` (and the parallel variant ~:632), `checkCollision({node_i->pose, node_j->pose}, true)` allocates a 2-element vector per node pair in the O(N_i·N_j) type-2-edge loop. Use a reusable 2-element buffer. **Critical:** the parallel variant runs multi-threaded — the buffer MUST be `thread_local` (or per-task local), never shared across threads. Rebuild, parity gate into `outputs/tier2c/`, and additionally re-run the LEGO ADG/TPG construction (which exercises this path) to confirm identical node/edge counts under both single- and multi-threaded construction.

- [ ] **Step 4: Commit Phase 5**

```bash
git add mr_planner_core/src mr_planner_core/include
git commit -m "perf: linear pairing, single-robot connect fast-path, TPG thread-local scratch (Tier-2, each benchmark-verified)"
```

**Phase 5 checkpoint:** report per-change before/after runtimes and confirmed parity for all three. Note any change that was reverted for failing parity. STOP for review.

---

## Phase 6: Upstream Proposal Document

**Rationale:** Deliver a written technical proposal for VAMP's maintainers describing how the fork's multi-robot support could become a generic upstream template feature with hooks for CBS/TPG/ADG. No code changes to `vamp/`.

**Files:**
- Create: `docs/upstream/vamp-multirobot-proposal.md`

- [ ] **Step 1: Write the proposal**

Structure:
1. **Motivation** — transparent multi-robot planning from user-supplied robots, with low-level hooks so CBS/TPG/ADG can be built on top without forking VAMP.
2. **What already works and is additive** — `collision/multi_robot.hh` composes independently-compiled per-robot sphere models as a variadic `std::tuple` (`MultiRobotState<RobotT, rake>` = config + base_transform + attachment), does compile-time recursive pairwise SIMD sphere sweeps, and builds *only* on VAMP's existing per-robot kernel contract (`sphere_fk`, `fkcc`, `fkcc_attach`, `fkcc_debug` — all already upstream). No changes to VAMP codegen or per-robot kernels. There is no fused multi-robot URDF.
3. **Proposed upstream API surface** — the `fkcc_multi_{all,self,cross}[_attach]` variadic templates, `MultiRobotState`, `MultiRobotCollisionFilter` (runtime ACM via named allow-lists), the `register_robot<Robot>()` runtime registry (advanced hook), and a NEW first-class `MultiRobotTeam` helper exposing `check_pair(i,j)` / `check_subset(active)` so downstreams stop hand-rolling the O(N²) function-pointer dispatch tables that `mr_planner_core`'s `VampInstance` currently precomputes.
4. **The two hard problems upstream must solve:** (a) the compile-time↔runtime bridge for arbitrary N and CBS subset queries; (b) heterogeneous teams + `LinkMapping` emitted by cricket codegen (currently `link_mapping.hh` is hand-maintained per robot).
5. **Friction points ranked** — reproduce the ranked list: compile-time-only robot set; homogeneous-only codegen (`generate_vamp_robot_plugin.py` emits `VampInstance<Robot×N>`); hand-maintained `LinkMapping`; coupling to `mr_planner` types / plugin ABI (`vamp_plugin_api.h`) which stays downstream; the `thread_local EnvironmentTransformCache` hidden state (make opt-in; note it strips heightfields/pointclouds from the transformed copy); the GP4 robot + reformatted `panda.hh` diff noise (contribute robots via the normal process).
6. **What stays downstream** — `VampInstance`, plugin ABI, cricket/foam scripts, hardcoded presets (the reference consumer).
7. **Staged upstreaming plan** — (i) land the additive collision core behind a `VAMP_ENABLE_MULTI_ROBOT` option; (ii) add `MultiRobotTeam` subset dispatch; (iii) move `LinkMapping` into cricket codegen; (iv) heterogeneous codegen; (v) split debug/contact plumbing into a separate header.

Reference exact file paths in the `vamp/` submodule for each claim so a VAMP maintainer can navigate directly.

- [ ] **Step 2: Commit Phase 6**

```bash
git add docs/upstream/vamp-multirobot-proposal.md
git commit -m "docs: add proposal for upstreaming multi-robot collision support into VAMP"
```

**Phase 6 checkpoint:** proposal ready to share with VAMP maintainers. STOP for review.

---

## Final Steps

- [ ] **Verify the full build + smoke once more on the final tree**

```bash
cmake --build mr_planner_core/build -j && cmake --build mr_planner_lego/build -j
bash mr_planner_core/scripts/regression/run_python_smoke.sh 2>&1 | tail -20
```

- [ ] **Summarize before/after benchmark numbers** for the user (planning, shortcutting, LEGO TPG/ADG), confirming result parity and reporting speedups.

- [ ] **Present branch `release-prep` for the user to review and merge** (do not merge to `main` without explicit approval).

## Self-Review Notes

- **Spec coverage:** All four user asks are covered — release readiness (Phase 1), cleanup/comments/docs/README (Phases 1–3), behavior-preserving perf across planning/shortcutting/LEGO+TPG (Phases 4–5, both Tier-1 SAFE and Tier-2 gated per the user's "Safe + Tier-2" choice), and the upstream-merge analysis as a written proposal (Phase 6). License = Apache-2.0 per the user's choice.
- **The `docs/` ~628MB video bloat** is intentionally NOT actioned here (it requires history rewriting); it is flagged to the user separately as a decision, not folded into this plan.
- **Parity oracle:** Phase 0 freezes baseline numbers; every perf change diffs against them. Runtime improvement is the goal; *result* changes fail the gate and revert.
