# Proposal: First-Class Multi-Robot Collision Checking in VAMP

**Status:** Draft for discussion with VAMP maintainers
**Authors:** VAMP-MR team (Philip Huang, Chenrui Gao, Jiaoyang Li)
**Scope:** Upstreaming the multi-robot composite collision layer developed for VAMP-MR (IROS 2026) into VAMP proper.

---

## 1. Motivation

VAMP delivers state-of-the-art single-robot collision checking by compiling a
robot's forward-kinematics + sphere model into SIMD kernels (`fkcc`,
`fkcc_attach`, `fkcc_debug`). VAMP-MR extends this to **teams of manipulators**:
given N robots (each an independently compiled VAMP robot) and their base
transforms, it answers self / environment / and **inter-robot** collision
queries, still fully SIMD, and exposes enough structure for higher-level
multi-robot algorithms — Conflict-Based Search (CBS), Temporal Plan Graphs
(TPG), and Action Dependency Graphs (ADG) — to be built on top.

The goal of this proposal is a design where a VAMP user can, transparently:

1. **Plan for a multi-robot team** by supplying robots (ideally from a
   multi-robot URDF / per-robot URDFs + base transforms) with robot-robot
   collision checks compiled in, and
2. **Build advanced multi-robot features** (CBS/TPG/ADG) on stable low-level
   hooks, without forking VAMP.

The key encouraging finding: **the multi-robot layer is purely additive.** It
builds only on VAMP's *existing* per-robot kernel contract and does not modify
VAMP's codegen or per-robot kernels.

---

## 2. What already exists in the fork, and why it is clean

The fork (`vamp-mr` branch) adds multi-robot support primarily in one header:

- `vamp/src/impl/vamp/collision/multi_robot.hh` — the composite collision engine.
- `vamp/src/impl/vamp/bindings/multi_robot_runtime.hh`, `bindings/multi_robot.cc` — a runtime robot registry + Python bindings.
- `vamp/src/impl/vamp/robots/link_mapping.hh` — per-robot sphere→link-name mapping (for named/ACM filtering).

Supporting edits: `collision/attachments.hh` (posed-attachment spheres),
`collision/shapes.hh`, and binding registration hooks
(`bindings/{init.hh.in, python.cc.in, robot.cc.in}`) that call
`register_robot<Robot>()`.

### 2.1 There is no fused multi-robot URDF

Crucially, VAMP-MR does **not** compile a single fused N-robot kinematic model.
Each robot keeps its own independently compiled sphere model; robots are
**composed at the collision-check call site**. The unit of composition is:

```cpp
template <typename RobotT, std::size_t rake>
struct MultiRobotState {
    using RobotType = RobotT;
    typename RobotType::template ConfigurationBlock<rake> configuration;
    Eigen::Isometry3f base_transform{Eigen::Isometry3f::Identity()};
    const collision::Attachment<float>* attachment = nullptr;
};
```

The public API is a variadic pack of heterogeneous robot states:

```cpp
template <std::size_t rake, typename... States>
auto fkcc_multi_all(const Environment<FloatVector<rake>>& env, const States&... states) -> bool;
// + an overload taking a MultiRobotCollisionFilter&
// siblings: fkcc_multi_self (no env), fkcc_multi_cross (SIMD lane-shifted whole-edge sweep),
//           debug_fkcc_multi_all (named-collision reporting)
```

Per-robot pipeline (`detail::process_robot_state`):
1. transform the shared environment into the robot's base frame (cached per
   transform in a `thread_local EnvironmentTransformCache`),
2. run the robot's own compiled `fkcc` / `fkcc_attach` for self + environment,
3. `sphere_fk` → `apply_transform` (SIMD) to place spheres in the world frame,
4. inter-robot checks then run over the tuple of world-frame sphere sets via a
   compile-time recursive pairwise sweep (`spheres_cross_variant<I>` →
   `spheres_cross_pair_variant<I,J>`), doing brute-force `sphere_sphere_sql2`
   across all sphere pairs, staying fully SIMD (each coordinate is a
   `FloatVector<rake>`). Attachments get their own cross products.

N robots + base transforms are combined purely as a `std::tuple` of per-robot
`Spheres`, never a fused entity. This is why the layer is additive: it only
requires the per-robot `sphere_fk`, `fkcc`, `fkcc_attach`, and `fkcc_debug`
statics that upstream VAMP already ships.

### 2.2 Runtime registry for composition without compile-time types

Python and dlopen'd plugins do not know robot types at compile time, so each
robot type registers a type-erased entry:

```cpp
struct RegistryEntry {
    std::string name;
    PrepareFn fn;         // &prepare_state<Robot>
    DebugFn   debug_fn;
    std::size_t dimension, sphere_count;
    LinkLookupFn link_lookup;
};
register_robot<Robot>();  // keyed by Robot::name
```

`prepare_state<Robot>` runs the same per-robot pipeline and emits world-frame
`std::vector<Sphere<float>>` (scalar, lane 0). The Python `fkcc_multi_all` takes
a list of `{robot, configuration, transform, attachment, label}` dicts and does
an all-pairs scalar sweep. This path is the "advanced hook" for
composition/inspection; it is not SIMD across the rake (it is a debug / single-
query path), whereas the templated path is the fast one.

### 2.3 Runtime allowed-collisions (ACM/SRDF equivalent)

`MultiRobotCollisionFilter` holds per-robot, per-sphere allow-lists of object
names (plus attachment allow-lists, `"*"` wildcard). `collisions_allowed()`
re-runs `fkcc_debug` to obtain named collisions and checks each against the
allow-list — this is how SRDF-style allowed collisions are honored at runtime.

---

## 3. How a downstream (mr_planner_core) consumes this today

`mr_planner_core` wraps the compile-time API in its own variadic class
`VampInstance<RobotTs...>` (`mr_planner_core/include/mr_planner/backends/vamp_instance.h`).
It stores `std::array<Eigen::Isometry3f, kRobotCount> base_transforms_`, builds
`MultiRobotState`s, and calls `fkcc_multi_{self,all,cross}`.

**The compile-time↔runtime bridge for CBS.** CBS/TPG only ever check one robot
or a pair at a time. Because the fast path needs the robot pack at compile time,
`VampInstance` precomputes O(N²) dispatch tables — arrays of function pointers to
`subsetCollisionImpl<Index>` / `subsetCollisionImpl<A,B>` instantiations — and a
runtime "active robot" list indexes into them (`subsetCollisionSwitch`). **This
hand-rolled bridge is the single most important thing that should become a
first-class VAMP facility** (see §4), so that every downstream doesn't reinvent
it.

**Bring-your-own-robot, today:** `scripts/plugins/generate_vamp_robot_plugin.py`
spherizes a URDF (foam), generates an SRDF ACM, runs cricket's `fkcc_gen` to emit
a VAMP robot header, then compiles a plugin that instantiates
`VampInstance<Robot, Robot, …>` (the **same** robot struct repeated N times) with
a C-ABI factory, loaded via `dlopen`. Robot-robot collision is not compiled from
a multi-robot URDF; it emerges from the generic pairwise sphere sweep.

---

## 4. Proposed upstream API surface

Minimal generic surface for CBS/TPG/ADG to build on without forking:

```cpp
namespace vamp::collision {

  template <typename RobotT, std::size_t rake>
  struct MultiRobotState;             // { configuration, base_transform, attachment }

  struct MultiRobotCollisionFilter;   // per-robot named allow-lists (runtime ACM)

  // Core queries (already present in the fork — keep):
  template <std::size_t rake, typename... S> bool fkcc_multi_all (const Environment<FloatVector<rake>>&, const S&...);
  template <std::size_t rake, typename... S> bool fkcc_multi_self(const S&...);
  template <std::size_t rake, typename... S> bool fkcc_multi_cross(const S&...);        // whole-edge SIMD sweep
  template <std::size_t rake, typename... S> DebugResult debug_fkcc_multi_all(const Environment<FloatVector<rake>>&, const S&...);

  // NEW: first-class runtime-subset dispatch so downstreams stop hand-rolling O(N^2) tables.
  template <typename... Robots>
  struct MultiRobotTeam {
      explicit MultiRobotTeam(std::array<Eigen::Isometry3f, sizeof...(Robots)> base_transforms);

      // The CBS hot path: check an arbitrary robot or pair by runtime index.
      template <std::size_t rake> bool check_single(std::size_t i, const Config& qi, const Environment<...>&, bool self) const;
      template <std::size_t rake> bool check_pair  (std::size_t i, std::size_t j, const Config& qi, const Config& qj, const Environment<...>&, bool self) const;
      // General subset (TPG/ADG): check a set of active robots against each other + env.
      template <std::size_t rake> bool check_subset(std::span<const std::size_t> active, std::span<const Config> qs, const Environment<...>&, bool self) const;
  };
}

namespace vamp::robots {
  template <typename Robot> struct LinkMapping;   // codegen-emitted, NOT hand-written
}
```

Plus the runtime `register_robot<Robot>()` registry as the advanced hook for
Python/plugin composition. On top of this, CBS supplies conflict-driven subsets,
and TPG/ADG consume the debug/contact results — all without touching VAMP
internals.

---

## 5. The two genuinely hard problems

1. **Compile-time↔runtime bridge.** The fast SIMD path requires the full robot
   type pack at compile time. Scaling to arbitrary N or heterogeneous teams
   means either explicit instantiation or the dlopen plugin trick, and CBS's
   "check subset {i,j}" only works because of the O(N²) function-pointer tables.
   `MultiRobotTeam` (§4) is the proposed home for this so it is written once,
   correctly, in VAMP.

2. **Heterogeneous teams + codegen-emitted `LinkMapping`.** The plugin generator
   emits `VampInstance<Robot×N>` — one struct repeated; heterogeneous teams
   (e.g. Panda + GP4) are only reachable via hand-written presets. And
   `link_mapping.hh` hardcodes `sphere_to_link` arrays per robot with
   `static_assert`s on `n_spheres`. For a real bring-your-own-URDF flow, cricket
   (`fkcc_gen`) must **emit the `LinkMapping` alongside the robot header**, and
   the codegen path must be able to emit a heterogeneous pack (or a per-subteam
   registry).

---

## 6. Friction points, ranked by difficulty

1. **(Hard) Compile-time-only robot set / no runtime-N generic entry point.**
   Addressed by `MultiRobotTeam` subset dispatch (§4/§5.1).
2. **(Hard) Homogeneous-only codegen.** `generate_vamp_robot_plugin.py` emits a
   repeated single struct; heterogeneous teams need codegen work (§5.2).
3. **(Medium) `LinkMapping<Robot>` is hand-maintained.** Must be cricket-emitted.
4. **(Medium) Coupling to downstream types & plugin ABI.** `VampInstance`
   implements `mr_planner`'s `PlanInstance`, and the plugin boundary
   (`vamp_plugin_api.h`, ABI version) is an mr_planner concept. VAMP should
   expose the multi-robot primitives as a stable public C++/Python surface,
   leaving the `PlanInstance`/plugin wrapper downstream.
5. **(Medium) `thread_local EnvironmentTransformCache`.** A hidden per-transform
   cache; fine for mr_planner's usage but reviewers will scrutinize correctness
   under changing environments, and it silently strips heightfields/pointclouds
   from the transformed copy. Make it explicit/opt-in.
6. **(Low) Robot + formatting cruft.** The new GP4 robot and a reformatted
   `panda.hh` create noisy diffs; new robots should go through the normal
   upstream robot-contribution process, not ride along with this feature.

---

## 7. What stays downstream

`VampInstance`, the plugin ABI (`vamp_plugin_api.h`,
`mr_planner_vamp_plugin_get_api`), the cricket/foam plugin-generator scripts, and
the hardcoded presets remain in `mr_planner_core` as the reference *consumer* —
concrete evidence of the API surface VAMP needs to expose, but not part of VAMP.

---

## 8. Suggested staged upstreaming plan

1. Land the additive collision core (`multi_robot.hh`: `MultiRobotState`,
   `MultiRobotCollisionFilter`, `fkcc_multi_*`, `debug_fkcc_multi_all`) behind a
   `VAMP_ENABLE_MULTI_ROBOT` CMake option. Split the debug/contact-visualization
   plumbing (`DebugCollisionPose`, `make_pose_from_capsule/cuboid`,
   `compute_contacts`) into a separate header so the core stays lean.
2. Add the `MultiRobotTeam` subset-dispatch helper (the compile-time↔runtime
   bridge) as the CBS/TPG/ADG-facing hook.
3. Move `LinkMapping` generation into cricket (`fkcc_gen`) so it is emitted, not
   hand-written.
4. Extend the codegen path to heterogeneous teams (multi-robot URDF → per-robot
   headers + a composed team type / registry).
5. Stabilize the runtime registry (`register_robot<Robot>()`) as the documented
   Python/plugin composition API; make `EnvironmentTransformCache` opt-in and
   documented.

**Bottom line:** the collision math (the `multi_robot.hh` templates +
`MultiRobotState` + filter + `LinkMapping` trait) is clean, additive, and builds
only on VAMP's existing per-robot kernel contract — a strong upstream candidate.
The hard part is not the collision math but the compile-time↔runtime bridge and
making `LinkMapping` codegen-emitted; solving those two delivers the "transparent
multi-robot from a URDF, with low-level hooks for CBS/TPG/ADG" vision.
