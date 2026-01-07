#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CORE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
REPO_ROOT="${CORE_ROOT}"
if [[ -d "${CORE_ROOT}/../mr_planner_lego" && -d "${CORE_ROOT}/../mr_planner_ros" ]]; then
  REPO_ROOT="$(cd "${CORE_ROOT}/.." && pwd)"
fi
CORE_SRC="${CORE_ROOT}"
if [[ "${REPO_ROOT}" != "${CORE_ROOT}" ]]; then
  CORE_SRC="${REPO_ROOT}/mr_planner_core"
fi

KEEP="${MR_PLANNER_PLUGIN_SMOKE_KEEP:-0}"
SMOKE_DIR="${MR_PLANNER_PLUGIN_SMOKE_DIR:-}"
if [[ -z "${SMOKE_DIR}" ]]; then
  SMOKE_DIR="$(mktemp -d "/tmp/mr_planner_plugin_smoke.XXXXXX")"
fi

trap 'code=$?; if [[ $code -eq 0 && "${KEEP}" != "1" ]]; then rm -rf "${SMOKE_DIR}"; else echo "[info] outputs kept at ${SMOKE_DIR}" >&2; fi' EXIT

prefix="${MR_PLANNER_PLUGIN_SMOKE_PREFIX:-}"
core_cmake_dir=""

if [[ -z "${prefix}" ]]; then
  prefix="${SMOKE_DIR}/install"
  core_build="${SMOKE_DIR}/build_core"

  mkdir -p "${prefix}"

  echo "[info] building+installing mr_planner_core to ${prefix}" >&2
  cmake -S "${CORE_SRC}" -B "${core_build}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${prefix}"
  cmake --build "${core_build}" -j
  cmake --install "${core_build}"
fi

for d in "${prefix}/lib/cmake/mr_planner_core" "${prefix}/lib64/cmake/mr_planner_core"; do
  if [[ -d "${d}" ]]; then
    core_cmake_dir="${d}"
    break
  fi
done
if [[ -z "${core_cmake_dir}" ]]; then
  echo "[error] could not locate installed mr_planner_core cmake package under ${prefix}" >&2
  exit 2
fi

echo "[info] locating vamp/robots/panda.hh" >&2
panda_header=""
for p in /usr/local/include/vamp/robots/panda.hh /usr/include/vamp/robots/panda.hh; do
  if [[ -f "${p}" ]]; then
    panda_header="${p}"
    break
  fi
done
if [[ -z "${panda_header}" ]]; then
  echo "[error] could not locate vamp/robots/panda.hh; install VAMP headers first." >&2
  exit 2
fi

plugin_dir="${SMOKE_DIR}/panda_two_plugin"
plugin_env="panda_two_plugin_smoke"
planning_time="${MR_PLANNER_PLUGIN_SMOKE_PLANNING_TIME:-5}"
seed="${MR_PLANNER_PLUGIN_SMOKE_SEED:-1}"

echo "[info] building panda x2 runtime plugin env (${plugin_env})" >&2
python3 "${CORE_SRC}/scripts/plugins/generate_vamp_robot_plugin.py" \
  --env-name "${plugin_env}" \
  --environment-name panda_two \
  --move-group panda_multi_arm \
  --robot-groups panda0_arm,panda1_arm \
  --hand-groups panda0_hand,panda1_hand \
  --num-robots 2 \
  --robot-header "${panda_header}" \
  --robot-struct Panda \
  --base-transforms "${CORE_SRC}/scripts/plugins/config/panda_two_base_transforms.json" \
  --mr-planner-core-dir "${core_cmake_dir}" \
  --output-dir "${plugin_dir}" >/dev/null

builtin_sample_out="${SMOKE_DIR}/builtin_panda_two_sample"
builtin_out="${SMOKE_DIR}/builtin_panda_two"
plugin_out="${SMOKE_DIR}/plugin_panda_two"
mkdir -p "${builtin_sample_out}" "${builtin_out}" "${plugin_out}"

echo "[info] planning with built-in env panda_two (sampling start/goal)" >&2
builtin_log="${builtin_sample_out}/plan.log"
"${prefix}/bin/mr_planner_core_plan" \
  --vamp-environment panda_two \
  --planner composite_rrt \
  --planning-time "${planning_time}" \
  --shortcut-time 0.0 \
  --seed "${seed}" \
  --num-robots 2 \
  --dof 7 \
  --output-dir "${builtin_sample_out}" >/dev/null 2>"${builtin_log}"

start_goal_json="$(
python3 - "${builtin_log}" <<'PY'
import re
import sys

txt = open(sys.argv[1], "r", encoding="utf-8", errors="replace").read()
m0 = re.search(r'\[info\]\s+sampled --start\s+"([^"]+)"', txt)
m1 = re.search(r'\[info\]\s+sampled --goal\s+"([^"]+)"', txt)
if not (m0 and m1):
    raise SystemExit("failed to extract sampled start/goal from log")
print(f"{m0.group(1)}\n{m1.group(1)}")
PY
)"
start="$(echo "${start_goal_json}" | sed -n '1p')"
goal="$(echo "${start_goal_json}" | sed -n '2p')"

if [[ -z "${start}" || -z "${goal}" ]]; then
  echo "[error] failed to parse sampled start/goal" >&2
  exit 3
fi

echo "[info] planning with built-in env panda_two using sampled start/goal" >&2
"${prefix}/bin/mr_planner_core_plan" \
  --vamp-environment panda_two \
  --planner composite_rrt \
  --planning-time "${planning_time}" \
  --shortcut-time 0.0 \
  --seed "${seed}" \
  --start "${start}" \
  --goal "${goal}" \
  --output-dir "${builtin_out}" >/dev/null

echo "[info] planning with plugin env ${plugin_env} using the same start/goal" >&2
export MR_PLANNER_VAMP_ENV_PATH="${plugin_dir}"
"${prefix}/bin/mr_planner_core_plan" \
  --vamp-environment "${plugin_env}" \
  --planner composite_rrt \
  --planning-time "${planning_time}" \
  --shortcut-time 0.0 \
  --seed "${seed}" \
  --start "${start}" \
  --goal "${goal}" \
  --output-dir "${plugin_out}" >/dev/null
unset MR_PLANNER_VAMP_ENV_PATH

for f in "${builtin_out}/solution.csv" "${plugin_out}/solution.csv"; do
  if [[ ! -f "${f}" ]]; then
    echo "[error] missing expected output: ${f}" >&2
    exit 4
  fi
done

python3 - "${builtin_out}/solution.csv" "${plugin_out}/solution.csv" >/dev/null <<'PY'
import csv
import sys
from typing import List

TOL = 1e-6

def nonempty(tokens) -> List[str]:
    return [c.strip() for c in tokens if c is not None and c.strip() != ""]

def read_header(path: str) -> List[str]:
    with open(path, "r", newline="") as f:
        header = next(csv.reader(f))
    return nonempty(header)

def read_rows(path: str) -> List[List[float]]:
    rows: List[List[float]] = []
    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        next(reader)  # header
        for raw in reader:
            row = nonempty(raw)
            if not row:
                continue
            rows.append([float(x) for x in row])
    return rows

h0 = read_header(sys.argv[1])
h1 = read_header(sys.argv[2])
if h0 != h1:
    raise SystemExit(f"CSV headers differ\\n{sys.argv[1]}: {h0[:8]}...\\n{sys.argv[2]}: {h1[:8]}...")
if not h0 or h0[0] != "time":
    raise SystemExit("invalid CSV header")
qcols = [c for c in h0[1:] if c.startswith("q")]
if len(qcols) != 14:
    raise SystemExit(f"expected 14 joint columns (2 robots x 7 dof), got {len(qcols)}")

r0 = read_rows(sys.argv[1])
r1 = read_rows(sys.argv[2])
if len(r0) != len(r1):
    raise SystemExit(f"row count differs: {len(r0)} vs {len(r1)}")
if not r0:
    raise SystemExit("no rows in CSV")

max_diff = 0.0
max_at = (0, 0)
for i, (a, b) in enumerate(zip(r0, r1)):
    if len(a) != len(b):
        raise SystemExit(f"row {i} column count differs: {len(a)} vs {len(b)}")
    for j, (xa, xb) in enumerate(zip(a, b)):
        d = abs(xa - xb)
        if d > max_diff:
            max_diff = d
            max_at = (i, j)

if max_diff > TOL:
    i, j = max_at
    raise SystemExit(
        f"CSV values differ (max_abs_diff={max_diff} > {TOL}) at row={i} col={j}: {r0[i][j]} vs {r1[i][j]}"
    )

print("ok")
PY

echo "[info] plugin smoke checks passed" >&2
