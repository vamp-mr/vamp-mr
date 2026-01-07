#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

usage() {
  cat <<'EOF'
Generate a Kinova Gen3 VAMP plugin environment (3 robots) and run mr_planner_core_plan.

This is a user-facing helper script. It assumes you already installed:
  - mr_planner_core (CMake install prefix, default: /usr/local)
  - cricket (fkcc_gen + templates), co-installed under that prefix or available in PATH
  - a spherized URDF (via foam) and matching SRDF for a *single* Gen3 robot

The 3-robot base transforms default to mr_planner_core/scripts/plugins/config/kortex_three_base_transforms.json,
which is derived from:
  /home/philip/catkin_ws/src/arcs_kortex/kortex_three_description/urdf/kortex_three.xacro

Usage:
  mr_planner_core/scripts/plugins/run_kinova_gen3_pipeline.sh --spherized-urdf <file> --srdf <file> --end-effector <frame> [options]

Options:
  --mr-prefix <path>        mr_planner_core install prefix (default: /usr/local)
  --output-dir <path>       Output directory (default: /tmp/mr_planner_kinova_plugin.XXXXXX)
  --env-name <name>         Environment name / JSON filename (default: kinova_gen3_three)
  --num-robots <int>        Number of robots (default: 3)
  --move-group <name>       Move group name (default: arm)
  --robot-groups <csv>      Robot group names (default: arm0,arm1,...)
  --base-transforms <json>  Base transforms JSON (default: config/kortex_three_base_transforms.json)
  --planner <name>          Planner (default: composite_rrt)
  --planning-time <sec>     Planner time (default: 2)
  --shortcut-time <sec>     Shortcut time (default: 0.1)
EOF
}

MR_PREFIX="/usr/local"
OUT_DIR=""
ENV_NAME="kinova_gen3_three"
NUM_ROBOTS=3
MOVE_GROUP="arm"
ROBOT_GROUPS=""
BASE_TRANSFORMS="${SCRIPT_DIR}/config/kortex_three_base_transforms.json"
SPHERIZED_URDF=""
SRDF=""
END_EFFECTOR=""
PLANNER="composite_rrt"
PLANNING_TIME=2
SHORTCUT_TIME=0.1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mr-prefix) MR_PREFIX="${2:-}"; shift 2 ;;
    --output-dir) OUT_DIR="${2:-}"; shift 2 ;;
    --env-name) ENV_NAME="${2:-}"; shift 2 ;;
    --num-robots) NUM_ROBOTS="${2:-}"; shift 2 ;;
    --move-group) MOVE_GROUP="${2:-}"; shift 2 ;;
    --robot-groups) ROBOT_GROUPS="${2:-}"; shift 2 ;;
    --base-transforms) BASE_TRANSFORMS="${2:-}"; shift 2 ;;
    --spherized-urdf) SPHERIZED_URDF="${2:-}"; shift 2 ;;
    --srdf) SRDF="${2:-}"; shift 2 ;;
    --end-effector) END_EFFECTOR="${2:-}"; shift 2 ;;
    --planner) PLANNER="${2:-}"; shift 2 ;;
    --planning-time) PLANNING_TIME="${2:-}"; shift 2 ;;
    --shortcut-time) SHORTCUT_TIME="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[error] unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "${SPHERIZED_URDF}" || -z "${SRDF}" || -z "${END_EFFECTOR}" ]]; then
  echo "[error] required: --spherized-urdf, --srdf, --end-effector" >&2
  usage
  exit 2
fi

if [[ -z "${OUT_DIR}" ]]; then
  OUT_DIR="$(mktemp -d "/tmp/mr_planner_kinova_plugin.XXXXXX")"
fi
mkdir -p "${OUT_DIR}"

PLAN_BIN="${MR_PREFIX}/bin/mr_planner_core_plan"
if [[ ! -x "${PLAN_BIN}" ]]; then
  echo "[error] could not find mr_planner_core_plan at ${PLAN_BIN}" >&2
  echo "       install mr_planner_core first (see mr_planner_core/scripts/setup/install_mr_planner_core.sh)" >&2
  exit 2
fi

if [[ -z "${ROBOT_GROUPS}" ]]; then
  ROBOT_GROUPS="$(python3 - <<PY
import sys
print(",".join([f"arm{i}" for i in range(int("${NUM_ROBOTS}"))]))
PY
)"
fi

PLUGIN_OUT="${OUT_DIR}/plugin"
PLAN_OUT="${OUT_DIR}/plan_out"
mkdir -p "${PLUGIN_OUT}" "${PLAN_OUT}"

echo "[info] generating VAMP plugin + env json -> ${PLUGIN_OUT}" >&2
python3 "${REPO_ROOT}/mr_planner_core/scripts/plugins/generate_vamp_robot_plugin.py" \
  --env-name "${ENV_NAME}" \
  --move-group "${MOVE_GROUP}" \
  --robot-groups "${ROBOT_GROUPS}" \
  --num-robots "${NUM_ROBOTS}" \
  --base-transforms "${BASE_TRANSFORMS}" \
  --spherized-urdf "${SPHERIZED_URDF}" \
  --srdf "${SRDF}" \
  --end-effector "${END_EFFECTOR}" \
  --mr-planner-core-prefix "${MR_PREFIX}" \
  --output-dir "${PLUGIN_OUT}" > "${PLUGIN_OUT}/summary.json"

ENV_JSON="${PLUGIN_OUT}/${ENV_NAME}.json"
if [[ ! -f "${ENV_JSON}" ]]; then
  echo "[error] env json not found: ${ENV_JSON}" >&2
  exit 2
fi

VAMP_DOF="$(python3 -c "import json; print(json.load(open('${ENV_JSON}'))['vamp_dimension'])" 2>/dev/null || echo 7)"
echo "[info] planning with env=${ENV_JSON} (num_robots=${NUM_ROBOTS}, dof=${VAMP_DOF})" >&2

"${PLAN_BIN}" \
  --vamp-environment "${ENV_JSON}" \
  --planner "${PLANNER}" \
  --planning-time "${PLANNING_TIME}" \
  --shortcut-time "${SHORTCUT_TIME}" \
  --seed 1 \
  --num-robots "${NUM_ROBOTS}" \
  --dof "${VAMP_DOF}" \
  --output-dir "${PLAN_OUT}" >/dev/null

echo "[info] done" >&2
echo "[info] outputs: ${OUT_DIR}" >&2
echo "[info] plugin env json: ${ENV_JSON}" >&2
echo "[info] plan outputs: ${PLAN_OUT}" >&2

echo "[info] python usage (direct JSON path):" >&2
cat <<EOF
# If \`import mr_planner_core\` fails, see mr_planner_core/README.md for PYTHONPATH setup.
python3 - <<'PY'
import mr_planner_core

env = mr_planner_core.VampEnvironment("${ENV_JSON}", seed=1)
res = env.plan(planner="${PLANNER}", planning_time=float("${PLANNING_TIME}"), shortcut_time=float("${SHORTCUT_TIME}"), seed=1, write_tpg=False)
print(res["solution_csv"])
PY
EOF
