#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

usage() {
  cat <<'EOF'
Build + install VAMP-MR components.

What it does (default):
  1) Install mr_planner_core into /usr/local (build dir: mr_planner_core/build)
  2) Build + install mr_planner_lego against the installed mr_planner_core (build dir: mr_planner_lego/build)

Optionally:
  - Build + install bundled VAMP first (from ./vamp) with --with-vamp

Examples:
  # system-wide install (uses sudo if needed)
  scripts/setup/install_vamp_mr.sh --prefix /usr/local

  # user-space install
  scripts/setup/install_vamp_mr.sh --prefix "${HOME}/.local"

  # also build + install bundled VAMP
  scripts/setup/install_vamp_mr.sh --prefix /usr/local --with-vamp

Options:
  --prefix <path>           Install prefix (default: /usr/local)
  --type <Release|Debug>    Build type (default: Release)
  --with-vamp               Build + install ./vamp into the same prefix

  --core-build-dir <path>   (default: <repo>/mr_planner_core/build)
  --lego-build-dir <path>   (default: <repo>/mr_planner_lego/build)
  --vamp-build-dir <path>   (default: <repo>/vamp/build)

  --sudo                    Force sudo for install steps
  --no-sudo                 Never use sudo (errors if prefix is not writable)

Notes:
  - mr_planner_lego is configured with MR_PLANNER_LEGO_USE_BUNDLED_CORE=OFF and
    CMAKE_PREFIX_PATH=<prefix> so it uses the installed mr_planner_core.
EOF
}

PREFIX="/usr/local"
BUILD_TYPE="Release"

WITH_VAMP=0
CORE_BUILD_DIR="${REPO_ROOT}/mr_planner_core/build"
LEGO_BUILD_DIR="${REPO_ROOT}/mr_planner_lego/build"
VAMP_BUILD_DIR="${REPO_ROOT}/vamp/build"

FORCE_SUDO=0
NO_SUDO=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix) PREFIX="${2:-}"; shift 2 ;;
    --type) BUILD_TYPE="${2:-}"; shift 2 ;;
    --with-vamp) WITH_VAMP=1; shift ;;
    --core-build-dir) CORE_BUILD_DIR="${2:-}"; shift 2 ;;
    --lego-build-dir) LEGO_BUILD_DIR="${2:-}"; shift 2 ;;
    --vamp-build-dir) VAMP_BUILD_DIR="${2:-}"; shift 2 ;;
    --sudo) FORCE_SUDO=1; shift ;;
    --no-sudo) NO_SUDO=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[error] unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ "${FORCE_SUDO}" == "1" && "${NO_SUDO}" == "1" ]]; then
  echo "[error] --sudo and --no-sudo are mutually exclusive" >&2
  exit 2
fi

need_sudo=0
if [[ "${NO_SUDO}" == "1" ]]; then
  need_sudo=0
elif [[ "${FORCE_SUDO}" == "1" ]]; then
  need_sudo=1
else
  if [[ -e "${PREFIX}" ]]; then
    [[ -w "${PREFIX}" ]] || need_sudo=1
  else
    parent_dir="$(dirname "${PREFIX}")"
    [[ -w "${parent_dir}" ]] || need_sudo=1
  fi
fi

SUDO=""
if [[ "${need_sudo}" == "1" ]]; then
  if ! command -v sudo >/dev/null 2>&1; then
    echo "[error] prefix ${PREFIX} is not writable and sudo is not available." >&2
    echo "        rerun with a writable --prefix, or install sudo." >&2
    exit 2
  fi
  SUDO="sudo"
fi

echo "[info] prefix: ${PREFIX}" >&2
echo "[info] build type: ${BUILD_TYPE}" >&2

if [[ "${WITH_VAMP}" == "1" ]]; then
  echo "[info] building + installing bundled VAMP -> ${PREFIX}" >&2
  cmake -S "${REPO_ROOT}/vamp" -B "${VAMP_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DVAMP_INSTALL_CPP_LIBRARY=ON -DVAMP_BUILD_PYTHON_BINDINGS=OFF
  cmake --build "${VAMP_BUILD_DIR}" -j
  ${SUDO} cmake --install "${VAMP_BUILD_DIR}"
fi

echo "[info] installing mr_planner_core -> ${PREFIX} (build dir: ${CORE_BUILD_DIR})" >&2
"${REPO_ROOT}/mr_planner_core/scripts/setup/install_mr_planner_core.sh" \
  --prefix "${PREFIX}" \
  --build-dir "${CORE_BUILD_DIR}" \
  --type "${BUILD_TYPE}" \
  $([[ "${FORCE_SUDO}" == "1" ]] && echo --sudo || true) \
  $([[ "${NO_SUDO}" == "1" ]] && echo --no-sudo || true)

echo "[info] building + installing mr_planner_lego -> ${PREFIX} (build dir: ${LEGO_BUILD_DIR})" >&2
cmake -S "${REPO_ROOT}/mr_planner_lego" -B "${LEGO_BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DMR_PLANNER_LEGO_USE_BUNDLED_CORE=OFF

cmake --build "${LEGO_BUILD_DIR}" -j
${SUDO} cmake --install "${LEGO_BUILD_DIR}"
