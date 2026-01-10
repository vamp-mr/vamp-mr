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

KEEP="${MR_PLANNER_PY_SMOKE_KEEP:-0}"
SMOKE_DIR="${MR_PLANNER_PY_SMOKE_DIR:-}"
if [[ -z "${SMOKE_DIR}" ]]; then
  SMOKE_DIR="$(mktemp -d "/tmp/mr_planner_py_smoke.XXXXXX")"
fi

trap 'code=$?; if [[ $code -eq 0 && "${KEEP}" != "1" ]]; then rm -rf "${SMOKE_DIR}"; else echo "[info] outputs kept at ${SMOKE_DIR}" >&2; fi' EXIT

USE_EXISTING="${MR_PLANNER_PY_SMOKE_USE_EXISTING:-0}"
EXISTING_PREFIX="${MR_PLANNER_PY_SMOKE_PREFIX:-}"
EXISTING_CORE_BUILD_DIR="${MR_PLANNER_PY_SMOKE_CORE_BUILD_DIR:-}"
EXISTING_LEGO_BUILD_DIR="${MR_PLANNER_PY_SMOKE_LEGO_BUILD_DIR:-}"

prefix="${SMOKE_DIR}/install"
core_build="${SMOKE_DIR}/build_core"
lego_build="${SMOKE_DIR}/build_lego"
DID_BUILD_INSTALL=0

pick_first_existing_dir() {
  for d in "$@"; do
    if [[ -n "${d}" && -d "${d}" ]]; then
      echo "${d}"
      return 0
    fi
  done
  return 1
}

if [[ -n "${EXISTING_PREFIX}" || "${USE_EXISTING}" == "1" || -n "${EXISTING_CORE_BUILD_DIR}" ]]; then
  echo "[info] using existing build/install (skipping rebuild)" >&2

  # Prefer an explicit prefix when provided (e.g., /usr/local or a custom install prefix).
  if [[ -n "${EXISTING_PREFIX}" ]]; then
    prefix="${EXISTING_PREFIX}"
  else
    # Create a small synthetic prefix used only for locating CLI binaries in python_smoke.py.
    prefix="${SMOKE_DIR}/prefix"
    mkdir -p "${prefix}/bin"
  fi

  core_build_candidates=()
  if [[ -n "${EXISTING_CORE_BUILD_DIR}" ]]; then
    core_build_candidates+=("${EXISTING_CORE_BUILD_DIR}")
  fi
  core_build_candidates+=(
    "${CORE_SRC}/build"
    "${REPO_ROOT}/build/mr_planner_core"
  )
  core_build="$(pick_first_existing_dir "${core_build_candidates[@]}" || true)"

  lego_build_candidates=()
  if [[ -n "${EXISTING_LEGO_BUILD_DIR}" ]]; then
    lego_build_candidates+=("${EXISTING_LEGO_BUILD_DIR}")
  fi
  lego_build_candidates+=(
    "${REPO_ROOT}/mr_planner_lego/build"
    "${REPO_ROOT}/build/mr_planner_lego"
  )
  lego_build="$(pick_first_existing_dir "${lego_build_candidates[@]}" || true)"

  # If we are not using an installed prefix, try to expose the locally built lego CLIs via prefix/bin.
  if [[ -d "${prefix}/bin" && -n "${lego_build}" ]]; then
    for exe in mr_planner_lego_assign mr_planner_lego_plan; do
      if [[ -x "${lego_build}/${exe}" && ! -e "${prefix}/bin/${exe}" ]]; then
        ln -s "${lego_build}/${exe}" "${prefix}/bin/${exe}"
      fi
    done
  fi
else
  mkdir -p "${prefix}"

  echo "[info] building+installing mr_planner_core (with python) to ${prefix}" >&2
  cmake -S "${CORE_SRC}" -B "${core_build}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${prefix}" \
    -DMR_PLANNER_CORE_ENABLE_PYTHON=ON
  cmake --build "${core_build}" -j
  cmake --install "${core_build}"

  core_cmake_dir=""
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

  if [[ -d "${REPO_ROOT}/mr_planner_lego" ]]; then
    echo "[info] building+installing mr_planner_lego (find_package core) to ${prefix}" >&2
    cmake -S "${REPO_ROOT}/mr_planner_lego" -B "${lego_build}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${prefix}" \
      -DMR_PLANNER_LEGO_USE_BUNDLED_CORE=OFF \
      -Dmr_planner_core_DIR="${core_cmake_dir}"
    cmake --build "${lego_build}" -j
    cmake --install "${lego_build}"
  else
    echo "[info] mr_planner_lego not found under ${REPO_ROOT}; running core-only python smoke checks" >&2
  fi

  DID_BUILD_INSTALL=1
fi

if [[ -n "${EXISTING_PREFIX}" || "${DID_BUILD_INSTALL}" == "1" ]]; then
  py_purelib="$(python3 -c "import sysconfig; p='${prefix}'; print(sysconfig.get_path('purelib', vars={'base': p, 'platbase': p}))")"
  py_platlib="$(python3 -c "import sysconfig; p='${prefix}'; print(sysconfig.get_path('platlib', vars={'base': p, 'platbase': p}))")"

  export PYTHONPATH="${py_platlib}:${py_purelib}:${PYTHONPATH:-}"
  export LD_LIBRARY_PATH="${prefix}/lib:${prefix}/lib64:${LD_LIBRARY_PATH:-}"
else
  # Build-tree mode: use the pybind output folder directly.
  if [[ -z "${core_build}" ]]; then
    echo "[error] could not infer a core build dir; set MR_PLANNER_PY_SMOKE_CORE_BUILD_DIR" >&2
    exit 2
  fi
  if [[ ! -f "${core_build}/python/mr_planner_core/__init__.py" ]]; then
    echo "[error] missing ${core_build}/python/mr_planner_core/__init__.py; rebuild core with -DMR_PLANNER_CORE_ENABLE_PYTHON=ON" >&2
    exit 2
  fi
  export PYTHONPATH="${core_build}/python:${PYTHONPATH:-}"
  ld_path="${core_build}"
  if [[ -n "${lego_build}" ]]; then
    ld_path="${ld_path}:${lego_build}"
  fi
  export LD_LIBRARY_PATH="${ld_path}:${LD_LIBRARY_PATH:-}"
fi

echo "[info] running python smoke tests (PYTHONPATH=${PYTHONPATH})" >&2
suite="${MR_PLANNER_PY_SMOKE_SUITE:-smoke}"
pass_suite=1
for arg in "$@"; do
  if [[ "${arg}" == "--suite" ]]; then
    pass_suite=0
    break
  fi
done

cmd=(python3 "${SCRIPT_DIR}/python_smoke.py" --repo-root "${REPO_ROOT}" --prefix "${prefix}" --work-dir "${SMOKE_DIR}")
if [[ "${pass_suite}" == "1" ]]; then
  cmd+=(--suite "${suite}")
fi
cmd+=("$@")
"${cmd[@]}"

echo "[info] python smoke checks passed" >&2
echo "[info] plugin smoke is separate: MR_PLANNER_PLUGIN_SMOKE_PREFIX=\"${prefix}\" ${SCRIPT_DIR}/run_plugin_smoke.sh" >&2
