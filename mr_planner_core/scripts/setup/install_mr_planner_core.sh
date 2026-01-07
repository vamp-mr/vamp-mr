#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

usage() {
  cat <<'EOF'
Build and install mr_planner_core (and optionally cricket) into a CMake prefix.

Examples:
  # system-wide (may require sudo)
  mr_planner_core/scripts/setup/install_mr_planner_core.sh --prefix /usr/local

  # user-space install
  mr_planner_core/scripts/setup/install_mr_planner_core.sh --prefix "${HOME}/.local"

  # also install cricket artifacts (fkcc_gen + templates) so plugin generation works without extra flags
  mr_planner_core/scripts/setup/install_mr_planner_core.sh --prefix /usr/local --install-cricket --cricket-dir "${HOME}/Code/cricket"

Options:
  --prefix <path>         Install prefix (default: /usr/local)
  --build-dir <path>      Build directory (default: /tmp/mr_planner_core_build.XXXXXX)
  --type <Release|Debug>  Build type (default: Release)
  --no-python             Disable Python bindings
  --no-vamp               Disable VAMP backend (also disables CLI apps + Python bindings)
  --sudo                  Force sudo for install/copy steps
  --no-sudo               Never use sudo (errors if prefix is not writable)

  --install-cricket       Copy/install cricket (fkcc_gen + templates) into the same prefix
  --cricket-dir <path>    Path to cricket repo (default: ~/Code/cricket)
  --cricket-build <path>  Cricket build dir (default: <build-dir>/cricket_build)

Notes:
  - mr_planner_core uses FetchContent to download pybind11 when Python is enabled.
  - mr_planner_core expects VAMP to be discoverable via CMake (find_package(vamp)).
  - cricket depends on pinocchio; when building cricket, this script will create a micromamba env from
    cricket's environment.yaml under <prefix>/share/mr_planner_core/cricket/micromamba.
EOF
}

PREFIX="/usr/local"
BUILD_DIR=""
BUILD_TYPE="Release"
ENABLE_PYTHON=1
ENABLE_VAMP=1
FORCE_SUDO=0
NO_SUDO=0

INSTALL_CRICKET=0
CRICKET_DIR="${HOME}/Code/cricket"
CRICKET_BUILD=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix) PREFIX="${2:-}"; shift 2 ;;
    --build-dir) BUILD_DIR="${2:-}"; shift 2 ;;
    --type) BUILD_TYPE="${2:-}"; shift 2 ;;
    --no-python) ENABLE_PYTHON=0; shift ;;
    --no-vamp) ENABLE_VAMP=0; ENABLE_PYTHON=0; shift ;;
    --sudo) FORCE_SUDO=1; shift ;;
    --no-sudo) NO_SUDO=1; shift ;;
    --install-cricket) INSTALL_CRICKET=1; shift ;;
    --cricket-dir) CRICKET_DIR="${2:-}"; shift 2 ;;
    --cricket-build) CRICKET_BUILD="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[error] unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "${BUILD_DIR}" ]]; then
  BUILD_DIR="$(mktemp -d "/tmp/mr_planner_core_build.XXXXXX")"
fi

echo "[info] build dir: ${BUILD_DIR}" >&2
echo "[info] install prefix: ${PREFIX}" >&2

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
  # If the prefix doesn't exist, check if we can create it (parent must be writable).
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
  echo "[info] using sudo for install into ${PREFIX}" >&2
else
  # If the user forbids sudo, fail fast when the prefix isn't writable.
  if [[ "${NO_SUDO}" == "1" ]]; then
    if [[ -e "${PREFIX}" ]]; then
      if [[ ! -w "${PREFIX}" ]]; then
        echo "[error] --no-sudo was specified but prefix is not writable: ${PREFIX}" >&2
        exit 2
      fi
    else
      parent_dir="$(dirname "${PREFIX}")"
      if [[ ! -w "${parent_dir}" ]]; then
        echo "[error] --no-sudo was specified but cannot create prefix under: ${parent_dir}" >&2
        exit 2
      fi
    fi
  fi
fi

cmake -S "${REPO_ROOT}/mr_planner_core" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DMR_PLANNER_CORE_ENABLE_VAMP="$([[ "${ENABLE_VAMP}" == "1" ]] && echo ON || echo OFF)" \
  -DMR_PLANNER_CORE_ENABLE_PYTHON="$([[ "${ENABLE_PYTHON}" == "1" ]] && echo ON || echo OFF)"

cmake --build "${BUILD_DIR}" -j
${SUDO} cmake --install "${BUILD_DIR}"

if [[ "${INSTALL_CRICKET}" != "1" ]]; then
  echo "[info] installed mr_planner_core only (cricket skipped)" >&2
  exit 0
fi

if [[ ! -d "${CRICKET_DIR}" ]]; then
  echo "[error] cricket repo not found: ${CRICKET_DIR} (use --cricket-dir)" >&2
  exit 2
fi

if [[ -z "${CRICKET_BUILD}" ]]; then
  CRICKET_BUILD="${BUILD_DIR}/cricket_build"
fi

TEMPLATES_SRC=""
for d in "${CRICKET_DIR}/resources/templates" "${CRICKET_DIR}/templates"; do
  if [[ -d "${d}" ]]; then
    TEMPLATES_SRC="${d}"
    break
  fi
done
if [[ -z "${TEMPLATES_SRC}" ]]; then
  echo "[error] could not find cricket templates under ${CRICKET_DIR}/resources/templates" >&2
  exit 2
fi

FKCC_GEN=""
for p in "${CRICKET_DIR}/build/fkcc_gen" "${CRICKET_DIR}/build-release/fkcc_gen" "${CRICKET_BUILD}/fkcc_gen"; do
  if [[ -x "${p}" ]]; then
    FKCC_GEN="${p}"
    break
  fi
done

if [[ -z "${FKCC_GEN}" ]]; then
  echo "[info] fkcc_gen not found; installing cricket via micromamba (pinocchio dependency)" >&2

  CRICKET_INSTALL_ROOT="${PREFIX}/share/mr_planner_core/cricket"
  MICROMAMBA_ROOT="${CRICKET_INSTALL_ROOT}/micromamba"
  MICROMAMBA_BIN="${MICROMAMBA_ROOT}/bin/micromamba"
  MICROMAMBA_PKGS="${MICROMAMBA_ROOT}/root"
  CRICKET_ENV="${MICROMAMBA_ROOT}/envs/cricket"
  CRICKET_ENV_FKCC="${CRICKET_ENV}/bin/fkcc_gen"

  # 1) Install micromamba into the prefix (self-contained).
  if [[ ! -x "${MICROMAMBA_BIN}" ]]; then
    echo "[info] downloading micromamba -> ${MICROMAMBA_BIN}" >&2
    tmp_mamba="${BUILD_DIR}/micromamba_download"
    rm -rf "${tmp_mamba}"
    mkdir -p "${tmp_mamba}"

    archive="${tmp_mamba}/micromamba.tar.bz2"
    if command -v curl >/dev/null 2>&1; then
      curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest -o "${archive}"
    elif command -v wget >/dev/null 2>&1; then
      wget -qO "${archive}" https://micro.mamba.pm/api/micromamba/linux-64/latest
    else
      echo "[error] neither curl nor wget is available to download micromamba" >&2
      exit 2
    fi
    tar -xvjf "${archive}" -C "${tmp_mamba}" bin/micromamba >/dev/null

    ${SUDO} mkdir -p "${MICROMAMBA_ROOT}/bin"
    ${SUDO} cp -f "${tmp_mamba}/bin/micromamba" "${MICROMAMBA_BIN}"
    ${SUDO} chmod +x "${MICROMAMBA_BIN}"
  fi

  # 2) Create the conda environment under the prefix if missing.
  if [[ ! -d "${CRICKET_ENV}" ]]; then
    echo "[info] creating micromamba env at ${CRICKET_ENV}" >&2
    ${SUDO} "${MICROMAMBA_BIN}" create -y \
      -r "${MICROMAMBA_PKGS}" \
      -p "${CRICKET_ENV}" \
      -f "${CRICKET_DIR}/environment.yaml" >/dev/null
  fi

  # 3) Build+install cricket into the env so fkcc_gen runs with its runtime deps.
  if [[ ! -x "${CRICKET_ENV_FKCC}" ]]; then
    echo "[info] building cricket fkcc_gen -> ${CRICKET_ENV_FKCC}" >&2
    # Use a dedicated build folder for micromamba builds to avoid reusing a
    # previous system CMake cache (which can poison dependency detection).
    CRICKET_BUILD_MAMBA="${CRICKET_BUILD}/micromamba"
    export CPM_SOURCE_CACHE="${BUILD_DIR}/cricket_cpm_cache"
    "${MICROMAMBA_BIN}" run -p "${CRICKET_ENV}" cmake -S "${CRICKET_DIR}" -B "${CRICKET_BUILD_MAMBA}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${CRICKET_ENV}" >/dev/null
    "${MICROMAMBA_BIN}" run -p "${CRICKET_ENV}" cmake --build "${CRICKET_BUILD_MAMBA}" -j >/dev/null
    ${SUDO} "${MICROMAMBA_BIN}" run -p "${CRICKET_ENV}" cmake --install "${CRICKET_BUILD_MAMBA}" >/dev/null
  fi

  if [[ ! -x "${CRICKET_ENV_FKCC}" ]]; then
    echo "[error] fkcc_gen not found after micromamba build: ${CRICKET_ENV_FKCC}" >&2
    exit 2
  fi

  # 4) Install a small wrapper into <prefix>/bin so other scripts can just call fkcc_gen.
  echo "[info] installing fkcc_gen wrapper -> ${PREFIX}/bin/fkcc_gen" >&2
  wrapper_tmp="${BUILD_DIR}/fkcc_gen_wrapper.sh"
  cat >"${wrapper_tmp}" <<EOF_WRAPPER
#!/usr/bin/env bash
set -euo pipefail
MICROMAMBA_BIN="${MICROMAMBA_BIN}"
CRICKET_ENV="${CRICKET_ENV}"
exec "\${MICROMAMBA_BIN}" run -p "\${CRICKET_ENV}" "\${CRICKET_ENV}/bin/fkcc_gen" "\$@"
EOF_WRAPPER
  ${SUDO} mkdir -p "${PREFIX}/bin"
  ${SUDO} cp -f "${wrapper_tmp}" "${PREFIX}/bin/fkcc_gen"
  ${SUDO} chmod +x "${PREFIX}/bin/fkcc_gen"

  # Continue with templates install below, and finish.
else
  # If the user already has a built fkcc_gen, install it directly.
  echo "[info] using existing fkcc_gen: ${FKCC_GEN}" >&2

  echo "[info] installing fkcc_gen -> ${PREFIX}/bin" >&2
  ${SUDO} mkdir -p "${PREFIX}/bin"
  ${SUDO} cp -f "${FKCC_GEN}" "${PREFIX}/bin/fkcc_gen"
fi

echo "[info] installing cricket templates -> ${PREFIX}/share/mr_planner_core/cricket/templates" >&2
${SUDO} mkdir -p "${PREFIX}/share/mr_planner_core/cricket"
${SUDO} rm -rf "${PREFIX}/share/mr_planner_core/cricket/templates"
${SUDO} cp -r "${TEMPLATES_SRC}" "${PREFIX}/share/mr_planner_core/cricket/templates"

echo "[info] installed mr_planner_core + cricket into ${PREFIX}" >&2
