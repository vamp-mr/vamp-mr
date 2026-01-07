#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

extra_args=()
have_min_rate=0
for arg in "$@"; do
  if [[ "${arg}" == "--bench-min-success-rate" ]]; then
    have_min_rate=1
    break
  fi
done
if [[ "${have_min_rate}" == "0" ]]; then
  extra_args+=(--bench-min-success-rate 0.8)
fi

exec "${SCRIPT_DIR}/run_python_smoke.sh" --suite benchmark "${extra_args[@]}" "$@"
