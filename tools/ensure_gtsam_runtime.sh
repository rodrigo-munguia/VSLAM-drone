# shellcheck shell=bash
# Idempotent GTSAM runtime setup for integrated launch / colcon.
#
# Usage (from repo root or any caller that sets EKF_REPO_ROOT):
#   # shellcheck source=tools/ensure_gtsam_runtime.sh
#   source "$ROOT/tools/ensure_gtsam_runtime.sh"
#   ekf_ensure_gtsam_runtime "$ROOT"   # exit 1 on failure if required
#
# Resolution order (first hit wins for prefix; existing LD_LIBRARY_PATH kept):
#   1. Already resolvable libgtsam.so.4 on LD_LIBRARY_PATH / ldconfig
#   2. tools/gtsam_prefix.env (untracked local file; see .example)
#   3. GTSAM_PREFIX if already set and contains lib/libgtsam.so.4
#   4. Repo-relative candidates (no absolute machine paths in this script):
#        $ROOT/deps/gtsam-4.2.0
#        $ROOT/../deps/gtsam-4.2.0
#        $ROOT/../../deps/gtsam-4.2.0
#
# Does not vendor GTSAM. Safe to source repeatedly (EKF_GTSAM_RUNTIME_CONFIGURED).

ekf_gtsam_lib_resolvable() {
  local p
  local IFS=':'
  # shellcheck disable=SC2086
  for p in ${LD_LIBRARY_PATH:-}; do
    [[ -n "$p" && -e "$p/libgtsam.so.4" ]] && return 0
  done
  if command -v ldconfig >/dev/null 2>&1; then
    if ldconfig -p 2>/dev/null | grep -q 'libgtsam\.so\.4'; then
      return 0
    fi
  fi
  return 1
}

ekf_gtsam_prepend_prefix() {
  local prefix="$1"
  if [[ -z "$prefix" || ! -d "$prefix/lib" ]]; then
    return 1
  fi
  if [[ ! -e "$prefix/lib/libgtsam.so.4" ]]; then
    return 1
  fi
  # Canonicalize for child-shell exports (avoid repo-relative ../../ noise).
  prefix="$(cd "$prefix" && pwd)"
  export GTSAM_PREFIX="$prefix"
  export CMAKE_PREFIX_PATH="${prefix}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  return 0
}

ekf_ensure_gtsam_runtime() {
  local root="${1:-}"
  if [[ -z "$root" ]]; then
    echo "error: ekf_ensure_gtsam_runtime requires repository root path" >&2
    return 1
  fi

  if [[ "${EKF_GTSAM_RUNTIME_CONFIGURED:-}" == "1" ]]; then
    if ekf_gtsam_lib_resolvable; then
      return 0
    fi
    # Prior configure claimed success but lib is missing now — fall through.
    unset EKF_GTSAM_RUNTIME_CONFIGURED
  fi

  if ekf_gtsam_lib_resolvable; then
    export EKF_GTSAM_RUNTIME_CONFIGURED=1
    return 0
  fi

  local env_file="$root/tools/gtsam_prefix.env"
  if [[ -f "$env_file" ]]; then
    # Preserve caller -u; env file assigns PATH-style vars.
    set +u
    set -a
    # shellcheck disable=SC1090
    source "$env_file"
    set +a
    set -u
  fi

  if ekf_gtsam_lib_resolvable; then
    export EKF_GTSAM_RUNTIME_CONFIGURED=1
    return 0
  fi

  local cand
  for cand in \
    "${GTSAM_PREFIX:-}" \
    "$root/deps/gtsam-4.2.0" \
    "$root/../deps/gtsam-4.2.0" \
    "$root/../../deps/gtsam-4.2.0"
  do
    [[ -n "$cand" ]] || continue
    if ekf_gtsam_prepend_prefix "$cand"; then
      break
    fi
  done

  if ekf_gtsam_lib_resolvable; then
    export EKF_GTSAM_RUNTIME_CONFIGURED=1
    echo "GTSAM runtime: using GTSAM_PREFIX=${GTSAM_PREFIX}"
    return 0
  fi

  echo "error: libgtsam.so.4 not found on LD_LIBRARY_PATH (required for --globalslam true)." >&2
  echo "  Install/build GTSAM 4.2.x, then either:" >&2
  echo "    cp tools/gtsam_prefix.env.example tools/gtsam_prefix.env" >&2
  echo "    # edit GTSAM_PREFIX in tools/gtsam_prefix.env, then re-run" >&2
  echo "  or:" >&2
  echo "    export GTSAM_PREFIX=/path/to/gtsam-4.2.0" >&2
  echo "    export LD_LIBRARY_PATH=\"\$GTSAM_PREFIX/lib:\${LD_LIBRARY_PATH:-}\"" >&2
  echo "  See tools/gtsam_prefix.env.example." >&2
  return 1
}
