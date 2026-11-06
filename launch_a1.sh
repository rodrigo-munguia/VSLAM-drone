#!/usr/bin/env bash
# Compatibility wrapper → ./launch2.sh (conventional install/ overlay).
# Prefer ./launch2.sh directly. Override with EKF_INSTALL_BASE if needed.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export EKF_INSTALL_BASE="${EKF_INSTALL_BASE:-install}"
exec "$ROOT/launch2.sh" "$@"
