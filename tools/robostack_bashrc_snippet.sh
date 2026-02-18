#!/usr/bin/env bash
# Source this file from ~/.bashrc to make RoboStack activation convenient.
# Example:
#   source /absolute/path/to/sentry-navigation/tools/robostack_bashrc_snippet.sh

SENTRY_WS_DEFAULT="/data/home/sim6g/sentry/sentry-navigation"
SENTRY_WS="${SENTRY_WS:-${SENTRY_WS_DEFAULT}}"
SENTRY_RS_ACTIVATE="${SENTRY_WS}/tools/activate_robostack.sh"

srs() {
  if [ ! -f "${SENTRY_RS_ACTIVATE}" ]; then
    echo "[ERROR] Missing ${SENTRY_RS_ACTIVATE}" >&2
    return 1
  fi
  # shellcheck source=/dev/null
  source "${SENTRY_RS_ACTIVATE}"
}

# Optional auto-activation when entering the workspace.
# Enable with: export AUTO_SENTRY_RS=1
if [ "${AUTO_SENTRY_RS:-0}" = "1" ]; then
  case "${PWD}" in
    "${SENTRY_WS}"|${SENTRY_WS}/*)
      if [ -f "${SENTRY_RS_ACTIVATE}" ]; then
        srs
      fi
      ;;
  esac
fi

