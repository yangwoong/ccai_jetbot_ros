#!/usr/bin/env bash
set -euo pipefail

UNIT_FILE="${CCAI_NVARGUS_UNIT_FILE:-/etc/systemd/system/nvargus-daemon.service}"
BROKEN_LINE='Environment="enableCamInfiniteTimeout=1"'

run_as_root() {
  if [ "$(id -u)" = "0" ]; then
    "$@"
  else
    sudo "$@"
  fi
}

if [ ! -f "${UNIT_FILE}" ]; then
  # Not every host runs nvargus-daemon (e.g. USB-only setups); nothing to fix.
  exit 0
fi

if ! grep -qF "${BROKEN_LINE}" "${UNIT_FILE}" 2>/dev/null; then
  echo "[nvargus-fix] ${UNIT_FILE} already clean"
  exit 0
fi

echo "[nvargus-fix] found '${BROKEN_LINE}' in ${UNIT_FILE} - this breaks Argus" \
     "CaptureSession creation on this L4T/imx219 combo. Removing it." >&2

run_as_root cp "${UNIT_FILE}" "${UNIT_FILE}.bak.$(date +%s)"
run_as_root sed -i '/Environment="enableCamInfiniteTimeout=1"/d' "${UNIT_FILE}"
run_as_root systemctl daemon-reload
run_as_root systemctl restart nvargus-daemon
echo "[nvargus-fix] nvargus-daemon restarted without enableCamInfiniteTimeout" >&2
