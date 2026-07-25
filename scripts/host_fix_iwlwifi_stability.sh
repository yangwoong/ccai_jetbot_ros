#!/usr/bin/env bash
set -euo pipefail

# Fixes a confirmed repeating Wi-Fi crash on this host's Intel AC 8265 card:
#
#   iwlwifi 0000:01:00.0: Queue 2 stuck for 10000 ms.
#   iwlwifi 0000:01:00.0: Microcode SW error detected. Restarting 0x2000000.
#
# Root-caused on real hardware (2026-07-25): NOT PCIe ASPM (disabling ASPM
# both at runtime and via the `pcie_aspm=off` kernel boot parameter did not
# stop the crash), NOT CPU starvation (tegrastats showed 25-35% CPU during
# a crash window), NOT weak signal (-28 to -29 dBm, tx retries/failed near
# zero). The actual trigger is a burst of several simultaneous new network
# connections over Wi-Fi at once - reproduced by unplugging the host's
# Ethernet cable while SSH'd in over Wi-Fi, which makes every connection
# that was preferring the lower-metric eth0 route (telegram_bridge_node's
# poll/send, vlm_client_node's cloud calls, apt/NTP, etc.) suddenly retry
# over wlan0 all at once. This matches a well-known class of iwlwifi bugs
# in the 802.11n frame-aggregation/block-ack queue path on this driver+
# firmware combo (loaded firmware version 22.391740.0) under bursty
# traffic. Disabling 11n aggregation fixes it.
#
# CONFIRMED FIX: reproduced the crash reliably (Ethernet-unplug test),
# applied this modprobe.d option file, rebooted, ran the same Ethernet-
# unplug test again with zero crashes.
#
# This requires a REBOOT to take effect (module options only apply when
# the iwlwifi module is next loaded) - unlike the disproven ASPM mitigation
# this replaces, there is no live/runtime equivalent for this option, so
# this script cannot "fix it now" the way host_fix_nvargus_daemon.sh can.

CONF_FILE="${CCAI_IWLWIFI_CONF_FILE:-/etc/modprobe.d/iwlwifi.conf}"
DESIRED_LINE='options iwlwifi 11n_disable=1 power_save=0 uapsd_disable=1'

run_as_root() {
  if [ "$(id -u)" = "0" ]; then
    "$@"
  else
    sudo "$@"
  fi
}

if [ -f "${CONF_FILE}" ] && grep -qF "${DESIRED_LINE}" "${CONF_FILE}" 2>/dev/null; then
  echo "[iwlwifi-stability-fix] ${CONF_FILE} already has the fix applied"
  exit 0
fi

echo "[iwlwifi-stability-fix] writing ${CONF_FILE} - REBOOT REQUIRED for this to take effect" >&2
run_as_root sh -c "echo '${DESIRED_LINE}' > '${CONF_FILE}'"
echo "[iwlwifi-stability-fix] wrote ${CONF_FILE}. Run 'sudo reboot' before relying on Wi-Fi stability." >&2
