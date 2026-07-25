#!/usr/bin/env bash
set -euo pipefail

# Mitigates a repeating "iwlwifi ... Queue N stuck for 10000 ms" /
# "Microcode SW error detected. Restarting" crash loop on this host's Intel
# AC 8265 Wi-Fi card, observed starting right when `docker run` brings up the
# D435i (RSUSB/libusb backend, raw USB bus access) plus visual_odom_node's
# per-frame CPU load (ORB+Kabsch) at the same time. That combination spikes
# USB bus traffic and CPU load simultaneously at container startup, and the
# iwlwifi log shows repeated "L1 Enabled - LTR Enabled" (PCIe ASPM power
# state transitions) right before each crash - a known iwlwifi failure mode
# when the PCIe link is aggressively power-managed under bus contention.
#
# This has NOT been confirmed as the definitive root cause on this exact
# hardware (Jetson Nano carrier board + M.2 8265) - it's a correlation-based
# hypothesis matching a well-documented class of iwlwifi/ASPM bugs. If
# disabling ASPM doesn't stop the crash loop, the next suspect is shared
# power-rail stress (D435i USB + Wi-Fi module drawing from the same rail
# under load) rather than anything PCIe-specific.
#
# Setting the aspm policy to "performance" disables ASPM's L0s/L1 power
# states link-wide (not just for the Wi-Fi device) - this takes effect
# immediately with no reboot needed, and is safe to reapply on every run.

POLICY_FILE="${CCAI_PCIE_ASPM_POLICY_FILE:-/sys/module/pcie_aspm/parameters/policy}"

run_as_root() {
  if [ "$(id -u)" = "0" ]; then
    "$@"
  else
    sudo "$@"
  fi
}

if [ ! -f "${POLICY_FILE}" ]; then
  # Kernel doesn't expose the aspm policy knob (or ASPM support isn't built
  # in) - nothing this script can do.
  echo "[iwlwifi-aspm-fix] ${POLICY_FILE} not present, skipping" >&2
  exit 0
fi

current="$(cat "${POLICY_FILE}" 2>/dev/null | tr -d '\n' || true)"
if echo "${current}" | grep -q '\[performance\]'; then
  echo "[iwlwifi-aspm-fix] pcie_aspm policy already 'performance'"
  exit 0
fi

echo "[iwlwifi-aspm-fix] current pcie_aspm policy: ${current}" >&2
if run_as_root sh -c "echo performance > '${POLICY_FILE}'" 2>/dev/null; then
  echo "[iwlwifi-aspm-fix] set pcie_aspm policy to 'performance' (ASPM L0s/L1 disabled)" >&2
else
  echo "[iwlwifi-aspm-fix] warning: could not write ${POLICY_FILE} (BIOS/UEFI ASPM override, or no permission) - continuing anyway" >&2
fi
