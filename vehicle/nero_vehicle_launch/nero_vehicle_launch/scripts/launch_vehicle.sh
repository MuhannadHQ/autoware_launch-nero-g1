#!/usr/bin/env bash
# =============================================================================
# launch_vehicle.sh -- nero_vehicle
#
# Check that the chassis is reachable over CAN, then bring up the vehicle
# interface exactly as it runs inside the full Autoware stack
# (vehicle_interface.launch.xml: yhs_autoware_bridge + yhs_can_control).
#
# Publishes /vehicle/status/{velocity,steering,gear,control_mode,
# turn_indicators}_status, /vehicle/status/battery_status and /chassis_info_fb
# -- the vehicle topics the logging simulator needs from a bag. Companion of
# the sensor kits' launch_sensors.sh: run both for an Autoware-replayable bag.
#
# NOTE: with no Autoware control command the bridge keeps sending its safe-stop
# (velocity 0, brake 100) to the chassis. Never run this while the full
# Autoware stack is up: that would be two CAN bridges writing to the chassis.
#
#   Usage
#     launch_vehicle.sh [options] [extra ros2-launch args key:=value ...]
#
#     --check-only      run the checks and exit (0 = all OK)
#     --skip-checks     launch without checking
#     --force           launch even if a check fails
#     --can IFACE       CAN interface (default pcan0, as in yhs_can_control cfg.yaml)
#     --probe-sec N     seconds to listen for CAN frames (default 3)
#     -h, --help
#
#   Environment (all optional)
#     AUTOWARE_SETUP   autoware workspace setup.bash to source
#     DRIVERS_SETUP    drivers overlay setup.bash to source
#     AUTOWARE_ENV     per-machine env file (RMW, CYCLONEDDS_URI, ...)
#     ROS_DOMAIN_ID    defaults to 42
#
#   Exit codes: 0 launched / checks passed, 1 a check failed, 2 usage/env error.
#
# Ctrl-C stops both nodes (ros2 launch handles the signal).
# =============================================================================
set -euo pipefail

PKG="nero_vehicle_launch"
LAUNCH_FILE="vehicle_interface.launch.xml"
CAN_IF="pcan0"

AUTOWARE_SETUP="${AUTOWARE_SETUP:-/work/autoware/v0.44.2/autoware-nero-g1/install/setup.bash}"
DRIVERS_SETUP="${DRIVERS_SETUP:-/work/autoware/v0.44.2/drivers/install/setup.bash}"
AUTOWARE_ENV="${AUTOWARE_ENV:-$HOME/.autoware_env.sh}"

# ----------------------------------------------------------------- options
CHECK_ONLY=0; SKIP_CHECKS=0; FORCE=0; PROBE_SEC=3
EXTRA_ARGS=()

usage() { sed -n '2,/^# ====.*$/p' "$0" | sed -n '2,$p' | sed 's/^# \{0,1\}//' | sed '$d'; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check-only)   CHECK_ONLY=1 ;;
    --skip-checks)  SKIP_CHECKS=1 ;;
    --force)        FORCE=1 ;;
    --can)          CAN_IF="$2"; shift ;;
    --probe-sec)    PROBE_SEC="$2"; shift ;;
    -h|--help)      usage; exit 0 ;;
    *:=*)           EXTRA_ARGS+=("$1") ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

# ----------------------------------------------------------------- output
if [[ -t 1 ]]; then C_OK=$'\e[32m'; C_ERR=$'\e[31m'; C_WARN=$'\e[33m'; C_HDR=$'\e[1m'; C_END=$'\e[0m'
else C_OK=""; C_ERR=""; C_WARN=""; C_HDR=""; C_END=""; fi
hdr()  { echo; echo "${C_HDR}== $*${C_END}"; }
ok()   { echo "  ${C_OK}[ OK ]${C_END} $*"; }
warn() { echo "  ${C_WARN}[WARN]${C_END} $*"; }
fail() { echo "  ${C_ERR}[FAIL]${C_END} $*"; FAILED=1; }
FAILED=0

# ----------------------------------------------------------------- environment
# Same chain as launch_sensors.sh / the launcher GUI: per-machine env file,
# autoware workspace, drivers overlay.
setup_env() {
  set +u  # colcon setup scripts reference unset variables
  # shellcheck source=/dev/null
  [[ -f "$AUTOWARE_ENV" ]] && source "$AUTOWARE_ENV"
  if ! command -v ros2 >/dev/null 2>&1 || ! ros2 pkg prefix "$PKG" >/dev/null 2>&1; then
    [[ -f "$AUTOWARE_SETUP" ]] || { echo "AUTOWARE_SETUP not found: $AUTOWARE_SETUP" >&2; exit 2; }
    # shellcheck source=/dev/null
    source "$AUTOWARE_SETUP"
  fi
  if ! ros2 pkg prefix yhs_autoware_bridge >/dev/null 2>&1; then
    # shellcheck source=/dev/null
    [[ -f "$DRIVERS_SETUP" ]] && source "$DRIVERS_SETUP"
  fi
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  set -u
  for p in "$PKG" yhs_autoware_bridge yhs_can_control; do
    ros2 pkg prefix "$p" >/dev/null 2>&1 || { echo "package not found in sourced workspaces: $p" >&2; exit 2; }
  done
}

# Prefer the launch file next to this script (source tree); else the installed one.
resolve_launch() {
  local here; here="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
  if [[ -f "$here/../launch/$LAUNCH_FILE" ]]; then
    echo "$here/../launch/$LAUNCH_FILE"
  else
    echo "$(ros2 pkg prefix "$PKG")/share/$PKG/launch/$LAUNCH_FILE"
  fi
}

# ----------------------------------------------------------------- checks
check_can() {
  hdr "CAN ($CAN_IF)"
  local state
  state="$(ip -br link show "$CAN_IF" 2>/dev/null | awk '{print $2}')"
  if [[ -z "$state" ]]; then fail "$CAN_IF does not exist (PCAN driver loaded? see nero-jetson-setup/scripts/setup_pcan_driver.sh)"; return; fi
  if [[ "$state" != "UP" ]]; then fail "$CAN_IF is $state (try: sudo ip link set $CAN_IF up)"; return; fi
  ok "$CAN_IF is UP"
  if ! command -v candump >/dev/null 2>&1; then warn "candump not installed (can-utils): cannot verify chassis traffic"; return; fi
  local frames
  frames="$(timeout "$PROBE_SEC" candump "$CAN_IF" 2>/dev/null | wc -l || true)"
  if [[ "$frames" -gt 0 ]]; then ok "chassis is talking: $frames frames in ${PROBE_SEC}s"
  else fail "no CAN frames on $CAN_IF in ${PROBE_SEC}s -- chassis powered? cable?"; fi
}

check_conflicts() {
  hdr "Already-running nodes"
  local nodes clash=""
  nodes="$(timeout 15 ros2 node list 2>/dev/null || true)"
  for n in /bridge_node /yhs_can_control_node; do
    grep -qx "$n" <<<"$nodes" && clash+=" $n"
  done
  # Also catch a bridge started outside ROS discovery (e.g. a different domain).
  pgrep -f "yhs_autoware_bridge/lib/yhs_autoware_bridge/bridge_node" >/dev/null && clash+=" (bridge_node process)"
  if [[ -n "$clash" ]]; then fail "vehicle interface already running (Autoware stack up?):$clash -- two CAN bridges must never run at once"
  else ok "no vehicle interface running on ROS_DOMAIN_ID=$ROS_DOMAIN_ID"; fi
}

# ----------------------------------------------------------------- main
setup_env
LAUNCH_PATH="$(resolve_launch)"

if [[ $SKIP_CHECKS -eq 0 ]]; then
  echo "${C_HDR}Vehicle check: $PKG${C_END}"
  check_can
  check_conflicts
  echo
  if [[ $FAILED -ne 0 ]]; then
    if [[ $CHECK_ONLY -eq 1 || $FORCE -eq 0 ]]; then
      echo "${C_ERR}Vehicle check FAILED${C_END} -- not launching (use --force to launch anyway)."
      exit 1
    fi
    echo "${C_WARN}Vehicle check FAILED -- launching anyway (--force).${C_END}"
  else
    echo "${C_OK}All vehicle checks passed.${C_END}"
  fi
  [[ $CHECK_ONLY -eq 1 ]] && exit 0
fi

CMD=(ros2 launch "$LAUNCH_PATH" "${EXTRA_ARGS[@]}")
hdr "Launching"
printf '  %q' "${CMD[@]}"; echo; echo
exec "${CMD[@]}"
