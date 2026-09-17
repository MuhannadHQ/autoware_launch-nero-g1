#!/usr/bin/env bash
# =============================================================================
# launch_sensors.sh -- fairy_e1r_airy_left_sensor_kit
#
# Check that every sensor of this kit is reachable, then bring the drivers up
# exactly as they run inside the full Autoware stack (same node names,
# namespaces, topics and TF) via launch/sensing_standalone.launch.xml.
#
#   Kit inventory
#     Fairy       top    192.168.1.202  MSOP 6677  DIFOP 7766
#     E1R         front  192.168.1.200  MSOP 6699  DIFOP 7788
#     Airy Left   left   192.168.1.203  MSOP 6666  DIFOP 7755
#     Septentrio  GNSS   /dev/serial/by-id/usb-Septentrio_*-if00 (+ -if02 NTRIP)
#     Xsens MTi   IMU    /dev/serial/by-id/usb-Xsens_*
#
#   Usage
#     launch_sensors.sh [options] [extra ros2-launch args key:=value ...]
#
#     --check-only          run the sensor checks and exit (0 = all OK)
#     --skip-checks         launch without checking
#     --force               launch even if a check fails
#     --gnss-module MOD     septentrio (default) | xsens
#     --no-gnss             do not launch GNSS/IMU (and do not check them)
#     --no-lidar            do not launch the LiDAR drivers (and do not check them)
#     --no-tf               do not start robot_state_publisher (TF from elsewhere)
#     --vehicle-model NAME  vehicle description prefix (default nero_vehicle)
#     --probe-sec N         seconds to listen for LiDAR UDP packets (default 3)
#     -h, --help
#
#   Environment (all optional)
#     AUTOWARE_SETUP   autoware workspace setup.bash to source
#     DRIVERS_SETUP    drivers overlay setup.bash to source
#     AUTOWARE_ENV     per-machine env file (RMW, CYCLONEDDS_URI, ...)
#     ROS_DOMAIN_ID    defaults to 42 (drivers/scripts/source.sh convention)
#
#   Exit codes: 0 launched / checks passed, 1 a check failed, 2 usage/env error.
#
# Ctrl-C stops every node (ros2 launch handles the signal).
# =============================================================================
set -euo pipefail

KIT_PKG="fairy_e1r_airy_left_sensor_kit_launch"
LAUNCH_FILE="sensing_standalone.launch.xml"

# name | namespace | ip | msop port | difop port
LIDARS=(
  "Fairy|top|192.168.1.202|6677|7766"
  "E1R|front|192.168.1.200|6699|7788"
  "Airy Left|left|192.168.1.203|6666|7755"
)
LIDAR_SUBNET="192.168.1."
SEPTENTRIO_DEV_GLOB="/dev/serial/by-id/usb-Septentrio_*-if00"
SEPTENTRIO_NTRIP_GLOB="/dev/serial/by-id/usb-Septentrio_*-if02"
XSENS_DEV_GLOB="/dev/serial/by-id/usb-Xsens_*"

AUTOWARE_SETUP="${AUTOWARE_SETUP:-/work/autoware/v0.44.2/autoware-nero-g1/install/setup.bash}"
DRIVERS_SETUP="${DRIVERS_SETUP:-/work/autoware/v0.44.2/drivers/install/setup.bash}"
AUTOWARE_ENV="${AUTOWARE_ENV:-$HOME/.autoware_env.sh}"

# ----------------------------------------------------------------- options
CHECK_ONLY=0; SKIP_CHECKS=0; FORCE=0
GNSS_MODULE="septentrio"; LAUNCH_GNSS=true; LAUNCH_LIDAR=true; LAUNCH_TF=true
VEHICLE_MODEL="nero_vehicle"; PROBE_SEC=3
EXTRA_ARGS=()

usage() { sed -n '2,/^# ====.*$/p' "$0" | sed -n '2,$p' | sed 's/^# \{0,1\}//' | sed '$d'; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check-only)   CHECK_ONLY=1 ;;
    --skip-checks)  SKIP_CHECKS=1 ;;
    --force)        FORCE=1 ;;
    --gnss-module)  GNSS_MODULE="$2"; shift ;;
    --no-gnss)      LAUNCH_GNSS=false ;;
    --no-lidar)     LAUNCH_LIDAR=false ;;
    --no-tf)        LAUNCH_TF=false ;;
    --vehicle-model) VEHICLE_MODEL="$2"; shift ;;
    --probe-sec)    PROBE_SEC="$2"; shift ;;
    -h|--help)      usage; exit 0 ;;
    *:=*)           EXTRA_ARGS+=("$1") ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done
case "$GNSS_MODULE" in septentrio|xsens) ;; *) echo "--gnss-module must be septentrio or xsens" >&2; exit 2 ;; esac

# ----------------------------------------------------------------- output
if [[ -t 1 ]]; then C_OK=$'\e[32m'; C_ERR=$'\e[31m'; C_WARN=$'\e[33m'; C_HDR=$'\e[1m'; C_END=$'\e[0m'
else C_OK=""; C_ERR=""; C_WARN=""; C_HDR=""; C_END=""; fi
hdr()  { echo; echo "${C_HDR}== $*${C_END}"; }
ok()   { echo "  ${C_OK}[ OK ]${C_END} $*"; }
warn() { echo "  ${C_WARN}[WARN]${C_END} $*"; }
fail() { echo "  ${C_ERR}[FAIL]${C_END} $*"; FAILED=1; }
FAILED=0

# ----------------------------------------------------------------- environment
# Same chain the launcher GUI and drivers/scripts/source.sh use:
# per-machine env file, autoware workspace, drivers overlay.
setup_env() {
  set +u  # colcon setup scripts reference unset variables
  # shellcheck source=/dev/null
  [[ -f "$AUTOWARE_ENV" ]] && source "$AUTOWARE_ENV"
  if ! command -v ros2 >/dev/null 2>&1 || ! ros2 pkg prefix "$KIT_PKG" >/dev/null 2>&1; then
    [[ -f "$AUTOWARE_SETUP" ]] || { echo "AUTOWARE_SETUP not found: $AUTOWARE_SETUP" >&2; exit 2; }
    # shellcheck source=/dev/null
    source "$AUTOWARE_SETUP"
  fi
  if ! ros2 pkg prefix septentrio_gnss_bridge >/dev/null 2>&1; then
    # shellcheck source=/dev/null
    [[ -f "$DRIVERS_SETUP" ]] && source "$DRIVERS_SETUP"
  fi
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  set -u
  for p in "$KIT_PKG" tier4_vehicle_launch autoware_launch "${VEHICLE_MODEL}_description" \
           fairy_nebula_driver e1r_nebula_driver airy_nebula_driver; do
    ros2 pkg prefix "$p" >/dev/null 2>&1 || { echo "package not found in sourced workspaces: $p" >&2; exit 2; }
  done
}

# Prefer the launch file next to this script (source tree) so the script can be
# tested before the package is rebuilt; otherwise use the installed one.
resolve_launch() {
  local here; here="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
  if [[ -f "$here/../launch/$LAUNCH_FILE" ]]; then
    echo "$here/../launch/$LAUNCH_FILE"
  else
    echo "$(ros2 pkg prefix "$KIT_PKG")/share/$KIT_PKG/launch/$LAUNCH_FILE"
  fi
}

# ----------------------------------------------------------------- probes
# Listen on a UDP port for up to $2 seconds; prints: packets <n> <src-ip> |
# busy (already bound: a driver is running) | none.
udp_probe() {
  python3 - "$1" "$2" <<'PY'
import socket, sys, time
port, secs = int(sys.argv[1]), float(sys.argv[2])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    s.bind(("0.0.0.0", port))
except OSError as e:
    print("busy" if e.errno in (98, 13) else f"error {e}"); sys.exit(0)
s.settimeout(0.5)
n, src, end = 0, None, time.monotonic() + secs
while time.monotonic() < end:
    try:
        _, addr = s.recvfrom(65535)
        n += 1; src = src or addr[0]
        if n >= 20: break
    except socket.timeout:
        pass
print(f"packets {n} {src}" if n else "none")
PY
}

check_network() {
  hdr "Host network"
  if ip -4 -o addr 2>/dev/null | grep -q " inet ${LIDAR_SUBNET//./\\.}"; then
    ok "host has an address on ${LIDAR_SUBNET}0/24"
  else
    warn "no host address on ${LIDAR_SUBNET}0/24 (see nero-jetson-setup/scripts/setup_lidar_network.sh)"
  fi
}

check_lidars() {
  hdr "LiDARs (ping + ${PROBE_SEC}s UDP listen on the MSOP port)"
  local entry name ns ip msop difop pingr probe
  for entry in "${LIDARS[@]}"; do
    IFS='|' read -r name ns ip msop difop <<<"$entry"
    if ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then pingr="ping ok"; else pingr="no ping reply"; fi
    probe="$(udp_probe "$msop" "$PROBE_SEC")"
    case "$probe" in
      packets*)
        read -r _ n src <<<"$probe"
        if [[ "$src" == "$ip" ]]; then ok "$name ($ns, $ip): $n packets on udp/$msop, $pingr"
        else warn "$name ($ns): packets on udp/$msop come from $src, expected $ip ($pingr)"; fi ;;
      busy)  fail "$name ($ns): udp/$msop already bound -- is a $name driver already running?" ;;
      none)  fail "$name ($ns, $ip): no packets on udp/$msop in ${PROBE_SEC}s, $pingr" ;;
      *)     fail "$name ($ns): probe error: $probe" ;;
    esac
  done
}

check_gnss() {
  hdr "GNSS / IMU (gnss_module=$GNSS_MODULE)"
  local d
  if [[ "$GNSS_MODULE" == "septentrio" ]]; then
    d=$(compgen -G "$SEPTENTRIO_DEV_GLOB" | head -1 || true)
    if [[ -n "$d" ]]; then ok "Septentrio receiver: $d"; else fail "Septentrio receiver not found ($SEPTENTRIO_DEV_GLOB)"; fi
    d=$(compgen -G "$SEPTENTRIO_NTRIP_GLOB" | head -1 || true)
    if [[ -n "$d" ]]; then ok "Septentrio RTCM/NTRIP port: $d"; else warn "Septentrio RTCM port not found ($SEPTENTRIO_NTRIP_GLOB) -- no RTK corrections"; fi
  fi
  # The Xsens runs in both modes (gnss_imu, or imu_only next to Septentrio).
  d=$(compgen -G "$XSENS_DEV_GLOB" | head -1 || true)
  if [[ -n "$d" ]]; then ok "Xsens MTi: $d"; else fail "Xsens MTi not found ($XSENS_DEV_GLOB)"; fi
}

check_conflicts() {
  hdr "Already-running nodes"
  local nodes
  nodes="$(timeout 15 ros2 node list 2>/dev/null || true)"
  local clash=""
  for n in /pointcloud_container /robot_state_publisher /sensing/lidar/concatenate_data /sensing/lidar/top/fairy_ros_wrapper_node /sensing/lidar/front/e1r_ros_wrapper_node /sensing/lidar/left/airy_left_ros_wrapper_node /sensing/gnss/septentrio_gnss_driver /sensing/gnss/xsens_mti_node; do
    grep -qx "$n" <<<"$nodes" && clash+=" $n"
  done
  if [[ -n "$clash" ]]; then fail "nodes already running on ROS_DOMAIN_ID=$ROS_DOMAIN_ID:$clash"
  else ok "no conflicting nodes on ROS_DOMAIN_ID=$ROS_DOMAIN_ID"; fi
}

# ----------------------------------------------------------------- main
setup_env
LAUNCH_PATH="$(resolve_launch)"

if [[ $SKIP_CHECKS -eq 0 ]]; then
  echo "${C_HDR}Sensor check: $KIT_PKG${C_END}"
  check_network
  [[ "$LAUNCH_LIDAR" == true ]] && check_lidars
  [[ "$LAUNCH_GNSS" == true ]] && check_gnss
  check_conflicts
  echo
  if [[ $FAILED -ne 0 ]]; then
    if [[ $CHECK_ONLY -eq 1 || $FORCE -eq 0 ]]; then
      echo "${C_ERR}Sensor check FAILED${C_END} -- not launching (use --force to launch anyway)."
      exit 1
    fi
    echo "${C_WARN}Sensor check FAILED -- launching anyway (--force).${C_END}"
  else
    echo "${C_OK}All sensor checks passed.${C_END}"
  fi
  [[ $CHECK_ONLY -eq 1 ]] && exit 0
fi

CMD=(ros2 launch "$LAUNCH_PATH"
     "vehicle_model:=$VEHICLE_MODEL" "gnss_module:=$GNSS_MODULE"
     "launch_gnss:=$LAUNCH_GNSS" "launch_lidar:=$LAUNCH_LIDAR" "launch_tf:=$LAUNCH_TF"
     "${EXTRA_ARGS[@]}")
hdr "Launching"
printf '  %q' "${CMD[@]}"; echo; echo
exec "${CMD[@]}"
