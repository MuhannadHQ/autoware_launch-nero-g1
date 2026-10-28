#!/bin/bash
# usb2_hub.sh - run the external USB hub at USB 2.0 so its USB 3 link cannot jam the GNSS.
#
# The sensor kit's driver launchers (sensing.launch.xml, sensing_standalone.launch.xml) start this
# once with the drivers. It checks what is connected through the external hub (Genesys GL3523:
# USB 2 half 05e3:0610, USB 3 half 05e3:0626; PCAN, Xsens and the Kingston on nero-jetson) and,
# if the USB 3 half is on, switches that half off. Everything on the hub then runs at USB 2.0.
#
# Why: the hub's USB 3 link jams the Septentrio's L1 (the receiver itself is on a Jetson port).
# 2026-10-28 on nero-jetson: a hub replug at 14:37 took main RF power 9 -> 3 and tracking to 0;
# with the USB 3 half off, RF was back to 9 in 1 s and RTK fixed by 14:42. Receiver resets do
# not help.
#
#   usb2_hub.sh                 check the hub, make it USB 2.0 (needs the one-time install)
#   sudo bash usb2_hub.sh install     one-time: root copy in /usr/local/sbin + a sudoers entry
#   sudo bash usb2_hub.sh uninstall   remove both (the hub comes back at USB 3 at the next replug)
#
# The switch-off is root-only (sysfs "authorized"), so the check runs the root copy through
# `sudo -n /usr/local/sbin/nero-usb2-hub switch`; that exact command is all the sudoers entry allows.
set -u
USB3_HALF=05e3:0626
USB2_HALF=05e3:0610
SYS="${NERO_USB_SYSFS:-/sys/bus/usb/devices}"   # override for tests only
ROOT_COPY=/usr/local/sbin/nero-usb2-hub
SUDOERS=/etc/sudoers.d/nero-usb2-hub
SELF="$(readlink -f "${BASH_SOURCE[0]}")"

say() { echo "[usb2_hub] $*"; }

devs_with() {  # <vid:pid> -> sysfs dirs of the USB devices with that id
    local d
    for d in "$SYS"/*; do
        [ -f "$d/idVendor" ] || continue
        [ "$(cat "$d/idVendor"):$(cat "$d/idProduct")" = "$1" ] && echo "$d"
    done
}

through() {  # <hub sysfs dir> -> one line per device connected through it
    local n c
    n="$(basename "$1")"
    for c in "$SYS/$n".*; do
        [ -f "$c/idVendor" ] || continue
        say "    $(basename "$c") $(cat "$c/idVendor"):$(cat "$c/idProduct") $(cat "$c/product" 2>/dev/null) $(cat "$c/speed")M"
    done
}

show() {
    local d
    for d in $(devs_with $USB2_HALF); do say "  USB 2 half $(basename "$d"):"; through "$d"; done
    for d in $(devs_with $USB3_HALF); do
        if [ "$(cat "$d/authorized")" = 1 ]; then say "  USB 3 half $(basename "$d"): ON"; through "$d"
        else say "  USB 3 half $(basename "$d"): off"; fi
    done
}

switch() {  # root: switch off every USB 3 half that is on
    local d rc=0
    for d in $(devs_with $USB3_HALF); do
        [ "$(cat "$d/authorized")" = 1 ] || continue
        if echo 0 > "$d/authorized"; then
            logger -t nero-usb2-hub "USB 3 half of the external hub ($(basename "$d")) switched off: hub runs at USB 2.0" 2>/dev/null
        else
            rc=1
        fi
    done
    return $rc
}

check() {
    local d on=0
    if [ -z "$(devs_with $USB2_HALF)$(devs_with $USB3_HALF)" ]; then say "no external hub connected"; return 0; fi
    for d in $(devs_with $USB3_HALF); do [ "$(cat "$d/authorized")" = 1 ] && on=1; done
    say "connected through the external hub:"
    show
    if [ "$on" = 0 ]; then say "hub already at USB 2.0"; return 0; fi
    if [ "$(id -u)" = 0 ]; then switch; else sudo -n "$ROOT_COPY" switch; fi
    for d in $(devs_with $USB3_HALF); do
        if [ "$(cat "$d/authorized")" = 1 ]; then
            say "ERROR: the hub's USB 3 half is still on (it jams the GNSS). One-time fix: sudo bash $SELF install"
            return 1
        fi
    done
    sleep 3   # what was on the USB 3 half re-enumerates on the USB 2 half
    say "hub switched to USB 2.0:"
    show
    if [ -f "$ROOT_COPY" ] && ! cmp -s "$SELF" "$ROOT_COPY"; then
        say "note: $ROOT_COPY differs from $SELF; re-run: sudo bash $SELF install"
    fi
}

case "${1:-check}" in
    check) check ;;
    switch)
        [ "$(id -u)" = 0 ] || { say "switch needs root"; exit 2; }
        switch ;;
    install)
        [ "$(id -u)" = 0 ] || { say "run with sudo"; exit 2; }
        user="${SUDO_USER:-nero}"
        install -o root -g root -m 0755 "$SELF" "$ROOT_COPY" || exit 1
        tmp="$(mktemp)"
        echo "$user ALL=(root) NOPASSWD: $ROOT_COPY switch" > "$tmp"
        visudo -cf "$tmp" >/dev/null || { rm -f "$tmp"; say "sudoers entry rejected by visudo"; exit 1; }
        install -o root -g root -m 0440 "$tmp" "$SUDOERS"; rm -f "$tmp"
        say "installed $ROOT_COPY and $SUDOERS ($user may run '$ROOT_COPY switch')"
        sudo -u "$user" sudo -n "$ROOT_COPY" switch && say "checked: $user can switch the hub without a password" ;;
    uninstall)
        [ "$(id -u)" = 0 ] || { say "run with sudo"; exit 2; }
        rm -f "$SUDOERS" "$ROOT_COPY"
        say "removed $ROOT_COPY and $SUDOERS" ;;
    *) sed -n '2,19p' "$SELF"; exit 2 ;;
esac
