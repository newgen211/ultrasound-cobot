#!/usr/bin/env bash
# Pi side of one sweep, in two steps so the Mac can switch networks between them.
#
#   ./flight.sh net
#       Bring wlan0 onto the probe's network (eth0 and ZeroTier untouched) and
#       print WLAN0=<ip>, the address the Mac uses once it joins that network too.
#
#   ./flight.sh fly <launch.jsonl> [speed] [rate] [delay]
#       Play the launch file and print EXEC=<pose_logs/exec_<stamp>.jsonl>.
#
# Capture runs on the Mac: the Cast SDK needs a GL context this Pi cannot give it
# (see ONE_BUTTON.md). The frames land on the Mac; only this exec log comes back.
set -euo pipefail
cd ~/Documents/ultrasound-cobot
source probe.env

case "${1:-}" in
net)
    sudo nmcli con down Hotspot-1 2>/dev/null || true
    sudo nmcli con up "$PROBE_CON"
    ping -c1 -W2 "$PROBE_IP" >/dev/null || { echo "ABORT: probe unreachable"; exit 1; }
    ip=$(ip -4 -br addr show wlan0 | awk '{print $3}' | cut -d/ -f1)
    [ -n "$ip" ] || { echo "ABORT: wlan0 has no address"; exit 1; }
    echo "WLAN0=$ip"
    ;;
fly)
    LAUNCH=${2:?launch file}; SPEED=${3:-25}; RATE=${4:-2}; DELAY=${5:-8}
    [ -s "$LAUNCH" ] || { echo "ABORT: no launch file $LAUNCH"; exit 1; }
    before=$(ls -t pose_logs/exec_*.jsonl 2>/dev/null | head -1 || true)
    python3 execute_sweep.py "$LAUNCH" --speed "$SPEED" --rate "$RATE" --delay "$DELAY" --no-prompt
    EXEC=$(ls -t pose_logs/exec_*.jsonl | head -1)
    [ "$EXEC" != "$before" ] || { echo "ABORT: no new exec log, the arm never played"; exit 1; }
    [ -s "${EXEC%.jsonl}_meta.json" ] || { echo "ABORT: $EXEC has no meta json"; exit 1; }
    echo "EXEC=$EXEC"
    ;;
*)
    echo "usage: flight.sh net | flight.sh fly <launch.jsonl> [speed] [rate] [delay]"; exit 2
    ;;
esac
