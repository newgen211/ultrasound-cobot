#!/usr/bin/env bash
# flight.sh <launch.jsonl> <section N> [speed] [rate] [delay]
# Pi side of one sweep: probe on wlan0, eth0 untouched, serial arm.
# Capture -> arm -> stop capture -> pair the exec log into the section folder.
set -euo pipefail
LAUNCH=$1; SEC=$2; SPEED=${3:-25}; RATE=${4:-2}; DELAY=${5:-8}
cd ~/Documents/ultrasound-cobot
source probe.env
[ -s "$LAUNCH" ] || { echo "ABORT: no launch file $LAUNCH"; exit 1; }
SESS=clarius_sessions/section_$SEC
[ -e "$SESS" ] && { echo "ABORT: $SESS exists"; exit 1; }
echo "== network"
nmcli con down Hotspot-1 2>/dev/null || true
nmcli con up "$PROBE_CON"
ping -c1 -W2 "$PROBE_IP" >/dev/null || { echo "ABORT: probe unreachable"; exit 1; }
ip -4 addr show wlan0 | grep inet
echo "== capture"
mkdir -p clarius_sessions
python3.10 cast_headless.py --section "$SEC" --ip "$PROBE_IP" --port "$PROBE_PORT" > "$SESS.capture.log" 2>&1 &
CAP=$!
sleep 6
n=$(ls "$SESS"/raw_*.bin 2>/dev/null | wc -l)
[ "$n" -gt 0 ] || { kill "$CAP" 2>/dev/null; cat "$SESS.capture.log"; echo "ABORT: no frames"; exit 1; }
python3 - "$SESS" <<'EOF'
import json,sys,glob
f=sorted(glob.glob(sys.argv[1]+"/raw_*.json"))[0]
n=json.load(open(f))["imu_sample_count"]; print("imu_sample_count",n)
sys.exit(0 if n>0 else 1)
EOF
echo "== arm"
python3 execute_sweep.py "$LAUNCH" --speed "$SPEED" --rate "$RATE" --delay "$DELAY" --no-prompt
echo "== stop capture"
kill -INT "$CAP"; wait "$CAP" || true
echo "== pair"
EXEC=$(ls -t pose_logs/exec_*.jsonl | head -1); META=${EXEC%.jsonl}_meta.json
cp "$EXEC" "$SESS/exec.jsonl"; cp "$META" "$SESS/exec_meta.json"
echo "section_$SEC frames=$(ls "$SESS"/raw_*.bin | wc -l) exec=$(wc -l < "$EXEC") from $EXEC"
