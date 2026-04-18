#!/usr/bin/env bash
# Start an autonomous driving session.
# Usage: ./start_session.sh [track_name] [event_type] [num_laps]
# Example: ./start_session.sh track_20260404_013726.csv trackdrive 1

TRACK=${1:-track_20260404_013726.csv}
EVENT=${2:-trackdrive}
LAPS=${3:-1}
API=http://localhost:8000

wait_connected() {
    echo "Waiting for UE5..."
    for i in $(seq 1 30); do
        STATUS=$(curl -s "$API/api/sim/status" 2>/dev/null)
        CONNECTED=$(echo "$STATUS" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('connected','false'))" 2>/dev/null)
        if [ "$CONNECTED" = "True" ] || [ "$CONNECTED" = "true" ]; then
            echo "Connected! fps=$(echo "$STATUS" | python3 -c "import sys,json; print(json.load(sys.stdin).get('fps',0))" 2>/dev/null)"
            return 0
        fi
        sleep 2
    done
    echo "ERROR: Could not connect to UE5 after 60s"
    return 1
}

wait_connected || exit 1

echo "Loading track: $TRACK"
curl -s -X POST "$API/api/track/$TRACK/load" | python3 -c "import sys,json; d=json.load(sys.stdin); print(f\"  Loaded {d['result']['cones']} cones\")" 2>/dev/null

echo "Activating RES..."
curl -s -X POST "$API/api/res/activate" > /dev/null

echo "Starting event: $EVENT ($LAPS laps)"
curl -s -X POST "$API/api/event/start" \
    -H "Content-Type: application/json" \
    -d "{\"event_type\":\"$EVENT\",\"num_laps\":$LAPS}" | python3 -c "import sys,json; d=json.load(sys.stdin); print(f\"  {d}\")" 2>/dev/null

sleep 1
echo "Releasing RES (enables api_control)..."
curl -s -X POST "$API/api/res/release" > /dev/null

echo "Done. Monitoring..."
for i in $(seq 1 10); do
    sleep 2
    curl -s "$API/api/vehicle/state" | python3 -c "
import sys, json
d = json.load(sys.stdin)
x, y = d.get('x',0), d.get('y',0)
spd = d.get('speed', 0)
thr = d.get('controls', {}).get('throttle', 0)
steer = d.get('controls', {}).get('steering', 0)
print(f'  speed={spd:.2f} x={x:.1f} y={y:.1f} thr={thr:.2f} steer={steer:.2f}')
" 2>/dev/null
done
