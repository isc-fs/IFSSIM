#!/usr/bin/env python3
"""
Closed-loop reset test:
  1. Load track → car aligned to start gate (orange cones)
  2. Stop pipeline so it doesn't fight manual commands
  3. Drive straight >5m (background ros_pub + teleport kick to break ω=0)
  4. Steer gently to change heading
  5. Brake and wait until fully stopped
  6. Activate EBS via /signal/ebs
  7. Wait for bridge command client to be connected
  8. Call ROS /reset service
  9. Verify car is back at start gate (within 1m)
"""
import subprocess, time, math, json, urllib.request, sys

API = "http://localhost:8000"
CONTAINER = "ifssim-dv_pipeline_stack-1"
ROS_SETUP = "source /opt/ros/humble/setup.bash && source /dv_pipeline_stack_ws/install/setup.bash"


def get(path):
    with urllib.request.urlopen(f"{API}{path}", timeout=5) as r:
        return json.loads(r.read())


def post(path, body=None):
    data = json.dumps(body).encode() if body else b""
    req = urllib.request.Request(f"{API}{path}", data=data,
          headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read())


def ros_pub(throttle, steering, brake, times, rate=40):
    """Blocking publish — runs in the calling thread."""
    cmd = (f"{ROS_SETUP} && "
           f"ros2 topic pub --times {times} /control_command fs_msgs/msg/ControlCommand "
           f"'{{header: {{stamp: {{sec: 0}}, frame_id: \"\"}}, "
           f"throttle: {throttle}, steering: {steering}, brake: {brake}}}' "
           f"--rate {rate}")
    subprocess.run(["docker", "exec", CONTAINER, "bash", "-c", cmd],
                   capture_output=True)


def ros_pub_bg(throttle, steering, brake, times, rate=40):
    """Non-blocking publish — returns Popen handle."""
    cmd = (f"{ROS_SETUP} && "
           f"ros2 topic pub --times {times} /control_command fs_msgs/msg/ControlCommand "
           f"'{{header: {{stamp: {{sec: 0}}, frame_id: \"\"}}, "
           f"throttle: {throttle}, steering: {steering}, brake: {brake}}}' "
           f"--rate {rate}")
    return subprocess.Popen(["docker", "exec", CONTAINER, "bash", "-c", cmd],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def ros_ebs():
    cmd = f"{ROS_SETUP} && ros2 topic pub --once /signal/ebs std_msgs/msg/Empty '{{}}'"
    subprocess.run(["docker", "exec", CONTAINER, "bash", "-c", cmd], capture_output=True)


def ros_reset():
    cmd = f"{ROS_SETUP} && ros2 service call /reset fs_msgs/srv/Reset"
    r = subprocess.run(["docker", "exec", CONTAINER, "bash", "-c", cmd],
                       capture_output=True, text=True)
    return "success=True" in r.stdout


def pose():
    return get("/api/vehicle/pose")


def state():
    return get("/api/vehicle/state")


def kick():
    """Fire simSetVehiclePose self-teleport to break the Chaos ω=0 degenerate state."""
    p = pose()
    post("/api/vehicle/teleport", {"x": p["x"], "y": p["y"], "z": p["z"]})


def drive_with_kick(throttle, steering, duration_s=3.0, rate=40):
    """
    Start publishing control commands in background.  Poll until the throttle
    command actually reaches UE5 (CachedControls updated), then fire the kick.
    This ensures the Chaos ω=0 break happens while throttle is active.
    """
    times = int(duration_s * rate)
    proc = ros_pub_bg(throttle, steering, 0.0, times, rate)
    # Wait until UE5 sees our throttle (ros2 CLI startup can take >500ms)
    deadline = time.time() + 3.0
    while time.time() < deadline:
        s = state()
        if abs(s["controls"].get("throttle", 0) - throttle) < 0.05 \
                and s["controls"].get("brake", 1) < 0.05:
            break
        time.sleep(0.1)
    kick()            # Fire while throttle is active and brake is clear
    time.sleep(0.25)  # Let the kick take effect in the physics tick
    proc.wait()       # Wait for publishing to finish


def brake_to_stop(max_iters=8):
    for _ in range(max_iters):
        ros_pub(throttle=0.0, steering=0.0, brake=1.0, times=40)  # 1 s at 40 Hz
        s = state()
        if s.get("speed", 1) < 0.1:
            return s
    return state()


def wait_bridge_connected(timeout=15):
    """Wait until sim reports connected (bridge command client is up)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            s = get("/api/sim/status")
            if s.get("connected"):
                return True
        except Exception:
            pass
        time.sleep(1)
    return False


def dist(a, b):
    return math.sqrt((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2)


# ── 1. Load track → car at start gate ────────────────────────────────────────
print("\n=== Step 1: load track ===")
result = post("/api/track/ci_test.csv/load")
print(f"  cones={result['result']['cones']}  car_aligned={result['result']['car_aligned']}")
time.sleep(0.5)   # Let the teleport settle
start = pose()
print(f"  Start gate: x={start['x']:.4f}  y={start['y']:.4f}")
s0 = state()
print(f"  handbrake={s0['controls']['handbrake']}  speed={s0['speed']:.2f}")

# ── 2. Stop pipeline + clear EBS latch + release plugin EBS ─────────────────
print("\n=== Step 2: stop pipeline + clear EBS ===")
try:
    r = post("/api/pipeline/stop")
    print(f"  pipeline: {r}")
except Exception as e:
    print(f"  (pipeline not running: {e})")
# Clear bridge EBS latch unconditionally — a previous test run may have left
# ebs_triggered_=true which silently drops all /control_command forwarding.
ros_pub_ebs_reset = (f"{ROS_SETUP} && ros2 topic pub --once /signal/ebs_reset "
                     f"std_msgs/msg/Empty '{{}}'")
subprocess.run(["docker", "exec", CONTAINER, "bash", "-c", ros_pub_ebs_reset],
               capture_output=True)
# Also release the plugin-level EBS so bApiControlEnabled flips back to true.
# A previous test run's `activateEbs` (or this run's loadTrack-time release
# which doesn't update bApiControlEnabled) can leave the plugin rejecting
# every setCarControls, so the bridge would forward throttle commands but
# UE5 would silently drop them. Send releaseEbs directly via the plugin RPC.
release_ebs = (f"python3 -c \"import socket; s=socket.socket(); "
               f"s.connect(('host.docker.internal', 41451)); "
               f"s.sendall(b'releaseEbs\\n'); print(s.recv(64).decode().strip())\"")
subprocess.run(["docker", "exec", CONTAINER, "bash", "-c", release_ebs],
               capture_output=True)
print("  EBS latch cleared + plugin EBS released")
time.sleep(0.5)   # Let pipeline processes quiesce and EBS reset propagate

# ── 3. Drive straight ────────────────────────────────────────────────────────
print("\n=== Step 3: drive straight ===")
drive_with_kick(throttle=0.35, steering=0.0, duration_s=3.0)
s = brake_to_stop()
mid = pose()
d_straight = dist(mid, start)
print(f"  Stopped: x={mid['x']:.3f}  y={mid['y']:.3f}  dist={d_straight:.1f}m  speed={s.get('speed', 0):.2f}")
assert d_straight > 5.0, f"Car barely moved ({d_straight:.1f}m) — check kick/API-control"

# ── 4. Steer to change heading ────────────────────────────────────────────────
print("\n=== Step 4: kick + low throttle + steer ===")
drive_with_kick(throttle=0.15, steering=0.4, duration_s=2.0)
s = brake_to_stop()
pre = state()
d_total = dist(pre, start)
print(f"  Stopped after steer: x={pre['x']:.3f}  y={pre['y']:.3f}  speed={pre.get('speed', 0):.2f}")
print(f"  Total displacement from start gate: {d_total:.1f}m")

# ── 5. Activate EBS ───────────────────────────────────────────────────────────
print("\n=== Step 5: activate EBS ===")
ros_ebs()
time.sleep(1.0)
s = state()
print(f"  handbrake={s.get('controls', {}).get('handbrake')}  speed={s.get('speed', 0):.2f}")

# ── 6. Confirm bridge is connected ────────────────────────────────────────────
print("\n=== Step 6: confirm bridge is connected ===")
ok = wait_bridge_connected(timeout=15)
print(f"  connected={ok}")
if not ok:
    print("  ERROR: bridge not connected, aborting")
    sys.exit(1)

# ── 7. Call /reset ────────────────────────────────────────────────────────────
print("\n=== Step 7: call /reset ===")
success = ros_reset()
print(f"  success={success}")
time.sleep(0.5)

# ── 8. Verify ────────────────────────────────────────────────────────────────
print("\n=== Step 8: verify ===")
after = state()
d_reset = dist(after, start)
print(f"  Start gate:  x={start['x']:.4f}  y={start['y']:.4f}")
print(f"  After reset: x={after['x']:.4f}  y={after['y']:.4f}  speed={after.get('speed', 0):.2f}")
print(f"  Distance from start gate: {d_reset:.4f}m")

if not success:
    print("\n  FAIL ✗ — reset returned success=False")
    sys.exit(1)
elif d_reset < 1.0:
    print("\n  PASS ✓")
else:
    print(f"\n  FAIL ✗ — {d_reset:.2f}m from gate after reset")
    sys.exit(1)
