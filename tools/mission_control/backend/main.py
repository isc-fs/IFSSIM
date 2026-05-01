"""
IFSSIM Mission Control — FastAPI Backend

Unified API for simulator management, event control, scoring, and telemetry.
Run: uvicorn main:app --host 0.0.0.0 --port 8000
"""

import os
import sys
import glob
import json
import io
import base64
import asyncio
import shutil
import threading
import time
from datetime import datetime
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query, Header, HTTPException, Depends, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel

from sim_client import SimConnection
from scoring import compute_scoring

# Built-in tracks that ship with the simulator — not deletable, auto-configure event type
BUILTIN_TRACKS = {
    "acceleration.csv": "acceleration",
    "skidpad.csv": "skidpad",
}

# Track generator path
TRACK_GEN_PATH = os.path.abspath(os.environ.get("TRACK_GEN_PATH",
    os.path.join(os.path.dirname(__file__), "..", "..", "random-track-generator")))
TRACKS_DIR = os.path.abspath(os.environ.get("TRACKS_DIR",
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "Content", "tracks")))
PIPELINE_CTL_FILE = "/pipeline_ctrl/enable"
# Path UE5 uses to load the file — must be the host-side absolute path (UE5 runs on host, not in Docker)
UE5_TRACKS_DIR = os.environ.get("UE5_TRACKS_DIR", TRACKS_DIR)

SIM_HOST = os.environ.get("IFSSIM_HOST", os.environ.get("SIM_HOST", "127.0.0.1"))
SIM_PORT = int(os.environ.get("IFSSIM_PORT", os.environ.get("SIM_PORT", "41451")))

# Optional API key. When set, every mutating endpoint (sim control, event
# control, RES, pipeline, track load/gen/delete) and the telemetry WS
# require a matching key. Read-only status endpoints stay open so a
# monitor dashboard can attach without the key. When unset, the backend
# logs a single warning and runs wide-open — fine for closed-network dev,
# not for track day on untrusted wifi.
MC_API_KEY = os.environ.get("IFSSIM_MC_API_KEY", "").strip()

# CORS origins. Comma-separated list; default restricts to the frontend's
# docker-compose-published address. Use the old wildcard `*` explicitly
# if an external tool needs it — never silently anymore.
_DEFAULT_CORS = "http://localhost:3000,http://127.0.0.1:3000"
MC_CORS_ORIGINS = [o.strip() for o in
    os.environ.get("IFSSIM_MC_CORS_ORIGINS", _DEFAULT_CORS).split(",")
    if o.strip()]

app = FastAPI(title="IFSSIM Mission Control", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=MC_CORS_ORIGINS,
    allow_methods=["*"],
    allow_headers=["*"],
)

if not MC_API_KEY:
    print("[mc] IFSSIM_MC_API_KEY is empty — running unauthenticated. "
          "Set it before exposing the backend beyond localhost.",
          file=sys.stderr, flush=True)


def require_api_key(x_api_key: Optional[str] = Header(default=None)):
    """FastAPI dependency — validates X-API-Key header against MC_API_KEY.
    No-op if the server wasn't configured with a key (dev mode)."""
    if not MC_API_KEY:
        return
    if x_api_key != MC_API_KEY:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="missing or invalid X-API-Key",
        )


def ws_api_key_ok(api_key: Optional[str]) -> bool:
    """WebSocket auth check. Browsers can't attach custom headers to
    a WebSocket handshake, so WS uses a query-param token instead."""
    if not MC_API_KEY:
        return True
    return api_key == MC_API_KEY

sim = SimConnection(SIM_HOST, SIM_PORT)

# State
session_log = []
res_active = False
home_pose = {"x": 0.0, "y": 0.0, "z": 0.3, "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0}  # ENU spawn pose
_home_pose_captured = False  # set once we snapshot the pawn's map placement (or user pins a home)
_sim_was_connected = False   # tracks connect/disconnect transitions so we re-capture after a UE5 restart

# Compound-operation lock. FastAPI runs sync `def` handlers in a worker
# thread pool, so two clients can land in `event_start`, `sim_reset` or
# the RES endpoints simultaneously. Each of those does N sequential
# RPCs *plus* mutates `res_active` / `current_event` / `PIPELINE_CTL_FILE`,
# and we don't want one handler to interleave with another mid-update
# (visible as RES toggle flicker, ghost pipeline restarts, and a
# split-brain `res_active` flag). All wire-level RPC calls go through
# `SimConnection._lock` already, so this lock is purely for the
# *compound* state.
_state_lock = threading.Lock()

# `track_generate` calls `os.chdir` (the third-party generator uses
# `__file__`-relative paths internally). chdir is process-global, so a
# concurrent track-generate would clobber the other's CWD, and any
# unrelated handler that runs in between would see the wrong CWD. The
# generator is also cpu-heavy, so we don't want to share `_state_lock`
# with the fast RES endpoints. Dedicated lock.
_gen_lock = threading.Lock()


def _ensure_home_pose_captured():
    """Snapshot the pawn's map-placement pose as home.

    Runs from the polled status/state endpoints so the capture happens within the
    first poll after UE5 connects — before the pawn has had time to drive away
    from its spawn. On UE5 disconnect (Stop/Play cycle), the captured flag is
    reset so the next connect re-captures fresh."""
    global home_pose, _home_pose_captured, _sim_was_connected
    is_conn = sim.is_connected()
    if _sim_was_connected and not is_conn:
        _home_pose_captured = False
    _sim_was_connected = is_conn
    if _home_pose_captured or not is_conn:
        return
    try:
        pose = sim.get_vehicle_pose()
    except Exception:
        return
    if not pose:
        return
    home_pose = {
        "x": pose.get("x", 0.0),
        "y": pose.get("y", 0.0),
        "z": 0.3,  # fixed lift so reset never spawns at ground level
        "qw": pose.get("qw", 1.0),
        "qx": pose.get("qx", 0.0),
        "qy": pose.get("qy", 0.0),
        "qz": pose.get("qz", 0.0),
    }
    _home_pose_captured = True
_STATE_FILE = os.path.join(TRACKS_DIR, ".ifssim_state.json")

def _load_state():
    try:
        with open(_STATE_FILE) as f:
            return json.load(f)
    except Exception:
        return {}

def _save_state(data: dict):
    try:
        s = _load_state()
        s.update(data)
        with open(_STATE_FILE, "w") as f:
            json.dump(s, f)
    except Exception:
        pass

current_event = _load_state().get("event", "unknown")


# === Pydantic Models ===

class EventSetup(BaseModel):
    event_type: str
    num_laps: int = 10

class TrackGenerate(BaseModel):
    n_points: int = 50
    n_regions: int = 30
    max_bound: int = 150
    name: str = ""

class TeleportRequest(BaseModel):
    x: float
    y: float
    z: float


# === Sim Status ===

@app.get("/api/sim/status")
def sim_status():
    connected = sim.is_connected()
    status = sim.get_status() if connected else {}
    _ensure_home_pose_captured()
    return {
        "connected": connected,
        "map": status.get("map", "unknown"),
        "fps": status.get("fps", 0),
        "paused": status.get("paused", False),
        "api_control": status.get("api_control", False),
    }

@app.post("/api/sim/pause", dependencies=[Depends(require_api_key)])
def sim_pause():
    try:
        sim.pause()
        return {"ok": True, "paused": True}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)

@app.post("/api/sim/resume", dependencies=[Depends(require_api_key)])
def sim_resume():
    try:
        sim.resume()
        return {"ok": True, "paused": False}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)

@app.post("/api/sim/reset", dependencies=[Depends(require_api_key)])
def sim_reset():
    """Soft reset: teleport car back to start position without crashing UE5.
    The destructive RPC 'reset' triggers a full level reload which crashes the
    UE5 editor. Use teleport + disable API control instead."""
    global res_active
    if not sim.is_connected():
        return {"ok": False, "error": "sim not connected"}
    with _state_lock:
        # Stop pipeline so control node stops publishing commands
        try:
            os.remove(PIPELINE_CTL_FILE)
        except FileNotFoundError:
            pass
        _ensure_home_pose_captured()
        try:
            sim.res_activate()
            # Position-only teleport. The full sim.teleport(...) variant
            # round-trips ENU↔UE5 quaternions through FSDSCoord and we
            # consistently observe a ~90° rotation drift on the way back
            # — the pawn ends up facing perpendicular to its spawn
            # heading, which makes Stanley's yaw_error term permanently
            # wrong and the controller steers off the corridor on the
            # first tick. Keeping the orientation untouched preserves
            # whatever yaw the pawn already has from loadTrack /
            # spawn_at_start_gate, which is the orientation the
            # autonomy was calibrated against.
            sim.teleport_pos(home_pose["x"], home_pose["y"], home_pose["z"])
            res_active = False
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
        log_event("reset", "Soft reset: pipeline stopped, car teleported to start")
    return {"ok": True}


# === Event Control ===

@app.get("/api/event/state")
def event_state():
    try:
        return sim.get_referee_state()
    except Exception:
        return {}

@app.post("/api/event/set", dependencies=[Depends(require_api_key)])
def event_set(setup: EventSetup):
    global current_event
    try:
        result = sim.set_event(setup.event_type, setup.num_laps)
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
    current_event = setup.event_type
    _save_state({"event": current_event})
    log_event("event_set", f"{setup.event_type} ({setup.num_laps} laps)")
    return result

@app.post("/api/event/start", dependencies=[Depends(require_api_key)])
def event_start(setup: EventSetup):
    global current_event, res_active
    if not sim.is_connected():
        return JSONResponse({"ok": False, "error": "Simulator not connected"}, status_code=503)
    with _state_lock:
        try:
            # Stop any running pipeline so the launch sequence below starts
            # the autonomy from a clean slate (cone_graph_slam, control,
            # path_planning all relaunched → SLAM re-runs INIT_CALIBRATING).
            try:
                os.remove(PIPELINE_CTL_FILE)
            except FileNotFoundError:
                pass
            # Park the car under EBS while the pipeline boots. cone_graph_slam
            # requires 3 s of stationary IMU samples to estimate accel/gyro
            # bias correctly; if the car moves during that window the bias
            # estimate locks in the body-frame launch acceleration and every
            # subsequent LiDAR scan trips DA-failure spikes. EBS holds the
            # handbrake on all four wheels until we explicitly release it
            # below, after SLAM has reported SLAM_RUNNING.
            sim.res_activate()
            res_active = True
            sim.set_event(setup.event_type, setup.num_laps)
            sim.resume()
            # Start the pipeline now (still EBS-locked). The control node
            # publishes /signal/ebs_reset on init which clears the bridge's
            # ebs_triggered_ flag from any prior session.
            os.makedirs("/pipeline_ctrl", exist_ok=True)
            open(PIPELINE_CTL_FILE, "w").close()
            # Wait for cone_graph_slam to clear INIT_CALIBRATING. We don't
            # have a status topic yet, so we wait the worst-case timing:
            # ~1 s for the launch process to fork all nodes + 3 s for the
            # IMU calibration window itself + 0.5 s margin. This is the
            # ONLY moment in event_start where the car is guaranteed to
            # be stationary, so any drift here corrupts the SLAM bias.
            time.sleep(4.5)
            # SLAM should now be in SLAM_RUNNING with a clean bias. Release
            # EBS, hand control to the autonomy, and let the velocity
            # controller ramp the EMRAX from rest. No pre-seated throttle
            # and no velocity-kick teleport: the launch is fully closed-
            # loop on the autonomy's first /control_command tick after the
            # rear-axle friction lock is broken by EMRAX shaft torque
            # alone (rear FrictionForceMultiplier was lowered to 1.0 in
            # the same change that removed the kick — see FSDSWheelRear.cpp).
            sim.res_release()
            res_active = False
            try:
                sim._cmd("enableApiControl 1")
            except Exception:
                pass
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
        current_event = setup.event_type
        _save_state({"event": current_event})
        # The plugin's SetEventType clamps lap counts per event type
        # (Acceleration/Autocross → 1, Skidpad → 4, Trackdrive → requested).
        # Echo the referee's actual required_laps so the UI reflects what
        # the sim will enforce, not what we asked for.
        try:
            ref = sim.get_referee_state()
            actual_laps = int(ref.get("required_laps", setup.num_laps))
        except Exception:
            actual_laps = setup.num_laps
        log_event("event_start", f"{setup.event_type} started ({actual_laps} laps)")
    return {"ok": True, "event": setup.event_type, "laps": actual_laps}


# === RES (Remote Emergency Stop) ===

@app.post("/api/res/activate", dependencies=[Depends(require_api_key)])
def res_activate_endpoint():
    global res_active
    with _state_lock:
        # Stop pipeline first so control node stops sending throttle commands
        try:
            os.remove(PIPELINE_CTL_FILE)
        except FileNotFoundError:
            pass
        try:
            sim.res_activate()
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
        res_active = True
        log_event("res", "EMERGENCY STOP activated")
    return {"ok": True, "res": "activated", "res_active": True}

@app.post("/api/res/release", dependencies=[Depends(require_api_key)])
def res_release_endpoint():
    global res_active
    with _state_lock:
        # Re-check under the lock — without it, an `event_start` running
        # concurrently could clear the flag between the check and the
        # release, and we'd issue a redundant `releaseEbs` to the sim.
        if not res_active:
            return JSONResponse(
                {"ok": False, "error": "RES is not active"},
                status_code=400,
            )
        try:
            sim.res_release()
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
        res_active = False
        log_event("res", "RES released")
    return {"ok": True, "res": "released", "res_active": False}

@app.get("/api/res/status")
def res_status():
    return {"res_active": res_active}


# === Pipeline ===

@app.post("/api/pipeline/start", dependencies=[Depends(require_api_key)])
def pipeline_start():
    os.makedirs("/pipeline_ctrl", exist_ok=True)
    open(PIPELINE_CTL_FILE, "w").close()
    log_event("pipeline", "Pipeline started")
    return {"ok": True, "pipeline": "started"}

@app.post("/api/pipeline/stop", dependencies=[Depends(require_api_key)])
def pipeline_stop():
    try:
        os.remove(PIPELINE_CTL_FILE)
    except FileNotFoundError:
        pass
    log_event("pipeline", "Pipeline stopped")
    return {"ok": True, "pipeline": "stopped"}

@app.get("/api/pipeline/status")
def pipeline_status():
    return {"enabled": os.path.exists(PIPELINE_CTL_FILE)}


# === Vehicle ===

@app.get("/api/vehicle/state")
def vehicle_state():
    try:
        state = sim.get_vehicle_state()
    except Exception:
        return {}
    _ensure_home_pose_captured()
    return state

@app.get("/api/vehicle/pose")
def vehicle_pose():
    try:
        return sim.get_vehicle_pose()
    except Exception:
        return {}

@app.post("/api/vehicle/teleport", dependencies=[Depends(require_api_key)])
def vehicle_teleport(req: TeleportRequest):
    try:
        # Position-only — sim.teleport() defaults to qw=1 (identity quaternion)
        # which would snap the car to face East regardless of its current
        # heading. teleport_pos sends `simSetVehiclePose x y z` without the
        # quaternion, which the plugin treats as a position-only teleport
        # and preserves the actor's existing yaw across ResetVehicleState.
        sim.teleport_pos(req.x, req.y, req.z)
        return {"teleported": True}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)


# === Track Manager ===

def parse_track_csv(filepath):
    cones = {"blue": [], "yellow": [], "big_orange": [], "small_orange": []}
    try:
        with open(filepath, "r") as f:
            for line in f:
                parts = line.strip().split(",")
                if len(parts) < 3:
                    continue
                cone_type = parts[0].strip()
                x, y = float(parts[1]), float(parts[2])
                if cone_type in cones:
                    cones[cone_type].append((x, y))
    except Exception:
        pass
    return cones

@app.get("/api/track/list")
def track_list():
    tracks = []
    for f in sorted(glob.glob(os.path.join(TRACKS_DIR, "*.csv"))):
        name = os.path.basename(f)
        cones = parse_track_csv(f)
        total = sum(len(v) for v in cones.values())
        tracks.append({
            "name": name, "path": os.path.abspath(f), "cones": total,
            "blue": len(cones["blue"]), "yellow": len(cones["yellow"]),
            "orange": len(cones["big_orange"]) + len(cones["small_orange"]),
            "builtin": name in BUILTIN_TRACKS,
            "event_type": BUILTIN_TRACKS.get(name),
        })
    return tracks

@app.get("/api/track/{name}/preview")
def track_preview(name: str):
    filepath = os.path.join(TRACKS_DIR, name)
    if not os.path.exists(filepath):
        return JSONResponse({"error": "Track not found"}, status_code=404)

    cones = parse_track_csv(filepath)
    plot = generate_track_plot(cones)
    if plot:
        return {"image": plot}
    return JSONResponse({"error": "Failed to generate plot"}, status_code=500)

@app.post("/api/track/{name}/load", dependencies=[Depends(require_api_key)])
def track_load(name: str):
    global current_event
    filepath = os.path.abspath(os.path.join(TRACKS_DIR, name))
    if not os.path.exists(filepath):
        return JSONResponse({"error": "Track not found"}, status_code=404)
    ue5_path = os.path.join(UE5_TRACKS_DIR, name)
    event_type = BUILTIN_TRACKS.get(name)
    with _state_lock:
        result = sim.load_track(ue5_path)
        if event_type:
            sim.set_event(event_type)
            current_event = event_type
            _save_state({"event": current_event})
        # Stop pipeline on track load (stale SLAM map would be invalid for new track)
        try:
            os.remove(PIPELINE_CTL_FILE)
        except FileNotFoundError:
            pass
        log_event("track_load", f"Loaded {name}" + (f" (event: {event_type})" if event_type else ""))
    return {"result": result, "track": name, "event_type": event_type}


@app.post("/api/vehicle/capture_home", dependencies=[Depends(require_api_key)])
def capture_home():
    """Capture current vehicle pose as the home/reset position."""
    global home_pose, _home_pose_captured
    pose = sim.get_vehicle_pose()
    if not pose:
        return JSONResponse({"ok": False, "error": "sim not connected"}, status_code=503)
    home_pose = {
        "x": pose.get("x", 0.0),
        "y": pose.get("y", 0.0),
        "z": 0.3,
        "qw": pose.get("qw", 1.0),
        "qx": pose.get("qx", 0.0),
        "qy": pose.get("qy", 0.0),
        "qz": pose.get("qz", 0.0),
    }
    _home_pose_captured = True
    return {"ok": True, "home_pose": home_pose}

@app.delete("/api/track/{name}", dependencies=[Depends(require_api_key)])
def track_delete(name: str):
    if "/" in name or "\\" in name or ".." in name:
        return JSONResponse({"error": "Invalid name"}, status_code=400)
    if name in BUILTIN_TRACKS:
        return JSONResponse({"error": "Cannot delete built-in track"}, status_code=400)
    filepath = os.path.join(TRACKS_DIR, name)
    if os.path.exists(filepath):
        os.remove(filepath)
        return {"deleted": name}
    return JSONResponse({"error": "Not found"}, status_code=404)

@app.post("/api/track/generate", dependencies=[Depends(require_api_key)])
def track_generate(params: TrackGenerate):
    try:
        sys.path.insert(0, TRACK_GEN_PATH)
        from track_generator import TrackGenerator
        from utils import Mode, SimType

        name_base = params.name.strip() or f"track_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        # output_yaml() builds its path as: os.path.realpath(os.path.dirname(__file__)) + output_location
        # __file__ is inside TRACK_GEN_PATH (read-only mount), so we use a traversal to
        # redirect output into a writable tmpdir: TRACK_GEN_PATH + /../../tmp/xxx = /tmp/xxx
        import tempfile
        tmp_dir = tempfile.mkdtemp()
        rel_to_gen = os.path.relpath(tmp_dir, TRACK_GEN_PATH)
        output_location = "/" + rel_to_gen  # e.g. "/../../tmp/tmpXXXXXX"

        # `os.chdir` is process-global. Two concurrent track-generate
        # calls would race on CWD; an unrelated handler running in the
        # same process between chdir-in and chdir-out would also see
        # the wrong CWD. Serialize with a dedicated lock and restore
        # CWD in finally so an exception inside `create_track()`
        # doesn't leave the whole backend stuck in TRACK_GEN_PATH.
        with _gen_lock:
            orig_dir = os.getcwd()
            os.chdir(TRACK_GEN_PATH)
            try:
                gen = TrackGenerator(
                    n_points=params.n_points, n_regions=params.n_regions,
                    min_bound=10., max_bound=float(params.max_bound),
                    mode=Mode.RANDOM, plot_track=False, visualise_voronoi=False,
                    create_output_file=True, output_location=output_location,
                    sim_type=SimType.FSDS
                )
                gen.create_track()
            finally:
                os.chdir(orig_dir)

        gen_file = os.path.join(tmp_dir, "random_track.csv")

        if os.path.exists(gen_file):
            os.makedirs(TRACKS_DIR, exist_ok=True)
            dest = os.path.join(TRACKS_DIR, f"{name_base}.csv")
            shutil.move(gen_file, dest)
            try:
                shutil.rmtree(tmp_dir)
            except Exception:
                pass

            cones = parse_track_csv(dest)
            total = sum(len(v) for v in cones.values())
            log_event("track_generate", f"Generated {name_base}.csv ({total} cones)")
            return {"name": f"{name_base}.csv", "cones": total}

        return JSONResponse({"error": "Generation failed"}, status_code=500)
    except ImportError as e:
        return JSONResponse({"error": f"Track generator not found: {e}"}, status_code=500)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)


# === Scoring ===

@app.get("/api/scoring/summary")
def scoring_summary(
    t_best: Optional[float] = Query(None),
    uss: bool = Query(False),
):
    ref = sim.get_referee_state()
    if not ref:
        return {"error": "No referee data"}

    event = ref.get("event", "unknown")
    lap_times = ref.get("lap_times", [])
    doo = ref.get("doo_counter", 0)
    oc = ref.get("oc_counter", 0)

    return compute_scoring(event, lap_times, doo, oc, t_best, uss=uss)


# === Session Log ===

def log_event(event_type: str, message: str):
    session_log.append({
        "timestamp": datetime.now().isoformat(),
        "type": event_type,
        "message": message,
    })

@app.get("/api/session/log")
def get_session_log():
    return session_log

@app.get("/api/session/export")
def export_session_log():
    """Export session log as JSON download."""
    return Response(
        content=json.dumps(session_log, indent=2),
        media_type="application/json",
        headers={"Content-Disposition": f"attachment; filename=session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"}
    )


# === WebSocket Telemetry ===

@app.websocket("/ws/telemetry")
async def telemetry_ws(websocket: WebSocket, api_key: Optional[str] = Query(default=None)):
    # WebSocket handshakes from browsers can't carry custom headers, so
    # the API key (if the server is configured with one) arrives as a
    # query parameter. Reject before accept so the handshake never
    # completes for an unauthenticated caller. The literal 1008 is the
    # "policy violation" WS close code — hardcoded here rather than
    # using `status.WS_1008_POLICY_VIOLATION` because the loop below
    # reuses the name `status` for `sim.get_status()`, which would
    # shadow the fastapi `status` module under Python's function-scope
    # rule and UnboundLocalError this line.
    if not ws_api_key_ok(api_key):
        await websocket.close(code=1008)
        return
    await websocket.accept()
    try:
        while True:
            try:
                # Gather state from sim. The three calls go through
                # SimConnection's threading.Lock, so a slow compound
                # operation in another worker (event_start does ~5
                # sequential RPCs while holding the lock) would freeze
                # the event loop here for hundreds of ms — visible in
                # the UI as telemetry "skipping". Run them on the
                # thread pool so the loop stays responsive while we
                # block on the wire.
                vehicle = await asyncio.to_thread(sim.get_vehicle_state)
                ref = await asyncio.to_thread(sim.get_referee_state)
                sim_status = await asyncio.to_thread(sim.get_status)

                data = {
                    "speed": vehicle.get("speed", 0),
                    "rpm": vehicle.get("rpm", 0),
                    "gear": vehicle.get("gear", 0),
                    "x": vehicle.get("x", 0),
                    "y": vehicle.get("y", 0),
                    "z": vehicle.get("z", 0),
                    "throttle": vehicle.get("controls", {}).get("throttle", 0),
                    "steering": vehicle.get("controls", {}).get("steering", 0),
                    "brake": vehicle.get("controls", {}).get("brake", 0),
                    # Regen telemetry (motor-side). regen_torque/power reflect
                    # what the sim is currently absorbing; regen_avail_torque
                    # is the cap at the current ω_motor (motor-peak or
                    # cell-power-limited, whichever binds); regen_max_*_limit
                    # are the hardware ceilings from settings.json.
                    "regen_torque": vehicle.get("regen_torque", 0),
                    "regen_power": vehicle.get("regen_power", 0),
                    "regen_avail_torque": vehicle.get("regen_avail_torque", 0),
                    "regen_max_torque": vehicle.get("regen_max_torque", 0),
                    "regen_max_power": vehicle.get("regen_max_power", 0),
                    "doo": ref.get("doo_counter", 0),
                    "oc": ref.get("oc_counter", 0),
                    "laps": ref.get("laps", 0),
                    "required_laps": ref.get("required_laps", 0),
                    "finished": ref.get("finished", False),
                    "event": current_event,
                    "fps": sim_status.get("fps", 0),
                    "paused": sim_status.get("paused", False),
                    "res_active": res_active,
                    "pipeline_enabled": os.path.exists(PIPELINE_CTL_FILE),
                }

                await websocket.send_json(data)
            except Exception:
                # Sim disconnected OR the primary send above just failed
                # on an already-closed socket. Try one keep-alive-style
                # error beacon; if that also fails the WS is gone and we
                # bail out of the loop rather than spinning forever on
                # RuntimeError.
                try:
                    await websocket.send_json({"error": "sim_disconnected"})
                except Exception:
                    break

            await asyncio.sleep(0.2)  # 5Hz
    except WebSocketDisconnect:
        pass


# === Track Plot Helper ===

def generate_track_plot(cones):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(1, 1, figsize=(8, 6))
        fig.patch.set_facecolor("#1a1a1a")
        ax.set_facecolor("#111111")

        if cones.get("blue"):
            bx, by = zip(*cones["blue"])
            ax.plot(bx, by, "o", color="#4488ff", markersize=4, label=f'Blue ({len(cones["blue"])})')
        if cones.get("yellow"):
            yx, yy = zip(*cones["yellow"])
            ax.plot(yx, yy, "o", color="#ffb81c", markersize=4, label=f'Yellow ({len(cones["yellow"])})')
        if cones.get("big_orange"):
            ox, oy = zip(*cones["big_orange"])
            ax.plot(ox, oy, "^", color="#ff6600", markersize=10, label=f'Orange ({len(cones["big_orange"])})')
        if cones.get("small_orange"):
            sx, sy = zip(*cones["small_orange"])
            ax.plot(sx, sy, "v", color="#ff8800", markersize=7)

        ax.set_aspect("equal")
        ax.set_xlabel("X (meters)", color="#888")
        ax.set_ylabel("Y (meters)", color="#888")
        ax.tick_params(colors="#888")
        ax.legend(loc="upper right", facecolor="#222", edgecolor="#333", labelcolor="#ccc", fontsize=9)
        ax.grid(True, alpha=0.15, color="#ffb81c")
        for spine in ax.spines.values():
            spine.set_color("#333")

        buf = io.BytesIO()
        fig.savefig(buf, format="png", bbox_inches="tight", dpi=120, facecolor=fig.get_facecolor())
        plt.close(fig)
        buf.seek(0)
        return base64.b64encode(buf.read()).decode()
    except Exception as e:
        print(f"Plot error: {e}")
        return None


# === Startup ===

@app.on_event("startup")
def startup():
    print(f"IFSSIM Mission Control | http://localhost:8000")
    print(f"Sim: {SIM_HOST}:{SIM_PORT}")
    print(f"Tracks: {TRACKS_DIR}")
    print(f"Docs: http://localhost:8000/docs")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
