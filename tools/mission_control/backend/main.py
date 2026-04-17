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
from datetime import datetime
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query
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
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "random-track-generator")))
TRACKS_DIR = os.path.abspath(os.environ.get("TRACKS_DIR",
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "Content", "tracks")))
# Path UE5 uses to load the file — must be the host-side absolute path (UE5 runs on host, not in Docker)
UE5_TRACKS_DIR = os.environ.get("UE5_TRACKS_DIR", TRACKS_DIR)

SIM_HOST = os.environ.get("SIM_HOST", "127.0.0.1")
SIM_PORT = int(os.environ.get("SIM_PORT", "41451"))

app = FastAPI(title="IFSSIM Mission Control", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

sim = SimConnection(SIM_HOST, SIM_PORT)

# State
session_log = []
res_active = False


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
    return {
        "connected": connected,
        "map": status.get("map", "unknown"),
        "fps": status.get("fps", 0),
        "paused": status.get("paused", False),
        "api_control": status.get("api_control", False),
    }

@app.post("/api/sim/pause")
def sim_pause():
    try:
        sim.pause()
    except Exception:
        pass
    return {"ok": True, "paused": True}

@app.post("/api/sim/resume")
def sim_resume():
    try:
        sim.resume()
    except Exception:
        pass
    return {"ok": True, "paused": False}

@app.post("/api/sim/reset")
def sim_reset():
    global res_active
    if not sim.is_connected():
        return {"ok": False, "error": "sim not connected"}
    try:
        sim.reset()
        res_active = False
    except Exception:
        return {"ok": False, "error": "reset failed"}
    log_event("reset", "Simulation reset")
    return {"ok": True}


# === Event Control ===

@app.get("/api/event/state")
def event_state():
    try:
        return sim.get_referee_state()
    except Exception:
        return {}

@app.post("/api/event/set")
def event_set(setup: EventSetup):
    try:
        result = sim.set_event(setup.event_type, setup.num_laps)
    except Exception:
        result = {"error": "sim not connected"}
    log_event("event_set", f"{setup.event_type} ({setup.num_laps} laps)")
    return result

@app.post("/api/event/start")
def event_start(setup: EventSetup):
    """Set event type and resume simulation."""
    try:
        sim.set_event(setup.event_type, setup.num_laps)
        sim.resume()
    except Exception:
        pass
    log_event("event_start", f"{setup.event_type} started ({setup.num_laps} laps)")
    return {"ok": True, "event": setup.event_type, "laps": setup.num_laps}


# === RES (Remote Emergency Stop) ===

@app.post("/api/res/activate")
def res_activate_endpoint():
    global res_active
    try:
        sim.res_activate()
    except Exception:
        pass
    res_active = True
    log_event("res", "EMERGENCY STOP activated")
    return {"ok": True, "res": "activated", "res_active": True}

@app.post("/api/res/release")
def res_release_endpoint():
    global res_active
    try:
        sim.res_release()
    except Exception:
        pass
    res_active = False
    log_event("res", "RES released")
    return {"ok": True, "res": "released", "res_active": False}

@app.get("/api/res/status")
def res_status():
    return {"res_active": res_active}


# === Vehicle ===

@app.get("/api/vehicle/state")
def vehicle_state():
    try:
        return sim.get_vehicle_state()
    except Exception:
        return {}

@app.get("/api/vehicle/pose")
def vehicle_pose():
    try:
        return sim.get_vehicle_pose()
    except Exception:
        return {}

@app.post("/api/vehicle/teleport")
def vehicle_teleport(req: TeleportRequest):
    try:
        sim.teleport(req.x, req.y, req.z)
    except Exception:
        pass
    return {"teleported": True}


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

@app.post("/api/track/{name}/load")
def track_load(name: str):
    filepath = os.path.abspath(os.path.join(TRACKS_DIR, name))
    if not os.path.exists(filepath):
        return JSONResponse({"error": "Track not found"}, status_code=404)
    ue5_path = os.path.join(UE5_TRACKS_DIR, name)
    result = sim.load_track(ue5_path)
    event_type = BUILTIN_TRACKS.get(name)
    if event_type:
        sim.set_event(event_type)
    log_event("track_load", f"Loaded {name}" + (f" (event: {event_type})" if event_type else ""))
    return {"result": result, "track": name, "event_type": event_type}

@app.delete("/api/track/{name}")
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

@app.post("/api/track/generate")
def track_generate(params: TrackGenerate):
    try:
        sys.path.insert(0, TRACK_GEN_PATH)
        from track_generator import TrackGenerator
        from utils import Mode, SimType

        name_base = params.name.strip() or f"track_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        orig_dir = os.getcwd()
        os.chdir(TRACK_GEN_PATH)
        temp_rel = "_temp_gen"
        os.makedirs(temp_rel, exist_ok=True)

        gen = TrackGenerator(
            n_points=params.n_points, n_regions=params.n_regions,
            min_bound=10., max_bound=float(params.max_bound),
            mode=Mode.RANDOM, plot_track=False, visualise_voronoi=False,
            create_output_file=True, output_location=f"/{temp_rel}",
            sim_type=SimType.FSDS
        )
        gen.create_track()

        gen_file = os.path.join(TRACK_GEN_PATH, temp_rel, "random_track.csv")
        os.chdir(orig_dir)

        if os.path.exists(gen_file):
            os.makedirs(TRACKS_DIR, exist_ok=True)
            dest = os.path.join(TRACKS_DIR, f"{name_base}.csv")
            shutil.move(gen_file, dest)
            try:
                shutil.rmtree(os.path.join(TRACK_GEN_PATH, temp_rel))
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
def scoring_summary(t_best: Optional[float] = Query(None)):
    ref = sim.get_referee_state()
    if not ref:
        return {"error": "No referee data"}

    event = ref.get("event", "unknown")
    lap_times = ref.get("lap_times", [])
    doo = ref.get("doo_counter", 0)
    oc = ref.get("oc_counter", 0)

    return compute_scoring(event, lap_times, doo, oc, t_best)


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
async def telemetry_ws(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            try:
                # Gather state from sim
                vehicle = sim.get_vehicle_state()
                ref = sim.get_referee_state()
                status = sim.get_status()

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
                    "doo": ref.get("doo_counter", 0),
                    "oc": ref.get("oc_counter", 0),
                    "laps": ref.get("laps", 0),
                    "required_laps": ref.get("required_laps", 0),
                    "finished": ref.get("finished", False),
                    "event": ref.get("event", "unknown"),
                    "fps": status.get("fps", 0),
                    "paused": status.get("paused", False),
                    "res_active": res_active,
                }

                await websocket.send_json(data)
            except Exception:
                # Sim disconnected, send empty
                await websocket.send_json({"error": "sim_disconnected"})

            await asyncio.sleep(0.1)  # 10Hz
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
