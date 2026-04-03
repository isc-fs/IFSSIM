"""
IFSSIM Track Manager — Web UI for generating and loading tracks.
Run: python app.py
Open: http://localhost:5050
"""

import os
import sys
import socket
import json
import glob
import io
import base64
import shutil
from datetime import datetime
from flask import Flask, render_template, request, jsonify

# Add track generator to path
TRACK_GEN_PATH = os.path.abspath(os.environ.get("TRACK_GEN_PATH",
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "random-track-generator")))
if os.path.exists(TRACK_GEN_PATH):
    sys.path.insert(0, TRACK_GEN_PATH)

TRACKS_DIR = os.path.abspath(os.environ.get("TRACKS_DIR",
    os.path.join(os.path.dirname(__file__), "..", "..", "Content", "tracks")))

SIM_HOST = "127.0.0.1"
SIM_PORT = 41451

app = Flask(__name__)


def sim_command(cmd):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(3)
        s.connect((SIM_HOST, SIM_PORT))
        s.sendall((cmd + "\n").encode())
        import time; time.sleep(0.2)
        data = s.recv(65536).decode().strip()
        s.close()
        return data
    except Exception as e:
        return json.dumps({"error": str(e)})


def parse_track_csv(filepath):
    cones = {"blue": [], "yellow": [], "big_orange": [], "small_orange": []}
    try:
        with open(filepath, "r") as f:
            for line in f:
                parts = line.strip().split(",")
                if len(parts) < 3:
                    continue
                cone_type = parts[0].strip()
                x = float(parts[1])
                y = float(parts[2])
                if cone_type in cones:
                    cones[cone_type].append((x, y))
    except:
        pass
    return cones


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


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/tracks")
def list_tracks():
    tracks = []
    for f in sorted(glob.glob(os.path.join(TRACKS_DIR, "*.csv"))):
        name = os.path.basename(f)
        cones = parse_track_csv(f)
        total = sum(len(v) for v in cones.values())
        tracks.append({
            "name": name, "path": os.path.abspath(f), "cones": total,
            "blue": len(cones["blue"]), "yellow": len(cones["yellow"]),
            "orange": len(cones["big_orange"]) + len(cones["small_orange"])
        })
    return jsonify(tracks)


@app.route("/api/track/<name>/preview")
def track_preview(name):
    filepath = os.path.join(TRACKS_DIR, name)
    if not os.path.exists(filepath):
        return jsonify({"error": "Track not found"}), 404
    cones = parse_track_csv(filepath)
    plot = generate_track_plot(cones)
    if plot:
        return jsonify({"image": plot})
    return jsonify({"error": "Failed to generate plot"}), 500


@app.route("/api/track/<name>/load", methods=["POST"])
def load_track(name):
    filepath = os.path.abspath(os.path.join(TRACKS_DIR, name))
    if not os.path.exists(filepath):
        return jsonify({"error": "Track not found"}), 404
    result = sim_command(f"loadTrack {filepath}")
    return jsonify({"result": result, "track": name})


@app.route("/api/generate", methods=["POST"])
def generate_track():
    try:
        from track_generator import TrackGenerator
        from utils import Mode, SimType

        data = request.json or {}
        n_points = data.get("n_points", 50)
        n_regions = data.get("n_regions", 30)
        max_bound = data.get("max_bound", 150)
        name_base = data.get("name", "").strip()
        if not name_base:
            name_base = f"track_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        # Save current dir, cd to track generator, generate, cd back
        orig_dir = os.getcwd()
        os.chdir(TRACK_GEN_PATH)

        temp_rel = "_temp_gen"
        os.makedirs(temp_rel, exist_ok=True)

        track_gen = TrackGenerator(
            n_points=n_points, n_regions=n_regions,
            min_bound=10., max_bound=float(max_bound),
            mode=Mode.RANDOM, plot_track=False, visualise_voronoi=False,
            create_output_file=True, output_location=f"/{temp_rel}",
            sim_type=SimType.FSDS
        )
        track_gen.create_track()

        gen_file = os.path.join(TRACK_GEN_PATH, temp_rel, "random_track.csv")
        os.chdir(orig_dir)

        if os.path.exists(gen_file):
            os.makedirs(TRACKS_DIR, exist_ok=True)
            dest_file = os.path.join(TRACKS_DIR, f"{name_base}.csv")
            shutil.move(gen_file, dest_file)
            try:
                shutil.rmtree(os.path.join(TRACK_GEN_PATH, temp_rel))
            except:
                pass

            cones = parse_track_csv(dest_file)
            total = sum(len(v) for v in cones.values())
            return jsonify({"name": f"{name_base}.csv", "cones": total})

        return jsonify({"error": f"File not found: {gen_file}"}), 500
    except ImportError as e:
        return jsonify({"error": f"Track generator not found at {TRACK_GEN_PATH}: {e}"}), 500
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/track/<name>/delete", methods=["DELETE"])
def delete_track(name):
    filepath = os.path.join(TRACKS_DIR, name)
    if os.path.exists(filepath):
        os.remove(filepath)
        return jsonify({"deleted": name})
    return jsonify({"error": "Track not found"}), 404


@app.route("/api/sim/status")
def sim_status():
    result = sim_command("ping")
    connected = result == "true"
    return jsonify({"connected": connected})


if __name__ == "__main__":
    print(f"IFSSIM Track Manager | http://localhost:5050")
    print(f"Tracks: {TRACKS_DIR}")
    print(f"Generator: {TRACK_GEN_PATH}")
    app.run(host="0.0.0.0", port=5050, debug=True)
