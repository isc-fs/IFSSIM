"""Read /imu and /motor_rpm from an MCAP bag (a file, or a folder of them).

The car's manual bags are several GB and take minutes to read, so a cache
folder keeps the arrays as .npz, keyed by the bag's path, size and mtime.
"""

from __future__ import annotations

import glob
import hashlib
import os
from typing import Dict, Optional

import numpy as np

KEYS = ("t", "lt", "ax", "ay", "az", "gx", "gy", "gz", "rt", "rpm")


def _files(path: str):
    return sorted(glob.glob(os.path.join(path, "*.mcap"))) if os.path.isdir(path) else [path]


def _cache_name(path: str) -> str:
    h = hashlib.sha1()
    for f in _files(path):
        st = os.stat(f)
        h.update(f"{os.path.abspath(f)}|{st.st_size}|{int(st.st_mtime)}".encode())
    return f"imu_{h.hexdigest()[:16]}.npz"


def load(path: str, cache_dir: Optional[str] = None) -> Dict[str, np.ndarray]:
    cache = os.path.join(cache_dir, _cache_name(path)) if cache_dir else None
    if cache and os.path.exists(cache):
        return dict(np.load(cache))

    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    v = {k: [] for k in KEYS}
    for f in _files(path):
        with open(f, "rb") as fh:
            reader = make_reader(fh, decoder_factories=[DecoderFactory()])
            for _, ch, m, msg in reader.iter_decoded_messages(topics=["/imu", "/motor_rpm"]):
                if ch.topic == "/imu":
                    v["t"].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
                    v["lt"].append(m.log_time * 1e-9)
                    a, g = msg.linear_acceleration, msg.angular_velocity
                    v["ax"].append(a.x); v["ay"].append(a.y); v["az"].append(a.z)
                    v["gx"].append(g.x); v["gy"].append(g.y); v["gz"].append(g.z)
                else:
                    v["rt"].append(m.log_time * 1e-9)
                    v["rpm"].append(msg.data)
    d = {k: np.asarray(x, dtype=float) for k, x in v.items()}
    if not len(d["t"]) or not len(d["rt"]):
        raise ValueError(f"{path}: no /imu or no /motor_rpm messages")
    if cache:
        os.makedirs(cache_dir, exist_ok=True)
        np.savez(cache, **d)
    return d
