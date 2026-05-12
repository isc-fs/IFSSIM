"""Helper for tools/smoke/test_control_smoke.py.

Run inside the dv_pipeline_stack container — needs rclpy, fs_msgs.

Usage:
    python3 _count_control_msgs.py <path-to-rosbag2.db3>

Prints "<total> <nontrivial> <steering_nonzero>" — counts of:
    total                  — all /control_command messages
    nontrivial             — throttle > 0 OR brake > 0
    steering_nonzero       — steering != 0

Lives as a separate file so the smoke test runner doesn't have to
inline-quote a python script through docker exec.
"""

from __future__ import annotations

import sqlite3
import sys

from rclpy.serialization import deserialize_message
from fs_msgs.msg import ControlCommand


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: _count_control_msgs.py <db3_path>", file=sys.stderr)
        sys.exit(2)
    db_path = sys.argv[1]

    db = sqlite3.connect(db_path)
    cur = db.cursor()
    cur.execute("SELECT id FROM topics WHERE name=?", ("/control_command",))
    row = cur.fetchone()
    if row is None:
        print("0 0 0")
        return
    tid = row[0]

    cur.execute("SELECT data FROM messages WHERE topic_id=?", (tid,))
    total = 0
    nontrivial = 0
    steering_nonzero = 0
    for (data,) in cur:
        total += 1
        msg = deserialize_message(data, ControlCommand)
        if msg.throttle > 0.0 or msg.brake > 0.0:
            nontrivial += 1
        if msg.steering != 0.0:
            steering_nonzero += 1
    print(f"{total} {nontrivial} {steering_nonzero}")


if __name__ == "__main__":
    main()
