"""Helper for tools/smoke/test_path_planning_smoke.py.

Run inside the dv_pipeline_stack container — needs rclpy, nav_msgs.

Usage:
    python3 _count_path_msgs.py <path-to-rosbag2.db3>

Prints "<total> <nontrivial>" — the count of /Path messages and the
count of those with len(poses) >= 2. Lives as a separate file so the
smoke test runner doesn't have to inline-quote a python script through
docker exec.
"""

from __future__ import annotations

import sqlite3
import sys

from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: _count_path_msgs.py <db3_path>", file=sys.stderr)
        sys.exit(2)
    db_path = sys.argv[1]

    db = sqlite3.connect(db_path)
    cur = db.cursor()
    cur.execute("SELECT id FROM topics WHERE name=?", ("/Path",))
    row = cur.fetchone()
    if row is None:
        print("0 0")
        return
    tid = row[0]

    cur.execute("SELECT data FROM messages WHERE topic_id=?", (tid,))
    total = 0
    nontrivial = 0
    for (data,) in cur:
        total += 1
        msg = deserialize_message(data, Path)
        if len(msg.poses) >= 2:
            nontrivial += 1
    print(f"{total} {nontrivial}")


if __name__ == "__main__":
    main()
