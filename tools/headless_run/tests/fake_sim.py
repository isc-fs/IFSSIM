"""Stands in for the sim: answers the line protocol on a port; can crash on cue."""
import socket, sys, threading, time

port = int(sys.argv[1])
die_after = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
boot_s = float(sys.argv[3]) if len(sys.argv) > 3 else 0.2
slow_s = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0  # sim clock at half speed this long
if die_after < 0:          # never comes up
    sys.exit(7)
time.sleep(boot_s)
srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", port))
srv.listen(8)
t_up = time.monotonic()


def sim_time():
    up = time.monotonic() - t_up
    return 1.0 + (up / 2 if up < slow_s else slow_s / 2 + (up - slow_s))


if die_after:
    threading.Timer(die_after, lambda: __import__("os")._exit(9)).start()
while True:
    c, _ = srv.accept()
    line = c.makefile().readline().strip()
    status = '{"map":"fake","fps":60.0,"paused":false,"sim_time":%.3f}' % sim_time()
    reply = {"ping": "true", "getSimStatus": status}.get(line, "{}")
    c.sendall((reply + "\n").encode())
    c.close()
