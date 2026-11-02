"""Stands in for the sim: answers the line protocol on a port; can crash on cue."""
import socket, sys, threading, time

port = int(sys.argv[1])
die_after = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
boot_s = float(sys.argv[3]) if len(sys.argv) > 3 else 0.2
if die_after < 0:          # never comes up
    sys.exit(7)
time.sleep(boot_s)
srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", port))
srv.listen(8)
if die_after:
    threading.Timer(die_after, lambda: __import__("os")._exit(9)).start()
while True:
    c, _ = srv.accept()
    line = c.makefile().readline().strip()
    reply = {"ping": "true", "getSimStatus": '{"map":"fake","fps":60.0,"paused":false}'}.get(line, "{}")
    c.sendall((reply + "\n").encode())
    c.close()
