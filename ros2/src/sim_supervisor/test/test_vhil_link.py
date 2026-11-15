"""vhil_link: the client of the emulated boards' server (no ROS needed)."""

import json
import socket
import threading
import time

from sim_supervisor.vhil_link import VhilLink, address_from_env


class Boards:
    """A stand-in for tools/vhil_boards/server.py: speaks its JSON lines."""

    def __init__(self):
        self.srv = socket.create_server(("127.0.0.1", 0))
        self.port = self.srv.getsockname()[1]
        self.received: list[dict] = []
        self.conns: list[socket.socket] = []
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            conn, _ = self.srv.accept()
            self.conns.append(conn)
            threading.Thread(target=self._read, args=(conn,), daemon=True).start()

    def _read(self, conn):
        for line in conn.makefile("r"):
            self.received.append(json.loads(line))

    def say(self, **state):
        self.conns[-1].sendall((json.dumps(state) + "\n").encode())

    def drop(self):
        conn = self.conns.pop()
        conn.shutdown(socket.SHUT_RDWR)  # close() alone keeps the makefile's descriptor open
        conn.close()


def wait(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_the_address_comes_from_the_environment():
    assert address_from_env({}) is None
    assert address_from_env({"IFSSIM_VHIL_BOARDS": "127.0.0.1:47800"}) == ("127.0.0.1", 47800)
    assert address_from_env({"IFSSIM_VHIL_BOARDS": ":47801"}) == ("127.0.0.1", 47801)


def test_state_from_the_boards_and_commands_to_them():
    b = Boards()
    link = VhilLink(("127.0.0.1", b.port))
    assert link.assi() is None and link.mission() == -1 and link.torque_nm() is None  # silent: nothing claimed
    assert wait(lambda: b.conns)
    b.say(assi=3, mission=1, torque_nm=-89)
    assert wait(link.fresh)
    assert (link.assi(), link.mission(), link.torque_nm()) == (3, 1, -89.0)
    link.send(cmd="ctrl", throttle=0.4, steer=0.0)
    assert wait(lambda: {"cmd": "ctrl", "throttle": 0.4, "steer": 0.0} in b.received)


def test_the_operator_panel_is_replayed_when_the_boards_come_back():
    b = Boards()
    link = VhilLink(("127.0.0.1", b.port))
    assert wait(lambda: b.conns)
    link.send(cmd="mission", idx=2)
    link.send(cmd="intent", v=1)
    link.send(cmd="dv_status", v=2)  # not part of the panel: not replayed
    assert wait(lambda: len(b.received) == 3)
    b.drop()
    b.received.clear()
    assert wait(lambda: b.conns and link._sock is not None, 5.0)
    assert wait(lambda: len(b.received) >= 2)
    assert {"cmd": "mission", "idx": 2} in b.received and {"cmd": "intent", "v": 1} in b.received
    assert not any(m["cmd"] == "dv_status" for m in b.received)
