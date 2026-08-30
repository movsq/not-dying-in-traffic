"""Dashboard backend.

Read-only view onto the record plane plus a single write endpoint for the big
red button. Runs in its own process at low priority; it is a display, and a
display must not be able to stall a control loop no matter what it does.
"""
from __future__ import annotations
import json, os, queue, threading, time, http.server, socketserver, urllib.parse
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from carctl.plant import Plant
from carctl import safety, msgen

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

subscribers: list[queue.Queue] = []
STATE = {"halted": False, "last_good_seq": 0}


def _broadcast(payload: dict) -> None:
    dead = []
    for q in subscribers:
        try:
            q.put_nowait(payload)
        except queue.Full:
            dead.append(q)          # a slow browser is dropped, never waited on
    for q in dead:
        subscribers.remove(q)


def _drive_forever() -> None:
    while True:
        plant, latched = Plant(), set()
        for _ in range(180):
            if STATE["halted"]:
                time.sleep(0.1); continue
            f = plant.step()
            inc = safety.detect(f, plant.true_light())
            if inc and inc.kind in latched:
                inc = None
            elif inc:
                latched.add(inc.kind)
            if f.reversible:
                STATE["last_good_seq"] = f.seq
            _broadcast({
                "seq": f.seq, "kmh": round(f.pose.v * 3.6, 1),
                "road": f.road, "maneuver": f.maneuver,
                "steer": round(f.pose.steer, 3),
                "x": round(f.pose.x, 1), "y": round(f.pose.y, 1),
                "light": f.sensors.light_state,
                "clearance": round(f.sensors.lidar_min_range, 1),
                "reversible": f.reversible,
                "subject": msgen.subject(f),
                "checkpoints": f.checkpoints,
                "incident": None if not inc else
                    {"kind": inc.kind, "detail": inc.detail,
                     "owner": inc.subsystem},
                "last_good_seq": STATE["last_good_seq"],
            })
            time.sleep(0.1)


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=HERE, **kw)

    def log_message(self, *a):  # keep the console clean
        pass

    def do_GET(self):
        if self.path == "/events":
            q: queue.Queue = queue.Queue(maxsize=20)
            subscribers.append(q)
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    data = q.get()
                    self.wfile.write(b"data: " +
                                     json.dumps(data).encode() + b"\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                if q in subscribers:
                    subscribers.remove(q)
            return
        if self.path == "/" or self.path == "":
            self.path = "/index.html"
        return super().do_GET()

    def do_POST(self):
        if self.path != "/revert-hard":
            self.send_error(404); return
        # `git revert --hard` is not a git command. The button is not a git
        # command either. What it actually does is: stop committing new frames,
        # bring the vehicle to a controlled stop, and hand the record plane to
        # the incident tooling. Naming it after a destructive git flag is the
        # honest label for "this ends the drive".
        STATE["halted"] = True
        _broadcast({"halted": True,
                    "note": "minimal-risk manoeuvre engaged; "
                            f"record frozen at seq {STATE['last_good_seq']}"})
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({
            "ok": True,
            "action": "minimal_risk_manoeuvre",
            "record_frozen_at_seq": STATE["last_good_seq"],
            "physically_undone": False,
        }).encode())


if __name__ == "__main__":
    threading.Thread(target=_drive_forever, daemon=True).start()
    port = int(os.environ.get("PORT", "8420"))
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(("127.0.0.1", port), Handler) as httpd:
        print(f"dashboard on http://127.0.0.1:{port}")
        httpd.serve_forever()
