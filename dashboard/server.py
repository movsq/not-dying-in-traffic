"""Dashboard backend.

Read-only view onto the record plane plus two write endpoints. Runs in its own
process; it is a display, and a display must not be able to stall a control
loop no matter what it does.

The write endpoints are guarded. A localhost bind is not access control: any
page the operator has open in the same browser can POST to 127.0.0.1, and a
plain form POST is a CORS simple request, so nothing preflights it and nothing
blocks it. Halting a vehicle from a random web page is not a feature. Every
mutating request needs a same-origin Host, an Origin that is either absent or
ours, and a token minted at startup and only ever handed to the served page.
"""
from __future__ import annotations
import json, os, queue, secrets, threading, time, http.server, socketserver
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from carctl.plant import Plant
from carctl import safety, msgen

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PORT = int(os.environ.get("PORT", "8420"))
TOKEN = secrets.token_urlsafe(16)

subscribers: list[queue.Queue] = []
subs_lock = threading.Lock()
STATE = {"halted": False, "last_good_seq": 0}


def _broadcast(payload: dict) -> None:
    with subs_lock:
        targets = list(subscribers)
    dead = []
    for q in targets:
        try:
            q.put_nowait(payload)
        except queue.Full:
            dead.append(q)          # a slow browser is dropped, never waited on
    if dead:
        with subs_lock:
            for q in dead:
                if q in subscribers:
                    subscribers.remove(q)


def _drive_forever() -> None:
    while True:
        # A halt must not be consumed by the drive loop counting down its
        # frames in the background. Park here until someone resumes.
        while STATE["halted"]:
            time.sleep(0.2)
        plant, latched = Plant(), set()
        STATE["last_good_seq"] = 0     # new drive, new seq numbering
        for _ in range(180):
            if STATE["halted"]:
                break
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

    # ---- guards ------------------------------------------------------------
    def _authorised(self) -> str | None:
        """Return a refusal reason, or None if the request may proceed."""
        host = (self.headers.get("Host") or "").strip()
        if host not in (f"127.0.0.1:{PORT}", f"localhost:{PORT}"):
            return f"unexpected Host {host!r}"      # blocks DNS rebinding
        origin = self.headers.get("Origin")
        if origin and origin not in (f"http://127.0.0.1:{PORT}",
                                     f"http://localhost:{PORT}"):
            return f"cross-origin request from {origin!r}"
        token = self.headers.get("X-Revert-Token")
        if not token or not secrets.compare_digest(token, TOKEN):
            return "missing or bad X-Revert-Token"
        return None

    def _deny(self, why: str) -> None:
        self.send_response(403)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({"ok": False, "refused": why}).encode())

    def _json(self, payload: dict) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(payload).encode())

    # ---- reads -------------------------------------------------------------
    def do_GET(self):
        if self.path == "/events":
            q: queue.Queue = queue.Queue(maxsize=20)
            with subs_lock:
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
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                with subs_lock:
                    if q in subscribers:
                        subscribers.remove(q)
            return
        if self.path in ("/", "", "/index.html"):
            # The token reaches the page and nowhere else. Requiring it as a
            # custom header also forces a CORS preflight on any cross-origin
            # attempt, which we never answer.
            with open(os.path.join(HERE, "index.html"), "rb") as fh:
                body = fh.read().replace(b"__REVERT_TOKEN__", TOKEN.encode())
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        return super().do_GET()

    # ---- writes ------------------------------------------------------------
    def do_POST(self):
        if self.path not in ("/revert-hard", "/resume"):
            self.send_error(404); return
        why = self._authorised()
        if why:
            self._deny(why); return

        if self.path == "/resume":
            STATE["halted"] = False
            _broadcast({"resumed": True})
            self._json({"ok": True, "action": "resumed"})
            return

        # `git revert --hard` is not a git command. The button is not a git
        # command either. What it actually does: stop committing new frames,
        # bring the vehicle to a controlled stop, and hand the record plane to
        # the incident tooling. Naming it after a destructive git flag is the
        # honest label for "this ends the drive".
        STATE["halted"] = True
        _broadcast({"halted": True,
                    "note": "minimal-risk manoeuvre engaged; "
                            f"record frozen at seq {STATE['last_good_seq']}"})
        self._json({"ok": True,
                    "action": "minimal_risk_manoeuvre",
                    "record_frozen_at_seq": STATE["last_good_seq"],
                    "physically_undone": False})


if __name__ == "__main__":
    threading.Thread(target=_drive_forever, daemon=True).start()
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(("127.0.0.1", PORT), Handler) as httpd:
        print(f"dashboard on http://127.0.0.1:{PORT}")
        httpd.serve_forever()
