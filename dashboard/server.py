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

Reads are guarded too, on the Host alone. That check used to sit only on the
write path, which left the two routes that matter most wide open: `GET /` is
what hands out the token, and `/events` is the live 10 Hz position feed. A
page that rebinds DNS to 127.0.0.1 could read both, so the guard that was
described as blocking DNS rebinding was not on the requests being rebound.
"""
from __future__ import annotations
import json, os, queue, secrets, threading, time, http.server
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

HEARTBEAT_S = 5.0          # idle gap before the stream proves it is still alive
_STOP = object()           # sentinel: this subscriber has been dropped


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
        for q in dead:
            # Wake the handler so it closes the socket and the browser's
            # EventSource reconnects. Dropping the queue and walking away left
            # that thread parked in get() forever with the socket still open,
            # so the page froze on a stale frame while continuing to display
            # "streaming @ 10 Hz" -- stale state shown as live, on a safety
            # display. Make room first: the queue is full, that is why we are
            # here.
            try:
                q.get_nowait()
            except queue.Empty:
                pass
            try:
                q.put_nowait(_STOP)
            except queue.Full:
                pass


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


class Handler(http.server.BaseHTTPRequestHandler):
    # Deliberately not SimpleHTTPRequestHandler. Inheriting a static file
    # server meant every path this class did not claim was served off disk
    # from the dashboard directory; the two routes below are the whole API.

    def log_message(self, *a):  # keep the console clean
        pass

    # ---- guards ------------------------------------------------------------
    def _host_ok(self) -> str | None:
        """Host check. Applied to EVERY route, reads included.

        This used to live only inside _authorised(), which only do_POST
        called, so the line commented "blocks DNS rebinding" blocked nothing
        on a GET -- and GET / is the route that hands out the token.
        """
        host = (self.headers.get("Host") or "").strip()
        if host not in (f"127.0.0.1:{PORT}", f"localhost:{PORT}"):
            return f"unexpected Host {host!r}"
        return None

    def _route(self) -> str:
        """The request path, without query or fragment, lowercased.

        Matching self.path exactly sent `/?v=2` and `/INDEX.HTML` to the
        static handler, which served index.html carrying the literal
        __REVERT_TOKEN__ -- a page that streams telemetry and looks perfectly
        healthy while every halt it sends is refused forever.
        """
        path = self.path.split("#", 1)[0].split("?", 1)[0]
        return path.rstrip("/").lower() or "/"

    def _authorised(self) -> str | None:
        """Return a refusal reason, or None if the request may proceed."""
        why = self._host_ok()
        if why:
            return why
        origin = self.headers.get("Origin")
        if origin and origin not in (f"http://127.0.0.1:{PORT}",
                                     f"http://localhost:{PORT}"):
            return f"cross-origin request from {origin!r}"
        token = self.headers.get("X-Revert-Token")
        # Headers are latin-1 decoded and compare_digest raises TypeError on a
        # non-ASCII str, which escaped the handler and closed the connection
        # with no response at all. A non-ASCII token is simply a wrong token.
        if (not token or not token.isascii()
                or not secrets.compare_digest(token, TOKEN)):
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
        why = self._host_ok()
        if why:
            self._deny(why); return
        route = self._route()
        if route == "/events":
            q: queue.Queue = queue.Queue(maxsize=20)
            with subs_lock:
                subscribers.append(q)
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    try:
                        data = q.get(timeout=HEARTBEAT_S)
                    except queue.Empty:
                        # An SSE comment. Proves the socket is still there,
                        # and lets a peer that went away surface as an OSError
                        # instead of parking this thread in get() forever.
                        self.wfile.write(b": keep-alive\n\n")
                        self.wfile.flush()
                        continue
                    if data is _STOP:
                        break      # dropped by _broadcast; let the page reconnect
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
        if route in ("/", "/index.html"):
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
        # No fallthrough to a static file handler. Serving the dashboard
        # directory meant GET /server.py returned this file verbatim, guards
        # and all, to anyone who asked.
        self.send_error(404)

    # ---- writes ------------------------------------------------------------
    def do_POST(self):
        route = self._route()
        if route not in ("/revert-hard", "/resume"):
            self.send_error(404); return
        why = self._authorised()
        if why:
            self._deny(why); return

        if route == "/resume":
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
    # SO_REUSEADDR means something different on Windows: a second process can
    # bind a port another process is already serving, and requests go to
    # whichever the kernel picks. Launching twice then leaves a stale server
    # answering, with whatever guards the old code had. Fail the second bind.
    class _Server(http.server.ThreadingHTTPServer):
        # ThreadingHTTPServer, not ThreadingTCPServer: the latter leaves
        # daemon_threads False and block_on_close True, so server_close()
        # joins every handler thread. One SSE stream parked in q.get() then
        # wedged the process on Ctrl-C while still holding the port -- and
        # with reuse off, the relaunch below fails rather than shadowing it,
        # so the operator was left with no dashboard until they killed a PID.
        allow_reuse_address = (os.name != "nt")

    with _Server(("127.0.0.1", PORT), Handler) as httpd:
        print(f"dashboard on http://127.0.0.1:{PORT}")
        httpd.serve_forever()
