"""Dashboard backend.

Read-only view onto the record plane plus two write endpoints. Runs in its own
process; it is a display, and a display must not be able to stall a control
loop no matter what it does.

What this process actually drives: its own `Plant`, in a background thread,
for display. There is no vehicle behind it and no git command anywhere in this
file. The halt endpoint stops that simulated drive and freezes the *displayed*
record state -- the seq the page shows as the last good one -- and hands the
real record to the incident tooling in carctl. It moves no refs, writes no
commits and reverts nothing. Saying otherwise on a safety display is the same
defect as showing stale state as live: the operator would be told an operation
happened that no code here performs.

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
import errno, json, os, queue, secrets, threading, time, http.server
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
STATE = {"halted": False, "last_good_seq": 0, "halt_note": ""}

# The last telemetry frame that went out. A subscriber that connects between
# broadcasts has to be told where the vehicle is; it cannot wait to be told.
LAST_PAYLOAD: dict | None = None

HEARTBEAT_S = 5.0          # idle gap before the stream proves it is still alive
_STOP = object()           # sentinel: this subscriber has been dropped


def _broadcast(payload: dict) -> None:
    global LAST_PAYLOAD
    if "seq" in payload:
        # Only telemetry frames are state. "halted" and "resumed" are events:
        # replaying one to a subscriber that arrived afterwards would announce
        # a transition that did not happen while it was listening.
        LAST_PAYLOAD = payload
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


def _seed_payload() -> dict:
    """The first frame a new subscriber gets, written before the queue loop.

    EventSource fires onopen the moment the response headers land, so the page
    starts claiming a live stream immediately. If nothing follows -- the drive
    is halted and nothing is being broadcast at all, or the first frame is
    still up to 100 ms away -- the operator gets "streaming @ 10 Hz" over a
    column of dashes and a live halt button. That is the same defect the drop
    path above was fixed for, stale state shown as live on a safety display,
    arriving from the other end: never having had state rather than having
    stopped getting it. Say what is true at the moment of connection.
    """
    if STATE["halted"]:
        return {"halted": True, "note": STATE["halt_note"],
                "record_frozen_at_seq": STATE["last_good_seq"]}
    if LAST_PAYLOAD is not None:
        return LAST_PAYLOAD
    # Nothing has been driven yet. There is no pose to report, but "not
    # halted" is still a fact the page cannot work out on its own.
    return {"halted": False}


def _drive_forever() -> None:
    while True:
        # A halt must not be consumed by the drive loop counting down its
        # frames in the background. Park here until someone resumes.
        while STATE["halted"]:
            time.sleep(0.2)
        plant, latched = Plant(), set()
        STATE["last_good_seq"] = 0     # new drive, new seq numbering
        # Reset with the seq numbering, for the same reason: the door is a
        # property of this drive's history, not of the process.
        one_way_door = False
        prev = None                    # previous frame, for detect's closing-speed gate
        for _ in range(180):
            if STATE["halted"]:
                break
            # Ground truth is read BEFORE the step, for the reason loop.py
            # spells out at its own call site: step() ends by advancing the
            # plant to the next tick, so asked afterwards the oracle answers
            # for tick i+1 and detect() judges frame_i against a world 100 ms
            # into its own future. loop.drive, cmd_incident and cmd_replay all
            # sample in this order; this was the fourth call site, still
            # sampling late. A light phase boundary landing on a tick edge
            # then makes this display's incidents disagree with the record's
            # on the very same deterministic script -- a safety display
            # contradicting the plane of record it claims to show.
            truth = plant.true_light()
            f = plant.step()
            fresh = [i for i in safety.detect(f, truth, prev)
                     if i.kind not in latched]
            prev = f
            for i in fresh:
                latched.add(i.kind)
            inc = fresh[0] if fresh else None
            # The latch stops at the first one-way door and stays there. `!`
            # means the history stops being invertible *from there on*, so the
            # reversible frames after an irreversible one are not somewhere the
            # record can be taken back to -- getting there would have to undo
            # the curb strike on the way. Advancing past it (111, 118, 121...)
            # offered the operator a rollback target that the project's own
            # definition of the marker says does not exist.
            if not f.reversible:
                one_way_door = True
            elif not one_way_door:
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
        # A HEAD response carries no body by definition; writing one here
        # would leave bytes in the socket that the client counts as the start
        # of the next response.
        if self.command != "HEAD":
            self.wfile.write(json.dumps({"ok": False, "refused": why}).encode())

    def _json(self, payload: dict) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(payload).encode())

    # ---- reads -------------------------------------------------------------
    def _serve_index(self, head: bool) -> None:
        """The index route, for GET and HEAD alike.

        One builder, one set of headers. do_HEAD used to carry its own copy of
        this minus the body write, so every header or token-substitution
        change had to be mirrored by hand -- and a HEAD whose headers drift
        from GET's is a HEAD nothing can use to decide whether to fetch the
        page.

        The token reaches the page and nowhere else. Requiring it as a custom
        header also forces a CORS preflight on any cross-origin attempt, which
        we never answer.
        """
        with open(os.path.join(HERE, "index.html"), "rb") as fh:
            body = fh.read().replace(b"__REVERT_TOKEN__", TOKEN.encode())
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        # This body carries the halt token. A cached copy is that token
        # sitting in a disk cache long after the process that minted it
        # exited, and the stale page served back from it is one whose
        # every halt is refused while it looks perfectly healthy.
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if not head:                # no body on HEAD: that is the whole point
            self.wfile.write(body)

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
                self.wfile.write(b"data: " +
                                 json.dumps(_seed_payload()).encode() + b"\n\n")
                self.wfile.flush()
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
            self._serve_index(head=False)
            return
        # No fallthrough to a static file handler. Serving the dashboard
        # directory meant GET /server.py returned this file verbatim, guards
        # and all, to anyone who asked.
        self.send_error(404)

    def do_HEAD(self):
        # Same guard, same routes, headers only. Without this method
        # BaseHTTPRequestHandler answered HEAD with its own 501 before
        # _host_ok() ever ran -- a response carrying the Server and Date
        # headers to any Host at all, which is exactly the read the guard
        # above exists to refuse. It also announced the process to a rebound
        # page that GET would have turned away.
        why = self._host_ok()
        if why:
            self._deny(why); return
        route = self._route()
        if route in ("/", "/index.html"):
            self._serve_index(head=True)
            return
        # /events falls through to 404 on purpose: EventSource never issues
        # HEAD, and answering 200 with an unconsumed stream would hold a
        # handler thread for nothing.
        self.send_error(404)

    def _unsupported(self):
        """Everything else: guarded first, then 405.

        These went to the inherited 501 as well, so PUT/DELETE/OPTIONS from
        any Host got a reply -- and OPTIONS in particular is what a browser
        sends to probe. Nothing here is ever going to grow a PUT, so the
        answer is Method Not Allowed rather than Not Implemented, and it is
        only given to a caller that cleared the Host check.
        """
        why = self._host_ok()
        if why:
            self._deny(why); return
        self.send_response(405)
        self.send_header("Content-Type", "application/json")
        # 405 without Allow is an incomplete 405, and the list is the two
        # methods this server has.
        self.send_header("Allow", "GET, HEAD, POST")
        self.end_headers()
        self.wfile.write(json.dumps(
            {"ok": False, "refused": f"method {self.command} not allowed"}
        ).encode())

    do_PUT = do_DELETE = do_OPTIONS = do_PATCH = _unsupported

    # ---- writes ------------------------------------------------------------
    def do_POST(self):
        # Authorisation first, routing second. The other order answered an
        # unauthorised POST to an unknown path with a bare 404, which
        # contradicts the rule this module states at the top: EVERY mutating
        # request needs a same-origin Host. A guard that only covers the paths
        # that happen to exist is not a uniform guard, and the 404/403 split
        # tells an unauthorised caller which routes are real.
        why = self._authorised()
        if why:
            self._deny(why); return
        route = self._route()
        if route not in ("/revert-hard", "/resume"):
            self.send_error(404); return

        if route == "/resume":
            STATE["halted"] = False
            STATE["halt_note"] = ""
            _broadcast({"resumed": True})
            self._json({"ok": True, "action": "resumed"})
            return

        # `git revert --hard` is not a git command. The button is not a git
        # command either. What it actually does: stop committing new frames,
        # bring the vehicle to a controlled stop, and hand the record plane to
        # the incident tooling. Naming it after a destructive git flag is the
        # honest label for "this ends the drive".
        STATE["halted"] = True
        # Kept on STATE, not only broadcast. A broadcast reaches whoever was
        # listening at the time; a page opened one second later has to be told
        # the same sentence, and _seed_payload() is the only thing that can
        # tell it.
        STATE["halt_note"] = ("minimal-risk manoeuvre engaged; "
                              f"record frozen at seq {STATE['last_good_seq']}")
        _broadcast({"halted": True, "note": STATE["halt_note"],
                    "record_frozen_at_seq": STATE["last_good_seq"]})
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

    try:
        server = _Server(("127.0.0.1", PORT), Handler)
    except OSError as exc:
        # The POSIX half of the WinError 10048 story in the README. There,
        # reuse-on-Windows let a second launch silently shadow the first and
        # the operator tested against the wrong build; here errno 98 refuses
        # the bind instead, which is the right outcome reported as a
        # ten-line traceback that reads like the dashboard crashed. It did
        # not: the port is taken, almost always by the dashboard already
        # running, and that is one line to say.
        if exc.errno == errno.EADDRINUSE:
            print(f"port {PORT} is already in use -- another dashboard is "
                  f"probably still running. Stop it, or set PORT.",
                  file=sys.stderr, flush=True)
            raise SystemExit(1)
        raise
    with server as httpd:
        # flush=True: stdout to a pipe or a log file is block-buffered, so the
        # one line telling the operator where the dashboard is sat in the
        # buffer until the process exited. A readiness banner that arrives at
        # shutdown is not a readiness banner.
        print(f"dashboard on http://127.0.0.1:{PORT}", flush=True)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            # Ctrl-C is how this process is meant to be stopped. Ending a
            # deliberate shutdown with a traceback trains the operator to
            # ignore tracebacks from a safety display.
            print("\ndashboard stopped", flush=True)
