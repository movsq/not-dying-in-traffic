"""The record plane.

Hard rule: the control loop never calls git. `git commit` forks, execs, builds
an index and fsyncs — tens of milliseconds with an unbounded tail. That does
not belong anywhere near a 100 ms deadline.

So the loop hands frames to a bounded queue and moves on. A committer thread
drains that queue into a single long-lived `git fast-import` process, which
takes commits over a pipe and appends them without ever touching the index or
the working tree. Throughput is tens of thousands of commits/sec, so the queue
is empty in steady state and the loop's cost is one enqueue.

Two consequences worth knowing about:

  * Refs only become visible to other git commands at a `checkpoint`. We
    checkpoint every CHECKPOINT_EVERY commits, so `git log` trails live state
    by up to that many frames. Visibility latency traded for throughput.
  * The queue drops oldest on overflow rather than blocking. A stalled disk
    must cost you history, never a control cycle. Drops are counted and
    reported; a drop is a defect, not a normal condition.
"""
from __future__ import annotations
import queue, subprocess, threading, os
from .state import Frame
from . import msgen

CHECKPOINT_EVERY = 50   # frames — 5 s of driving
QUEUE_DEPTH = 512       # frames — ~51 s of backlog before we start dropping

IDENT = b"not-dying-in-traffic <vsedlacek1337@gmail.com>"


def _data(payload: bytes) -> bytes:
    return b"data %d\n" % len(payload) + payload


class Committer:
    """Owns the fast-import process. One instance per drive."""

    def __init__(self, repo: str, ref: str = "refs/heads/main"):
        self.repo = repo
        self.ref = ref.encode()
        self.q: queue.Queue[Frame | None] = queue.Queue(maxsize=QUEUE_DEPTH)
        self.dropped = 0
        self.committed = 0
        self._need_from = False
        self._thread: threading.Thread | None = None
        self._proc: subprocess.Popen | None = None

    # ---- control-loop side -------------------------------------------------
    def submit(self, frame: Frame) -> None:
        """Called from the control thread. Never blocks, never raises."""
        try:
            self.q.put_nowait(frame)
        except queue.Full:
            try:
                self.q.get_nowait()      # drop oldest
                self.dropped += 1
                self.q.put_nowait(frame)
            except (queue.Empty, queue.Full):
                self.dropped += 1

    # ---- committer-thread side ---------------------------------------------
    def start(self) -> None:
        # fast-import treats a ref it has not seen as new and roots the first
        # commit, which loses every previous drive. Continue the existing tip
        # explicitly so drives append into one history and bisect works across
        # them.
        existing = subprocess.run(
            ["git", "rev-parse", "--verify", "--quiet", self.ref.decode()],
            cwd=self.repo, capture_output=True, text=True)
        self._need_from = existing.returncode == 0
        self._proc = subprocess.Popen(
            ["git", "fast-import", "--date-format=raw", "--quiet", "--done"],
            cwd=self.repo, stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
        )
        self._thread = threading.Thread(target=self._drain, daemon=True)
        self._thread.start()

    def _emit(self, f: Frame) -> bytes:
        out = [b"commit " + self.ref + b"\n",
               b"committer " + IDENT + b" %d +0200\n" % f.t_wall_s,
               _data(msgen.message(f).encode())]
        if self._need_from:
            out.append(b"from " + self.ref + b"^0\n")
            self._need_from = False
        # After the first, no `from` line. fast-import parents each commit on
        # the head it is already tracking, which is the linear history we want.
        for path, content in f.tree().items():
            out.append(b"M 100644 inline " + path.encode() + b"\n")
            out.append(_data(content.encode()))
        out.append(b"\n")
        return b"".join(out)

    def _drain(self) -> None:
        assert self._proc and self._proc.stdin
        stdin = self._proc.stdin
        since_ckpt = 0
        while True:
            f = self.q.get()
            if f is None:
                break
            stdin.write(self._emit(f))
            self.committed += 1
            since_ckpt += 1
            if since_ckpt >= CHECKPOINT_EVERY:
                stdin.write(b"checkpoint\n")
                since_ckpt = 0
            stdin.flush()
        stdin.write(b"done\n")
        stdin.flush()
        stdin.close()

    def stop(self) -> None:
        self.q.put(None)
        if self._thread:
            self._thread.join(timeout=30)
        if self._proc:
            _, err = self._proc.communicate(timeout=30)
            if self._proc.returncode != 0:
                raise RuntimeError(f"fast-import failed: {err.decode()[:2000]}")
