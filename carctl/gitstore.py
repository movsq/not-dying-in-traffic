"""The record plane.

Hard rule: the control loop never calls git. `git commit` forks, execs, builds
an index and fsyncs. Tens of milliseconds, with a tail you cannot bound. That
does not belong anywhere near a 100 ms deadline.

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

The same stream also writes refs/heads/lineage: one commit carrying only
models.json, every time a checkpoint changes. It goes down the same pipe
rather than through a separate git call so that a promotion and the first
frame that ran under it become visible at the same checkpoint. main has a
retention window and that ref does not, so once main is pruned the lineage ref
is the only surviving record of when a model shipped. See lineage.py.
"""
from __future__ import annotations
import queue, subprocess, tempfile, threading, time
from .state import Frame
from . import lineage, msgen

CHECKPOINT_EVERY = 50   # frames, 5 s of driving
QUEUE_DEPTH = 512       # frames, about 51 s of backlog before we start dropping

IDENT = b"not-dying-in-traffic <vsedlacek1337@gmail.com>"


def _data(payload: bytes) -> bytes:
    return b"data %d\n" % len(payload) + payload


def _tz_offset(epoch_s: int) -> bytes:
    """This machine's real UTC offset at that instant.

    fast-import's raw format takes `<epoch> <offset>`. The epoch was always
    correct, but the offset was hardcoded to +0200, so every commit written
    outside Prague summer time displayed an hour off its own timestamp.
    """
    off = -(time.altzone if time.localtime(epoch_s).tm_isdst else time.timezone)
    sign = "+" if off >= 0 else "-"
    off = abs(off)
    return f"{sign}{off // 3600:02d}{off % 3600 // 60:02d}".encode()


class Committer:
    """Owns the fast-import process. One instance per drive."""

    def __init__(self, repo: str, ref: str = "refs/heads/main",
                 lineage_ref: str = lineage.REF, drive_tag: str = ""):
        self.repo = repo
        self.ref = ref.encode()
        self.lineage_ref = lineage_ref.encode()
        # Named at drive start, created at drive end. The lineage commits
        # written mid-drive have to say which drive they happened during, and
        # a tag that does not exist yet still has a name.
        self.drive_tag = drive_tag
        self.q: queue.Queue[Frame | None] = queue.Queue(maxsize=QUEUE_DEPTH)
        self.dropped = 0
        self.committed = 0
        self.promotions = 0
        self._need_from = False
        self._lineage_from = False
        self._models: str | None = None
        self.error: Exception | None = None
        self._alive = True
        self._stderr = None
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
            except queue.Empty:
                # The committer drained the queue between the failed put and
                # this get, so there is room now and nothing needs evicting.
                # Catching Empty together with Full counted a drop here and
                # then discarded the frame we were handed -- dropping the
                # NEWEST into a queue that had just emptied. That is the
                # reverse of the documented policy, and the newest frame is
                # the one closest to whatever caused the stall.
                pass
            try:
                self.q.put_nowait(frame)
            except queue.Full:
                self.dropped += 1        # refilled again; this frame is lost

    # ---- committer-thread side ---------------------------------------------
    def start(self) -> None:
        # fast-import treats a ref it has not seen as new and roots the first
        # commit, which loses every previous drive. Continue the existing tip
        # explicitly so drives append into one history and bisect works across
        # them.
        existing = subprocess.run(
            ["git", "rev-parse", "--verify", "--quiet", self.ref.decode()],
            cwd=self.repo, capture_output=True, text=True, encoding="utf-8")
        self._need_from = existing.returncode == 0
        # What the lineage ref already says is in force. Comparing against
        # this rather than against the first frame is what makes a checkpoint
        # swapped while the vehicle was parked show up as a promotion instead
        # of vanishing into the gap between two drives.
        self._models = lineage.models_at(self.repo, self.lineage_ref.decode())
        self._lineage_from = self._models is not None
        # stderr goes to a file, not a pipe. Nothing reads a pipe until
        # communicate() at the very end, so a few hundred warning lines fill
        # the OS buffer, fast-import blocks writing stderr, stops reading
        # stdin, and the committer thread blocks writing to it. Deadlock at
        # the end of a drive, with the loop already finished.
        self._stderr = tempfile.TemporaryFile()
        self._proc = subprocess.Popen(
            ["git", "fast-import", "--date-format=raw", "--quiet", "--done"],
            cwd=self.repo, stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=self._stderr,
        )
        self._thread = threading.Thread(target=self._drain, daemon=True)
        self._thread.start()

    def _emit(self, f: Frame) -> bytes:
        out = [b"commit " + self.ref + b"\n",
               b"committer " + IDENT + b" %d " % f.t_wall_s
               + _tz_offset(f.t_wall_s) + b"\n",
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

    def _emit_lineage(self, f: Frame, models: str) -> bytes:
        """One commit carrying models.json and nothing else."""
        changed = lineage.changes(self._models, models)
        msg = lineage.message(changed, f.t_wall_s, self.drive_tag, f.seq)
        out = [b"commit " + self.lineage_ref + b"\n",
               b"committer " + IDENT + b" %d " % f.t_wall_s
               + _tz_offset(f.t_wall_s) + b"\n",
               _data(msg.encode())]
        if self._lineage_from:
            # Same trap as main: a fresh fast-import roots the first commit on
            # a ref it has not seen, which would orphan every promotion the
            # vehicle has ever recorded.
            out.append(b"from " + self.lineage_ref + b"^0\n")
            self._lineage_from = False
        out.append(b"M 100644 inline models.json\n")
        out.append(_data(models.encode()))
        out.append(b"\n")
        return b"".join(out)

    def _drain(self) -> None:
        assert self._proc and self._proc.stdin
        stdin = self._proc.stdin
        since_ckpt = 0
        try:
            while True:
                f = self.q.get()
                if f is None:
                    break
                models = f.models_json()
                if models != self._models:
                    # Before the frame, not after: if the stream dies here the
                    # record shows a promotion with no frames under it, which
                    # is readable. The other order shows frames running a
                    # checkpoint nothing recorded shipping.
                    stdin.write(self._emit_lineage(f, models))
                    self._models = models
                    self.promotions += 1
                stdin.write(self._emit(f))
                self.committed += 1
                since_ckpt += 1
                if since_ckpt >= CHECKPOINT_EVERY:
                    stdin.write(b"checkpoint\n")
                    since_ckpt = 0
                stdin.flush()
            stdin.write(b"done\n")
            stdin.flush()
        except (BrokenPipeError, OSError, ValueError) as exc:
            # fast-import died under us. Record it and let stop() report it
            # rather than dying silently and leaving the queue to fill.
            self.error = exc
        finally:
            self._alive = False
            try:
                stdin.close()
            except OSError:
                pass

    def _stderr_text(self) -> str:
        if not self._stderr:
            return ""
        self._stderr.seek(0)
        return self._stderr.read().decode("utf-8", "replace")[:2000]

    def stop(self) -> None:
        # A blocking put here hangs forever if the drain thread already died
        # and left the queue full. The sentinel is best effort; the thread is
        # joined with a timeout either way.
        if self._alive:
            try:
                self.q.put(None, timeout=5)
            except queue.Full:
                pass
        if self._thread:
            self._thread.join(timeout=30)
            if self._thread.is_alive():
                raise RuntimeError("committer thread did not finish in 30 s; "
                                   f"{self.q.qsize()} frames still queued")
        if self._proc:
            self._proc.wait(timeout=30)
            if self._proc.returncode != 0 or self.error:
                # Formatted separately. Concatenating the two branches glued
                # the exit code onto the errno text, so a broken pipe on exit
                # 2 read "fast-import failed ([Errno 32] Broken pipe2)".
                why = (repr(self.error) if self.error
                       else f"exit {self._proc.returncode}")
                raise RuntimeError(
                    f"fast-import failed ({why}): {self._stderr_text()}")
        if self._stderr:
            self._stderr.close()
