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
import contextlib, os, queue, subprocess, tempfile, threading, time
from .state import Frame, FRAME_PATHS
from . import lineage, msgen
from .publish import PUBLIC_IDENT

CHECKPOINT_EVERY = 50   # frames, 5 s of driving
QUEUE_DEPTH = 512       # frames, about 51 s of backlog before we start dropping

IDENT = b"not-dying-in-traffic <vsedlacek1337@gmail.com>"

# FRAME_PATHS is imported, not restated. This is the side that has to
# recognise a tree somebody else wrote, and it used to carry its own copy of
# the four names: the writer and the checker held the same fact twice, which
# is the same fact only until one of them is edited.


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

    def abandon(self) -> None:
        """Kill the fast-import child, now, without waiting for anything.

        stop() is the orderly path and it can spend a minute in joins. This is
        for the caller who has already decided not to wait -- loop.py's second
        Ctrl-C. Killing the child rather than simply walking away is the same
        reasoning stop()'s own timeout paths use: an orphaned fast-import holds
        a half-written pack open, and on Windows an open pack is an unlinkable
        one, so the next `carctl maintain` fails its repack for a reason that
        has nothing to do with maintenance.
        """
        proc = self._proc
        if proc is not None:
            with contextlib.suppress(OSError):
                proc.kill()

    # ---- committer-thread side ---------------------------------------------
    def _rev(self, ref: str) -> str:
        """The sha `ref` resolves to in this repo, or "" if it does not.

        Three copies of this incantation lived inline, each spelled slightly
        differently and each reading its answer out of a different field --
        returncode in one place, stdout in another. They all mean the same
        thing, and the two questions they decide (root a new ref, or continue
        an existing one) are the ones where getting the polarity backwards
        orphans every commit already on the ref.
        """
        return subprocess.run(
            ["git", "rev-parse", "--verify", "--quiet", ref], cwd=self.repo,
            capture_output=True, text=True, encoding="utf-8").stdout.strip()

    def _check_frame_tree(self) -> None:
        """Refuse to append frames onto a tip that is not a frame.

        _emit now writes `deleteall`, so a polluted tip can no longer ride
        forward into the frames appended to it -- but that makes this check
        more useful, not redundant. Driving in a repo with the source checked
        into main is the easy mistake, because that is the branch this package
        is developed on, and the two possible answers to it are "bury the
        source under 170 frame commits and say nothing" or "refuse and say
        which repo you are in". Silently burying it is the worse one: the
        commits are indistinguishable from a real drive's afterwards, and the
        operator learns nothing about having run the command in the wrong
        place. Before the first commit rather than after the drive, because
        after the drive the damage is the history.
        """
        r = subprocess.run(["git", "ls-tree", "--name-only", self.ref.decode()],
                           cwd=self.repo, capture_output=True, text=True,
                           encoding="utf-8")
        if r.returncode != 0:
            raise RuntimeError(f"cannot read the tree at {self.ref.decode()}: "
                               f"{r.stderr.strip()}")
        extra = sorted({n.strip() for n in r.stdout.splitlines()
                        if n.strip()}.difference(FRAME_PATHS))
        if extra:
            shown = ", ".join(extra[:6]) + (", ..." if len(extra) > 6 else "")
            raise RuntimeError(
                f"{self.ref.decode()} is not a frame history: its tip carries "
                f"{shown}. Every frame of this drive would inherit that tree, "
                "because frames are appended onto it rather than replacing "
                "it. Run carctl from the repository the record lives in, not "
                "from the one the source lives in.")

    def _seed_lineage(self) -> None:
        """Adopt origin's lineage before anything reads the local ref.

        `git clone` materialises only HEAD's branch, so a clone of this repo
        arrives with refs/remotes/origin/lineage and no refs/heads/lineage at
        all. models_at() then found nothing in force: the first frame of the
        first drive read as "record 4 checkpoints" rather than as the rollback
        off the promoted checkpoint that it actually is, the promotion it
        wrote rooted a second parallel lineage, and every promotion the fleet
        had already published was orphaned by the act of driving once. blame
        then answered from the provisional main path instead of the ref built
        to outlive it.
        """
        local = self.lineage_ref.decode()
        if self._rev(local):
            return
        # `refs/remotes/origin/lineage` was hardcoded here, which made the
        # remote's NAME load-bearing: the review's point is that a clone whose
        # remote is called `upstream`, or `fleet`, found nothing to continue
        # and quietly rooted a lineage parallel to the published one -- the
        # exact failure this method exists to prevent, reintroduced by a
        # spelling. Ask lineage which remote refs actually resolve instead.
        candidates = lineage.remote_refs(self.repo)
        name = local.rsplit("/", 1)[-1]
        origin = f"refs/remotes/origin/{name}"
        if origin in candidates:
            # Still preferred when it is there. `origin` is what a clone calls
            # the place it came from, and a repo with several remotes has an
            # answer to "whose lineage is this" that guessing does not.
            upstream = origin
        elif len(candidates) == 1:
            upstream = candidates[0]
        else:
            # None of them, or several with no origin among them. Neither is a
            # published lineage this drive can be sure it is continuing, and
            # picking one at random is how a renamed remote roots the parallel
            # lineage in the first place. Say so and root a local one, which is
            # at least visibly a new lineage rather than a silent fork.
            print(f"warning: no published lineage was found to continue "
                  f"({len(candidates)} remote lineage ref(s) resolve, none of "
                  f"them unambiguous); this drive's promotions will start a "
                  f"lineage of their own")
            return
        seed = subprocess.run(["git", "update-ref", local, self._rev(upstream)],
                              cwd=self.repo, capture_output=True, text=True,
                              encoding="utf-8")
        if seed.returncode != 0:
            # Not fatal -- the drive can still record its promotions -- but it
            # is about to root a lineage parallel to the published one, and
            # that is worth saying out loud rather than discovering later.
            print(f"warning: could not seed {local} from {upstream}: "
                  f"{seed.stderr.strip()}; this drive's promotions will start "
                  "a lineage of their own")

    def start(self) -> None:
        # fast-import treats a ref it has not seen as new and roots the first
        # commit, which loses every previous drive. Continue the existing tip
        # explicitly so drives append into one history and bisect works across
        # them.
        self._need_from = bool(self._rev(self.ref.decode()))
        if self._need_from:
            self._check_frame_tree()
        # Before models_at, not after: what the local ref says is the whole
        # input to the promotion decision, and on a fresh clone it says
        # nothing until this has run.
        self._seed_lineage()
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
        # Launched into its own process group, which is the only reason a
        # Ctrl-C can be survived at all. A terminal sends SIGINT to the entire
        # foreground process group, so in the shared group fast-import died on
        # the same keystroke that ended the drive -- and `done`, which the
        # drain thread writes and which is what makes the last
        # <= CHECKPOINT_EVERY frames durable, can only be written to a child
        # that outlived the ^C. Interrupting a drive used to cost up to 5 s of
        # frames for no reason but the signal's blast radius.
        #
        # Two spellings because the two platforms have no shared one:
        # start_new_session is setsid(), which does not exist on Windows,
        # where a new group is a CreateProcess flag instead. Both mean "not in
        # the terminal's group"; neither is a no-op we could skip.
        detached = ({"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP}
                    if os.name == "nt" else {"start_new_session": True})
        self._proc = subprocess.Popen(
            ["git", "fast-import", "--date-format=raw", "--quiet", "--done"],
            cwd=self.repo, stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=self._stderr, **detached,
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
        #
        # `deleteall` before the M lines, so the tree of a frame commit is
        # equal to Frame.tree() by construction instead of by inheritance.
        # Without it fast-import starts each commit from the parent's tree and
        # merely overlays the four files, so anything sitting below the tip --
        # a source checkout, a file some other tool wrote, a path a previous
        # version of this code emitted -- rides forward into every frame of
        # every drive from here on, invisibly, because no frame ever mentions
        # it. _check_frame_tree still refuses such a tip up front: this makes
        # the pollution unable to spread, not acceptable.
        out.append(b"deleteall\n")
        for path, content in f.tree().items():
            out.append(b"M 100644 inline " + path.encode() + b"\n")
            out.append(_data(content.encode()))
        out.append(b"\n")
        return b"".join(out)

    def _emit_lineage(self, f: Frame, models: str,
                      changed: list[tuple[str, str | None, str | None]]) -> bytes:
        """One commit carrying models.json and nothing else.

        `changed` is passed in rather than recomputed: the caller has to look
        at it before deciding there is a promotion to write at all.
        """
        msg = lineage.message(changed, f.t_wall_s, self.drive_tag, f.seq)
        # The fleet identity, not IDENT. lineage is the one ref designed to be
        # pushed and then kept forever, so it has to be publishable by
        # construction -- and construction is here, at the only place these
        # commits are ever written. Scrubbing it afterwards is not available:
        # rewriting an identity changes every sha on a ref whose whole value
        # is that its shas are stable references from blame. publish.audit()
        # checks for exactly this and would refuse the push, which is a late
        # and useless place to learn it. main keeps IDENT deliberately: those
        # frames carry a location feed and are never published.
        out = [b"commit " + self.lineage_ref + b"\n",
               b"committer " + PUBLIC_IDENT + b" %d " % f.t_wall_s
               + _tz_offset(f.t_wall_s) + b"\n",
               _data(msg.encode())]
        if self._lineage_from:
            # Same trap as main: a fresh fast-import roots the first commit on
            # a ref it has not seen, which would orphan every promotion the
            # vehicle has ever recorded.
            out.append(b"from " + self.lineage_ref + b"^0\n")
            self._lineage_from = False
        # "models.json and nothing else" is the docstring's claim, and this is
        # what makes it true of the tree rather than only of the M line below.
        # Inheriting the parent's tree would have let anything that ever
        # reached this ref -- backfill's output, a hand-made commit -- stay on
        # it forever, on the one ref that is published and then kept for good.
        out.append(b"deleteall\n")
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
                    changed = lineage.changes(self._models, models)
                    if not changed:
                        # The bytes differ but the checkpoints do not:
                        # different key order, indentation, a key the ref
                        # carries that the frame does not. That is not a
                        # promotion and there is nothing to say about it, but
                        # it must not be able to end the drive either -- an
                        # empty change set raises out of message(), and this
                        # thread dying at frame 0 committed the whole drive to
                        # nothing. Take the new text as the state in force and
                        # carry on.
                        self._models = models
                    else:
                        # Before the frame, not after: if the stream dies here
                        # the record shows a promotion with no frames under it,
                        # which is readable. The other order shows frames
                        # running a checkpoint nothing recorded shipping.
                        stdin.write(self._emit_lineage(f, models, changed))
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
        try:
            if self._thread:
                self._thread.join(timeout=30)
                if self._thread.is_alive():
                    # Kill the child before giving up on the thread. Raising
                    # with fast-import still running leaves an orphan holding a
                    # half-written pack open, and on Windows an open pack is an
                    # unlinkable one: the next `carctl maintain` fails its
                    # repack for a reason that has nothing to do with
                    # maintenance.
                    if self._proc:
                        self._proc.kill()
                    raise RuntimeError(
                        "committer thread did not finish in 30 s; "
                        f"{self.q.qsize()} frames still queued")
            if self._proc:
                try:
                    self._proc.wait(timeout=30)
                except subprocess.TimeoutExpired as exc:
                    # Same orphan, reached the other way: the thread wrote
                    # `done` and left, and fast-import is still chewing on it
                    # (or wedged). We are leaving either way, so leave nothing
                    # holding the pack.
                    self._proc.kill()
                    raise RuntimeError(
                        "fast-import did not exit within 30 s of `done` and "
                        f"was killed: {self._stderr_text()}") from exc
                if self._proc.returncode != 0 or self.error:
                    # Formatted separately. Concatenating the two branches glued
                    # the exit code onto the errno text, so a broken pipe on exit
                    # 2 read "fast-import failed ([Errno 32] Broken pipe2)".
                    why = (repr(self.error) if self.error
                           else f"exit {self._proc.returncode}")
                    raise RuntimeError(
                        f"fast-import failed ({why}): {self._stderr_text()}")
        finally:
            # The stderr file is read by _stderr_text() on the raising paths
            # above, so it closes here rather than before them -- but it closes
            # on every path, including those.
            if self._stderr:
                self._stderr.close()
