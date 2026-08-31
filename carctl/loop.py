"""The 100 ms loop.

Absolute deadlines, never `sleep(0.1)`. Sleeping a fixed interval accumulates
every tick's overrun into permanent drift, and a drifting safety loop lies
about its own timestamps. We schedule against a fixed epoch and measure the
jitter, because the loop's real product is not the commits, it is the promise
that a frame exists for every 100 ms of the drive.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import re, subprocess, time
from .plant import Plant, DT
from .gitstore import Committer
from . import lineage, retain, safety

# 20 ticks, and the test below is strict, so a kind has to be clear for 21
# consecutive ticks -- 2.1 s -- before the next occurrence counts as a new
# event. Stated in ticks because that is what the loop counts; the seconds are
# a consequence of DT and drift with it.
RELATCH_TICKS = 20

# `Drive: drive-0006`, as lineage.message() writes it into every promotion.
_DRIVE_TRAILER = re.compile(r"^Drive: drive-(\d+)\s*$", re.M)


@dataclass
class DriveReport:
    ticks: int = 0
    overruns: int = 0
    max_jitter_ms: float = 0.0
    max_submit_us: float = 0.0
    incidents: list = field(default_factory=list)
    dropped: int = 0
    committed: int = 0
    promotions: int = 0
    tag: str = ""
    # Whether the pacing that produces overruns and jitter actually ran.
    # Under --fast nothing is scheduled against a deadline, so both counters
    # keep their initial zeros -- and a zero that was never measured reads
    # exactly like a clean result, which is the one thing a timing report must
    # not be able to say by accident.
    realtime: bool = True
    # The drive ended on a signal rather than on its tick count. Everything
    # below it in this report is still true, just of a shorter drive.
    interrupted: bool = False


def _lineage_high_water(repo: str) -> int:
    """The largest drive number named by a promotion on either lineage ref.

    The tag is deleted by retention and never pushed, so the `Drive:` trailer
    is the only place the counter survives a clone: a clone that reads only
    local tags starts again at drive-0001 while origin/lineage is already
    saying "during drive-0006". Reusing a number makes that phrase permanently
    ambiguous on a ref that is never pruned -- two different drives, one name,
    and no way left to tell which promotion belonged to which.

    Both refs, because the local one may not exist yet on a fresh clone and
    the remote one goes stale the moment this vehicle drives.
    """
    best = 0
    remote = "refs/remotes/origin/" + lineage.REF.rsplit("/", 1)[-1]
    for ref in (lineage.REF, remote):
        r = subprocess.run(["git", "log", "--format=%B", ref], cwd=repo,
                           capture_output=True, text=True, encoding="utf-8")
        if r.returncode != 0:
            continue        # ref does not resolve; nothing to learn from it
        for m in _DRIVE_TRAILER.finditer(r.stdout):
            best = max(best, int(m.group(1)))
    return best


def _next_drive_tag(repo: str) -> str:
    """The name this drive will be tagged with, decided before it starts.

    Named up front because the lineage commits written mid-drive have to say
    which drive a promotion happened during, and the tag itself cannot exist
    until there is a tip to point it at. Retention deletes drive tags along
    with the frames they bound, so what those commits carry is the name as
    provenance, not a reference: "during drive-0011" stays readable after
    drive-0011 is gone.
    """
    existing = subprocess.run(["git", "for-each-ref", "--format=%(refname)",
                               "refs/tags/drive-*"], cwd=repo,
                              capture_output=True, text=True,
                              encoding="utf-8").stdout.split()
    # High-water mark, not a count. Counting reuses a number as soon as any
    # tag is deleted, so drive-0002 could end up pointing at a tip thousands
    # of commits after drive-0003 and tag order would stop matching drive
    # order, which is exactly what cmd_bisect reads to pick its endpoints.
    # Retention deletes old tags, which makes that a routine event rather than
    # a hypothetical one.
    nums = [int(t.rsplit("-", 1)[-1]) for t in existing
            if t.rsplit("-", 1)[-1].isdigit()]
    # Tags say what this clone has driven; lineage trailers say what the
    # vehicle has driven. The high-water mark is over both.
    nums.append(_lineage_high_water(repo))
    return f"drive-{max(nums, default=0) + 1:04d}"


def _tag_drive(repo: str, name: str, promotions: int) -> str:
    """One lightweight tag per drive. Bisecting a fleet history means landing
    on drive boundaries, not on some arbitrary frame in the middle of one."""
    r = subprocess.run(["git", "tag", name, "refs/heads/main"], cwd=repo,
                       capture_output=True, text=True, encoding="utf-8")
    if r.returncode != 0:
        # Swallowing this left drives silently unbounded for bisect.
        print(f"warning: could not tag this drive as {name}: "
              f"{r.stderr.strip()}")
        if promotions:
            # The lineage commits are already written and already name this
            # drive. The hazard is not that the name is missing -- the usual
            # reason tagging fails is that the name is taken -- it is that
            # `Drive: <name>` on a ref kept forever now points at whatever
            # other drive's tip owns it, so those promotions read as having
            # happened during a drive they had nothing to do with.
            print(f"warning: {promotions} lineage commit(s) from this drive "
                  f"name {name}, which is not this drive's tip")
        return ""
    return name


def drive(repo: str, seconds: float, realtime: bool = True,
          on_frame=None) -> DriveReport:
    plant = Plant()
    tag = _next_drive_tag(repo)
    committer = Committer(repo, drive_tag=tag)
    rep = DriveReport(realtime=realtime)

    # kind -> tick it was last true. A set that was only added to could never
    # re-arm; this expires.
    latched: dict[str, int] = {}
    # Ground truth is asked for, not required. `plant.true_light()` was called
    # unconditionally, so the loop could not be pointed at anything but the
    # simulator, which is exactly the claim that replacing the plant is all a
    # real vehicle would need. A plant without it falls back to what
    # perception reported, which is all a real vehicle can know at the time.
    oracle = getattr(plant, "true_light", None)
    epoch = time.monotonic()
    # round, not int. `int(2.9 / 0.1)` is 28, because 2.9/0.1 is
    # 28.999999999999996 in binary -- 196 of the first 600 tenth-second
    # durations lost their last tick, silently, from a loop whose stated
    # product is that a frame exists for every 100 ms of the drive.
    n_ticks = round(seconds / DT)

    # Tells maintenance to stay off the disk for the length of this drive.
    # Advisory one way only: it never blocks the drive, and it expires on its
    # own so a power cut cannot leave the vehicle unable to repack.
    with retain.drive_lock(repo, seconds):
        # Started here, not before the lock. Entering drive_lock resolves the
        # git dir, which raises if git is missing or this is not a repository
        # -- and started first, the fast-import child was already running with
        # nothing left to write `done` to it, so it died "stream ends early"
        # over a failure that happened before the drive began.
        committer.start()
        try:
            # Not a crash path. Ctrl-C and SIGTERM (cli turns the latter into
            # this same exception) are how a drive is ended early on purpose,
            # and an interrupted drive is still a drive: its frames are real,
            # its promotions happened, and it needs its tag as much as any
            # other -- see the tagging call at the bottom.
            for i in range(n_ticks):
                # Tick i is due one full period after the epoch, not at it.
                # `epoch + i * DT` made tick 0 due at the instant epoch was
                # sampled, so slack was always microseconds negative and every
                # drive reported a deadline overrun that never happened -- while
                # a real first-tick overrun stayed invisible underneath it. It
                # also ended the drive a tick early: --seconds 14 paced 13.9 s.
                deadline = epoch + (i + 1) * DT
                if realtime:
                    slack = deadline - time.monotonic()
                    if slack > 0:
                        time.sleep(slack)
                    else:
                        rep.overruns += 1
                    jitter = (time.monotonic() - deadline) * 1000
                    rep.max_jitter_ms = max(rep.max_jitter_ms, jitter)

                # Ground truth is read BEFORE the step, because step() ends by
                # advancing plant.t to the next tick: asked afterwards, the
                # oracle answered for tick i+1 and detect() was handed
                # (frame_i, truth_{i+1}) -- perception judged against a world
                # 100 ms into its own future. It is invisible in the shipped
                # scenario only because red_light_run also needs
                # stop_line_crossed, which step() computes internally at the
                # correct t, so the one tick where the mismatch could show is
                # already gated by a flag that agrees. A light phase boundary
                # landing one tick earlier, or any second detector reading the
                # oracle, turns it into a wrong answer with no symptom.
                truth = oracle() if oracle else ""

                frame = plant.step()

                # Safety runs before the commit. The record must never be the
                # thing standing between a hazard and the brakes.
                if oracle is None:
                    truth = frame.sensors.light_state
                seen = safety.detect(frame, truth)
                kinds_now = {s.kind for s in seen}
                # Release a latch once its condition has been clear for a while.
                # The set was only ever added to, so a second genuinely separate
                # incident of the same kind later in the same drive was discarded
                # as "still unfolding" however long the gap between them.
                for kind in [k for k, t in latched.items()
                             if k not in kinds_now and i - t > RELATCH_TICKS]:
                    del latched[kind]
                fresh = [s for s in seen if s.kind not in latched]
                for s in seen:                  # refresh while it persists
                    latched[s.kind] = i
                rep.incidents.extend(fresh)
                # on_frame still takes a single incident: the most severe one that
                # is new this tick, or None.
                inc = fresh[0] if fresh else None

                t0 = time.perf_counter()
                committer.submit(frame)
                rep.max_submit_us = max(rep.max_submit_us,
                                        (time.perf_counter() - t0) * 1e6)
                rep.ticks += 1
                if on_frame:
                    on_frame(frame, inc)
        except KeyboardInterrupt:
            # Recorded, not re-raised. Unwinding out of drive() threw away a
            # report that was entirely true -- the frames, the drops, the
            # promotions -- and skipped the tagging below, leaving exactly the
            # dangling "Drive: drive-NNNN" that _tag_drive's warning is about.
            # The caller decides what an interrupted drive should exit with;
            # this function's job is to finish telling the truth about it.
            rep.interrupted = True
        finally:
            # stop() has to run even on Ctrl-C, or on a raising on_frame -- cli.py
            # passes a closure. Skipping it left the daemon drain thread to die at
            # interpreter exit with `done` never written: fast-import then exits
            # "stream ends early" and every frame since the last checkpoint, up to
            # CHECKPOINT_EVERY and so 4.9 s of driving, is gone from the record.
            try:
                committer.stop()
            finally:
                # Recorded even if stop() raises. A drive that committed 140
                # frames and then failed to shut down cleanly should still be able
                # to tell you it committed 140 frames.
                rep.dropped = committer.dropped
                rep.committed = committer.committed
                rep.promotions = committer.promotions
    # A tag has to bound something. A drive that committed nothing would tag
    # the previous drive's tip, and five such runs left bisect with five names
    # for one commit -- endpoints that cannot be ordered and a range that is
    # empty whichever two you pick. An interrupted drive still tags, because
    # it did commit frames and they are as real as any others.
    if rep.committed:
        rep.tag = _tag_drive(repo, tag, rep.promotions)
    return rep
