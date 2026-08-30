"""The 100 ms loop.

Absolute deadlines, never `sleep(0.1)`. Sleeping a fixed interval accumulates
every tick's overrun into permanent drift, and a drifting safety loop lies
about its own timestamps. We schedule against a fixed epoch and measure the
jitter, because the loop's real product is not the commits, it is the promise
that a frame exists for every 100 ms of the drive.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import subprocess, time
from .plant import Plant, DT
from .gitstore import Committer
from . import retain, safety

RELATCH_TICKS = 20      # 2 s clear of a kind before it counts as a new event


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
            # The lineage commits are already written and already say this
            # name. Saying so is the difference between a stale label and a
            # stale label nobody knows about.
            print(f"warning: {promotions} lineage commit(s) from this drive "
                  f"name {name}, which now does not exist")
        return ""
    return name


def drive(repo: str, seconds: float, realtime: bool = True,
          on_frame=None) -> DriveReport:
    plant = Plant()
    tag = _next_drive_tag(repo)
    committer = Committer(repo, drive_tag=tag)
    rep = DriveReport()

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

                frame = plant.step()

                # Safety runs before the commit. The record must never be the
                # thing standing between a hazard and the brakes.
                truth = oracle() if oracle else frame.sensors.light_state
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
    rep.tag = _tag_drive(repo, tag, rep.promotions)
    return rep
