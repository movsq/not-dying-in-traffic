"""The 100 ms loop.

Absolute deadlines, never `sleep(0.1)` — sleeping a fixed interval accumulates
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
from . import safety


@dataclass
class DriveReport:
    ticks: int = 0
    overruns: int = 0
    max_jitter_ms: float = 0.0
    max_submit_us: float = 0.0
    incidents: list = field(default_factory=list)
    dropped: int = 0
    committed: int = 0


def _tag_drive(repo: str) -> None:
    """One lightweight tag per drive. Bisecting a fleet history means landing
    on drive boundaries, not on some arbitrary frame in the middle of one."""
    existing = subprocess.run(["git", "for-each-ref", "--format=%(refname)",
                               "refs/tags/drive-*"], cwd=repo,
                              capture_output=True, text=True).stdout.split()
    n = len(existing) + 1
    subprocess.run(["git", "tag", f"drive-{n:04d}", "refs/heads/main"],
                   cwd=repo, capture_output=True)


def drive(repo: str, seconds: float, realtime: bool = True,
          on_frame=None) -> DriveReport:
    plant = Plant()
    committer = Committer(repo)
    committer.start()
    rep = DriveReport()

    latched: set[str] = set()
    epoch = time.monotonic()
    n_ticks = int(seconds / DT)

    for i in range(n_ticks):
        deadline = epoch + i * DT
        if realtime:
            slack = deadline - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                rep.overruns += 1
            jitter = (time.monotonic() - deadline) * 1000
            rep.max_jitter_ms = max(rep.max_jitter_ms, jitter)

        frame = plant.step()

        # Safety runs before the commit. The record must never be the thing
        # standing between a hazard and the brakes.
        inc = safety.detect(frame, plant.true_light())
        if inc and inc.kind in latched:
            inc = None                      # same event, still unfolding
        elif inc:
            latched.add(inc.kind)
            rep.incidents.append(inc)

        t0 = time.perf_counter()
        committer.submit(frame)
        rep.max_submit_us = max(rep.max_submit_us,
                                (time.perf_counter() - t0) * 1e6)
        rep.ticks += 1
        if on_frame:
            on_frame(frame, inc)

    committer.stop()
    _tag_drive(repo)
    rep.dropped = committer.dropped
    rep.committed = committer.committed
    return rep
