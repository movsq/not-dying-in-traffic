"""git stash, for parallel parking.

The manoeuvre: stash the current trajectory, try an approach, pop if it worked.

The analogy holds further than it looks. `git stash pop` replays a saved change
onto a working tree that has moved on, and can conflict. Physically the working
tree is the street, and it *always* moves on: the gap closes, a cyclist
arrives, the car behind creeps forward. So a pop is a three-way merge --
stashed intent, saved base state, current world -- and it is allowed to fail.

A stash that cannot detect that conflict is worse than no stash at all, because
it replays a plan built for a world that is gone. Hence `preconditions`: the
facts the stashed plan depended on, checked again at pop time.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict, fields
import json, subprocess, uuid
from .state import Frame

TTL_S = 45.0   # a parking attempt older than this is stale by construction


@dataclass
class Preconditions:
    gap_length_m: float
    lead_vehicle_x: float
    follow_vehicle_x: float
    clearance_m: float

    def conflicts_with(self, other: "Preconditions") -> list[str]:
        c = []
        if other.gap_length_m < self.gap_length_m - 0.4:
            c.append(f"gap shrank {self.gap_length_m:.2f}m -> {other.gap_length_m:.2f}m")
        if abs(other.follow_vehicle_x - self.follow_vehicle_x) > 0.5:
            c.append("vehicle behind moved")
        if other.clearance_m < 0.3:
            c.append(f"clearance {other.clearance_m:.2f}m below margin")
        return c


# One id per process. The monotonic clock restarts at zero on every run, so a
# delta between two processes' readings is a meaningless number that happens
# to look entirely plausible. This is what tells the two cases apart.
RUN_ID = uuid.uuid4().hex[:12]


@dataclass
class StashEntry:
    id: str
    ref: str
    seq: int
    t_mono_ns: int
    return_pose: dict
    preconditions: dict
    attempt: int
    t_wall_s: int = 0
    run_id: str = ""        # empty on entries written before this existed


class ParkingStash:
    def __init__(self, repo: str):
        self.repo = repo

    def _git(self, *args: str) -> subprocess.CompletedProcess:
        return subprocess.run(["git", *args], cwd=self.repo,
                              capture_output=True, text=True, encoding="utf-8")

    def push(self, f: Frame, pre: Preconditions, attempt: int = 1) -> StashEntry:
        """Save the pose we can return to, and the facts we are betting on."""
        sid = uuid.uuid4().hex[:8]
        ref = f"refs/parking/{sid}"
        head = self._git("rev-parse", "--verify", "--quiet",
                         "refs/heads/main").stdout.strip()
        if not head:
            # Writing the meta ref anyway produced entries whose parking ref
            # did not exist, which list() happily returned and drop() could
            # not remove.
            raise RuntimeError("no refs/heads/main to anchor a stash to; "
                               "run a drive first")
        r = self._git("update-ref", ref, head)
        if r.returncode != 0:
            raise RuntimeError(f"could not create {ref}: {r.stderr.strip()}")
        entry = StashEntry(
            id=sid, ref=ref, seq=f.seq, t_mono_ns=f.t_mono_ns,
            t_wall_s=f.t_wall_s, run_id=RUN_ID,
            return_pose={"x": f.pose.x, "y": f.pose.y,
                         "heading": f.pose.heading, "v": 0.0},
            preconditions=asdict(pre), attempt=attempt)
        blob = self._git_hash(json.dumps(asdict(entry), indent=2) + chr(10))
        meta = self._git("update-ref", f"refs/parking-meta/{sid}", blob)
        if meta.returncode != 0:
            self._git("update-ref", "-d", ref)   # do not leave a half entry
            raise RuntimeError(f"could not record stash meta: {meta.stderr.strip()}")
        return entry

    def _git_hash(self, text: str) -> str:
        p = subprocess.run(["git", "hash-object", "-w", "--stdin"],
                           cwd=self.repo, input=text, capture_output=True,
                           text=True, encoding="utf-8")
        return p.stdout.strip()

    def list(self) -> list[StashEntry]:
        out = self._git("for-each-ref", "--format=%(refname)",
                        "refs/parking-meta/").stdout.split()
        entries = []
        for ref in out:
            oid = self._git("rev-parse", ref).stdout.strip()
            raw = json.loads(self._git("cat-file", "-p", oid).stdout)
            known = {fld.name for fld in fields(StashEntry)}
            entries.append(StashEntry(**{k: v for k, v in raw.items()
                                         if k in known}))
        return sorted(entries, key=lambda e: e.seq)

    def age_s(self, entry: StashEntry, now: Frame) -> float:
        """Monotonic time is per-process and restarts at zero, so an entry from
        an earlier run always looked brand new and the TTL never fired. Wall
        clock decides across processes; the monotonic clock still decides
        within one drive, where it is the trustworthy one.

        "Same run" used to be inferred -- a positive monotonic delta under an
        hour -- which is precisely what a restart also produces. A 44 s old
        stash then reported 7.6 s and popped clean, replaying a parking plan
        onto a street that had moved on. It is an identity check now.
        """
        if entry.run_id and entry.run_id == RUN_ID:
            return (now.t_mono_ns - entry.t_mono_ns) / 1e9
        # Across processes only the wall clock means anything, and t_wall_s
        # carries whole seconds, so the true age is somewhere in
        # [wall - 1, wall + 1]. A TTL wants the upper bound: truncating let a
        # 45.9 s entry report 45.0 and slip under a 45 s expiry.
        return float(now.t_wall_s - entry.t_wall_s) + 1.0

    def pop(self, entry: StashEntry, now: Frame, now_pre: Preconditions):
        """Three-way merge against the street. Returns (ok, conflicts)."""
        age = self.age_s(entry, now)
        conflicts = []
        if age > TTL_S:
            conflicts.append(f"stale: {age:.1f}s > TTL {TTL_S:.0f}s")
        conflicts += Preconditions(**entry.preconditions).conflicts_with(now_pre)
        if conflicts:
            return False, conflicts
        return True, []

    def sweep(self, now: Frame) -> list[str]:
        """Drop entries past their TTL. Nothing did this before, and the normal
        outcome of a parking attempt is CONFLICT with the stash kept, so
        refs/parking/* grew by one on every single run."""
        dropped = []
        for e in self.list():
            if self.age_s(e, now) > TTL_S:
                self.drop(e)
                dropped.append(e.ref)
        return dropped

    def drop(self, entry: StashEntry) -> None:
        self._git("update-ref", "-d", entry.ref)
        self._git("update-ref", "-d", f"refs/parking-meta/{entry.id}")
