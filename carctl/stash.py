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
from dataclasses import dataclass, asdict
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


@dataclass
class StashEntry:
    id: str
    ref: str
    seq: int
    t_mono_ns: int
    return_pose: dict
    preconditions: dict
    attempt: int


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
        head = self._git("rev-parse", "refs/heads/main").stdout.strip()
        self._git("update-ref", ref, head)
        entry = StashEntry(
            id=sid, ref=ref, seq=f.seq, t_mono_ns=f.t_mono_ns,
            return_pose={"x": f.pose.x, "y": f.pose.y,
                         "heading": f.pose.heading, "v": 0.0},
            preconditions=asdict(pre), attempt=attempt)
        blob = self._git_hash(json.dumps(asdict(entry), indent=2) + "\n")
        self._git("update-ref", f"refs/parking-meta/{sid}", blob)
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
            blob = ref.rsplit("/", 1)[-1]
            oid = self._git("rev-parse", ref).stdout.strip()
            entries.append(StashEntry(**json.loads(
                self._git("cat-file", "-p", oid).stdout)))
        return sorted(entries, key=lambda e: e.seq)

    def pop(self, entry: StashEntry, now: Frame, now_pre: Preconditions):
        """Three-way merge against the street. Returns (ok, conflicts)."""
        age = (now.t_mono_ns - entry.t_mono_ns) / 1e9
        conflicts = []
        if age > TTL_S:
            conflicts.append(f"stale: {age:.1f}s > TTL {TTL_S:.0f}s")
        conflicts += Preconditions(**entry.preconditions).conflicts_with(now_pre)
        if conflicts:
            return False, conflicts
        return True, []

    def drop(self, entry: StashEntry) -> None:
        self._git("update-ref", "-d", entry.ref)
        self._git("update-ref", "-d", f"refs/parking-meta/{entry.id}")
