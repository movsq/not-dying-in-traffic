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
from . import lineage, retain, safety

# 20 ticks, and the test below is strict, so a kind has to be clear for 21
# consecutive ticks -- 2.1 s -- before the next occurrence counts as a new
# event. Stated in ticks because that is what the loop counts; the seconds are
# a consequence of DT and drift with it.
RELATCH_TICKS = 20

# One minute of ticks. The drive lock is a lease, and this is how often the
# loop renews it -- see the call in the tick body.
LOCK_REFRESH_TICKS = 600


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
    #
    # The tag is deleted by retention and never pushed, so the `Drive:`
    # trailer is the only place the counter survives a clone: reading local
    # tags alone starts again at drive-0001 while origin/lineage already says
    # "during drive-0006", and a reused number makes that phrase permanently
    # ambiguous on a ref that is never pruned. high_water_drive scans the
    # trailers on the local lineage ref and on every remote one, newest first
    # and trailers only, so the counter survives a clone and a renamed remote
    # without ever walking an unbounded pile of commit bodies.
    nums.append(lineage.high_water_drive(repo))
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

    # The frame one tick back, which is what lets detect() see a range
    # shrinking rather than just a range. None on the first tick, and that is
    # a real state rather than a placeholder: there is no closing speed yet.
    prev = None

    # One try around the whole lock/start/tick region. A signal is
    # asynchronous, so there is no instruction in here it cannot land on, and
    # guarding the tick loop alone left windows on either side of it: a Ctrl-C
    # while drive_lock resolved the git dir, or inside committer.start(), or
    # during the shutdown below, unwound straight out of drive() -- past the
    # counters, past the tagging -- and the operator got a traceback where the
    # report of a real, if short, drive belonged. It also left exactly the
    # dangling "Drive: drive-NNNN" that _tag_drive's warning is about.
    try:
        # Tells maintenance to stay off the disk for the length of this drive.
        # Advisory one way only: it never blocks the drive, and it expires on
        # its own so a power cut cannot leave the vehicle unable to repack.
        with retain.drive_lock(repo, seconds) as refresh:
            try:
                # Started here, not before the lock. Entering drive_lock
                # resolves the git dir, which raises if git is missing or this
                # is not a repository -- and started first, the fast-import
                # child was already running with nothing left to write `done`
                # to it, so it died "stream ends early" over a failure that
                # happened before the drive began. Inside the try, because a
                # signal landing between Popen and the first tick would
                # otherwise leave that child running with nobody left to close
                # its stdin.
                committer.start()
                # Not a crash path. Ctrl-C and SIGTERM (cli turns the latter
                # into this same exception) are how a drive is ended early on
                # purpose, and an interrupted drive is still a drive: its
                # frames are real, its promotions happened, and it needs its
                # tag as much as any other -- see the tagging call at the
                # bottom.
                for i in range(n_ticks):
                    # Tick i is due one full period after the epoch, not at it.
                    # `epoch + i * DT` made tick 0 due at the instant epoch was
                    # sampled, so slack was always microseconds negative and
                    # every drive reported a deadline overrun that never
                    # happened -- while a real first-tick overrun stayed
                    # invisible underneath it. It also ended the drive a tick
                    # early: --seconds 14 paced 13.9 s.
                    deadline = epoch + (i + 1) * DT
                    if realtime:
                        slack = deadline - time.monotonic()
                        if slack > 0:
                            time.sleep(slack)
                        else:
                            rep.overruns += 1
                        jitter = (time.monotonic() - deadline) * 1000
                        rep.max_jitter_ms = max(rep.max_jitter_ms, jitter)

                    # Ground truth is read BEFORE the step, because step() ends
                    # by advancing plant.t to the next tick: asked afterwards,
                    # the oracle answered for tick i+1 and detect() was handed
                    # (frame_i, truth_{i+1}) -- perception judged against a
                    # world 100 ms into its own future. It is invisible in the
                    # shipped scenario only because red_light_run also needs
                    # stop_line_crossed, which step() computes internally at
                    # the correct t, so the one tick where the mismatch could
                    # show is already gated by a flag that agrees. A light
                    # phase boundary landing one tick earlier, or any second
                    # detector reading the oracle, turns it into a wrong answer
                    # with no symptom.
                    truth = oracle() if oracle else ""

                    frame = plant.step()

                    # Safety runs before the commit. The record must never be
                    # the thing standing between a hazard and the brakes.
                    if oracle is None:
                        truth = frame.sensors.light_state
                    seen = safety.detect(frame, truth, prev)
                    prev = frame
                    kinds_now = {s.kind for s in seen}
                    # Release a latch once its condition has been clear for a
                    # while. The set was only ever added to, so a second
                    # genuinely separate incident of the same kind later in the
                    # same drive was discarded as "still unfolding" however long
                    # the gap between them.
                    for kind in [k for k, t in latched.items()
                                 if k not in kinds_now and i - t > RELATCH_TICKS]:
                        del latched[kind]
                    fresh = [s for s in seen if s.kind not in latched]
                    for s in seen:          # hold the latch while it persists
                        latched[s.kind] = i
                    rep.incidents.extend(fresh)
                    # on_frame still takes a single incident: the most severe
                    # one that is new this tick, or None.
                    inc = fresh[0] if fresh else None

                    t0 = time.perf_counter()
                    committer.submit(frame)
                    rep.max_submit_us = max(rep.max_submit_us,
                                            (time.perf_counter() - t0) * 1e6)
                    rep.ticks += 1
                    if on_frame:
                        on_frame(frame, inc)
                    # The lock is a lease, and this is the renewal. It is what
                    # lets a 5-hour drive stay protected past any fixed cap the
                    # lock puts on a single lease, while a crash still frees
                    # maintenance within minutes instead of within whatever
                    # length the dead drive happened to declare. Once a minute,
                    # so the cost is nothing next to the tick it rides on.
                    if i and i % LOCK_REFRESH_TICKS == 0:
                        refresh()
            finally:
                # stop() has to run even on Ctrl-C, or on a raising on_frame --
                # cli.py passes a closure. Skipping it left the daemon drain
                # thread to die at interpreter exit with `done` never written:
                # fast-import then exits "stream ends early" and every frame
                # since the last checkpoint, up to CHECKPOINT_EVERY and so
                # 4.9 s of driving, is gone from the record.
                try:
                    committer.stop()
                except KeyboardInterrupt:
                    # A second signal, arriving while the first one's shutdown
                    # is draining the queue and joining threads. That is the
                    # operator insisting, and insisting must not buy them
                    # another 60 s of joins -- so give up on the orderly path
                    # and kill the child instead of waiting on it. The frames
                    # still in the queue are the accepted price of insisting;
                    # what is not negotiable is leaving fast-import behind
                    # holding a half-written pack open.
                    rep.interrupted = True
                    committer.abandon()
                finally:
                    # Recorded even if stop() raises. A drive that committed
                    # 140 frames and then failed to shut down cleanly should
                    # still be able to tell you it committed 140 frames.
                    rep.dropped = committer.dropped
                    rep.committed = committer.committed
                    rep.promotions = committer.promotions
    except KeyboardInterrupt:
        # Recorded, not re-raised. The caller decides what an interrupted drive
        # should exit with; this function's job is to finish telling the truth
        # about it, which is everything the counters above already say.
        rep.interrupted = True
    # A tag has to bound something. A drive that committed nothing would tag
    # the previous drive's tip, and five such runs left bisect with five names
    # for one commit -- endpoints that cannot be ordered and a range that is
    # empty whichever two you pick. An interrupted drive still tags, because
    # it did commit frames and they are as real as any others.
    #
    # One site, reached from every path above -- including the one where the
    # operator hit Ctrl-C twice -- because the tag is what stops the
    # promotions this drive already wrote from naming a name that nothing
    # carries. It is guarded in turn: a signal arriving during `git tag` is
    # the last window left, and there is nothing to unwind by then anyway.
    if rep.committed:
        try:
            rep.tag = _tag_drive(repo, tag, rep.promotions)
        except KeyboardInterrupt:
            rep.interrupted = True
    return rep
