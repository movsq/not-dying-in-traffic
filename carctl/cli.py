from __future__ import annotations
import argparse, json, os, re, subprocess, sys
from . import (blame as blamemod, lineage as lineagemod, loop as loopmod,
               publish as pubmod, retain as retainmod, safety,
               stash as stashmod)
from .plant import Plant
from .stash import Preconditions

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _git(*a):
    return subprocess.run(["git", *a], cwd=REPO, capture_output=True, text=True, encoding="utf-8").stdout


def _commit_for_seq(seq: int) -> tuple[str, str]:
    """Resolve a frame's seq to its commit in the most recent drive.

    Returns (sha, error). Indexing `git log --max-count=<seconds/0.1>`
    positionally assumed every drive was exactly as long as the flag passed to
    THIS command. Real drive lengths on this repo run 280, 140, 60, 30, so the
    index landed inside a previous drive -- and that wrong sha was then
    blamed, and record-reverted, against a frame that had no incident. The
    Seq: trailer is already in every message; read it.
    """
    out = _git("log", "--format=%H%x1f%B%x1e", "refs/heads/main")
    for entry in out.split("\x1e"):
        sha, _, body = entry.strip().partition("\x1f")
        m = re.search(r"^Seq: (\d+)$", body, re.M)
        if not m:
            continue
        found = int(m.group(1))
        if found == seq:
            return sha.strip(), ""
        if found < seq:
            # Walking back from the tip, seq descends within a drive. Dropping
            # below the target without matching means this drive ended before
            # that frame, so it is not the drive we are looking at.
            break
    return "", (f"seq {seq} is not in the most recent drive on "
                f"refs/heads/main; run a drive at least {seq + 1} frames long")


def cmd_drive(args):
    seen = []
    def on_frame(f, inc):
        if inc:
            seen.append((f, inc))
    rep = loopmod.drive(REPO, args.seconds, realtime=not args.fast,
                        on_frame=on_frame)
    print(f"ticks           {rep.ticks}")
    print(f"committed       {rep.committed}")
    print(f"dropped frames  {rep.dropped}")
    print(f"promotions      {rep.promotions}   <- commits on {lineagemod.REF}")
    print(f"deadline overruns {rep.overruns}")
    print(f"max jitter      {rep.max_jitter_ms:.2f} ms")
    print(f"max submit cost {rep.max_submit_us:.1f} us   <- the loop's entire git bill")
    print(f"tagged as      {rep.tag or 'UNTAGGED'}")
    kinds = {}
    for i in rep.incidents:
        kinds.setdefault(i.kind, []).append(i)
    print(f"\nincidents: {len(rep.incidents)} across {len(kinds)} kind(s)")
    for k, v in kinds.items():
        print(f"  {k:15s} first at seq {v[0].seq:4d}  {v[0].detail}")
        print(f"  {'':15s} owner: {v[0].subsystem}")


def cmd_incident(args):
    """Full incident walk-through: find it, blame it, decide the revert."""
    # Re-run the drive deterministically to recover the frame + incident.
    plant = Plant()
    hit = None
    now = None
    for _ in range(int(args.seconds / 0.1)):
        f = plant.step()
        for inc in safety.detect(f, plant.true_light()):
            if inc.kind == args.kind and hit is None:
                hit = (f, inc)
        if hit and f.seq == hit[0].seq + 8:
            now = f
            break
    else:
        now = f
    if not hit:
        print(f"no {args.kind} in this drive"); return
    frame, inc = hit
    csha, err = _commit_for_seq(frame.seq)
    if err:
        print(err); return

    print(f"INCIDENT  {inc.kind}  seq {inc.seq}  commit {csha[:10]}")
    print(f"  {inc.detail}")
    print(f"  subject: {_git('log', '-1', '--format=%s', csha).strip()}")
    print()
    print("git blame ->")
    for k, v in blamemod.attribute(REPO, csha, inc.subsystem).items():
        print(f"  {k:28s} {v}")
    print()
    verdict = safety.physical_revert(REPO, csha, now)
    print(f"physical revert ({(now.t_mono_ns-frame.t_mono_ns)/1e9:.1f}s later) ->")
    print(f"  allowed: {verdict.allowed}")
    print(f"  reason:  {verdict.reason}")
    print(f"  return path: {verdict.cost_m:.1f} m")
    print()
    wt = os.path.join(REPO, ".control")
    rr = safety.record_revert(REPO, csha, wt,
                              f"Incident: {inc.kind} ({inc.detail})")
    status = "ok" if rr.ok else "FAILED"
    print(f"record revert -> {status}  {rr.sha[:10]}  {rr.detail}")
    if rr.ok and not rr.already:
        print("  (record plane only; the physical world was not walked back)")


def cmd_park(args):
    st = stashmod.ParkingStash(REPO)
    plant = Plant()
    for _ in range(125):
        f = plant.step()
    swept = st.sweep(f)
    if swept:
        print(f"swept {len(swept)} stash entr(ies) past TTL")
    pre = Preconditions(gap_length_m=6.1, lead_vehicle_x=f.pose.x + 7.0,
                        follow_vehicle_x=f.pose.x - 1.2, clearance_m=0.55)
    entry = st.push(f, pre, attempt=1)
    print(f"git stash push -> {entry.ref}  (seq {entry.seq}, "
          f"return pose {entry.return_pose['x']:.1f},{entry.return_pose['y']:.1f})")
    print(f"  betting on: gap {pre.gap_length_m} m, clearance {pre.clearance_m} m")
    print()
    for _ in range(20):
        f2 = plant.step()
    worse = Preconditions(gap_length_m=5.4, lead_vehicle_x=pre.lead_vehicle_x,
                          follow_vehicle_x=pre.follow_vehicle_x - 0.9,
                          clearance_m=0.22)
    ok, conflicts = st.pop(entry, f2, worse)
    print(f"git stash pop  -> {'applied' if ok else 'CONFLICT'}")
    for c in conflicts:
        print(f"  conflict: {c}")
    if not conflicts:
        print("  trajectory replayed onto current world state")
    else:
        print("  -> stash kept, replanning attempt 2 (the gap is not the gap "
              "it was)")
    print()
    print("stash list:")
    for e in st.list():
        print(f"  {e.ref}  attempt {e.attempt}  seq {e.seq}")
    if args.drop:
        st.drop(entry); print("dropped")


def cmd_lineage(args):
    """The ref that outlives the frames: one commit per checkpoint promotion."""
    if args.backfill or args.rebuild:
        n, msg = lineagemod.backfill(REPO, rebuild=args.rebuild)
        print(msg)
        if not n:
            return
    if _git("rev-parse", "--verify", "--quiet", lineagemod.REF).strip() == "":
        print(f"{lineagemod.REF} does not exist. Seed it from the promotions "
              f"already on main with:  carctl lineage --backfill")
        return
    n = _git("rev-list", "--count", lineagemod.REF).strip()
    print(f"{lineagemod.REF}  {n} promotion(s), kept indefinitely")
    print()
    # One Promoted trailer per line. Run together on one line, a commit that
    # moved four checkpoints at once reads as one very long checkpoint id.
    print(_git("log", f"-n{args.n}", "--format=%h  %ad  %s%n"
               "            %(trailers:key=Promoted,valueonly,"
               "separator=%x0A            )"
               "%n            during %(trailers:key=Drive,valueonly)",
               "--date=format:%Y-%m-%d %H:%M", lineagemod.REF), end="")


def cmd_maintain(args):
    """Retention and repack. Runs only while the vehicle is stopped."""
    for line in retainmod.maintain(REPO, args.days, args.dry_run):
        print(line)


def cmd_bisect(args):
    tags = _git("for-each-ref", "--format=%(refname:short)",
                "refs/tags/drive-*").split()
    if len(tags) < 2:
        print("need at least two tagged drives"); return
    good, bad = args.good or tags[0], args.bad or tags[-1]
    print(f"{len(tags)} tagged drives, {good} .. {bad}")
    print()
    print(blamemod.bisect_script(REPO, good, bad), end="")
    print()
    print("emitted, not run. bisecting a moving vehicle is not a thing.")


def cmd_publish(args):
    sha = pubmod.build_public_ref(REPO)
    n = _git("rev-list", "--count", "refs/heads/public").strip()
    print(f"refs/heads/public  {sha[:10]}  {n} commits")
    print("  poses snapped to 25 m, commit times rounded to the minute")
    before = _git("show", "refs/heads/main:state.json").splitlines()
    after = _git("show", "refs/heads/public:state.json").splitlines()
    print("")
    print("  main   " + " ".join(l.strip() for l in before if '"x"' in l or '"y"' in l))
    print("  public " + " ".join(l.strip() for l in after if '"x"' in l or '"y"' in l))
    if not args.push:
        print("")
        print("not pushed. to publish:  carctl publish --push")
        return
    ok, msg = pubmod.push(REPO)
    print(("pushed" if ok else "push failed") + ": " + msg)


def cmd_log(args):
    print(_git("log", f"-n{args.n}", "--format=%h  %ad  %s",
               "--date=format:%H:%M:%S"))


def main(argv=None):
    # Street names are UTF-8. A cp1252 console would otherwise raise
    # UnicodeEncodeError mid-report.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass
    p = argparse.ArgumentParser(prog="carctl")
    sub = p.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("drive"); d.add_argument("--seconds", type=float, default=14.0)
    d.add_argument("--fast", action="store_true",
                   help="skip real-time pacing (CI / replay)")
    d.set_defaults(func=cmd_drive)

    i = sub.add_parser("incident")
    i.add_argument("--kind", default="red_light_run")
    i.add_argument("--seconds", type=float, default=14.0)
    i.set_defaults(func=cmd_incident)

    k = sub.add_parser("park"); k.add_argument("--drop", action="store_true")
    k.set_defaults(func=cmd_park)

    b = sub.add_parser("bisect")
    b.add_argument("--good"); b.add_argument("--bad")
    b.set_defaults(func=cmd_bisect)

    ln = sub.add_parser("lineage")
    ln.add_argument("-n", type=int, default=20)
    ln.add_argument("--backfill", action="store_true",
                    help="seed an empty ref from the promotions already on main")
    ln.add_argument("--rebuild", action="store_true",
                    help="replace an existing ref from main, keeping the old "
                         "tip under refs/backup/")
    ln.set_defaults(func=cmd_lineage)

    m = sub.add_parser("maintain")
    m.add_argument("--days", type=float, default=retainmod.MAIN_WINDOW_DAYS,
                   help="how much full-frame history main keeps")
    m.add_argument("--dry-run", action="store_true",
                   help="say what would be dropped, change nothing")
    m.set_defaults(func=cmd_maintain)

    pb = sub.add_parser("publish")
    pb.add_argument("--push", action="store_true",
                    help="actually push refs/heads/public to origin")
    pb.set_defaults(func=cmd_publish)

    l = sub.add_parser("log"); l.add_argument("-n", type=int, default=20)
    l.set_defaults(func=cmd_log)

    a = p.parse_args(argv)
    a.func(a)


if __name__ == "__main__":
    main()
