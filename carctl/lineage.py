"""The one ref that is never pruned.

Full frames on `main` have a retention window, because 10 Hz poses are both
enormous and a location feed for one named person. Checkpoint promotions do
not, because `git blame` on an incident has to reach back to the promotion of
the oldest checkpoint still in service, and the fleet's oldest is currently
about six months old. Prune below that and `git blame -L n,n models.json` runs
off the end of history and lands on the oldest surviving commit, which is a
confident wrong answer: worse than no answer, and exactly the failure blame
exists to prevent.

Eighteen months of full frames would be about 121 GB at the measured floor of
255.1 bytes per frame, and the floor is soft. So retention splits by file
rather than by time. models.json gets its own ref carrying that one file, one
commit per promotion. Four subsystems promoting maybe weekly is a rounding
error of disk, and it turns the retention window into a non-question for the
only thing that needed a long one.

The ref has to stand alone, because the main commit the promotion happened
during will be gone. Each commit here carries what changed, when, and the
drive it happened during. It carries no pose: this is the one ref kept
forever, and main is pruned partly because a permanent record of where the car
was is the thing we are trying not to have.
"""
from __future__ import annotations
import datetime, json, subprocess

REF = "refs/heads/lineage"


def _git(repo: str, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(["git", *args], cwd=repo, capture_output=True,
                          text=True, encoding="utf-8")


def models_at(repo: str, rev: str) -> str | None:
    """The models.json in force at `rev`, or None if there is none there."""
    r = _git(repo, "show", f"{rev}:models.json")
    return r.stdout if r.returncode == 0 else None


def changes(old: str | None,
            new: str) -> list[tuple[str, str | None, str | None]]:
    """(subsystem, checkpoint before, checkpoint after) for what moved.

    `old` is None for the first entry on the ref, where every subsystem is
    being recorded rather than promoted.

    The union of both sides, not just the new one: a subsystem that is dropped
    from models.json is as much a change to what is in force as one that moves,
    and iterating only `after` reported the retirement as nothing happening.
    An `after` of None is that retirement.
    """
    try:
        before = json.loads(old) if old else {}
    except ValueError:
        # A models.json we cannot read is not evidence we can diff against, so
        # record the whole set rather than silently reporting no promotions.
        before = {}
    after = json.loads(new)
    return [(k, before.get(k), after.get(k))
            for k in sorted(set(before) | set(after))
            if before.get(k) != after.get(k)]


def message(changed: list[tuple[str, str | None, str | None]], when_unix: int,
            drive_tag: str, seq: int) -> str:
    """The commit body. Everything blame reports later comes from here."""
    if not changed:
        raise ValueError("a lineage commit with no promotion in it")
    if len(changed) == 1:
        subsystem, before, after = changed[0]
        if after is None:
            subject = f"retire {subsystem}"
        elif before is None:
            subject = f"record {subsystem} at {after}"
        else:
            subject = f"promote {subsystem} to {after}"
    else:
        names = ", ".join(c[0] for c in changed)
        # Three verbs, because a set that is entirely appearing and a set that
        # is entirely going away are different events, and "promote" describes
        # neither. Mixed sets keep the general word.
        if all(c[1] is None for c in changed):
            verb = "record"
        elif all(c[2] is None for c in changed):
            verb = "retire"
        else:
            verb = "promote"
        subject = f"{verb} {len(changed)} checkpoints: {names}"
    when = datetime.datetime.fromtimestamp(when_unix).astimezone()
    lines = [subject, ""]
    for subsystem, before, after in changed:
        lines.append(f"Promoted: {subsystem} {before or '(none)'} -> "
                     f"{after if after is not None else '(retired)'}")
    # The commit timestamp says the same thing, but this ref outlives every
    # tool that wrote it and a trailer survives a rewrite of committer dates.
    lines.append(f"Promoted-At: {when.isoformat(timespec='seconds')}")
    # Named, not pointed at. The tag is deleted with the frames it bounds when
    # the retention window moves past it, so this is provenance rather than a
    # reference: "during drive-0011" stays readable after drive-0011 is gone.
    lines.append(f"Drive: {drive_tag or '(untagged)'}")
    lines.append(f"Frame: {seq}")
    return "\n".join(lines) + "\n"


def index(repo: str, ref: str = REF) -> list[tuple[str, int, str]]:
    """(commit, committer time, models.json blob) for every lineage entry.

    Newest first. One `git log` rather than a rev-parse per commit: this is
    read on every blame, and the ref is the one thing that grows without
    bound.
    """
    r = _git(repo, "log", "--format=%x01%H %ct", "--raw", "--no-abbrev",
             "--no-renames", ref, "--", "models.json")
    if r.returncode != 0:
        return []
    out, sha, ct = [], "", 0
    for line in r.stdout.splitlines():
        if line.startswith("\x01"):
            parts = line[1:].split()
            sha, ct = parts[0], int(parts[1])
        elif line.startswith(":"):
            # `:<oldmode> <newmode> <oldsha> <newsha> <status>\t<path>`
            fields = line.split()
            if len(fields) >= 4 and sha:
                out.append((sha, ct, fields[3]))
    return out


def entry_for(repo: str, models_blob: str, not_after: int,
              ref: str = REF) -> str:
    """The lineage commit that put `models_blob` in force, or "".

    Bounded by time because a fleet can roll a checkpoint back, which brings
    an earlier set of checkpoints round again under a second lineage commit.
    Taking the newest match unbounded would then explain an old incident with
    a promotion that had not happened yet.
    """
    for sha, ct, blob in index(repo, ref):
        if blob == models_blob and ct <= not_after:
            return sha
    return ""


def backfill(repo: str, ref: str = REF, source: str = "refs/heads/main",
             rebuild: bool = False) -> tuple[int, str]:
    """Seed the ref from the promotions already sitting on `source`.

    Retention refuses to prune frames whose promotions are recorded nowhere
    else, which is every frame written before this ref existed. Without a
    bootstrap that is a deadlock: main can never be pruned, because the
    evidence pruning would destroy has nowhere else to live. This moves it.

    Seeding stops at a ref that already exists, because the entries it would
    add are all older than the ones already there and a git history does not
    take insertions at the front. `rebuild` is the way out: the ref is derived
    from `source` in the first place, so rebuilding it from `source` loses
    nothing, as long as `source` is still whole. It is not whole once
    retention has pruned it, and that is the one case rebuilding refuses.
    """
    tip = _git(repo, "rev-parse", "--verify", "--quiet", ref).stdout.strip()
    if tip and not rebuild:
        return 0, (f"{ref} already exists with "
                   + _git(repo, "rev-list", "--count", ref).stdout.strip()
                   + " entries. A backfill would need to insert older "
                   "promotions in front of them; rebuild the ref from "
                   f"{source} instead")
    if tip:
        shallow = _git(repo, "rev-parse", "--is-shallow-repository")
        if shallow.stdout.strip() == "true":
            return 0, (f"{source} has already been pruned, so it is no longer "
                       f"a complete record of what {ref} holds; rebuilding "
                       "from it would drop the promotions that only survive "
                       "here")
        # The old ref is kept, not replaced in place. Same convention as
        # refs/backup/public-before-rebuild, and named after the tip so a
        # second rebuild cannot quietly overwrite the first one's copy.
        keep = f"refs/backup/lineage-before-{tip[:12]}"
        if _git(repo, "update-ref", keep, tip).returncode != 0:
            return 0, f"could not save the existing {ref} at {keep}"
    r = _git(repo, "log", "--reverse", "--format=%x01%H %ct", "--raw",
             "--no-abbrev", "--no-renames", source, "--", "models.json")
    if r.returncode != 0:
        return 0, f"cannot read {source}: {r.stderr.strip()}"

    from .gitstore import _data, _tz_offset          # same stream format
    # The public identity, not the car's own. This is the ref designed to
    # outlive the frames and leave the vehicle -- it is pushed beside public
    # and kept forever -- so it is written publishable from the start rather
    # than scrubbed on the way out; there is no fast-export filter on this
    # path to scrub it. main keeps the personal identity deliberately: that
    # one stays in the car. Imported here rather than at module scope because
    # publish imports this module, and a module-level import back would be a
    # cycle. It also makes `carctl lineage --rebuild` the migration for a ref
    # written before this was true: the entries are derived from main, so
    # rewriting them costs nothing but the identity line.
    from .publish import PUBLIC_IDENT

    # No `from` line anywhere in this stream: the first commit roots the ref
    # and fast-import parents the rest on what it is already tracking. A
    # rebuild moves the ref backwards to that new root, which is what --force
    # below is for.
    stream, prev, n = [], None, 0
    sha, ct = "", 0
    for line in r.stdout.splitlines():
        if line.startswith("\x01"):
            parts = line[1:].split()
            sha, ct = parts[0], int(parts[1])
            continue
        if not line.startswith(":") or not sha:
            continue
        blob = line.split()[3]
        content = _git(repo, "cat-file", "blob", blob).stdout
        changed = changes(prev, content)
        if not changed:
            continue
        prev = content
        msg = message(changed, ct, _drive_of(repo, sha), _seq_of(repo, sha))
        stream.append(b"commit " + ref.encode() + b"\n")
        stream.append(b"committer " + PUBLIC_IDENT + b" %d " % ct
                      + _tz_offset(ct) + b"\n")
        stream.append(_data(msg.encode()))
        stream.append(b"M 100644 inline models.json\n")
        stream.append(_data(content.encode()))
        stream.append(b"\n")
        n += 1
    if not n:
        return 0, f"no models.json history on {source} to seed from"
    stream.append(b"done\n")
    cmd = ["git", "fast-import", "--date-format=raw", "--quiet", "--done"]
    if tip:
        cmd.append("--force")
    imp = subprocess.run(cmd, cwd=repo, input=b"".join(stream),
                         capture_output=True)
    if imp.returncode != 0:
        raise RuntimeError("fast-import failed seeding the lineage ref: "
                           + imp.stderr.decode("utf-8", "replace")[:2000])
    verb = "rebuilt" if tip else "seeded"
    kept = (f", previous tip kept at refs/backup/lineage-before-{tip[:12]}"
            if tip else "")
    return n, f"{verb} {ref} with {n} promotion(s) from {source}{kept}"


def _drive_of(repo: str, sha: str) -> str:
    """The earliest drive tag that contains `sha`.

    A drive is tagged when it ends, so the first tag reachable from a frame is
    the drive that frame was part of. Sorted by number, not by tag order:
    for-each-ref sorts lexically and drive-0010 would come before drive-0009.
    """
    tags = _git(repo, "tag", "--contains", sha, "--list", "drive-*").stdout.split()
    numbered = sorted((int(t.rsplit("-", 1)[-1]), t) for t in tags
                      if t.rsplit("-", 1)[-1].isdigit())
    return numbered[0][1] if numbered else ""


def _seq_of(repo: str, sha: str) -> int:
    """The frame number from the commit's own Seq trailer."""
    body = _git(repo, "log", "-1", "--format=%B", sha).stdout
    for line in body.splitlines():
        if line.startswith("Seq: "):
            try:
                return int(line[5:])
            except ValueError:
                break
    return -1
