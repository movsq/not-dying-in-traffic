# not-dying-in-traffic

A car whose state history is a git history. It commits every 100 ms, reverts
when it does something wrong, blames a model checkpoint for the incident, and
stashes a trajectory to try parallel parking twice.

Most of it works. One part deliberately does not, and that part turned out to
be the interesting one.

## The split that everything else hangs off

There are two planes and they are allowed to disagree.

```
        control plane                          record plane
   +-----------------------+              +----------------------+
   |  100 ms loop          |  enqueue     |  committer thread    |
   |  plant -> safety ->   |------------->|  git fast-import     |
   |  actuators            |   25 us      |  (one long-lived     |
   |                       |              |   process, pipe fed) |
   |  never calls git      |              |                      |
   +-----------------------+              +----------+-----------+
                                                     | checkpoint
                                                     | every 50 frames
                                          +----------v-----------+
                                          |  refs/heads/main     |
                                          |  blame, bisect,      |
                                          |  revert, stash       |
                                          +----------+-----------+
                                                     | scrub + rewrite
                                          +----------v-----------+
                                          |  refs/heads/public   |--> GitHub
                                          +----------------------+
```

`git commit` forks a process, execs, builds an index and fsyncs. Tens of
milliseconds, with a tail you cannot bound. Nothing like that belongs inside a
100 ms safety deadline, so the loop never calls git at all. It puts a frame on
a bounded queue and gets on with driving. That enqueue costs 25 us in the worst
case I measured over a 140 frame drive.

A committer thread drains the queue into one long-lived `git fast-import`
process, which appends commits over a pipe and never touches the index or a
working tree. I expected commit throughput to be the hard part. It is not.
fast-import handles 10 Hz without noticing.

It does have one sharp edge. A fresh fast-import process treats a ref it has
not seen as brand new and roots the first commit, silently orphaning every
earlier drive. You have to emit `from refs/heads/main^0` on the first commit of
a session. I only found that by driving twice on the same repo.

When the queue fills it drops the oldest frame instead of blocking. A stalled
disk should cost you history, never a control cycle. Every drop gets counted,
because a drop means something is wrong.

It took two goes to actually do that. The overflow path was a get followed by a
put, and if the committer drained the queue in between, the get raised `Empty`
and the handler threw away the frame it had just been handed, into a queue that
was now empty. So it dropped the newest frame, which is the one closest to
whatever caused the stall, and counted it the same way.

## What each frame commits

Four files. The shape of `models.json` matters more than it looks.

`state.json` holds pose, speed, street, manoeuvre and a `reversible` flag.
`sensors.json` holds lidar, signal state, lateral offset and IMU.
`actuators.json` holds what we actually commanded, which is often not what we
meant. `models.json` holds one checkpoint per line, and that one formatting
choice is what makes blame useful later.

`lateral_offset` is the signed distance from the reference path the car is
supposed to be on, and `lane_ref` names which path that was. It used to be a
decorative sine of amplitude 0.12 m against an off-road threshold of 1.75 m,
which meant `off_road` was one of four incident kinds that nothing in the
scenario could trigger, and the planner was the one subsystem `git blame`
never got asked about. Making it real also meant the plant had to close the
loop on it: open loop steering cannot hold a lane, and the old script wandered
3.4 m of `x` across a street that is meant to be straight.

On a street the reference is the centre of the nearest lane the car is allowed
to be in. In a junction there is no such thing, and for a while the plant
reported 0.0 through the whole left turn, which does not read as "no
measurement", it reads as a lane held perfectly. A junction gets an arc
tangent to the centre of the lane the car enters on and the centre of the lane
it leaves on, with those two lane centres as the arc's own extensions past its
tangent points, which is what keeps the number continuous across the
manoeuvre. The plant steers to that arc rather than running the turn open
loop, because an open loop turn has no reference and any offset reported
against one would have been picked to fit whatever the car did. Tracking error
through the turn peaks at 0.38 m against the 1.75 m threshold.

```
seq 24  lat +0.000  ref Vinohradská:0            approaching
seq 32  lat -0.375  ref Vinohradská:0>Hlavní:0   mid turn, worst tracking error
seq 50  lat +0.094  ref Vinohradská:0>Hlavní:0   settling onto the exit lane
seq 60  lat +0.425  ref Hlavní:0                 back on a straight centreline
```

Parking is now the only manoeuvre with no reference path, because it leaves
the lane deliberately. `lane_ref` is empty on those frames and `detect` does
not evaluate `off_road`, rather than reading the placeholder 0.0 as a lane
held perfectly. The gap is recorded per frame instead of inferred, so the
history shows where `off_road` was not evaluated instead of implying it was
evaluated and passed.

Commit messages come from the manoeuvre classifier, in Conventional Commits
format. The `!` marker keeps its real meaning. It means the frame is
irreversible, so the history stops being invertible from there on.

```
feat: turned left onto Hlavní
feat: continued along Hlavní at 43 km/h
feat!: continued along Hlavní at 43 km/h     <- stop line crossed
chore!: parallel parking on Hlavní           <- curb strike
```

Reversibility gets decided when the frame is captured, by code that can see the
sensors. Not later, by code that is guessing.

There is one threshold for it, in `safety.py`, and `plant.py` and `msgen.py`
both read it. They used to keep their own copies, and the copies disagreed: the
plant compared the curb impulse against zero while the message generator
compared the accelerometer against 20, with gravity's 9.81 sitting in between
them. A small strike marked the frame irreversible and then could not name a
reason, and the commit went out saying `Irreversible-Reason: unknown`.

## Revert, and the half of it that cannot exist

`git revert` on a source tree always works. Any diff inverts. Physics does not
work that way, so revert splits in two.

**Record revert.** A real `git revert`, run in a throwaway detached worktree so
it can never race fast-import writing to `main`. What it produces is an
auditable statement that a frame was wrong, anchored under
`refs/reverts/<original>` so it does not sit unreachable on a detached HEAD.
Every step is checked. An earlier version swallowed three exit codes in a row
and printed a stale sha that looked like a fresh revert. The last one left
unchecked was the `update-ref` that does the anchoring, which turned out to be
the one that mattered. That ref is the only thing referencing the new commit,
and the worktree removal in the `finally` ran either way, so a failed anchor
returned ok, printed a sha, and left the commit unreachable and waiting to be
collected. If the anchor fails now the worktree stays, because at that moment
it is the only reference to the thing you just made.

**Physical revert.** Check the `reversible` flag on the frame you are
reverting. If it survives that, read the parent commit's `state.json`, treat
that pose as a goal, and check whether the goal is still inside the reachable
set from where the car is now. Drive there only if it is.

That first check used to read the parent's flag instead of the frame's, which
sounds like the same thing and is not. The one way door is recorded on the
frame where it happened, so reading the parent asked whether the world was
reversible one tick before the curb strike. It always was. `reversible: false`
never once blocked the revert of the frame that set it, and the `!` in the
subject line was decorative for this purpose. Here is what the red light
incident actually prints:

```
physical revert (0.8s later) ->
  allowed: False
  reason:  frame 110 is itself marked irreversible; the world cannot be walked back through it
  return path: 0.0 m
```

So the record got reverted and the world did not. Both of those are true at the
same time and the system reports both. Skip the reachability check and "revert"
becomes an unplanned manoeuvre wearing a reassuring name, which is worse than
having no revert.

When a frame does get as far as the reachability check, that check compares the
driven path against the lidar rather than the straight line. A car cannot
translate sideways, so returning to a pose off to one side means driving a
curve: across the scripted left turn the chord is 16.12 m and the arc is
18.42 m, and the old test was asking for 2.3 m less room than the manoeuvre
needs. It reads `lidar_min_bearing` now, which was written every frame and read
by nothing, so a return 0.4 rad off the path no longer refuses a clear one. And
a lidar sitting at its 40 m range limit is no longer reported as "occupied at
40.0 m". Not seeing anything as far as you can see is not the same as seeing
that it is clear, and the two deserve different answers.

## Blame

`models.json` puts one subsystem per line so that `git blame -L n,n` lands on
the commit that last changed that specific checkpoint. For a car that swaps
models over the air mid-drive, that commit is the moment the bad model shipped.

```
INCIDENT  red_light_run  seq 110  commit e7513a8aee
  crossed stop line at 43 km/h while perception reported 'green'

git blame ->
  subsystem                    perception
  models_json_line             3
  promoted_on_ref              refs/heads/lineage
  promoted_in                  16e16714803a594eefaba95d87cea484f0ff1993
  promoted_by_commit_subject   promote perception to ckpt-perception-2026.07.14-a91f
  checkpoint                   ckpt-perception-2026.07.14-a91f
  promoted_during_drive        drive-0010
  provenance                   shadow_km 9100, gate is 250000, promoted anyway
```

Fifty commits and five seconds before the car ran the light, an OTA agent
promoted a perception checkpoint with 3.6% of the shadow mileage its gate
required. Blame found it starting from nothing but the incident commit. That
is the one place where the git metaphor stops being a joke and earns its keep.

That blame runs on `refs/heads/lineage`, not on `main`. `main` has a retention
window and the promotion being looked for is routinely older than it, so a
`main` sha here would be an answer with an expiry date on it. See below.

The drive stages one fault per subsystem now, so all four owners in `OWNER`
get exercised rather than two. The planner asks for lane 2.6 of a two lane
road and holds it long enough to leave the roadway, which is `off_road`.
Something is already in the space it steered into, so the excursion produces a
near miss inside `MIN_CLEARANCE`, which is `collision` and belongs to
prediction. Then the perception checkpoint runs the light, and the controller
clips the curb while parking. They overlap on purpose:

```
seq 108  ['collision', 'off_road']
seq 109  ['collision', 'off_road']
seq 110  ['collision', 'red_light_run', 'off_road']
```

`detect` used to return on its first match, so seq 110 would have been a
`collision` and nothing else. The red light run would have been erased rather
than deprioritised, because `stop_line_crossed` is true for exactly one tick
and there is no later frame to catch it on, and blame would have been handed
prediction instead of perception. Being wrong about which subsystem to blame
is worse than reporting one incident fewer.

Drives append to one history rather than each starting fresh, and every drive
gets a `drive-NNNN` tag when it ends. That gives `git bisect` sensible places
to land, because you want to find the first bad drive, not some arbitrary frame
in the middle of one. `carctl bisect` writes the script out and stops there:

```
9 tagged drives, drive-0001 .. drive-0010

git -C /home/you/not-dying-in-traffic bisect start drive-0010 drive-0001
git -C /home/you/not-dying-in-traffic bisect run carctl replay --assert-no-incident

emitted, not run. bisecting a moving vehicle is not a thing.
```

The path and both revisions go through `shlex.quote` on the way out. They did
not, which is a silly thing to get wrong in a script whose entire purpose is to
be pasted into a shell: on Windows the shell ate the backslashes and `git -C`
received `C:Usersfixednot-dying-in-traffic`, a path with a space in it broke
the same way, and a revision containing a semicolon became a second command.

## Stash, for parallel parking

`git stash pop` replays saved work onto a tree that moved on, and it can
conflict. The street is the working tree here, and the street always moves on.
The gap closes. A cyclist arrives. The car behind creeps forward.

So a stash entry carries the facts its plan was betting on, plus a 45 s TTL.
Popping is a three way merge between stashed intent, saved base and current
world, and it is allowed to fail:

```
git stash push -> refs/parking/421184df  (seq 124, return pose 40.2,84.1)
  betting on: gap 6.1 m, clearance 0.55 m

git stash pop  -> CONFLICT
  conflict: gap shrank 6.10m -> 5.40m
  conflict: vehicle behind moved
  conflict: clearance 0.22m below margin
  -> stash kept, replanning attempt 2
```

A stash that cannot notice that conflict is worse than no stash, because it
confidently replays a plan built for a world that is gone.

The TTL runs on wall clock, not the simulated one. Simulated time restarts at
zero in each process, so an entry saved an hour ago used to compute an age of
0.0 s and pass the freshness check. `carctl park` also sweeps expired entries
before it starts, because the normal outcome of a parking attempt is a conflict
with the stash kept, and nothing was ever retiring them.

The fix for that was itself wrong for a while, in a way I liked. "Same process"
was inferred from the monotonic delta being positive and under an hour, which
is exactly what a restart also produces, since the monotonic clock comes back
up at zero. A 44 s old stash reported 7.6 s and popped clean. Entries carry the
id of the process that wrote them now, so it is an identity check rather than a
plausible looking guess.

The conflict check also reads all four facts it stores. It skipped
`lead_vehicle_x` entirely, so the car in front could roll back into the gap and
conflict with nothing, and the demo in `cli.py` copies that field forward
unchanged, which is why nothing ever noticed.

## The red button

`git revert --hard` is not a git command. The button is not one either. It
freezes the record at the last reversible frame and runs a minimal risk
manoeuvre. It is guarded by a same-origin check and a token minted at startup
that only ever reaches the served page. Binding to localhost is not access
control, and a plain form POST from any other page in the same browser is a
CORS simple request that nothing preflights. Halting a vehicle should take more
than an open tab.

For a while that guard was on the wrong requests. It lived inside the function
the POST handler called, and nothing else called it, so every GET went
unchecked. `GET /` handed the real token to any `Host` that asked for it, and
`/events` streamed the live 10 Hz feed to the same. The line I had commented as
blocking DNS rebinding was not on the requests that get rebound, which is the
kind of thing you only notice by asking what actually calls it. Reads are
checked now, and the route match ignores the query string and case, because
`/?v=2` used to miss the token substitution and fall through to the static file
handler, which served the page with the placeholder still in it: a dashboard
that streams, looks completely healthy, and refuses every halt you press. The
static handler is gone as well, since it also served `server.py` verbatim to
anyone who asked.

The page checks whether the halt was accepted. It did not, and a refusal is
valid JSON, so a 403 rendered as a completed halt: HALTED banner, red button
disabled, speed still updating underneath it. On the one control whose entire
job is stopping a car, a refusal and a success must not look the same.

One thing that bit me while testing the guard: `allow_reuse_address` means
something different on Windows. A second process can bind a port another is
already serving, so relaunching the dashboard left the old build answering
requests with the old build's guards, and my test passed against a server that
did not have the fix in it. The flag is now off on Windows, and a second launch
fails with WinError 10048 instead of quietly shadowing the first.

That fix had a second half I missed. `ThreadingTCPServer` does not use daemon
threads and joins all of them on close, so a single dashboard tab parked on the
event stream wedged shutdown while still holding the port, and with reuse off
the relaunch then failed too. Fixing the shadowing had turned it into a hang.
It is a `ThreadingHTTPServer` now and the stream has a heartbeat, so a tab that
went away surfaces as a dead socket instead of a thread waiting forever.

The confirmation dialog says what you asked for, and under it, the two numbers
that disagree.

```
are you sure? this cannot be undone (physically)

Record plane: rolls back 67 frame(s) to seq 109.
Physical plane: rolls back 0 m. The vehicle will run a
minimal-risk manoeuvre and stop where it is.
```

That second number is always 0 m. I think showing it is the most useful thing
on the whole dashboard.

## Publishing

The remote is set:

```
origin  https://github.com/movsq/not-dying-in-traffic.git
```

`main` never leaves the car. Exact poses at 10 Hz are a location feed for one
named person, and git history outlives anyone deleting it later. `public` is
what gets pushed, rebuilt from `main` by streaming `git fast-export` through a
filter and back into `git fast-import`.

Every channel that carries location gets rewritten, because a commit carries it
in more places than its tree. The blob, where poses snap to a 25 m grid. The
message, where the `Pose:` trailer snaps to the same grid. The identity, which
becomes an anonymous address with the timestamp rounded to the minute. I
originally scrubbed only the blob, which accomplished nothing. The exact
trajectory was sitting in the body of every commit right next to the coarsened
one, and the committer email was on all 560 of them.

Then it turned out I had not really scrubbed the blob either. The filter picked
its scrubber by sniffing the shape of the JSON, and `sensors.json` has no
`pose` key, so it was handed straight back. It reached `public` as literally
the same git object as on `main`, carrying `light_distance`, which is the
signed distance to a fixed landmark on a named street, at full double
precision. A 25 m grid on `state.json` is worth nothing sitting next to that.
The shape sniff was there because `fast-export` emits blobs with marks and only
names the path later, in the commit, so the path is not known at the moment the
payload goes past. The filter resolves marks to paths in a pre-pass now and
dispatches by filename, and a filename with no scrubber stops the build instead
of being published on the grounds that nobody thought about it.

```
main    state.json    "x": 41.846469430637235, "y": 89.7491853669169
        sensors.json  "light_distance": -20.134457378403937
        message       Pose: 41.846469,89.749185 @ 0.9498 rad
        committer     a real personal address

public  state.json    "x": 50.0, "y": 100.0
        sensors.json  "light_distance": -25.0
        message       Pose: 50.00,100.00 @ 0.9 rad
        committer     fleet@not-dying-in-traffic.invalid
```

The trailer carries six decimals on `main` now rather than two. The message and
the blob are scrubbed independently, so a 2 dp trailer could snap into a
different 25 m cell than the full precision pose did, and publishing two
different cells for one frame narrows the true coordinate far more than either
cell alone gives away.

`lane_ref` is the newest field to go through that lookup and it comes out
unchanged, deliberately: it names a street and a lane, and `state.json`
already publishes the street as `road`. The point of the table is that the
answer is written down, not that every answer is "scrub it".

`publish.audit()` re-reads the built ref and refuses the push if an unsnapped
pose or a non-public identity survived. A push is the one step you cannot take
back, so it gets a gate that does not depend on me remembering.

It could not have caught the `sensors.json` leak. It ran two `git log` commands
and never opened a tree, so of the three things it was supposed to be checking,
the blob was the one it could not see. It reads every published blob now and
checks that it is a fixed point of its own scrubber, which catches a scrubber
that did not run without needing to know why it did not run.

It also used to fail open. Every git call in it discarded its exit code, so a
ref that could not be read gave empty output, which filtered down to zero
problems, which reads exactly like a clean ref. Asking it about a ref that does
not exist got you "nothing wrong". It raises now. A gate that answers the same
way when it passes and when it never ran is worse than no gate, because only
one of those two readings is true and you cannot tell which one you got.

The push refspec is pinned in `.git/config` to
`refs/heads/public:refs/heads/main`, and `push.default` is `nothing`, so a bare
`git push` cannot reach `main` by accident.

Nothing has been pushed yet. The repo is private, which lowers the stakes but
does not change the design. Publishing stays a thing you type on purpose:

```bash
python -m carctl publish --push
```

## Retention, and the ref that outlives the frames

At 10 Hz the car commits 864,000 times a day. Across the 1210 commits on
`main` a frame costs 238.8 bytes packed, so a drive-day is about 197 MB and a
year about 70 GB. Treat that as a floor: these are short scripted drives whose
poses delta extremely well, and it is measured after a repack.

The number that decides the window is not a disk number. Blame has to reach
back to the promotion of the oldest checkpoint still in service, which today
is `ckpt-controller-2026.03.01-0b12`, about six months old. Prune below that
and `git blame -L n,n models.json` walks off the end of what survives and
lands on the oldest remaining commit: a confident wrong answer, which is worse
than no answer and is precisely the failure blame exists to prevent.
Eighteen months of full frames to satisfy that would be about 105 GB.

So retention splits by file rather than by time.

```
refs/heads/main      full frames, 14 days       ~2.8 GB at the floor,
                                                budget 10 GB for real ratios
refs/heads/lineage   models.json, forever       one commit per promotion
```

Four subsystems promoting maybe weekly is a rounding error of disk, and it
turns the window into a non-question for the only thing that needed a long
one. The lineage ref has to stand alone, because the frame the promotion
happened on will be gone:

```
16e1671  2026-08-30 07:17  promote perception to ckpt-perception-2026.07.14-a91f
            perception ckpt-perception-2026.05.30-1e4d -> ckpt-perception-2026.07.14-a91f
            during drive-0010
```

The drive is a name in the body, not a ref. Retention deletes the tag along
with the frames it bounds, so "during drive-0011" has to stay readable after
`drive-0011` does not exist. That is what forced the tag name to be decided at
the start of a drive rather than at the end: the lineage commits are written
mid-drive, and a tag cannot point at a tip that does not exist yet. It carries
no pose either. This is the one ref kept forever, and `main` is pruned partly
because a permanent record of where the car was is the thing we are trying not
to have.

`blame.attribute()` finds the entry by matching the frame's `models.json`
blob, so the two refs are literally the same git object and line numbers agree
without anything assuming they do. The match is bounded by the incident's own
commit time, because a rolled-back checkpoint brings an earlier set of
checkpoints round again under a second lineage commit, and the newest match
would otherwise explain an old incident with a promotion that had not happened
yet. Where there is no entry, which is every frame written before the ref
existed, it answers from `main` and labels the answer provisional.

Pruning uses a shallow boundary, not a rewrite and not a graft. A rewrite
changes every surviving commit's sha, and a sha is a frame's identity here:
`refs/reverts/<sha>` is what anchors an incident to the frame it happened on,
so a daily rewrite would invalidate yesterday's incident report. A graft
reclaims nothing, because git disables replace refs while packing on purpose,
and commit objects are 70% of the pack.

```
carctl maintain --days 0

stationary check: stopped, last frame reports 0.1 km/h
cut at 7de1ae5288 (0 frame(s) inside the 0 day window; held back to the
                   start of the most recent drive)
dropping 1210 of 1390 frame(s)
dropping 8 ref(s) that reach below the cut
expiring the reflogs of HEAD, refs/heads/main
pack 1.6 MB -> 0.2 MB
commit-graph dropped, not rebuilt: git does not write one for a shallow repository
multi-pack-index rewritten
```

The refs go because a ref is what keeps objects alive, and a leftover revert
anchor holds a whole chain of frames behind a commit that is on no branch at
all. Refs holding their own copy of the record get reported and left alone: a
backup ref is somebody's safety copy and a retention pass does not get to
decide about it. `public` is rebuilt from the pruned `main` rather than
deleted, and its reflog is expired only after that rebuild succeeds, because
`publish.py` keeps the previous public ref reachable through the reflog
exactly so a failed rebuild has a fallback.

The prune refuses if any checkpoint set among the frames being dropped has no
lineage entry dated at or before the frames that ran under it. That is the
same lookup blame does, run ahead of time. `carctl lineage --backfill` seeds
the ref from the promotions already on `main`, which is what makes the gate
satisfiable on a repo that predates it.

The last line of that output is the price. Git will not write a commit-graph
for a shallow repository, so once `main` has been pruned there is no
commit-graph for anything. That is a cache rather than the record, and it is
the right way round now that blame, the operation that has to reach furthest
back, runs on the small lineage ref. If walking `main` ever becomes the
binding cost, the answer is a shorter window, not a rewritten record.

Repacking is the other half, and the one that bites first. `fast-import`
writes a pack per session, nothing collapses them, and lookup cost grows with
the count. A geometric repack keeps the count logarithmic while rewriting only
the small packs, and keeps the multi-pack-index. The full repack belongs to
the prune, because that is what actually drops the objects.

Both halves run only while the vehicle is stopped, on two independent signals:
a lock file a drive writes for its own declared length, and the speed in the
last committed frame.

```
stationary check: the last frame on main reports 8.9 km/h; the vehicle is
                  moving, or the record is stale
refusing to touch the object store while the vehicle is not stopped
```

A repack competing with the committer for disk is exactly the stalled-disk
scenario the bounded queue drops frames on, so repacking mid-drive would
manufacture the failure the architecture exists to survive. The lock is
advisory in one direction only. It never blocks a drive, and it expires by
itself, so a power cut cannot leave a vehicle unable to repack until somebody
walks up to it.

## Running it

Python 3.9 or newer and `git` on `PATH`. No third party packages, no build
step. Written and run on Windows against 3.14.

It runs unchanged on Linux, as far as can be checked without a Linux box in
front of me. There is exactly one platform branch in the tree, the
`allow_reuse_address` line in `dashboard/server.py`, and its Linux value is
`True`, which is what `http.server.HTTPServer` sets by default anyway: the
branch exists to take the Windows behaviour away, not to add anything
elsewhere. Nothing else in the tree touches a platform API, no path is built
by hand, no source path differs only by case, and every text-mode subprocess
call names `encoding="utf-8"` explicitly, so a `LANG=C` shell cannot mangle
the street names. The suite runs from a directory whose name contains spaces.
The one version floor is `git` 2.32 for `repack --geometric`; older git takes
the full repack instead and says so. What I have not done is execute it on
Linux.

A fresh clone lands on `main`, which is drive data and contains no source. The
code is on `src`:

```bash
git clone https://github.com/movsq/not-dying-in-traffic.git
cd not-dying-in-traffic
git checkout src
```

Then, from the repo root. Use `python` rather than `python3` on Windows:

```bash
python3 -m carctl drive --seconds 14
```

```bash
python3 -m carctl incident --kind red_light_run
```

```bash
python3 -m carctl park
```

```bash
python3 -m carctl bisect
```

```bash
python3 -m carctl lineage
```

```bash
python3 -m carctl maintain --dry-run
```

```bash
python3 -m carctl publish
```

```bash
python3 dashboard/server.py
```

`drive` writes commits to `refs/heads/main` and `refs/heads/lineage` in
whatever repo you run it from, and `incident` writes a revert under
`refs/reverts/`. Neither touches a remote. `publish` rebuilds
`refs/heads/public` locally and stops there; it needs `--push` to reach the
network, and the audit gate runs first either way.

`maintain` deletes frames, so start with `--dry-run`. On a repo whose history
predates the lineage ref it will refuse to prune until you have run
`carctl lineage --backfill`, which is the right order: the promotions on
`main` are the evidence, and they have to be somewhere else before `main` goes.

## Numbers from a real 14 s drive

```
ticks             140
committed         140
dropped frames      0
promotions          2   <- commits on refs/heads/lineage
deadline overruns   0
max jitter       0.61 ms
max submit cost  26.3 us   <- the loop's entire git bill
tagged as        drive-0010
incidents           4  (off_road @ 102, collision @ 108,
                        red_light_run @ 110, curb_strike @ 131)
```

That overrun count used to read 1, on every drive I ever ran, sitting directly
above a max jitter of 0.62 ms. A 0.62 ms jitter cannot miss a 100 ms deadline,
and the two numbers disagreeing in the same block is what eventually gave it
away: tick zero was scheduled at the instant the epoch was sampled, so it was
already late by the time the loop looked. The count being permanently 1 also
meant a genuine first tick overrun had nowhere to show up, which is the part
that actually matters. The same off by one ended every drive a tick early, so
`--seconds 14` was pacing 13.9 s.

The curb strike moved from seq 132 to 131 because the plant derives time from
the tick count now instead of accumulating `t += 0.1`. That accumulation drifts
low, since 0.1 is not representable, so every scripted event in the scenario
was firing one tick late: the OTA swap at 61 rather than 60, the light at 103
rather than 102.

Two promotions on a fresh repo, not one: the first lineage commit records the
checkpoint set the drive started with, because the ref had nothing in it to
compare against. On a repo that has driven before, only the OTA swap is new.

At 10 Hz the car commits 864,000 times a day. That is what the retention and
repack policy above is for.

## Source

The code lives on `refs/heads/src`, not `main`. `main` is drive data, and
mixing the two would put source files in every frame commit and push them to
the public ref. `git log src` is the code history; `git log main` is the
driving.

## What is missing

The plant is a kinematic bicycle model on a scripted route, in `plant.py`.
Replace it with a real vehicle interface and nothing else in the codebase
notices, which is the one bit of the design I am confident about.

That was not actually true when I wrote it. The loop called
`plant.true_light()` on every tick, which is a ground truth oracle no real
vehicle has, so the one claim I was confident about was the one the code did
not support. It asks for an oracle now and falls back to what perception
reported when there is not one, which is all a real vehicle knows at the time.

The junction has a reference path now, an arc joining the two lane centres,
so `off_road` can fire inside a turn. There is one junction in `JUNCTIONS`
because there is one turn in the script. A real map has to come from
somewhere, and a real junction is not a constant-radius arc: a clothoid or a
spline is the shape, and the offset function is the only thing that would
change.

fast-import only makes refs visible at a `checkpoint`, so `git log` trails live
state by up to 50 frames. Drop `CHECKPOINT_EVERY` in `gitstore.py` if you want
fresher refs and can pay for the extra flushes.

14 days is a judgement, not a derivation. The lineage half is derived and
firm: it must outlive the oldest deployed checkpoint, and today that is six
months. The full-frame half is a forensics question, how long after an
incident anyone might still want 10 Hz poses on the vehicle rather than in
whatever they were uploaded to, and I picked a number rather than leave it
blank. `MAIN_WINDOW_DAYS` in `retain.py` is the one place to change it, and
`carctl maintain --days N` overrides it for a single pass. If the answer is
"we upload within a day and never read on-vehicle frames again", 3 days is
plenty and the budget drops with it. Shortening it is safe in a way it would
not have been before, because the thing that needed a long window no longer
lives on `main`.

Retention governs `refs/heads/main` and whatever reaches into it. A ref
holding an independent copy of the record, a backup of an earlier `public`
among them, is reported and left alone, because deciding about somebody's
safety copy is not a maintenance pass's job. Deciding about it is still
somebody's job.

`refs/reverts/*` is pruned with the frames it points into, which means an
incident's auditable statement expires with the frames it is about. That is
the same problem `models.json` had, and it has the same shape of answer: a
second standalone ref carrying incident records, with the detail, the owner
and the drive, and no pose. I have not written it.

The prune walks the dropped range to check lineage coverage, and it walks
every remaining ref to find the ones that reach below the cut. Both are
O(history) and both run while the vehicle is stopped, which is the only reason
that is affordable. At a real drive-day it is minutes, not seconds.
