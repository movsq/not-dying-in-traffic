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
a bounded queue and gets on with driving. That enqueue costs tens of
microseconds in the worst case. The original Windows capture measured
24.5 us. Repeated Linux drives measure between 20 and 50, and the drive
quoted below measured 22.4. With the committer deliberately SIGSTOPped for
the whole drive it peaked at 1.9 ms, still 50x inside the budget.

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

The committer can also die politely now. A lineage ref whose `models.json` was
a superset of the frame's, a retired subsystem or even a formatting
difference, used to raise inside the drain thread at frame 0. That killed the
thread silently. The drive ran to completion, committed nothing, and said so
only in an unrelated-looking error at the end. A formatting-only difference is
not a promotion and now just updates the comparison blob. A retirement is
recorded as one, because the ref that is never pruned has to be able to say a
subsystem stopped existing.

## What each frame commits

Four files. The shape of `models.json` matters more than it looks.

`state.json` holds pose, speed, street, manoeuvre and a `reversible` flag.
`sensors.json` holds lidar, signal state, lateral offset and IMU.
`actuators.json` holds what we actually commanded, which is often not what we
meant. `models.json` holds one checkpoint per line, and that one formatting
choice is what makes blame useful later.

`lateral_offset` is the signed distance from the reference path the car is
supposed to be on, and `lane_ref` names which path that was. It used to be a
decorative sine of amplitude 0.12 m against an off-road threshold of 1.75 m.
That meant `off_road` was one of four incident kinds that nothing in the
scenario could trigger, and the planner was the one subsystem `git blame`
never got asked about. Making it real also meant the plant had to close the
loop on it. Open loop steering cannot hold a lane, and the old script wandered
3.4 m of `x` across a street that is meant to be straight.

On a street the reference is the centre of the nearest lane the car is allowed
to be in. In a junction there is no such thing, and for a while the plant
reported 0.0 through the whole left turn, which does not read as "no
measurement", it reads as a lane held perfectly. A junction gets an arc
tangent to the centre of the lane the car enters on and the centre of the lane
it leaves on, with those two lane centres as the arc's own extensions past its
tangent points. That is what keeps the number continuous across the
manoeuvre. The plant steers to that arc rather than running the turn open
loop, because an open loop turn has no reference and any offset reported
against one would have been picked to fit whatever the car did. Tracking error
through the turn peaks at 0.38 m against the 1.75 m threshold.

```
seq 24  lat +0.000  ref Vinohradská:0            approaching
seq 32  lat -0.375  ref Vinohradská:0>Ječná:0    mid turn, worst tracking error
seq 50  lat +0.094  ref Vinohradská:0>Ječná:0    settling onto the exit lane
seq 60  lat +0.425  ref Ječná:0                  back on a straight centreline
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
feat: turned left onto Ječná
feat: continued along Ječná at 43 km/h
feat!: continued along Ječná at 43 km/h     <- stop line crossed
chore!: parallel parking on Ječná           <- curb strike
```

Reversibility gets decided when the frame is captured, by code that can see the
sensors. Not later, by code that is guessing.

There is one threshold for it, in `safety.py`, and `plant.py` and `msgen.py`
both read it. They used to keep their own copies, and the copies disagreed.
The plant compared the curb impulse against zero while the message generator
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
curve. Across the scripted left turn the chord is 15.93 m and the driven path
is 18.45 m. The one-arc model the check uses prices it at 17.18 m, about half
the deficit recovered. The S-curve remainder is left to the clearance margin
rather than papered over by a multiplier tuned on this one turn, which would
be wrong on every other geometry and wrong in the unsafe direction on a
tighter one. `lidar_min_bearing` gates the forward cone, so a forward return
0.4 rad off the path does not refuse a clear one. The caveat, learned by
asking what actually calls this, is that a revert's goal is the parent frame,
which is always behind, and the rear channel carries no bearing and so counts
as on-path, the conservative reading of not knowing. And a lidar sitting at
its 40 m range limit is no longer reported as "occupied at 40.0 m". Not
seeing anything as far as you can see is not the same as seeing that it is
clear, and the two deserve different answers.

## Blame

`models.json` puts one subsystem per line so that `git blame -L n,n` lands on
the commit that last changed that specific checkpoint. For a car that swaps
models over the air mid-drive, that commit is the moment the bad model shipped.

```
INCIDENT  red_light_run  seq 110  commit ef65f8ce52
  crossed stop line at 43 km/h while perception reported 'green'
  subject: feat!: continued along Ječná at 43 km/h

git blame ->
  subsystem                    perception
  models_json_line             3
  promoted_on_ref              refs/heads/lineage
  promoted_in                  80448912994a3768d1092cec9e93a8cf7724df72
  promoted_at_unix             1788133336
  promoted_by_commit_subject   promote perception to ckpt-perception-2026.07.14-a91f
  checkpoint                   ckpt-perception-2026.07.14-a91f
  promoted_during_drive        drive-0006
  provenance                   trained 2026-07-14, dataset eu-urban-v12+synthetic-lowsun,
                               shadow_km 9100 of a 250000 km gate, promoted anyway, ...
```

Fifty commits and five seconds before the car ran the light, an OTA agent
promoted a perception checkpoint with 3.6% of the shadow mileage its gate
required. Blame found it starting from nothing but the incident commit. That
is the one place where the git metaphor stops being a joke and earns its keep.

That blame runs on `refs/heads/lineage`, not on `main`. `main` has a retention
window and the promotion being looked for is routinely older than it, so a
`main` sha here would be an answer with an expiry date on it. See below.

The drive stages one fault per subsystem, so all four owners in `OWNER` get
exercised rather than two. The planner asks for lane 2.6 of a two lane road
and holds it long enough to leave the roadway, which is `off_road`. A van is
already parked in the space it steered into, so the excursion produces a near
miss inside `MIN_CLEARANCE`, which is `collision` and belongs to prediction.
Then the perception checkpoint runs the light, and the controller clips the
curb while parking. They overlap on purpose:

```
seq 108  ['collision', 'off_road']
seq 109  ['collision', 'off_road']
seq 110  ['red_light_run', 'off_road']
```

`detect` used to return on its first match, so seq 108 would have been a
`collision` and nothing else, and the off-road excursion still in progress
under it would have been erased rather than deprioritised. Several things can
be wrong at once, and a frame where two are wrong is not less interesting than
one. The list is ordered most severe first, and the order is written down as a
ranking rather than being whatever order the sensors happened to be tested in,
because the loop hands downstream tooling the head of that list.

The van, the parking bay and the curb are places, not times. They used to be
time gates, which made the world a stage rig. The van "appeared" at 10.0 s
whether or not the car was anywhere near it, so a drive that stopped for the
amber still collided, on schedule, with a van 9 m ahead of where it stood.
They are positions along the route now, and a car that stops short of them
records no incident. That is what makes `replay` below mean something.

Drives append to one history rather than each starting fresh, and every drive
gets a `drive-NNNN` tag when it ends. That gives `git bisect` sensible places
to land, because you want to find the first bad drive, not some arbitrary frame
in the middle of one. `carctl bisect` writes the script out and stops there:

```
2 tagged drives, drive-0007 .. drive-0008

git -C /home/fixed/not-dying-in-traffic bisect start drive-0008 drive-0007
git -C /home/fixed/not-dying-in-traffic bisect run carctl replay --assert-no-incident

emitted, not run. bisecting a moving vehicle is not a thing.
```

Tags never leave a clone, because the push refspecs carry branches, not tags.
A fresh clone has none until it drives, and the counter continues from the
`Drive:` trailers on the lineage ref rather than restarting at drive-0001.
That is why the first drive on this clone is drive-0007. Origin's lineage
already says drives 0001..0006 happened, and reusing a number would make
"during drive-0006" permanently ambiguous on the one ref that is never
pruned.

`carctl replay` is what makes the script actually runnable. At each bisect
step git leaves one frame's files in the working tree. Replay reads that
frame's `seq` and its `models.json`, re-drives the deterministic plant with
that checkpoint set pinned for the whole run, no OTA swap, and exits nonzero
if the record up to that frame contains an incident. Pinning is the point.
Incidents in this world are caused by which checkpoint is in force, so frames
committed under the good perception model replay clean end to end, frames
committed under the bad one do not, and bisect converges on the first frame
whose record shows the regression. `--kind red_light_run` narrows the assert
to one incident kind if you want the perception story alone.

The path and both revisions go through `shlex.quote` on the way out. They did
not, which is a silly thing to get wrong in a script whose entire purpose is to
be pasted into a shell. On Windows the shell ate the backslashes and `git -C`
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
git stash push -> refs/parking/a5603421  (seq 124, return pose 45.7,83.2)
  betting on: gap 6.1 m, clearance 0.55 m

git stash pop  -> CONFLICT
  conflict: gap shrank 6.10m -> 5.40m
  conflict: vehicle behind moved
  conflict: clearance 0.22m below margin
  -> stash kept, replanning attempt 2 (the gap is not the gap it was)
```

A stash that cannot notice that conflict is worse than no stash, because it
confidently replays a plan built for a world that is gone.

The TTL is the part that took three tries, each wrong in a way I liked. "Same
process" was first inferred from the monotonic delta being positive and under
an hour, which is exactly what a restart also produces, since the monotonic
clock comes back up at zero. A 44 s old stash reported 7.6 s and popped
clean. So entries carried the id of the process that wrote them, an identity
check instead of a plausible-looking guess. But the identity check was
arbitrating between two clocks, one of which was not a clock. `t_mono_ns` is
the tick count, simulated time, which stops when the loop does, so inside one
long-lived process a stash an hour old by every real measure reported 2.0 s
and popped clean through the branch that was supposed to be the trustworthy
one. Both clocks are read now and the staler answer wins. The wall catches
entries from an earlier process, where tick counts restart and a delta means
nothing. The tick clock catches a simulation running faster than the wall,
where an hour of street time passes in a second and the wall reading is the
lie. Taking the max needs no arbitrator to decide which regime it is in, and
the arbitrator was the part that kept being wrong. The wall stamps carry
whole seconds, so that side reads one second high on purpose. A TTL wants the
upper bound.

`carctl park` also sweeps expired entries before it starts, because the
normal outcome of a parking attempt is a conflict with the stash kept, and
nothing was ever retiring them.

The conflict check also reads all four facts it stores. It skipped
`lead_vehicle_x` entirely, so the car in front could roll back into the gap and
conflict with nothing, and the demo in `cli.py` copies that field forward
unchanged, which is why nothing ever noticed. The clearance rule is relative
as well as floored now. The old absolute-tolerance version fired on gaps that
were always tight and stayed that way, and stayed silent on a clearance that
halved, the exact event its comment claimed to catch, because a halving of a
tight gap is smaller than an absolute tolerance sized for a roomy one.

## The red button

`git revert --hard` is not a git command. The button is not one either. It
freezes the displayed record at the last reversible frame and runs a minimal
risk manoeuvre, and the frames after the freeze point are handed to the
incident tooling as suspect. The dashboard is a display over its own simulated
drive. It moves no refs and reverts nothing, which its page now also says.
The freeze point respects the one-way door. Once a drive has produced an
irreversible frame, the last-good seq stops advancing for the rest of that
drive, because `!` means the history stops being invertible from there on and
a freeze point past it would be an offer to walk back through the curb
strike. The button is guarded by a same-origin check and a token minted at
startup that only ever reaches the served page. Binding to localhost is not
access control, and a plain form POST from any other page in the same browser
is a CORS simple request that nothing preflights. Halting a vehicle should
take more than an open tab.

For a while that guard was on the wrong requests. It lived inside the function
the POST handler called, and nothing else called it, so every GET went
unchecked. `GET /` handed the real token to any `Host` that asked for it, and
`/events` streamed the live 10 Hz feed to the same. The line I had commented as
blocking DNS rebinding was not on the requests that get rebound, which is the
kind of thing you only notice by asking what actually calls it. Reads are
checked now, and the check runs before the route match, so an unauthorised
POST cannot even probe which routes exist. Methods the server does not
implement get the same treatment. HEAD answers like GET behind the Host
check, and PUT, DELETE, OPTIONS and PATCH are refused after it, because the
inherited 501 used to fire before any guard, so "applied to every route" was
only true of the routes that existed. The route match ignores the query
string and case, because `/?v=2` used to miss the token substitution and fall
through to the static file handler, which served the page with the placeholder
still in it. That page streams, looks completely healthy, and refuses every
halt you press. The static handler is gone as well, since it also served
`server.py` verbatim to anyone who asked.

The page checks whether the halt was accepted. It did not, and a refusal is
valid JSON, so a 403 rendered as a completed halt. HALTED banner, red button
disabled, speed still updating underneath it. On the one control whose entire
job is stopping a car, a refusal and a success must not look the same.

The same standard applies to a page that arrives late. A browser that
connected while the vehicle was halted used to fire `onopen`, print
"streaming @ 10 Hz" over readouts showing `-`, and leave the halt button
armed. Stale state dressed as live, on a safety display. Every new subscriber
gets a snapshot frame before the live events now, a reconnect cannot paint
"streaming" over a HALTED banner, and the token-bearing page goes out with
`Cache-Control: no-store`.

One thing that bit me while testing the guard. `allow_reuse_address` means
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

Record plane: freezes at seq 107 -- the 30 frame(s) after it
are handed to the incident tooling as suspect.
Physical plane: rolls back 0 m. The vehicle will run a
minimal-risk manoeuvre and stop where it is.
```

The 107 is the same whenever the click comes after the seq 108 near miss. It
is the last reversible frame before the drive's first one-way door, and the
latch holds there however long the drive runs on.

That second number is always 0 m. I think showing it is the most useful thing
on the whole dashboard. The numbers in the dialog are a prediction made at the
moment you clicked, and the stream keeps moving while you decide. The
confirmation that follows shows the seq the record actually froze at, which is
the server's answer rather than the page's guess.

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
main    state.json    "x": 45.99462893454763, "y": 86.06200615370916
        sensors.json  "light_distance": -17.048849869313287
        message       Pose: 45.994629,86.062006 @ 1.3461 rad
        committer     a real personal address

public  state.json    "x": 50.0, "y": 75.0
        sensors.json  "light_distance": -25.0
        message       Pose: 50.00,75.00 @ 1.3 rad
        committer     fleet@not-dying-in-traffic.invalid
```

The trailer carries six decimals on `main` rather than two. The message and
the blob are scrubbed independently, so a 2 dp trailer could snap into a
different 25 m cell than the full precision pose did, and publishing two
different cells for one frame narrows the true coordinate far more than either
cell alone gives away.

`lane_ref` is the newest field to go through that lookup and it comes out
unchanged, deliberately. It names a street and a lane, and `state.json`
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

`lineage` is pushed alongside `public`. It was designed to carry no pose
precisely so it could be kept forever, and a clone without it answers every
blame from the provisional `main` path. Shipping the scrubbed frames while
withholding the one ref that explains them would publish the puzzle and keep
the answer.

"Publishable by construction" turned out to be an argument, not a check, and
the argument only covered pose. Lineage is written by gitstore and
`lineage --backfill`, nowhere near the fast-export filter, so nothing ever
scrubbed its identities, and the personal committer address rode out on every
lineage commit that was ever pushed. That is the same channel the filter
exists to strip from `public`. Construction is honest now. Lineage commits
are written with the fleet identity at the only place they are ever written,
and the push runs a lineage-shaped audit for identity, tree and messages
beside the public one, refusing the whole push if either fails. A ref written
before that change still carries the address. `carctl lineage --rebuild` is
the migration, since the entries are derived from `main` and rewriting them
costs nothing but the identity line. Rewriting the copy already on the remote
is a push away once the local ref is rebuilt.

There are no pinned push refspecs, deliberately. Pinned refspecs would make a
bare `git push` succeed, publishing `public` and `lineage` while walking
straight past both audit gates. The guard is two other things, and it is
established by `carctl drive`, not only by `publish`, because the drive is
the command that creates the sensitive data and a clone that has driven once
has raw frames worth protecting before anyone thinks about publishing. First,
`push.default` is set to `nothing` in the repo's local config, unless the
local config already says something on purpose, so a bare `git push` fails
loudly. Second, a `pre-push` hook is installed that refuses any push of
anything but `refs/heads/public` to the remote's `main`. Git's own error for
`push.default=nothing` helpfully suggests naming a refspec, and `git push
origin main` is precisely the spelling that must not work. What remains is
`carctl publish --push`, which audits first.

The scrubbed `public` and the lineage ref have been pushed. A clone of this
repository carries the scrubbed copy as its `main`, which is what makes the
clone safe to hand out, whoever holds it. Whether the repo is private or
public lowers or raises the stakes but does not change the design. Publishing
a drive stays a thing you type on purpose:

```bash
python -m carctl publish --push
```

## Retention, and the ref that outlives the frames

At 10 Hz the car commits 864,000 times a day. Across the 1020 commits on
`main` a frame costs 255.1 bytes packed, so a drive-day is about 220 MB and a
year about 80 GB. Treat that as a floor. These are short scripted drives whose
poses delta extremely well, and it is measured after a repack.

The number that decides the window is not a disk number. Blame has to reach
back to the promotion of the oldest checkpoint still in service, which today
is `ckpt-controller-2026.03.01-0b12`, about six months old. Prune below that
and `git blame -L n,n models.json` walks off the end of what survives and
lands on the oldest remaining commit. A confident wrong answer is worse than
no answer, and it is precisely the failure blame exists to prevent. Eighteen
months of full frames to satisfy that would be about 121 GB.

So retention splits by file rather than by time.

```
refs/heads/main      full frames, 14 days       ~3.1 GB at the floor,
                                                budget 10 GB for real ratios
refs/heads/lineage   models.json, forever       one commit per promotion
```

Four subsystems promoting maybe weekly is a rounding error of disk, and it
turns the window into a non-question for the only thing that needed a long
one. The lineage ref has to stand alone, because the frame the promotion
happened on will be gone:

```
8044891  2026-08-31 01:42  promote perception to ckpt-perception-2026.07.14-a91f
            perception ckpt-perception-2026.05.30-1e4d -> ckpt-perception-2026.07.14-a91f
            during drive-0006
```

The drive is a name in the body, not a ref. Retention deletes the tag along
with the frames it bounds, so "during drive-0006" has to stay readable after
`drive-0006` does not exist. That is what forced the tag name to be decided at
the start of a drive rather than at the end. The lineage commits are written
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
changes every surviving commit's sha, and a sha is a frame's identity here.
`refs/reverts/<sha>` is what anchors an incident to the frame it happened on,
so a daily rewrite would invalidate yesterday's incident report. A graft
reclaims nothing, because git disables replace refs while packing on purpose,
and commit objects are over half the pack.

```
carctl maintain --days 0

stationary check: stopped, last frame reports 0.4 km/h
window: 0 day(s) (--days)
cut at 80a3db9b09 (170 frame(s) inside the 0 day window; held back to the start of the most recent drive)
dropping 1210 of 1380 frame(s)
dropping 3 ref(s) that reach below the cut
  still holds dropped frames (remote-tracking; retention cannot reclaim these objects -- `git update-ref -d <ref>` or repoint the remote if this clone's disk matters): refs/remotes/origin/HEAD
  still holds dropped frames (remote-tracking; retention cannot reclaim these objects -- `git update-ref -d <ref>` or repoint the remote if this clone's disk matters): refs/remotes/origin/main
rebuilt refs/heads/public from the pruned main
expiring the reflogs of HEAD, refs/heads/main, refs/heads/public
pack 0.9 MB -> 0.6 MB
commit-graph dropped, not rebuilt: git does not write one for a shallow repository
multi-pack-index rewritten
```

The remote-tracking lines are a clone telling the truth about itself. A ref
is what keeps objects alive, and `refs/remotes/origin/main` pins every frame
this pass just dropped, so on a clone the pack line barely moves until you
decide about the tracking refs too. They used to be filed under "somebody's
safety copy", which is an operator decision, and a clone's own bookkeeping is
not one. On the car there are no remote-tracking refs, and the drop is the
whole story.

The `window:` line names where the number came from, the flag, the
`CARCTL_WINDOW_DAYS` environment variable, or the default, because the window
decides which frames stop existing, and ambient configuration that can
shorten it has to be visible in the report it shortened. A garbled value is
refused loudly rather than silently falling back to 14, from either source.
The environment must be a positive number, and `--days` a finite one, zero or
more. Zero from a flag is an operator's explicit decision for one pass. Zero
from the environment would be a standing config that wipes every drive on
every pass. The two are held to different rules on purpose.

The refs go because a ref is what keeps objects alive, and a leftover revert
anchor holds a whole chain of frames behind a commit that is on no branch at
all. Refs holding their own copy of the record get reported and left alone. A
backup ref is somebody's safety copy, and a retention pass does not get to
decide about it. `public` is rebuilt from the pruned `main` rather than
deleted, and its reflog is expired only after that rebuild succeeds, because
`publish.py` keeps the previous public ref reachable through the reflog
exactly so a failed rebuild has a fallback.

The prune refuses if any checkpoint set among the frames being dropped has no
lineage entry dated at or before the frames that ran under it. That is the
same lookup blame does, run ahead of time. `carctl lineage --backfill` seeds
the ref from the promotions already on `main`, which is what makes the gate
satisfiable on a repo that predates it.

A prune that fails after it has started changing the repository says so. The
deleted refs and the shallow boundary land before the reflog expiry and the
repack, and any of those later steps can fail. A pack held open by a virus
scanner is the usual Windows way. The report that failure interrupts survives
it, marked as a prune that failed partway with the refs already gone, because
"not pruned" and "half pruned" need opposite responses and the operator only
gets to pick the right one if the report can tell them apart. The exit code
says the same thing. A pass that got past the point of no return and then
stopped exits non-zero with the full report, so a cron line cannot read "half
pruned" as success. That catch is for any exception, not only this module's
own. The public rebuild inside the destructive window raises the publish
module's types, and letting those out as tracebacks was exactly the bare "not
pruned" this paragraph promises not to give you.

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
a lock file a drive writes and renews, and the speed in the last committed
frame.

```
stationary check: the last frame on main reports 25.6 km/h; the vehicle is
                  moving, or the record is stale
window: 14 day(s) (MAIN_WINDOW_DAYS default)
refusing to touch the object store while the vehicle is not stopped
```

A repack competing with the committer for disk is exactly the stalled-disk
scenario the bounded queue drops frames on, so repacking mid-drive would
manufacture the failure the architecture exists to survive. The lock is
advisory in one direction only. It never blocks a drive. It is a lease
rather than a fixed expiry, claiming fifteen minutes at a time and renewed by
the loop once a minute for as long as the drive actually runs. A fixed expiry
had to pick between two failures. Honour a typo'd `--seconds 1e9` and lock
maintenance out until 2058, or cap it and leave a genuinely long drive
unprotected past the cap, with the speed gate passing the moment the vehicle
pauses at a red light. The lease has neither. Any drive length stays
protected while it is alive, and a crash frees maintenance within minutes.
Each lock carries a token, and a drive only removes the lock it wrote. Two
overlapping drives used to let whichever finished first delete the lock the
other was still protected by.

## Running it

Python 3.9 or newer and `git` on `PATH`. No third party packages, no build
step. Written and run on Windows against 3.14.

It has now been executed on Linux, and the first execution disagreed with
this section in more places than the code did. The loop, the committer and
the whole drive pipeline ran unchanged, and the max jitter came out 0.03 ms
against 0.64 on Windows. There are two platform branches in the tree now. One
is the `allow_reuse_address` line in `dashboard/server.py`, whose Linux value
is `True`, which is what `http.server.HTTPServer` sets by default anyway. The
other is the process-group detach in `gitstore.py`, `start_new_session` on
POSIX and a CreateProcess flag on Windows, because the two platforms have no
shared spelling for "not in the terminal's group". Both exist to take a
behaviour away, not to add one. Nothing else in the tree touches a platform
API, no path is built by hand, no source path differs only by case, and every
text-mode subprocess call names `encoding="utf-8"` explicitly, so a `LANG=C`
shell cannot mangle the street names. The one version floor is `git` 2.32 for
`repack --geometric`. Older git takes the full repack instead and says so.

A fresh clone lands on `src`, the code. The drive data is one branch over on
`main`, the scrubbed public copy, since that is what was published:

```bash
git clone https://github.com/movsq/not-dying-in-traffic.git
cd not-dying-in-traffic
```

Then, from the repo root. On Linux, substitute `python3` if that is what your
distribution calls it:

```bash
python -m carctl drive
```

The default drive is 17 s, long enough to run the whole script and come to
rest. That matters because `maintain` reads the last committed frame's speed
and refuses to touch the disk under a car that is still rolling. A drive cut
short ends moving, and maintenance then waits for one that did not. Cutting
one short is safe now from either direction. fast-import runs outside the
terminal's process group, so a ^C, which signals the whole group, no longer
kills the child mid-stream and costs the frames since the last checkpoint,
and SIGTERM arrives as the same clean shutdown. An interrupted drive commits
every frame it drove, prints its report, tags itself like any other, and
exits 130 so a script can tell a prefix from the drive it asked for. The
first drive on a clone also adopts origin's lineage ref before it reads it.
A clone materialises only `main`, and driving once used to orphan every
published promotion by rooting a second lineage next to it.

```bash
python -m carctl incident --kind red_light_run
```

```bash
python -m carctl park
```

```bash
python -m carctl bisect
```

```bash
python -m carctl lineage
```

```bash
python -m carctl maintain --dry-run
```

```bash
python -m carctl publish
```

```bash
python dashboard/server.py
```

`pip install .` is optional and adds a `carctl` entry point on `PATH`, which
is what the emitted bisect script calls. The repo every command operates on
is the one containing your working directory, resolved fresh per invocation.
It used to be derived from where the package was installed, which after
`pip install .` was site-packages, or worse, whichever repository happened to
contain the venv, and 170 frames of drive data landing silently in an
unrelated repo is the kind of bug you only get to be surprised by once.
`CARCTL_REPO` in the environment overrides the cwd lookup, for cron and
systemd units that have no meaningful working directory. A value that is not
a git repository is refused loudly, for the same reason `CARCTL_WINDOW_DAYS`
is. `python -m carctl` is the same thing without the install, with one
exception. Inside a `git bisect` checkout the working tree is a frame, not
the source, so `python -m carctl` has nothing to import there and the emitted
script needs the installed entry point.

`drive` writes commits to `refs/heads/main` and `refs/heads/lineage` in
whatever repo you run it from, and refuses if that repo's `main` is not a
frame history, because frames are appended onto the tip's tree and driving
with source checked into `main` would stamp `carctl/` into every frame
commit. `incident` writes a revert under `refs/reverts/`. Neither touches a
remote. `publish` rebuilds `refs/heads/public` locally and stops there. It
needs `--push` to reach the network, and the audit gates, public and lineage
both, run first either way. On a clone whose lineage predates the
public-identity change, that gate will tell you to run
`carctl lineage --rebuild` once before the first publish. The rebuild rewrites
every lineage sha. That is the point, the old shas carry the address. So the
first push afterwards is refused non-fast-forward against the remote's old
copy, and `publish --push` says so and prints the one deliberate force
command that replaces it. Pushed refs are the one place the rewrite has to be
typed on purpose.

`maintain` deletes frames, so start with `--dry-run`. A fresh drive writes its
own lineage entries, so a repo born under this code is always prunable. A repo
whose history predates the lineage ref will refuse to prune until you have run
`carctl lineage --backfill`, which is the right order. The promotions on
`main` are the evidence, and they have to be somewhere else before `main`
goes. On a clone the first drive seeds the ref from origin by itself, so the
gate is normally already satisfied. `--rebuild` is for replacing a ref that
exists and is wrong.

## Numbers from a real 17 s drive

Captured on Linux, from the first drive on a fresh clone:

```
ticks           170
committed       170
dropped frames  0
promotions      2   <- commits on refs/heads/lineage
deadline overruns 0
max jitter      0.03 ms
max submit cost 22.4 us   <- the loop's entire git bill
tagged as      drive-0007

incidents: 4 across 4 kind(s)
  off_road        first at seq  102  lateral offset -1.79 m from Ječná:2.6
                  owner: planner
  collision       first at seq  108  clearance=1.43 m
                  owner: prediction
  red_light_run   first at seq  110  crossed stop line at 43 km/h while perception reported 'green'
                  owner: perception
  curb_strike     first at seq  131  az=50.8 m/s^2
                  owner: controller
```

The same drive on Windows reported a max jitter of 0.64 ms, twenty times this
box's 0.03, and a submit cost of 24.5 us against this capture's 22.4. The
counts, seqs and incidents are identical, which is the portability claim
above in one line. Under `--fast` the jitter and overrun lines read "not
measured" instead of zero. Nothing was scheduled against a deadline, and a
zero that was never measured reads exactly like a clean result on the two
numbers this loop exists to produce.

This block used to say 5 incidents across 4 kinds. The fifth was 21
consecutive frames of `collision` starting at seq 149. The car had parked,
successfully, and sat stationary at 0.4 km/h, 1.40 m behind the van it had
deliberately parked behind, charged to prediction and invisible under a
report that prints one line per kind. A near miss is about closing speed, so
the detector now reads the range rate between frames, and a car that is
neither moving nor being closed on raises nothing.

That overrun count used to read 1, on every drive I ever ran, sitting directly
above a max jitter of 0.62 ms. A 0.62 ms jitter cannot miss a 100 ms deadline,
and the two numbers disagreeing in the same block is what eventually gave it
away. Tick zero was scheduled at the instant the epoch was sampled, so it was
already late by the time the loop looked. The count being permanently 1 also
meant a genuine first tick overrun had nowhere to show up, which is the part
that actually matters. The same off by one ended every drive a tick early, so
`--seconds 14` was pacing 13.9 s.

The curb strike moved from seq 132 to 131 because the plant derives time from
the tick count now instead of accumulating `t += 0.1`. That accumulation drifts
low, since 0.1 is not representable, so every scripted event in the scenario
was firing one tick late: the OTA swap at 61 rather than 60, the light at 103
rather than 102.

Two promotions in this drive, and neither is a human decision. The drive
starts on the checkpoint set the source ships, which differs from what the
lineage ref last saw, because the previous drive ended under the OTA swap, so
the first frame records a rollback. Then the swap lands again at 6.0 s. On a
fresh repo the first commit records the whole starting set instead, because
the ref has nothing to compare against.

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
somewhere, and a real junction is not a constant-radius arc. A clothoid or a
spline is the shape, and the offset function is the only thing that would
change.

fast-import only makes refs visible at a `checkpoint`, so `git log` trails live
state by up to 50 frames. Drop `CHECKPOINT_EVERY` in `gitstore.py` if you want
fresher refs and can pay for the extra flushes.

14 days is a judgement, not a derivation. The lineage half is derived and
firm. It must outlive the oldest deployed checkpoint, and today that is six
months. The full-frame half is a forensics question, how long after an
incident anyone might still want 10 Hz poses on the vehicle rather than in
whatever they were uploaded to, and I picked a number rather than leave it
blank. `MAIN_WINDOW_DAYS` in `retain.py` is the one place it lives,
`CARCTL_WINDOW_DAYS` in the environment overrides it per deployment, and
`carctl maintain --days N` overrides both for a single pass. Every pass
reports which of the three it used. If the answer is "we upload within a day
and never read on-vehicle frames again", 3 days is plenty, about 660 MB at
the floor, call it 2 GB with the same margin. Shortening it is safe in a way
it would not have been before, because the thing that needed a long window no
longer lives on `main`.

Retention governs `refs/heads/main` and whatever reaches into it. A ref
holding an independent copy of the record, a backup of an earlier `public`
among them, is reported and left alone, because deciding about somebody's
safety copy is not a maintenance pass's job. Deciding about it is still
somebody's job.

`refs/reverts/*` is pruned with the frames it points into, which means an
incident's auditable statement expires with the frames it is about. That is
the same problem `models.json` had, and it has the same shape of answer. A
second standalone ref would carry the incident records, with the detail, the
owner and the drive, and no pose. I have not written it.

The prune walks the dropped range to check lineage coverage, and it walks
every remaining ref to find the ones that reach below the cut. Both are
O(history) and both run while the vehicle is stopped, which is the only reason
that is affordable. At a real drive-day it is minutes, not seconds.
