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
INCIDENT  red_light_run  seq 110  commit 93d67cc765
  crossed stop line at 43 km/h while perception reported 'green'

git blame ->
  subsystem                    perception
  models_json_line             3
  promoted_in                  18030a49f032405ba37e7d6f98f2e859f88d3d9f
  promoted_by_commit_subject   feat: continued along Hlavní at 26 km/h
  checkpoint                   ckpt-perception-2026.07.14-a91f
  provenance                   shadow_km 9100, gate is 250000, promoted anyway
```

Fifty commits and five seconds before the car ran the light, an OTA agent
promoted a perception checkpoint with 3.6% of the shadow mileage its gate
required. Blame found it starting from nothing but the incident commit. That
is the one place where the git metaphor stops being a joke and earns its keep.

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

## Running it

Python 3.9 or newer and `git` on `PATH`. No third party packages, no build
step. Written and run on Windows against 3.14. It should run unchanged on
Linux: there is exactly one platform branch in the tree, the
`allow_reuse_address` line in `dashboard/server.py`, which is off on Windows
for the reason above and on everywhere else. Nothing else touches a
platform API, and paths go through `os.path` and `pathlib` throughout.

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
python3 -m carctl publish
```

```bash
python3 dashboard/server.py
```

`drive` writes commits to `refs/heads/main` in whatever repo you run it from,
and `incident` writes a revert under `refs/reverts/`. Neither touches a remote.
`publish` rebuilds `refs/heads/public` locally and stops there; it needs
`--push` to reach the network, and the audit gate runs first either way.

## Numbers from a real 14 s drive

```
ticks             140
committed         140
dropped frames      0
deadline overruns   0
max jitter       0.55 ms
max submit cost  27.0 us   <- the loop's entire git bill
tagged as        drive-0010
incidents           2  (red_light_run @ seq 110, curb_strike @ seq 131)
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

At 10 Hz the car commits 864,000 times a day. That number is the thing I would
worry about first if this ran for a week.

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

`off_road` is one of the four incident kinds and the scripted drive cannot
trigger it. `lateral_offset` in the plant is a decorative sine with an
amplitude of 0.12 m against a threshold of 1.75 m, so the planner is the one
subsystem `git blame` never gets asked about. Fixing that needs a lane model
rather than a bigger number, which is why it is in this section rather than
the previous one.

fast-import only makes refs visible at a `checkpoint`, so `git log` trails live
state by up to 50 frames. Drop `CHECKPOINT_EVERY` in `gitstore.py` if you want
fresher refs and can pay for the extra flushes.

There is no repack policy yet, and 864k commits a day needs one, along with a
retention window. I said I could not size it without a real drive-day, but half
of that was measurable from what is already here: across the 1210 commits on
`main`, a frame costs 238.8 bytes on disk after packing, so a drive-day is
about 197 MB and a year about 70 GB. Treat it as a floor. These are short
scripted drives whose poses delta extremely well, and it is measured after a
repack. The packs will bite before the disk does: `fast-import` writes one per
session, nothing collapses them, and lookup cost grows with the count. What I
still cannot guess is the retention window, because that is a question about
how far back you want `git blame` to reach, not about bytes.
