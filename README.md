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

## Revert, and the half of it that cannot exist

`git revert` on a source tree always works. Any diff inverts. Physics does not
work that way, so revert splits in two.

**Record revert.** A real `git revert`, run in a detached control worktree so
it can never race fast-import writing to `main`. It always succeeds. What it
produces is an auditable statement that a frame was wrong.

**Physical revert.** Read the parent commit's `state.json`, treat that pose as
a goal, and check whether the goal is still inside the reachable set from where
the car is now. Drive there only if it is. Here is what the red light incident
actually printed:

```
physical revert (0.8s later) ->
  allowed: False
  reason:  goal is 10.4 m behind at 37 km/h; reversing is inadmissible above 7 km/h
```

So the record got reverted and the world did not. Both of those are true at the
same time and the system reports both. Skip the reachability check and "revert"
becomes an unplanned manoeuvre wearing a reassuring name, which is worse than
having no revert.

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
2 tagged drives, drive-0001 .. drive-0002

git bisect start drive-0002 drive-0001
git bisect run carctl replay --assert-no-incident

emitted, not run. bisecting a moving vehicle is not a thing.
```

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

## The red button

`git revert --hard` is not a git command. The button is not one either. It
freezes the record at the last reversible frame and runs a minimal risk
manoeuvre. The confirmation dialog says what you asked for, and under it, the
two numbers that disagree.

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
filter and back into `git fast-import`. Poses snap to a 25 m grid, commit times
round to the minute, street names and manoeuvres and the reversible flag come
through untouched. Those are the parts worth reading anyway.

```
main   "x": 41.846469430637235, "y": 89.7491853669169
public "x": 50.0, "y": 100.0
```

The push refspec is pinned in `.git/config` to
`refs/heads/public:refs/heads/main`, and `push.default` is `nothing`, so a bare
`git push` cannot reach `main` by accident.

Nothing has been pushed yet. The repo is private, which lowers the stakes but
does not change the design. Publishing stays a thing you type on purpose:

```bash
python -m carctl publish --push
```

## Running it

```bash
python -m carctl drive --seconds 14
```

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
python -m carctl publish
```

```bash
python dashboard/server.py
```

## Numbers from a real 14 s drive

```
ticks             140
committed         140
dropped frames      0
deadline overruns   1
max jitter       0.62 ms
max submit cost  25.0 us   <- the loop's entire git bill
incidents           2  (red_light_run @ seq 110, curb_strike @ seq 132)
```

At 10 Hz the car commits 864,000 times a day. That number is the thing I would
worry about first if this ran for a week.

## What is missing

The plant is a kinematic bicycle model on a scripted route, in `plant.py`.
Replace it with a real vehicle interface and nothing else in the codebase
notices, which is the one bit of the design I am confident about.

fast-import only makes refs visible at a `checkpoint`, so `git log` trails live
state by up to 50 frames. Drop `CHECKPOINT_EVERY` in `gitstore.py` if you want
fresher refs and can pay for the extra flushes.

There is no repack policy yet, and 864k commits a day needs one, along with a
retention window. I did not guess at the numbers because sizing it needs a real
disk budget and a real drive-day, and I have neither. Give me either and I will
write it.
