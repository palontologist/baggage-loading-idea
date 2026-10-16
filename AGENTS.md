# Baggage Loading Challenge - SIGNATE

## Quick Push

```bash
cd /home/palontologist/Downloads/dev/baggage-loading-idea
python -c "import zipfile; z=zipfile.ZipFile('submit.zip','w',zipfile.ZIP_DEFLATED); z.write('submit/agent.py','agent.py'); z.close()"
git add submit/agent.py submit.zip AGENTS.md
git commit -m "feat: rewrite agent around the simulator's own placement rules"
git push origin main
```

## Files

- `submit/agent.py` - the whole agent (pure Python + NumPy, single file)
- `submit.zip` - what gets uploaded; contains `agent.py` at the archive root

## Testing

The simulator needs its dependencies; run from `simulator/` with its venv
active and `PYTHONPATH=.`:

```bash
cd simulator && source .venv/bin/activate
PYTHONPATH=. python -m scripts.run_test \
  --module-path agents/submit/ \
  --config-path configs/sample_config.json
```

`--module-path` is dotted relative to `simulator/`, so symlink the agent in:
`ln -s ../../submit simulator/agents/submit`.

Helper scripts live in `simulator/scripts/`:

- `live.py` - drive the real simulator in-process, no multiprocessing. Takes
  `BAG_CFG` (config path) and a task id. Fastest way to check a change.
- `live_sweep.py` - same, across several parameter settings.
- `pack_debug.py` - offline only: what the planner does, no physics.
- `probe.py` - dumps container planes and item statistics.
- `stuck.py` / `mismatch.py` - explain why placements are rejected.
- `composite.py` - **judge every change with this one.** Runs sample 000/001
  plus `robust_config.json` R1-R4 and prints the five components the platform
  actually averages, not just items placed. Takes about ten minutes for all six.
- `why_stop.py` - **use this before proposing any change.** Attributes every
  run's end to exactly one of the three process checks (`inclusion`,
  `transport`, `settle`) and reports how many items survived. Cheaper than
  `composite.py` and it points straight at the binding constraint.

The score is the unweighted mean of `fill`, `cog`, `stability`, `placement`
and `soft_item`, so a change that raises one and lowers another has to clear a
real bar. `num_placed` is reported but is *not* part of the score.

## Results (local simulator, sample_config.json)

| task | items placed | fill_score | optimize | policy |
| --- | --- | --- | --- | --- |
| 000 | 68.3% | 31.15 | 48 s | 0.2 s |
| 001 | 47.6% | 20.96 | - | 1.2 s |

Limits are 180 s for optimize and 8-10 s for policy. `_OPT_BUDGET` (150 s) and
`_POLICY_BUDGET` (3 s) are wall clock, so the agent returns in time whatever the
machine speed and how large the task is; a truncated plan is still a valid
permutation, so degradation is graceful.

Check changes against `configs/robust_config.json` (R1-R4: the sample item sets
with jittered dimensions, varied containers, lookaheads and `optimize` flags) as
well as the sample tasks. Two tuning wins that looked good on the samples were
reverted after the wider sweep - see below.

## How the agent works

The important part is that it reproduces the simulator's own placement rules
instead of approximating them, because a single rejected placement ends the whole
episode and zeroes everything after it.

1. **Geometry from the container's own planes.** Interior bounds and the
   containment test are read off `n_vecs` / `points`, the exact planes the
   simulator tests against. Nothing about wall thickness, buffer or the cut corner
   is assumed. (The "cut corner" turns out to be a bottom-left chamfer, so the
   left wall only exists above it.)

2. **Height map placement.** A 2D surface map gives every resting height in one
   sliding-max pass per orientation, so candidate positions for all items are
   cheap. Items may bridge gaps between neighbours; the map is stamped with the
   packing gap so the sliver of floor between two boxes still reads as being at
   their level.

3. **Only ever lay items on their smallest face.** A box stood on end seals the
   push-in corridor in front of it for everything behind it in that row.

4. **The push-in corridor is modelled exactly.** `feasible()` mirrors
   `PlacementValidator.check_transport_path`: insertion height, the clamped
   insertion x, the sweep from the opening to the target depth, the lateral slide,
   and the resting pose. Any candidate that would have been rejected is never
   offered.

5. **Stability is checked geometrically.** A box has to be held by enough of its
   underside, and that support has to sit under its middle. Without this, items
   that merely graze a neighbour are released in mid air, fall the full height of
   the container, fail the settle test, and end the episode.

6. **Ranking**: lowest layer first, then deepest (keeps the corridor open for
   what comes next), then the most level support, then flush against a wall or
   neighbour, then toward the centre for a low centre of gravity. Soft items go
   last so nothing is stacked on them; priority items are steered into priority
   containers when one exists.

7. **`optimize()` returns only an ordering.** `policy()` recomputes every
   position from the live settled state. Replaying precomputed coordinates broke
   as soon as anything settled a few millimetres out.

## What was tried and did not work

**Beam search over placement sequences** (tried 2026-10-09, reverted). A beam of
partial layouts carried forward per depth, branching over both which item goes
down next and where it goes. Measured four configurations on the sample tasks:

| variant | task 000 items | task 000 fill | plan time |
| --- | --- | --- | --- |
| greedy | 68.3% | 31.15 | 43 s |
| beam 2x3, plan replayed | 53.7% | 17.80 | 71 s |
| beam 2x3, order only | 56.1% | 22.44 | 71 s |
| beam 2x2, order only | 68.3% | 25.40 | 88 s |
| beam 4x2, order only | 51.2% | 17.03 | 88 s |

It looks better *offline* (a 2x3 beam placed 26 items versus the greedy's 25) and
then loses live. Two reasons:

- Branching over positions is worthless if `policy()` throws the positions away
  and re-derives them, and branching over items is worthless if `_best_move` is
  not allowed to look at every item first. Restricting the expansion to the
  biggest items made things strictly worse than the greedy's ranking.
- A replayed plan is brittle. The greedy recomputes each position against the
  poses the simulator has actually settled into and so self-corrects; a plan is a
  rigid sequence, and one item settling a few millimetres out invalidates every
  position after it.

Cost per search node is the binding constraint: evaluating every remaining item
costs about a second, so a beam wide enough to matter does not fit in the 180 s
`optimize` budget.

**Preferring positions that go in without a sideways slide** (tried 2026-10-10,
reverted). Only boxes whose centre lies inside the direct-entry span avoid a
lateral slide, and a slide seals that band for every later box at the same
height, so preferring direct entry looks like a strong win. On the sample tasks
it is: task 000 68.3% -> 70.7% of items, task 001 47.6% -> 52.4%. On the wider
sweep it is a gamble, not an improvement:

| config | slide=0 | slide=1e4 |
| --- | --- | --- |
| sample 000 | 68.3% | 70.7% |
| sample 001 | 47.6% | 52.4% |
| R1 | 41.5% | 39.0% |
| R2 | 97.6% | 100.0% |
| R3 | 56.1% | 26.8% |
| R4 | 26.2% | 26.2% |
| random 60 | 78.3% | 75.0% |
| **mean** | **59.4%** | **55.7%** |

It gains on the two tasks it was tuned on and collapses on R3, which is the same
items in the same container with dimensions jittered by 10% and `optimize` off.
Since the graded cases are unseen, a change that wins on average but has a 30
point downside on a plausible neighbouring configuration is not worth taking.

Lesson worth keeping: tune on `scripts/tune_live.py` against the physics, and
judge on the whole suite. The planner's own score is over optimistic, because it
evaluates positions against its model while `policy()` re-derives them from
settled poses - at one point this change read as 25 -> 34 items offline and
70.7% -> 70.7% live.

**Keeping cargo off soft bags** (tried 2026-10-10, reverted). The platform's
second-worst component is `soft_item` (~14/100 on the graded run), and soft bags
crush under load, so preferring spots whose underside does not rest on soft
cargo looks free. It is not. Three variants, all measured on `composite.py`:

| task | none | soft only | soft + priority |
| --- | --- | --- | --- |
| 000 items | 68.3% | **75.6%** | 68.3% |
| 000 softfree | **33.3%** | 0.0% | 22.2% |
| 000 fill | 31.15 | **37.63** | 26.50 |
| R3 items | **56.1%** | 56.1% | 26.8% |
| R1 items | 41.5% | 41.5% | **46.3%** |

"Soft only" is genuinely better on every raw axis for task 000 except the one it
targets, and identical everywhere else, which is the most seductive result in
this file - and it is still a loss, because the score is the mean of five
components and it trades 33 points of `softfree` for 6.5 of fill. Adding
priority cargo on top of that reintroduces the 30 point downside on R3.

The reason it fails is structural, and the same reason means the obvious fix
will not work either. On task 000 the planner places the first eight items,
then finds that **no ordinary item has a feasible spot** for four consecutive
steps, so it places four soft bags to fill gaps. Those four are what unblock the
next nineteen ordinary items, and the ordinary items then land on top of them.
Soft bags are load-bearing filler here: taking them out of the load path costs
more fill than the platform awards for not crushing them.

Reserving the whole column above a soft bag - nothing whose footprint overlaps
a soft bag's footprint - **does fix softfree, and the old runtime objection is
now gone.** The first attempt did the shadow test per candidate and could not
finish six configs in 40 minutes. `_Container._reserved` instead keeps each soft
bag's footprint as a rectangle taken straight out of `self.boxes` (so it is
recovered from the simulator's own poses and survives a restart) and tests
overlap at `O(candidates x soft bags)` - the same shape as the stability test
that already runs. Three configs now take 5m22s.

This is the only change in this file that gains on several components at once,
and it is the best result measured so far:

| | 000 placed | 000 fill | 000 softfree | 000 placement | 000 cog_z |
| --- | --- | --- | --- | --- | --- |
| top face only | 73.2% | 30.03 | 12.5% | 25.0% | 0.601 |
| whole column | 51.2% | 11.75 | **100.0%** | **100.0%** | **0.447** |

Across the nine-config suite the mean moves by placed -3.2, fill -2.4,
softfree **+28.2**, placement **+13.9**, cog_z -0.032. Six of nine scenes reach
100% on both softfree and placement, which are the values the official
reference agent reports. Only task 000 pays heavily (22 points of placed, 18 of
fill) and R3's placement slips 50% -> 33%.

The cost is concentrated in fill, so this only pays if the platform weights
softfree and placement at least comparably to fill. Since the score is a
weighted average we have never been able to read, that is the one assumption
this rests on.

**Mass-aware item ordering** (tried 2026-10-10, reverted). `agent.py` had no
reference to `mass` at all, while the centre-of-gravity component is explicitly
mass-weighted and `scripts/score_layout.py` already reports a mass-weighted
`cog_z`. Adding `_W_MASS * (_M_REF - mass)` to the candidate key is a safe shape:
it is constant across one item's candidates, so it never moves the box, it only
decides which item goes down next - heavy cargo sooner, so it lands low.

The mechanism works; the price does not. The key's positional terms sit at
`1e15` per cell of layer, `1e6` of depth and `1e3` of flush, so the weight has to
be swept across five orders of magnitude to find the regime:

| `_W_MASS` | 000 placed | 000 fill | 000 cog_z | R3 placed | R3 fill | R3 cog_z |
| --- | --- | --- | --- | --- | --- | --- |
| 0 (off) | 68.3% | 31.15 | 0.636 | 56.1% | 25.97 | 0.658 |
| 1 | 68.3% | 31.15 | 0.636 | 56.1% | 25.97 | 0.658 |
| 100 | 61.0% | 23.25 | 0.621 | 56.1% | 24.01 | 0.658 |
| 1e13 - 1e15 | 43.9% | 23.74 | 0.635 | 39.0% | 15.61 | 0.511 |

Anything above `1e3` swamps the flush and depth terms and simply sorts the whole
task heaviest-first. That does lower the centre of gravity (R3 0.658 -> 0.511,
001 0.582 -> 0.499) and takes `softfree` on task 000 from 33% to 100%, but it
costs 17-25 points of items placed, and centre of gravity is one of five
components. Mass is best used as a pure tie-break, and there it does nothing at
all, because the existing `level` term already puts cargo at the bottom and the
cheap way to lower a mass-weighted centroid is to fill the bottom densely.

**Ground-layer items can never score fill** (found 2026-10-10, not fixable).
Across the suite 55 placements are reported `not inside (hit boundary plane)`
and therefore contribute zero to `fill_score` while still counting towards
`num_placed_items`. 91% of them are on `plane0`, normal `[0,0,-1]` - the floor -
with a median overshoot of 0.0mm:

| cause | count |
| --- | --- |
| ground-layer graze, <=5mm | 50 |
| sunk 5-50mm (soft bags deforming) | 4 |
| other | 1 |

The mechanism: `containers.buffer = 0.0`, so the collision surface sits exactly
on the containment plane; the item is commanded 6mm up (`_CONTAIN`), accepted,
spawned 8cm up (`start_z = 0.08`) and falls onto the floor; then
`evaluator.py:19` reuses the *validator's placement* margin
(`inclusion_margin = -0.005`) against the *settled* pose, and a resting item's
bottom corner has `dots = 0.0`, which fails. Raising `_CONTAIN` cannot help -
commanding higher only means a longer fall to the same resting place.

This is a property of the harness, not of this agent, so it caps everyone
equally. It does mean the effective ceiling on `fill` is roughly 45-50 rather
than the 59-65 that item volume alone suggests, since the entire ground layer
scores zero however well it is packed. Any estimate of what is achievable here
has to assume that cap.

**One rejected placement ends the episode** (found 2026-10-10, explains most of
the variance). `env.py:216-236` sets `terminated = True` on *any* of the three
process checks failing - `check_inclusion`, `check_transport_path`, or
`place_item` reporting an unsafe settle. Nothing is salvaged after that. This is
why the platform status reads `Stopped in the middle. Did not satisfy
{'is_placed_safe', 'is_included', 'is_valid'}`, and it is the reason a run that
places 68% of items locally can place 50% on the platform: a single mismatch
between `feasible()` and the validator, or one bad settle, costs everything
downstream.

It also explains why every *aggressive* idea in this file lost. The direct-entry
slide preference, keeping cargo off soft bags, mass-aware ordering and
most-constrained-first all buy density by accepting marginal placements, and a
marginal placement is exactly what fails the settle test and ends the run. On a
score that averages components, trading "one extra item if it works" for "the
whole remainder if it does not" is a losing trade.

Consequently the highest-value tuning left is not a ranking weight. It is
`_MIN_SUPPORT` and `_CENTRE_TOL` in `_stable()`, which are the agent's only
model of the settle test: the validator accepts 30cm of displacement and 45
degrees (`displacement_threshold`, `angle_displacement_threshold`) and runs 300
physics steps, while `_stable()` guesses at support fraction and centroid
offset. Too strict and we reject placements that would have settled fine; too
loose and we propose ones that do not. That pair, not the candidate ordering, is
what decides how far each run gets.

**Lookahead is a cliff, not a dial** (found 2026-10-10). The spec allows 3 to 40
observable bags (`configs/robust_config.json` R5-R7 added to cover it; before
this the suite topped out at 10). Same items, same container, `optimize` off,
only `look_ahead` varying:

| scene | look_ahead | placed | fill |
| --- | --- | --- | --- |
| R7 | 3 | 29.3% | 8.38 |
| R3 | 8 | 56.1% | 25.97 |
| R6 | 20 | **73.2%** | 30.17 |
| R5 | 40 | 34.1% | 4.50 |

A 44-point spread from lookahead alone. R5 ends at 14 items with the stack only
0.94m high, i.e. one failed placement early. Treat any single-config number in
this file as having that much noise on it, and prefer changes that lift the
*worst* case rather than the mean.

**Most-constrained-first ordering** (tried 2026-10-10, reverted). `_best_move`
already computes how many spots an item can occupy and threw the count away.
Preferring the item with the fewest is a textbook bin-packing heuristic and is
free to evaluate. Over the lookahead cliff above it costs more than it returns:
mean placed falls from 48.2% to 33.5%, with R3 56.1 -> 26.8 and R6 73.2 -> 43.9.
Again a density gain bought by accepting marginal placements. The knob survives
as `_W_MCF = 0` because the count is genuinely free and the idea may behave
differently once the settle model is right.

**The relaxed fallback was throwing the settle test away** (tried 2026-10-10,
kept). `scripts/why_stop.py` settled which check ends each run: across the nine
config suite **not one ends on the transport path** - six die on the settle
test, two on inclusion, and only R2 (after the fix) finishes at all. Raising
`_MIN_SUPPORT` alone did almost nothing because `_online`'s fallback calls
`_best_move(..., relax=True)`, which skips `_stable()` entirely and then ranks
by the normal key - which prefers low, deep, flush spots, i.e. exactly the
bridging spots that fall.

Ranked by how much of the underside is actually carried instead, and with
`_MIN_SUPPORT` at 0.7: mean placed 53.8% -> 55.4%, mean fill 18.1 -> 19.3,
R7 29.3% -> 46.3%, and **R2 places all 42 items with `is_placed_safe: True`**,
which is the condition the official reference agent meets. R6 regresses
73.2% -> 61.0% and R4 is stubbornly unchanged at 26.2%, so this is progress
rather than a fix.

For calibration, the official `README.md` gives a reference result: fill 15.31,
cog 66.84, stability 72.07, placement 80.0, soft 100.0, `num_placed_items`
1.0, "The packing has been completed successfully" - a mean of 66.84. Our
fill of 29.5 is nearly double theirs, because fill is scene-dependent and the
reference scene is sparsely filled. Their placement 80 and soft 100 show both
components are readily attainable; they track how much you actually place.
The `README.md` also claims the fill containment test is looser than the
validator's, but the shipped `env.py:51` passes the validator's own
`inclusion_margin` (-0.005) to the evaluator, so locally they are the same.
