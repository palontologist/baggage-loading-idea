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

## Results (local simulator, sample_config.json)

| task | items placed | fill_score | optimize | policy |
| --- | --- | --- | --- | --- |
| 000 | 68.3% | 31.15 | 43 s | 0.16 s |
| 001 | 47.6% | 20.96 | - | 1.1 s |

Limits are 180 s for optimize and 8-10 s for policy, so there is headroom, but
`_OPT_BUDGET` / `_POLICY_BUDGET` in the agent keep it bounded regardless of input
size.

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
