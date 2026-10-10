# Agent Instructions for Baggage Loading Challenge

## Architecture

`agent.py` is the whole agent: pure Python plus NumPy, single file, no compiled
helper and no subprocess. Everything it needs is derived from the observation
dict the platform provides.

## How it works

The agent reproduces the simulator's own placement rules rather than
approximating them, because a single rejected placement ends the episode and
zeroes everything after it. See the repository `AGENTS.md` for the full
explanation; the short version:

- Interior geometry and the containment test come from `n_vecs` / `points`, the
  exact planes the simulator tests against.
- `_best_moves` scores every resting spot of an item with a cheap height map,
  then checks the survivors against a vectorised copy of
  `PlacementValidator.check_transport_path`.
- `_stable` requires real support under the middle of a box, because an item
  released in mid air falls the full height of the container and fails the settle
  test.
- `optimize()` returns only an item ordering. `policy()` recomputes every
  position from the live settled state.

## Dependencies

- Python 3.10+
- numpy >= 1.20 (uses `numpy.lib.stride_tricks.sliding_window_view`)

## Testing

Run type checking with mypy:

```bash
cd submit
mypy agent.py --config-file mypy.ini
```

Run the agent locally (requires the simulator):

```bash
cd simulator && source .venv/bin/activate
ln -s ../../submit agents/submit
PYTHONPATH=. python -m scripts.run_test \
  --module-path agents/submit/ \
  --config-path configs/sample_config.json
```
