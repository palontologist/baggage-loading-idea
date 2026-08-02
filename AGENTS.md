# Agent Submission Instructions for Baggage Loading Challenge

## Quick Push

```bash
cd /home/palontologist/Downloads/dev/baggage-loading-idea-clean
git add submit.zip AGENTS.md
git commit -m "feat: create submission.zip for SIGNA81211"
git push origin main
```

## Files Included

- `agent.py`: Hybrid Python + Rust agent with fallback logic
- `rust_brain_wrapper.sh`: Executable wrapper for Rust binary
- `rust_brain`: Compiled Rust binary (included for compatibility)
- `requirements.txt`: Python dependencies (numpy >= 1.20)
- `mypy.ini`: Type checking configuration
- `pyproject.toml`: Build and lint configuration
- `CLAUDE.md`: Agent instructions for this subtree
- `app/react-native/AGENTS.md`: Agent instructions for the React Native mobile client subtree
- `app/react-native/CLAUDE.md`: Agent guidance for the React Native mobile client subtree

## Security & Safety

- All data sent to the Rust binary passes through `_sanitize_request()` and item/container validation (`_validate_item`, `_validate_container`) before being written to stdin
- A rate limiter enforces a 0.5s cooldown between Rust binary calls to prevent backend abuse

## Testing

Run type checking:

```bash
cd submit
mypy agent.py --config-file mypy.ini
```

Run the simulator locally (if dependencies are installed):

```bash
cd simulator/simulator
python scripts/run_test.py \
  --module-path ../baggage-loading-idea-clean/submit \
  --config-path configs/sample_config.json
```

## Agent Strategy

This agent uses a **greedy grid-based placement** approach with several key improvements over a naive baseline:

1. **Full container space utilization** — Places items across the entire X-Y area of the container (not just the back half), avoiding the cut corner region.
2. **Multi-level stacking** — Items are placed on the floor first, then stacked on top of previously placed items at multiple Z levels.
3. **All 6 orientations tried** — Each item is evaluated in all 6 possible axis-aligned orientations to find the best fit.
4. **Proper 3D AABB collision detection** — Checks overlap in all three axes (XY and Z), unlike simpler implementations that only check XY.
5. **Cut corner avoidance** — The triangular cut corner at the back-left of the container is excluded from placement candidates.
6. **Coordinate conversion** — Packed item positions (in world coordinates) are converted to local container coordinates for accurate collision checking.
7. **Efficient grid search** — Systematic grid with early termination when floor-level placement is found.

## Performance

- **Optimization**: O(n log n) for intelligent item sorting
- **Placement**: Greedy best-fit with stability checks
- **Fallback**: Always available, no external dependencies
- **Time limit safe**: Under 8 seconds per episode

The agent is ready to submit to the SIGNATE platform!