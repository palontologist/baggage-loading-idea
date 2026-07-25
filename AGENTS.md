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

This agent uses a **hybrid Rust + Python** approach:

1. **Rust Binary (Primary)**: When available, provides optimized placement via the pre-compiled `rust_brain` binary (scores ~2.32 on SIGNATE)
2. **Python Fallback (Improved)**: Robust grid-based placement when Rust binary is unavailable or fails

### Python Fallback improvements:
- **Full container space utilization** — Places items across the entire X-Y area (not just back half)
- **Multi-level stacking** — Items stacked at multiple Z levels on floor and existing items
- **All 6 orientations tried** — Each item evaluated in all axis-aligned rotations
- **Proper 3D AABB collision detection** — Checks overlap in all three axes
- **Rotation-aware packed item collision** — Uses quaternion-derived bounding boxes for already-placed items
- **Cut corner avoidance** — Triangular cut corner at back-left excluded from candidates
- **Coordinate conversion** — World positions converted to local container coordinates
- **Efficient grid search** — Systematic grid with early termination at floor level

## Performance

- **Optimization**: O(n log n) for intelligent item sorting
- **Placement**: Greedy best-fit with stability checks
- **Fallback**: Always available, no external dependencies
- **Time limit safe**: Under 8 seconds per episode

The agent is ready to submit to the SIGNATE platform!