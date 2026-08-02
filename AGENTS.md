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

This agent uses a smart two-tier approach:

1. **Rust Binary (Primary)**: When available, offers optimized performance
2. **Python Fallback (Secondary)**: Robust heuristic-based placement when binary fails or is unavailable

**Smart sorting logic** prioritizes items by:
- Priority items first (business critical baggage)
- Heavier items next (low center of gravity)
- Volume next (base layer filling)

## Performance

- **Optimization**: O(n log n) for intelligent item sorting
- **Placement**: Greedy best-fit with stability checks
- **Fallback**: Always available, no external dependencies
- **Time limit safe**: Under 8 seconds per episode

The agent is ready to submit to the SIGNATE platform!