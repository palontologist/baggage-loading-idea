# Agent Instructions for Baggage Loading Challenge

## Architecture

This agent uses a hybrid Python + Rust approach for the SIGNATE Baggage Loading optimization challenge.

- `agent.py`: Main agent with Rust binary primary path and Python heuristic fallback
- `rust_brain`: Compiled Rust binary for optimized planning
- `rust_brain_wrapper.sh`: Shell wrapper that launches the Rust binary

## Security Boundary

All data sent to the Rust binary is validated through `_sanitize_request()` and item/container validation functions (`_validate_item`, `_validate_container`) before being written to stdin. This ensures untrusted input cannot inject malformed payloads into the backend.

## Cooldown Enforcement

A rate limiter (`COOLDOWN_SECONDS = 0.5s`) is enforced in `_send_request()` using `time.monotonic()`. If a call arrives before the cooldown period has elapsed, the method sleeps for the remaining duration. This prevents rapid successive calls that could abuse the backend.

## Fallback Behavior

If the Rust binary fails to spawn or returns an invalid response, the agent falls back to a pure Python heuristic:
- **Optimize**: Sorts items by priority (desc), mass (desc), volume (desc)
- **Policy**: Places items at position (0.5, 0.5, 0.5) in container 0

## Dependencies

- Python 3.9+
- numpy >= 1.20

## Testing

Run type checking with mypy:

```bash
cd submit
mypy agent.py --config-file mypy.ini
```

Run the agent locally (requires simulator):

```bash
python submit/agent.py
```