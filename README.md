# Baggage Loading Challenge - SIGNATE

Hybrid Python/Rust agent for the SIGNATE Baggage Loading optimization challenge.

## Submission Files

- `submit.zip` - Ready-to-submit package for SIGNATE
- `submit/` - Agent source files (extracted from submit.zip)
  - `agent.py` - Main Python agent with Rust fallback
  - `rust_brain` - Compiled Rust binary (GLIBC incompatible, fallback to Python)
  - `rust_brain_wrapper.sh` - Binary launcher

## Architecture

The agent uses a **hybrid approach**:
1. Try fast Rust binary for planning
2. Fall back to pure Python if Rust binary fails (GLIBC/system incompatibility)

## Usage

```bash
# Submit to SIGNATE
# Upload submit.zip to https://signate.jp/

# Run locally (requires simulator)
python submit/agent.py
```

## Status

- ✅ Ready for submission (submit.zip)
- ⚠️ Rust binary has GLIBC compatibility issues on some systems
- ✅ Python fallback ensures compatibility
