# Claude Agent Guidance — app/react-native/

## Overview

This directory contains the React Native mobile client port. It is a work-in-progress vertical slice that mirrors the existing Flutter app.

## Key Files

- `App.tsx` — Entry point and navigation
- `screens/` — Screen components
- `services/` — API and backend communication
- `types/` — TypeScript type definitions

## Backend / Auth Assumptions

- The RN app assumes a backend that enforces a safety contract on all agent-interaction requests.
- Authentication is handled via secure token storage; tokens are refreshed automatically.
- The backend validates and sanitizes all incoming data before processing.

## Safety & Privacy

- The AI Interjection flow sends transcript data to the agent backend. A clear prompt/data boundary is enforced: transcript text is wrapped in a structured envelope with metadata (timestamp, speaker ID, session token) before being sent. The backend validates the envelope schema and rejects malformed or unsanitized input.
- The cooldown timer is enforced on both the client and backend sides to prevent repeated agent calls.

## Validation

- Run `npx tsc --noEmit` for type checking.
- Run `npx eslint .` for linting.
- Run `npx prettier --check .` for formatting.
- All checks must pass before merging.

## Testing on Device/Emulator

- Test on both Android emulator and iOS simulator.
- Verify that the AI Interjection cooldown prevents repeated triggers.
- Verify that the prompt/data boundary rejects malformed transcript input.