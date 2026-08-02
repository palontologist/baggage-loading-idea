# React Native Mobile Client — Agent Instructions

## Status

The React Native port is in early development. It is a vertical slice that mirrors the existing Flutter app's core navigation and dataflow. The RN app is **not yet feature-complete** and should not be treated as a production release.

## Backend / Auth Assumptions

- The RN app communicates with the same backend as the Flutter app.
- Authentication tokens are passed via secure storage (react-native-keychain).
- All API calls go through a backend-side safety contract that validates and sanitizes incoming requests.
- The backend enforces rate limiting and content safety on all agent-interaction endpoints.

## Build & Validation

- Type checking: `npx tsc --noEmit`
- Linting: `npx eslint .`
- Formatting: `npx prettier --check .`
- Build: `npx react-native build` (requires Android SDK or Xcode)

## Contributor Notes

- Before merging changes that affect the AI Interjection flow, verify that the prompt/data boundary and backend safety contract are intact.
- The cooldown timer for AI Interjection must be enforced on both the client and backend sides.
- All new dependencies must pass typecheck and build validation before merge.