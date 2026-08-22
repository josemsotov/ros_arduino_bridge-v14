# GitHub Copilot Instructions

## Workspace context priority

- For structural, architectural, cross-file, or unfamiliar workspace questions, **reference and read `CODEBASE_MEMORY.md` first using workspace context**.
- Use that memory as the primary navigation map to select the smallest relevant set of active source files. Confirm implementation details in source before proposing or applying code.
- If active code and the memory disagree, follow active code, call out the mismatch, and recommend a focused update to `CODEBASE_MEMORY.md`.

## Token-efficient exploration

- Do not perform broad recursive reads unless targeted investigation cannot answer the request.
- Search for filenames, symbols, definitions, and call sites first; read only relevant ranges and direct dependencies.
- Prefer root V14 sources. Ignore `build_output/`, `BACKUPS/`, `MOTOR-INTERFACE-V-13/`, `.venv/`, `__pycache__/`, and `.codex_*/` unless the user explicitly places them in scope.
- Avoid reopening unchanged files and avoid reproducing large code blocks when a concise reference or patch is sufficient.
- For a local change, inspect the target, its direct callers/callees, applicable feature flags, pins/configuration, and the narrowest relevant tests.

## Codebase alignment

- Follow the architecture and development standards in `CODEBASE_MEMORY.md`.
- Preserve Arduino Mega 2560/AVR resource constraints, compile-time feature flags, header guards, and the dependency-sensitive include order in `Modules.h`.
- Keep the superloop non-blocking, validate serial input, constrain actuator values, retain timeout/emergency-stop behavior, and avoid unsafe changes to Timer5 PWM handling.
- Match established naming: preprocessor constants in `UPPER_SNAKE_CASE`; subsystem-prefixed or `camelCase` Arduino symbols; `PascalCase` Python classes and `snake_case` Python functions/variables.
- Treat generated binaries, backups, historical snapshots, virtual environments, and tool artifacts as non-source.
- When a change materially alters architecture, entry points, module dependencies, hardware boundaries, or communication flow, recommend updating `CODEBASE_MEMORY.md` in the same change.
