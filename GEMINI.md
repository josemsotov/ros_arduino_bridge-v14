# Gemini Workspace Instructions

## Mandatory context-first workflow

1. For every architectural, structural, cross-module, or unfamiliar codebase request, **read `CODEBASE_MEMORY.md` first, before scanning multiple workspace files**.
2. Treat `CODEBASE_MEMORY.md` as the navigation map, not as a substitute for source verification. Use it to identify the smallest set of authoritative files, then verify only the details relevant to the task.
3. If the memory conflicts with active source code, the active source wins. Mention the discrepancy and propose a focused memory correction.

## Context and token discipline

- Do not recursively read the whole workspace by default. Start with targeted filename search and symbol/text search.
- Open only the relevant sections of files identified by `CODEBASE_MEMORY.md`; expand scope incrementally when evidence requires it.
- Exclude generated, historical, backup, environment, and tool-artifact paths unless explicitly requested: `build_output/`, `BACKUPS/`, `MOTOR-INTERFACE-V-13/`, `.venv/`, `__pycache__/`, and `.codex_*/`.
- Prefer active root firmware files over similarly named historical copies.
- Do not repeatedly reread unchanged files. Reuse already established facts and summarize large findings instead of pasting source.
- For localized changes, inspect the target file, its direct callers/dependencies, and relevant configuration or tests only.
- Use search results to locate definitions and references before opening long headers or Python scripts.
- Never infer that the repository contains a full ROS2 application: validate boundaries described in `CODEBASE_MEMORY.md`.

## Implementation alignment

- Follow the architecture, dependency order, safety constraints, naming conventions, and generated-file boundaries documented in `CODEBASE_MEMORY.md`.
- Preserve Arduino Mega 2560 constraints, compile-time feature flags, the `Modules.h` include order, non-blocking loop behavior, serial validation, and motor fail-safe behavior.
- Verify changes with the narrowest relevant compile, test, or diagnostic command available. Do not modify backups or generated binaries as part of a source change.

## Memory maintenance

When the user introduces or requests a major architectural change—such as a new subsystem, communication path, entry point, directory boundary, hardware abstraction, or cross-module dependency—**explicitly suggest updating `CODEBASE_MEMORY.md`**. If editing documentation is within the requested scope, include that update; otherwise ask the user whether they want the map refreshed.

