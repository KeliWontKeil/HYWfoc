# Codex Workspace Guide

This file is the minimal entry point for Codex work in this repository. It does
not replace the project documents; it points to the current sources of truth and
records the local build path that is easy to forget.

## Read First

1. `AI_INITIALIZATION.md`
2. `docs/development.md`
3. `docs/architecture.md`
4. `NEXT_MISSION.md`
5. `.github/AGENTS.md` when a task needs a specialist review role

If documents disagree with code, use the code as the immediate truth and update
the affected document in the same change.

## Project Shape

- `foc_core/` is the platform-independent FOC library.
- `examples/GD32F303_FOCExplore/` is the current board instance.
- The root workspace is for repository management and documentation.
- The instance workspace under `examples/GD32F303_FOCExplore/software/` is the
  build, flash, and debug entry.

Keep the dependency direction intact: `LS -> L1 -> L2 -> L3 -> L5`.
`L1/L2/L3` may reach hardware only through `L3/foc_platform_api`.

## Build

Use the helper script instead of hard-coding the installed EIDE extension
version:

```powershell
.\tools\build_gd32f303.ps1
```

The script locates the latest installed `cl.eide-*` extension, patches a
temporary `builder.params` copy when the generated file references an old EIDE
model path, and runs `unify_builder.exe --rebuild`.

Acceptance remains: 0 errors, no newly introduced warnings, ROM <= 256 KB,
RAM <= 96 KB.

## Documentation Hygiene

The current documentation set has version drift. Treat `CHANGELOG.md` and
`README.md` as the best baseline indicators for release state, but verify any
behavioral claim against code before editing.

Avoid adding parallel fact-source documents. Prefer updating:

- `docs/architecture.md` for structure and dependency changes.
- `docs/development.md` for workflow changes.
- `docs/protocol-parameters-bilingual.md` for protocol and runtime parameter
  changes.
- `NEXT_MISSION.md` for active milestones.
