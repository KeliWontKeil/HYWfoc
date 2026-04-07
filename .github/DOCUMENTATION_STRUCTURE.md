# Documentation Structure Guide

This file is the master map of documentation ownership for this repository.

## Repository Documentation Ownership

| Scope | Path | Responsibility |
|---|---|---|
| Project entry and licensing | `README.md`, `LICENSE`, `CHANGELOG.md`, `NEXT_MISSION.md` | Project overview, legal license, release history, and active mission planning |
| AI/global governance | `copilot-instructions.md`, `AI_INITIALIZATION.md` | Always-on AI behavior, workflow conventions, and project context |
| Workflow orchestration | `.github/AGENTS.md`, `.github/WORKFLOW_CHECKLIST.md`, `.github/agents/*.agent.md`, `.github/prompts/*.prompt.md` | Agent roles, execution checklist, and reusable task prompts |
| Library technical docs | `docs/*.md` | Architecture, dependency tree, development process, protocol reference, migration/evaluation reports |
| Board/instance docs | `examples/<instance>/README.md`, `examples/<instance>/hardware/*.md`, `examples/<instance>/software/README.md` | Instance-specific hardware notes and standalone software package description |

## Current Structure Snapshot

```text
ProjectRoot/
├── README.md
├── LICENSE
├── CHANGELOG.md
├── NEXT_MISSION.md
├── copilot-instructions.md
├── AI_INITIALIZATION.md
├── .github/
│   ├── AGENTS.md
│   ├── WORKFLOW_CHECKLIST.md
│   ├── DOCUMENTATION_STRUCTURE.md
│   ├── agents/*.agent.md
│   └── prompts/*.prompt.md
├── docs/
│   ├── README.md
│   ├── architecture.md
│   ├── development.md
│   ├── structure-and-dependency-tree.md
│   ├── protocol-parameters-bilingual.md
│   ├── api-unused-interface-evaluation.md
│   └── library-structuring-p1.md
└── examples/GD32F303_FOCExplore/
    ├── README.md
    ├── hardware/
    │   ├── README.md
    │   └── hardware.md
    └── software/
        ├── README.md
        └── Project.code-workspace
```

## Update Rules

1. Keep one source of truth per topic; use links instead of duplicated prose.
2. Root `Project.code-workspace` is management-only; build/flash/debug guidance belongs to instance software docs.
3. Any architecture, interface, timing, or config-contract change must update `docs/` in the same iteration.
4. Any board pin/peripheral change must update `examples/GD32F303_FOCExplore/hardware/hardware.md`.
5. Any workflow/agent behavior change must update `.github` docs and `copilot-instructions.md` together.

## Reading Order

1. `README.md`
2. `docs/README.md`
3. `docs/architecture.md`
4. `docs/development.md`
5. `examples/GD32F303_FOCExplore/hardware/hardware.md`

Last Updated: 2026-04-06
