# Documentation Structure Guide

This file serves as a master index for the FOC project documentation. All documents are organized by category to avoid scattering and ensure consistency.

---

## 📁 Documentation Hierarchy

```
ProjectRoot/
├── copilot-instructions.md          # 🔴 [CRITICAL] AI instruction manifest - DO NOT MOVE
├── AI_INITIALIZATION.md             # 🤖 AI context & framework setup - DO NOT MOVE (Copilot discovery)
├── NEXT_MISSION.md                  # 📅 Current project phase & goals
├── CHANGELOG.md                     # 📝 Version history & releases
│
├── .github/
│   ├── AGENTS.md                    # 🤖 Agent orchestration & roles
│   ├── WORKFLOW_CHECKLIST.md        # ✅ Pre-code/build/commit checklists
│   ├── DOCUMENTATION_STRUCTURE.md   # 📑 This file - documentation index
│   ├── agents/
│   │   ├── primary-developer.agent.md
│   │   ├── foc-algorithm-review.agent.md
│   │   ├── architecture-review.agent.md
│   │   └── documentation-compliance.agent.md
│   └── prompts/
│       ├── led-state-visualization.prompt.md
│       ├── config-macro-consolidation.prompt.md
│       └── header-decoupling.prompt.md
│
├── docs/
│   ├── README.md                    # 📘 Project quick start & overview
│   ├── architecture.md              # 🏗️ System design & layering
│   ├── development.md               # 👨‍💻 Development workflow & procedures
│   ├── engineering/
│   │   └── dev-guidelines/
│   │       └── rules/               # 📏 Coding standards (EN/CN)
│   ├── protocol-parameters-bilingual.md   # 📡 Protocol reference (EN/CN)
│   └── structure-and-dependency-tree.md   # 🔗 Include graph & data structures
│
└── examples/
    └── GD32F303_FOCExplore/
        └── hardware/
            └── hardware.md          # 🔌 Hardware config & pin mappings
```

---

## 📋 Document Categories & Purpose

### Category 1: Critical Configuration (ProjectRoot)

| Document | Purpose | Audience | Update Frequency | Notes |
|----------|---------|----------|------------------|-------|
| **copilot-instructions.md** | Master AI instruction manifest. Defines project conventions, layering, naming, build/test procedures. **Do NOT move.** | AI Agents, Developers | Per release or structural change | Essential for AI Agent initialization |
| **AI_INITIALIZATION.md** | AI context & framework state. Provides AI Agents with project setup information. **Do NOT move.** | AI Agents, New Developers | Per framework change | Copilot discovery mechanism |
| **NEXT_MISSION.md** | Current sprint/phase breakdown. Contains active goals, priorities, deliverables. | Developers, AI Agents | Per sprint (weekly) | Source of truth for phase terminology |

### Category 2: Workflow & Process (.github/)

| Document | Purpose | Audience | Update Frequency |
|----------|---------|----------|------------------|
| **AGENTS.md** | Agent roles, responsibilities, trigger conditions, handoff rules | Developers, AI Agents | Per agent update |
| **WORKFLOW_CHECKLIST.md** | Pre-code/build/commit verification checklists | Developers | Per phase change |
| **DOCUMENTATION_STRUCTURE.md** | This file; master index of all documentation | Developers, AI Agents | As structure changes |
| **agents/*.agent.md** | Individual agent role definitions | AI Agents | Per agent refinement |
| **prompts/*.prompt.md** | Task-specific guidance prompts (named generically, phase-independent) | Developers, AI Agents | Per task update |

### Category 3: Technical Documentation (docs/)

| Document | Purpose | Audience | Update Frequency |
|----------|---------|----------|------------------|
| **README.md** | Project overview, features, quick start | New developers, stakeholders | Per release |
| **architecture.md** | Layering contract, timing, data flow, module responsibilities | Developers, AI Agents | Per architectural change |
| **development.md** | Build procedures, workflows, constraints, rules | Developers | Per process change |
| **examples/GD32F303_FOCExplore/hardware/hardware.md** | Pin assignments, peripherals, clock config, constraints | Hardware engineers, developers | Per hardware change |
| **protocol-parameters-bilingual.md** | Protocol specification, commands, parameters | Developers, integrators | Per protocol update |
| **structure-and-dependency-tree.md** | Include graph, type catalog, dependency analysis | Developers | Per structural change |

### Category 4: Development Guidelines (docs/engineering/dev-guidelines/rules/)

| Document Set | Purpose | Languages |
|---|---|---|
| **en/** | Coding standards, best practices, Anti-patterns (English) | English |
| **cn/** | Coding standards, best practices, Anti-patterns (Chinese) | Chinese (Simplified) |

---

## 🔗 Key Links & Cross-References

### For New Developers
1. Start here: [docs/README.md](../docs/README.md)
2. Then read: [copilot-instructions.md](../copilot-instructions.md)
3. Check current work: [NEXT_MISSION.md](../NEXT_MISSION.md)
4. Review rules: [docs/engineering/dev-guidelines/rules/en/](../docs/engineering/dev-guidelines/rules/en/) or [cn/](../docs/engineering/dev-guidelines/rules/cn/)

### For AI Agents
1. Agent definitions: [.github/AGENTS.md](.github/AGENTS.md)
2. Master instructions: [copilot-instructions.md](../copilot-instructions.md)
3. Current goals: [NEXT_MISSION.md](../NEXT_MISSION.md)
4. Checklists: [.github/WORKFLOW_CHECKLIST.md](.github/WORKFLOW_CHECKLIST.md)

### For Workflow Execution
1. Identify task: [NEXT_MISSION.md](../NEXT_MISSION.md)
2. Choose agents: [.github/AGENTS.md](.github/AGENTS.md)
3. Get task prompt: [.github/prompts/](./prompts/)
4. Follow checklist: [.github/WORKFLOW_CHECKLIST.md](.github/WORKFLOW_CHECKLIST.md)

---

## 📌 Important Principles

### ✅ DO
- Update documentation **synchronously** with code changes
- Keep phase/goal descriptions in **NEXT_MISSION.md**, not scattered in task prompts
- Refer to NEXT_MISSION.md when phase terminology changes
- Document temporary reports **inline** with work, then delete before next task

### ❌ DON'T
- Hardcode phase identifiers (P1.1, P1.3, etc.) in workflow documents
- Leave temporary test/work reports in the repository
- Scatter the same documentation across multiple locations (use links instead)
- Move `copilot-instructions.md` from project root
- Create documentation without updating this structure guide

---

## 🔄 Documentation Maintenance Tasks

### When updating documentation:
- [ ] Update this index if adding new documents
- [ ] Mark outdated content with deprecation notices
- [ ] Update CHANGELOG.md with documentation changes
- [ ] Ensure cross-document links are still valid
- [ ] Remove temporary reports/test files before committing

### When starting a new phase/sprint:
- [ ] Update NEXT_MISSION.md with new goals
- [ ] Review and update task prompts if phase structure changes
- [ ] Verify AGENTS.md still applies to new work
- [ ] Update WORKFLOW_CHECKLIST.md if processes change
- [ ] Delete any accumulated test/work reports

---

## 📊 Documentation Status

| Category | Status | Last Updated |
|----------|--------|---|
| Workflow & Process | ✅ Complete | 2026-03-31 |
| Technical Docs | ✅ Current | 2026-03-31 |
| Development Guidelines | ✅ Complete | (see dev-guidelines/) |
| Agent Definitions | ✅ Complete | 2026-03-31 |
| Task Prompts | ✅ Complete (generic naming) | 2026-03-31 |

---

**Last Updated**: 2026-03-31  
**Version**: 1.0  
**Audience**: Developers, AI Agents, Project Maintainers
