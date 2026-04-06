# Agent Orchestration Guide

This file defines the FOC project's agent ecosystem, their responsibilities, and **when to invoke them**.

---

## 🎯 Agent Overview & Trigger Conditions

### 1. **Primary Developer** (`primary-developer.agent.md`)

**Scope**: All routine development tasks across all layers

**When to invoke**:
- ✅ Feature implementation ("Add X-axis current limit")
- ✅ Bug fixes ("Fix PWM glitch at startup")
- ✅ Code refactoring (within single layer, e.g., CamelCase → snake_case)
- ✅ Configuration updates (macros, defaults)
- ✅ General task completion with build/test verification

**Handoff criteria**:
- 🔀 If changes involve **control algorithm behavior** → handoff to `foc-algorithm-review`
- 🔀 If changes involve **architectural coupling/dependencies** → handoff to `architecture-review`
- 🔀 If changes need **documentation alignment** → handoff to `documentation-compliance`

**Key responsibility**: End-to-end workflow (code → build → verify zero warnings)

---

### 2. **FOC Algorithm Specialist** (`foc-algorithm-review.agent.md`)

**Scope**: Algorithm correctness, control phenomena diagnosis, FOC math validation

**When to invoke**:
- ✅ Speed/torque loop oscillates or is sluggish
- ✅ Startup behavior unexpected (ramp-up, calibration)
- ✅ PID tuning or state-machine changes
- ✅ Sensor delay or timing assumptions change
- ✅ New control path added (position loop, Field-Weakening, etc.)
- ✅ 1ms period assumption needs verification

**File patterns (auto-suggest if editing)**:
- `Application/Source/foc_control*.c`
- `Application/Include/foc_config_control.h`
- Math or timing changes in `math_transforms.*`

**Key responsibility**: Ensure algorithm respects 1ms control period and L1/L2/L3→L4 contract

**Handoff criteria**:
- 🔀 If implementation needs structural refactoring → handoff to `architecture-review`
- 🔀 If build fails or needs general code cleanup → handoff to `primary-developer`

---

### 3. **Architecture & Structure Specialist** (`architecture-review.agent.md`)

**Scope**: Dependency management, layer enforcement, file organization, decoupling

**When to invoke**:
- ✅ Possible layer-boundary violations detected ("L2 including L4 device header?")
- ✅ Circular dependencies or tight coupling
- ✅ Scattered config macros across implementation files
- ✅ File organization refactoring (move drivers, consolidate headers)
- ✅ Forward-declaration opportunities or header reduction
- ✅ API contract violations between layers

**File patterns (auto-suggest if editing)**:
- `Application/Include/` headers
- `Application/Source/foc_platform_api.*`
- `Utilities/` driver organization
- `foc_config*.h` structure

**Key responsibility**: Maintain strict L1→L2→L3→L4 layering and minimize coupling

**Handoff criteria**:
- 🔀 If implementation of approved refactoring is needed → handoff to `primary-developer`
- 🔀 If algorithm logic changes simultaneously → handoff to `foc-algorithm-review`

---

### 4. **Documentation Compliance Specialist** (`documentation-compliance.agent.md`)

**Scope**: Documentation accuracy, code-docs alignment, drift detection

**When to invoke**:
- ✅ Code changed significantly; docs may be outdated
- ✅ API or config signature changed
- ✅ New files added (need docs entry)
- ✅ Architecture diagrams need updating
- ✅ README or NEXT_MISSION.md drifting from reality
- ✅ Doxygen comments missing or conflicting

**File patterns (auto-suggest if editing)**:
- `docs/` (any markdown changes)
- `NEXT_MISSION.md` or version tracking
- Header files with Doxygen blocks
- README or architecture diagrams

**Key responsibility**: Ensure documentation reflects implementation; detect and resolve drift

**Handoff criteria**:
- 🔀 If code changes are needed to match docs → handoff to `primary-developer` or task-specific agent

---

## 🔀 Agent Handoff Model

```
┌─────────────────────────────────────────────────────────┐
│                   User Request                          │
└────────────────────┬────────────────────────────────────┘
                     │
        ┌────────────┴────────────┐
        │ What is the nature?     │
        └────────────┬────────────┘
        
    ┌───┬────────┬────────────┬─────────────┐
    │   │        │            │             │
    v   v        v            v             v
 Algo  Struct   Docs       General    Multi-Check
 Issue  Issue   Issue      Code Task  Needed
    │   │        │            │             │
    │   │        │            │        Call Primary
    │   │        │            │        + Others
    │   │        │            │             │
    └─┬─┴───┬────┴─────────────┴─────────┬──┘
      │     │                            │
      v     v                            v
  FOC-Algo Arch-Review            PRIMARY-DEVELOPER
  Specialist (+ potential           (or task-specific)
             handoff to
             Primary or Docs)

Next → Verify build → Check docs alignment → Complete
```

---

## 📋 Common Workflows & Agent Chains

### Workflow 1: Add New Peripheral (e.g., CAN driver)
1. **Primary Developer** (or user) → outline plan
2. **Architecture Review** → validate placement in Utilities, check API contract
3. **Primary Developer** → implement driver
4. **Primary Developer** → integrate into `foc_platform_api`
5. **Documentation Compliance** → update `docs/hardware.md`
6. **Primary Developer** → final compile + test ✅

### Workflow 2: Fix Speed Loop Oscillation
1. **FOC Algorithm Specialist** → diagnose root cause (PID tuning, timing, saturation?)
2. **FOC Algorithm Specialist** → propose config macro changes
3. **Primary Developer** → implement and verify (if simple)  OR  specialist does it
4. **Documentation Compliance** → update NEXT_MISSION.md or docs if tuning constants change
5. **Primary Developer** → final compile + hardware test ✅

### Workflow 3: P1.3 Config Consolidation (Task-specific)
1. **Architecture Review** → scan for scattered defaults, create consolidation map
2. **Primary Developer** → move macros to `foc_config_*.h`, update includes
3. **Architecture Review** → verify no layer violations in new structure
4. **Primary Developer** → compile + verify zero warnings ✅
5. **Documentation Compliance** → update `docs/engineering/dev-guidelines/rules/` if new patterns established

### Workflow 4: P1.1 LED Visualization
1. **Primary Developer** (with LED-specific prompt) → implement state machine
2. **Architecture Review** → ensure LED driver properly isolated in L4
3. **Primary Developer** → compile + test on hardware
4. **Documentation Compliance** → add LED state table to `docs/README.md`  ✅

---

## 🔗 Related Files

- **Main instructions**: [copilot-instructions.md](../copilot-instructions.md)
- **Agent definitions**: `.github/agents/*.agent.md`
- **Project phases**: [NEXT_MISSION.md](../../NEXT_MISSION.md)
- **Build rules**: Tasks in `.vscode/tasks.json` or EIDE config
- **Code conventions**: [docs/engineering/dev-guidelines/rules/](../../docs/engineering/dev-guidelines/rules/)

---

## 📝 Checklist for Developers

Before starting a task, ask:

- [ ] **Is this primarily algorithm work?** → Consider FOC Algorithm Specialist
- [ ] **Is this primarily structural?** → Consider Architecture Review  
- [ ] **Is this primarily doc updates?** → Consider Documentation Compliance
- [ ] **Is this mixed or unclear?** → Start with Primary Developer (they'll coordinate)
- [ ] **Will this change build behavior or API?** → Plan for doc sync
- [ ] **Am I modifying L2 core algorithm?** → Verify 1ms period still valid

---

## 🚀 Invoking Agents Via Copilot Chat

**Option 1: Direct invocation**
```
@architecture-review: Review current layer coupling and suggest decoupling opportunities
```

**Option 2: Via Primary Developer delegation**
```
Add CAN driver support. When the structural changes are ready, delegate to @architecture-review for review.
```

**Option 3: Task-specific prompts**
```
Use the P1.1-LED-Visualization prompt to implement the state machine.
```

---

## 📌 Future Enhancements

- Add `applyTo` rules to agent YAML frontmatter for auto-suggestion
- Define pre-commit hooks for zero-warning verification
- Add P2 (library) phase workflow coordination
- Document multi-agent parallel workflows (e.g., P1.4 header refactor + algorithm tuning)
