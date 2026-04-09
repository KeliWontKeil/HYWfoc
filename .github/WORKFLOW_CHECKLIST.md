# Copilot Workflow Checklist & Guidelines

This document provides quick reference checklists for common development workflows on the FOC project. Follow these before submitting changes.

---

## 📋 Pre-Coding Checklist

Before starting any task, verify:

- [ ] **Read NEXT_MISSION.md**: Understand current phase and priorities
- [ ] **Review copilot-instructions.md**: Refresh on project conventions
- [ ] **Check your task scope**: Locate it in NEXT_MISSION.md for context and goals
- [ ] **Identify required agents**: Use `.github/AGENTS.md` to determine who to involve
- [ ] **Open the correct workspace**: use `examples/GD32F303_FOCExplore/software/Project.code-workspace` for build/flash/debug
- [ ] **Verify build is clean**: `eide.project.build` should show 0 errors and no newly introduced warnings

---

## 🔨 Coding Checklist

As you implement:

- [ ] **Follow naming conventions**: `Module_FunctionName()`, `camelCase` vars, `UPPER_CASE` macros
- [ ] **Add config macros to `foc_cfg_*.h`**: Never hardcode tuning constants in `.c`
- [ ] **Respect layer boundaries**: 
   - L1/L2/L3 access L4 **only** via `interface/foc_platform_api.h`
  - Do NOT include `gd32f30x_*.h` in public headers
   - Use `config/foc_shared_types.h` for cross-layer types
- [ ] **Document via Doxygen**: Add `/** @brief ... */` comments to new public functions
- [ ] **Include safety notes**: Comment any ISR assumptions, timing constraints, or memory limits

---

## ✅ Pre-Build Verification

Before running `eide.project.build`:

- [ ] **No syntax errors**: IDE shows no red squiggles
- [ ] **Includes are correct**: No circular dependency warnings
- [ ] **Macro definitions are unique**: Each config macro defined exactly once
- [ ] **Extern declarations clean**: Functions declared in headers before use

---

## 🏗️ Build & Verification Workflow

### Step 1: Compile
```bash
# EIDE task: "build" (or Ctrl+Shift+B)
eide.project.build
```

**Expected output**:
```
Finished: 0 error messages, and no newly introduced warnings
```

**If warnings appear**:
- [ ] Check warning type (unused variable, implicit cast, etc.)
- [ ] Fix at source (don't suppress)
- [ ] Re-run build until clean

### Step 2: Verify Memory Budget
```
ROM:   35.5KB / 256KB (13.9%) ✅
RAM:   2.5KB / 96KB (2.6%)   ✅
```

**Action if exceeded**:
- [ ] Run `eide.project.rebuild` (full rebuild, may show different results)
- [ ] Compare ROM/RAM before and after your changes
- [ ] If increase > 2KB ROM or > 512B RAM, justify or optimize

### Step 3: Validate Linker Output
```
No undefined references
No relocation errors
No duplicate symbols
```

**Action if errors**:
- [ ] Check all function declarations (`extern`)
- [ ] Verify platform API wrapped correctly
- [ ] Run `eide.project.rebuild` (hard reset)

### Step 4: Check Layer Boundaries (Optional, for structural changes)
```bash
# Ensure no L2/L3 headers expose L4 device headers
grep -r "^#include.*gd32f30x_" foc/include/
# Should return: nothing (or pre-approved exceptions)
```

---

## 📚 Documentation Sync Checklist

After code changes:

- [ ] **Updated relevant .md files**: architecture, hardware, development docs
- [ ] **Doxygen comments complete**: All new public APIs documented
- [ ] **NEXT_MISSION.md updated**: Reflect progress on phases
- [ ] **Checked for:** 
  - API signature changes? → Update examples/GD32F303_FOCExplore/hardware/hardware.md or development.md
   - New config macros? → Document in docs/development.md examples and note owning `foc/include/config/foc_cfg_*.h`
  - Timing assumptions changed? → Update docs/architecture.md
   - New peripheral added? → Update examples/GD32F303_FOCExplore/hardware/hardware.md with pin/config mapping

### Invoking Documentation Agent
```
@documentation-compliance: Check if my code changes align with project documentation.
Are there any new APIs, config changes, or timing assumptions that need doc updates?
```

---

## 🚀 Pre-Commit Checklist

**As a final gate before git commit:**

- [ ] **Code compiles**: `eide.project.build` → 0 errors, no newly introduced warnings
- [ ] **Memory in budget**: ROM < 256KB, RAM < 96KB (show measurements in commit message)
- [ ] **Layer boundaries intact**: No L2/L3 includes of L4 device headers
- [ ] **Tests passed**: Hardware validation completed (if applicable)
- [ ] **Documentation synced**: All files updated (run doc agent if uncertain)
- [ ] **Commit message clear**: Describes change scope and phase (e.g., "P1.3: Consolidate TORQUE_LIMIT macro")
- [ ] **No generated artifacts committed**: build/output/.hex and similar generated files stay ignored

### Post-Commit Governance

- [ ] **Local commit only**: Complete the cycle with local `git commit`
- [ ] **No push by default**: Do not run `git push` unless explicitly requested by the user
- [ ] **Revision increment**: Local revision part (`1.2.3` -> `.3`) increments by `+1` after each local commit
- [ ] **Commit correction path**: If current commit needs correction, use `git commit --amend` instead of creating a new standalone commit
- [ ] **Activation baseline awareness**: Governance is active for version baseline `>= 1.0.0`

---

## 🔄 Multi-Device Workflow (Multiple Agents)

Some tasks benefit from sequential agent invocations:

### Example: Modify FOC Algorithm
1. **You request**: "Reduce speed loop oscillation by 20%"
2. **Primary Developer** (or you): Initial diagnosis
3. **FOC Algorithm Specialist** (@foc-algorithm-review): Propose PID tuning changes
4. **Primary Developer**: Implement changes
5. **Documentation Compliance**: Verify docs still align
6. **You**: Final review + hardware test

### How to trigger multi-agent flow:
```
@primary-developer: Implement speed loop oscillation fix.
When ready for algorithm review, @foc-algorithm-review will validate correctness.
```

---

## 🐛 Bug Fix Workflow

When fixing a bug:

1. **Identify scope**:
   - [ ] Algorithm behavioral issue? → FOC Algorithm Specialist
   - [ ] Structural/coupling issue? → Architecture Review
   - [ ] General code issue? → Primary Developer
   - [ ] Unclear? → Primary Developer (they'll assess)

2. **Implement fix**:
   - [ ] Add root-cause comment in code
   - [ ] Update CHANGELOG.md with bug description
   - [ ] Test on hardware if control-path related

3. **Example prompt**:
   ```
   Bug: Speed ramp doesn't smooth on cold startup.
   Symptoms: [describe observable behavior]
   Suspected cause: [your hypothesis]
   
   @foc-algorithm-review: Diagnose and recommend fix.
   ```

---

## 🎯 Feature Implementation Workflow

Adding a new control feature (e.g., Field-Weakening):

1. **Design phase**:
   - [ ] Read NEXT_MISSION.md for feature priority
   - [ ] Outline algorithm changes + config macros
   - [ ] Request review: @architecture-review + @foc-algorithm-review

2. **Implementation phase**:
   - [ ] Implement in foc_control.c + new config header
   - [ ] Follow naming conventions + add Doxygen comments
   - [ ] Build + verify no newly introduced warnings

3. **Validation phase**:
   - [ ] Hardware test on actual motor
   - [ ] @documentation-compliance: Update docs
   - [ ] Final pre-commit check

---

## 📊 Task-Specific Prompts

Task prompts are located in `.github/prompts/`. Browse the directory or check [NEXT_MISSION.md](../NEXT_MISSION.md) to identify which task prompt applies:

```
.github/prompts/
├── led-state-visualization.prompt.md
├── config-macro-consolidation.prompt.md
└── header-decoupling.prompt.md
```

Invoke each prompt via the corresponding agent. Example:
```
@primary-developer: Use the led-state-visualization prompt.
@architecture-review: Use the config-macro-consolidation prompt.
```

---

## 🔗 Quick Reference Links

| Task | Resource |
|---|---|
| Project overview | [docs/README.md](../docs/README.md) |
| Architecture | [docs/architecture.md](../docs/architecture.md) |
| Current goals | [NEXT_MISSION.md](../NEXT_MISSION.md) |
| Build commands | EIDE tasks in VS Code |
| Code conventions | [docs/engineering/dev-guidelines/rules/](../docs/engineering/dev-guidelines/rules/) |
| Agent guide | [AGENTS.md](AGENTS.md) |
| Copilot instructions | [copilot-instructions.md](../copilot-instructions.md) |

---

## 🆘 Troubleshooting

### Build fails with warnings
- [ ] Check exact warning type (unused variable, cast, etc.)
- [ ] Fix at source in .c file
- [ ] ARM Compiler 5 is strict; some warnings require refactoring

### Memory exceeded
- [ ] Run `eide.project.rebuild` (full clean rebuild)
- [ ] Compare `.map` file before/after your changes
- [ ] Consider breaking feature into phases or optimizing algorithm

### Linker errors
- [ ] Verify function declarations in headers
- [ ] Check platform API wrappers are complete
- [ ] Ensure no circular includes between config headers

### Unclear which agent to use
- [ ] Default: **Primary Developer** for any routine task
- [ ] If specialized (algorithm/architecture/docs), specify in request
- [ ] Primary Developer will escalate if needed

---

## ✨ Best Practices

1. **Compile early, compile often**: Don't wait until end-of-day to discover warnings
2. **Use config macros aggressively**: Even single-use constants belong in foc_cfg_*.h
3. **Document as you code**: Doxygen comments + inline explanations
4. **Test incrementally**: Don't implement everything, then test once
5. **Respect layer boundaries**: Future P2 library stage depends on clean layering now

---

## 📞 Getting Help

- **Stuck on agent selection?** → Check [AGENTS.md](AGENTS.md)
- **Unclear requirements?** → Check [NEXT_MISSION.md](../NEXT_MISSION.md)
- **Code style questions?** → See [docs/engineering/dev-guidelines/rules/](../docs/engineering/dev-guidelines/rules/)
- **Build issues?** → See Troubleshooting section above
- **Algorithm questions?** → Read [docs/architecture.md](../docs/architecture.md) + foc-algorithm-review agent description

---

**Last updated**: 2026-04-09  
**Version**: v1.0.0  
**Phases covered**: See [NEXT_MISSION.md](../NEXT_MISSION.md)
**Active phases**: See [NEXT_MISSION.md](../NEXT_MISSION.md) for current work breakdown
