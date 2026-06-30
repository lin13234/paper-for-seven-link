# MMT Paper Bilingual Draft Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild the MMT manuscript as an English LaTeX draft and create a section-matched Chinese Markdown draft without inventing missing evidence.

**Architecture:** Treat the English LaTeX manuscript as the submission source and the Chinese Markdown file as a semantic mirror for author review. Reuse only claims supported by the planning notes, Chinese draft, and kinematic reports; encode missing figures, citations, PPO parameters, and physical experiments as explicit evidence markers.

**Tech Stack:** Elsevier `cas-sc` LaTeX, Markdown, PowerShell, `rg`, available TeX compiler.

## Global Constraints

- The paper uses seven sections: Introduction; Kinematic Modeling and Reconfiguration Analysis; Numerical Validation of the Two Motion Branches; Prototype Design and Curriculum-Learning-Based PPO Validation; Physical Prototype Validation; Discussion; Conclusions.
- Section 2 explains SVD theory and branch-reconfiguration mechanics; Section 3 retains numerical curves, fitted relations, manifolds, and singularity tables.
- Section 4 contains three subsections: prototype/simulation framework; Stage I test-rig branch discovery and ablation; Stage II Fourier-guided ground locomotion.
- Section 5 must not claim that unperformed physical experiments have been completed.
- English and Chinese drafts must share section order, equations, terminology, and evidence markers.
- Do not fabricate citations, numerical measurements, PPO hyperparameters, hardware specifications, or experimental results.

---

### Task 1: Lock evidence boundaries and terminology

**Files:**
- Create: `02论文正文/001paper/00_项目说明/论文术语与证据台账.md`
- Read: `02论文正文/001paper/00_项目说明/MMT论文全局写作规划.md`
- Read: `02论文正文/001paper/02_中文草稿/中文底稿.docx`
- Read: `02论文正文/001paper/01_主稿_LaTeX/paper.tex`
- Read: `03推导分析/Ultimate_Kinematics_Output/Final_Comprehensive_Report.tex`

**Interfaces:**
- Consumes: Existing author notes and derived kinematic claims.
- Produces: Canonical English/Chinese terms, notation, supported claims, disputed claims, and exact evidence-marker vocabulary used by Tasks 2–4.

- [ ] **Step 1: Create the terminology ledger**

Record canonical forms for:

- plane-symmetric spatial 7R closed-chain mechanism
- spatial symmetric branch
- degenerated planar branch
- passive Jacobian matrix \(\mathbf J_p\)
- singular value decomposition (SVD)
- minimum singular value \(\sigma_{\min}\)
- ill-conditioned or pseudo-singular configuration
- branch reconfiguration
- redundant actuation
- test rig
- curriculum learning
- Fourier-guided reference trajectory
- MuJoCo and proximal policy optimization (PPO)

- [ ] **Step 2: Record evidence status**

Classify every major claim as `supported`, `needs verification`, or `planned experiment`. In particular, flag the conflicting planar-branch angle relations and every unmeasured physical-prototype metric as not ready for declarative prose.

- [ ] **Step 3: Verify the ledger contains no unresolved unlabeled claims**

Run:

```powershell
rg -n "supported|needs verification|planned experiment" "02论文正文/001paper/00_项目说明/论文术语与证据台账.md"
```

Expected: every major claim row has exactly one evidence-status label.

### Task 2: Rebuild the English LaTeX manuscript

**Files:**
- Modify: `02论文正文/001paper/01_主稿_LaTeX/paper.tex`

**Interfaces:**
- Consumes: Task 1 terminology ledger and evidence classifications.
- Produces: Submission-oriented English manuscript skeleton with substantive evidence-backed prose and explicit evidence markers.

- [ ] **Step 1: Replace template front matter**

Use the working title:

```text
Kinematic Branch Reconfiguration of a Plane-Symmetric Spatial 7R Mechanism for Underactuated Mobile Locomotion
```

Remove template authors, affiliations, biographies, sample keywords, example theorem, sample table, and unrelated captions. Keep author metadata as clearly labeled author-supplied fields rather than invented identities.

- [ ] **Step 2: Write the seven-section structure**

Create the exact section order in the Global Constraints. Use the approved subsection boundaries, including the five-part Section 2, three-part Section 3, and three-part Section 4.

- [ ] **Step 3: Preserve and reorganize core equations**

Retain:

```latex
\mathbf{T}_1\mathbf{T}_2\cdots\mathbf{T}_7=\mathbf I
```

```latex
\mathbf J\dot{\mathbf q}=\mathbf 0
```

```latex
\mathbf J_p\dot{\mathbf q}_p=-\mathbf J_a\dot q_a
```

```latex
\mathbf J_p=\mathbf U\boldsymbol{\Sigma}\mathbf V^T
```

```latex
q_7=q_1,\qquad q_6=-q_2,\qquad q_5=-q_3
```

Keep long screw vectors and full Fourier coefficients out of the main narrative unless they are essential to the stated conclusion.

- [ ] **Step 4: Write Sections 2 and 3 with a strict method/results boundary**

Section 2 must define the SVD criterion, rank loss, instantaneous mobility, pseudo-singularity distinction, orthogonal joint-axis groups, and branch-selection mechanism. Section 3 must report continuation, singular-value curves, spatial-branch relations, planar-branch behavior, and manifold intersections.

- [ ] **Step 5: Write the two-stage curriculum-learning argument**

State the training sequence exactly:

```text
test-rig branch discovery and redundant-actuation ablation
-> Fourier fitting of joint coordination
-> Fourier-guided locomotion learning under ground contact
```

Do not present the Fourier reference as an independently discovered physical law; describe it as a fitted coordination target extracted from Stage I.

- [ ] **Step 6: Write the physical validation section as a protocol**

Use future or planned-study language. Include measurements for joint trajectories, branch switching, displacement, speed, step length, energy, repeatability, and simulation–experiment error, each with an explicit evidence marker.

- [ ] **Step 7: Audit unsupported claims and template residue**

Run:

```powershell
rg -n "Krishnan|Trivandrum|Munnar|quadrupole exciton|Example Matrix|Theorem and|Author biography|XXX|perfect alignment|completely proves" "02论文正文/001paper/01_主稿_LaTeX/paper.tex"
```

Expected: no matches.

### Task 3: Create the synchronized Chinese draft

**Files:**
- Create: `02论文正文/001paper/02_中文草稿/中文同步底稿.md`

**Interfaces:**
- Consumes: Final structure and claims from Task 2.
- Produces: Chinese semantic mirror with identical numbering, equations, and evidence markers.

- [ ] **Step 1: Mirror every heading**

Use bilingual heading lines in the form:

```markdown
## 2. 运动学建模与可重构分析
<!-- Kinematic Modeling and Reconfiguration Analysis -->
```

- [ ] **Step 2: Translate meaning rather than English syntax**

Keep one paragraph per job: background, gap, method, result, mechanism, implication, or limitation. Preserve canonical notation and abbreviations.

- [ ] **Step 3: Mirror all missing-input markers**

Every English evidence marker must have a Chinese counterpart at the corresponding location. Physical prototype results remain planned, not completed.

- [ ] **Step 4: Verify heading correspondence**

Run:

```powershell
rg -n "^\\\\section|^\\\\subsection" "02论文正文/001paper/01_主稿_LaTeX/paper.tex"
rg -n "^## |^### " "02论文正文/001paper/02_中文草稿/中文同步底稿.md"
```

Expected: the same seven top-level sections and matching subsection order.

### Task 4: Run manuscript QA

**Files:**
- Verify: `02论文正文/001paper/01_主稿_LaTeX/paper.tex`
- Verify: `02论文正文/001paper/02_中文草稿/中文同步底稿.md`
- Verify: `02论文正文/001paper/00_项目说明/论文术语与证据台账.md`

**Interfaces:**
- Consumes: Drafts from Tasks 1–3.
- Produces: Evidence-backed, structurally synchronized files ready for the author to add figures, citations, and measurements.

- [ ] **Step 1: Check LaTeX environment balance**

Run a PowerShell count of `\begin{...}` and `\end{...}` tokens and verify that each environment has matching counts.

- [ ] **Step 2: Compile when a TeX engine is available**

Run:

```powershell
pdflatex -interaction=nonstopmode -halt-on-error paper.tex
```

from `02论文正文/001paper/01_主稿_LaTeX`.

Expected: successful PDF generation. If the Elsevier class or bibliography file is unavailable, report the exact missing dependency and retain the structural checks.

- [ ] **Step 3: Audit evidence markers**

Run:

```powershell
rg -n "Evidence needed|Author input required|Planned experiment|TODO" "02论文正文/001paper/01_主稿_LaTeX/paper.tex" "02论文正文/001paper/02_中文草稿/中文同步底稿.md"
```

Expected: markers occur only where source evidence is genuinely absent, especially citations, hardware parameters, PPO details, figures, and physical experiments.

- [ ] **Step 4: Check the working tree**

Run:

```powershell
git status --short
```

Expected: only the intended manuscript, Chinese draft, ledger, plan, and design-addendum files are modified or untracked.
