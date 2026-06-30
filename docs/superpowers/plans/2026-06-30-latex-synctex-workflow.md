# LaTeX SyncTeX Workflow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a VS Code and LaTeX Workshop workflow that compiles the current paper, previews it in the internal PDF viewer, and supports SyncTeX forward and inverse navigation.

**Architecture:** PowerShell scripts under `tools/` provide editor-independent build, watch, and clean commands around `latexmk`. Workspace-local VS Code configuration invokes those scripts and configures LaTeX Workshop's internal PDF viewer and SyncTeX behavior. An acceptance test checks prerequisites, parses the JSON configuration, builds the real manuscript, and verifies both PDF and SyncTeX artifacts.

**Tech Stack:** PowerShell 7/Windows PowerShell, TeX Live 2025, `latexmk`, `pdflatex`, VS Code, LaTeX Workshop, JSON with comments (`jsonc`).

## Global Constraints

- Default main document: `02论文正文/001paper/01_主稿_LaTeX/paper.tex`.
- Use `latexmk -pdf -synctex=1 -interaction=nonstopmode -file-line-error`.
- Keep build outputs beside the main `.tex` file to preserve SyncTeX paths.
- Use the LaTeX Workshop internal tab viewer; do not install SumatraPDF.
- Do not modify manuscript content.
- Clean only standard auxiliary files and retain the generated PDF.
- Support paths containing Chinese characters.
- Do not overwrite user-level VS Code settings.

---

### Task 1: Build and clean scripts

**Files:**
- Create: `tools/latex_build.ps1`
- Create: `tools/latex_clean.ps1`
- Test: `tools/tests/test_latex_scripts.ps1`

**Interfaces:**
- Consumes: optional `-Main <path>`, `-Watch`, and `-OpenPdf` parameters.
- Produces: `paper.pdf`, `paper.synctex.gz`, and the original `latexmk` exit code.

- [ ] **Step 1: Write the failing script acceptance test**

Create `tools/tests/test_latex_scripts.ps1` with assertions that:

```powershell
$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$build = Join-Path $root 'tools\latex_build.ps1'
$clean = Join-Path $root 'tools\latex_clean.ps1'

if (-not (Test-Path -LiteralPath $build)) {
    throw "Missing build script: $build"
}
if (-not (Test-Path -LiteralPath $clean)) {
    throw "Missing clean script: $clean"
}
```

- [ ] **Step 2: Run the test and confirm the missing-script failure**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/tests/test_latex_scripts.ps1
```

Expected: non-zero exit with `Missing build script`.

- [ ] **Step 3: Implement `tools/latex_build.ps1`**

Implement:

```powershell
[CmdletBinding()]
param(
    [string]$Main = '02论文正文\001paper\01_主稿_LaTeX\paper.tex',
    [switch]$Watch,
    [switch]$OpenPdf
)

$ErrorActionPreference = 'Stop'
$workspaceRoot = Split-Path -Parent $PSScriptRoot
$mainPath = if ([System.IO.Path]::IsPathRooted($Main)) {
    $Main
} else {
    Join-Path $workspaceRoot $Main
}
$mainPath = [System.IO.Path]::GetFullPath($mainPath)

if (-not (Test-Path -LiteralPath $mainPath -PathType Leaf)) {
    Write-Error "LaTeX main file not found: $mainPath"
    exit 2
}

$latexmk = Get-Command latexmk -ErrorAction SilentlyContinue
if ($null -eq $latexmk) {
    Write-Error 'latexmk was not found on PATH. Verify the TeX Live installation.'
    exit 3
}

$mainDirectory = Split-Path -Parent $mainPath
$mainName = Split-Path -Leaf $mainPath
$pdfPath = [System.IO.Path]::ChangeExtension($mainPath, '.pdf')
$arguments = @(
    '-pdf',
    '-synctex=1',
    '-interaction=nonstopmode',
    '-file-line-error',
    '-halt-on-error'
)
if ($Watch) {
    $arguments += @('-pvc', '-view=none')
}
$arguments += $mainName

Push-Location $mainDirectory
try {
    & $latexmk.Source @arguments
    $exitCode = $LASTEXITCODE
} finally {
    Pop-Location
}

if ($exitCode -ne 0) {
    Write-Error "LaTeX build failed with exit code $exitCode."
    exit $exitCode
}
if (-not $Watch -and -not (Test-Path -LiteralPath $pdfPath)) {
    Write-Error "Build finished without producing PDF: $pdfPath"
    exit 4
}
if ($OpenPdf -and -not $Watch) {
    $code = Get-Command code -ErrorAction SilentlyContinue
    if ($null -eq $code) {
        Write-Warning 'VS Code command was not found; PDF was built but not opened.'
    } else {
        & $code.Source --reuse-window $pdfPath
    }
}
exit 0
```

- [ ] **Step 4: Implement `tools/latex_clean.ps1`**

Implement the same main-path validation, then run:

```powershell
Push-Location $mainDirectory
try {
    & $latexmk.Source -c $mainName
    $exitCode = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $exitCode
```

- [ ] **Step 5: Extend and run the script acceptance test**

Add checks for `latexmk` and the real build:

```powershell
& powershell -NoProfile -ExecutionPolicy Bypass -File $build
if ($LASTEXITCODE -ne 0) {
    throw "Build script failed with exit code $LASTEXITCODE"
}

$outputDirectory = Join-Path $root '02论文正文\001paper\01_主稿_LaTeX'
foreach ($name in @('paper.pdf', 'paper.synctex.gz')) {
    $path = Join-Path $outputDirectory $name
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Expected build artifact is missing: $path"
    }
}
Write-Host 'LaTeX script acceptance test passed.'
```

Run the test and expect exit code `0` with the success message.

### Task 2: VS Code and LaTeX Workshop configuration

**Files:**
- Modify: `.gitignore`
- Create: `.vscode/settings.json`
- Create: `.vscode/tasks.json`
- Create: `.vscode/extensions.json`
- Test: `tools/tests/test_vscode_latex_config.ps1`

**Interfaces:**
- Consumes: `tools/latex_build.ps1`, `tools/latex_clean.ps1`, LaTeX Workshop.
- Produces: workspace tasks and internal-viewer SyncTeX configuration.

- [ ] **Step 1: Write the failing configuration test**

The test must require the three `.vscode` files and parse each after removing
full-line `//` comments. It must assert:

```powershell
$settings.'latex-workshop.view.pdf.viewer' -eq 'tab'
$settings.'latex-workshop.latex.autoBuild.run' -eq 'onSave'
$settings.'latex-workshop.view.pdf.internal.synctex.keybinding' -eq 'ctrl-click'
$settings.'latex-workshop.synctex.afterBuild.enabled' -eq $true
```

Expected initial result: failure because `.vscode/settings.json` is missing.

- [ ] **Step 2: Allowlist workspace configuration in `.gitignore`**

Replace the blanket `.vscode/` rule with:

```gitignore
.vscode/*
!.vscode/settings.json
!.vscode/tasks.json
!.vscode/extensions.json
```

- [ ] **Step 3: Create `.vscode/settings.json`**

Configure one `latexmk` tool and recipe:

```json
{
  "latex-workshop.latex.autoBuild.run": "onSave",
  "latex-workshop.latex.outDir": "%DIR%",
  "latex-workshop.view.pdf.viewer": "tab",
  "latex-workshop.view.pdf.internal.synctex.keybinding": "ctrl-click",
  "latex-workshop.synctex.afterBuild.enabled": true,
  "latex-workshop.latex.autoClean.run": "never",
  "latex-workshop.latex.tools": [
    {
      "name": "latexmk-synctex",
      "command": "latexmk",
      "args": [
        "-pdf",
        "-synctex=1",
        "-interaction=nonstopmode",
        "-file-line-error",
        "-halt-on-error",
        "%DOC%"
      ]
    }
  ],
  "latex-workshop.latex.recipes": [
    {
      "name": "latexmk with SyncTeX",
      "tools": ["latexmk-synctex"]
    }
  ],
  "latex-workshop.latex.recipe.default": "first"
}
```

- [ ] **Step 4: Create `.vscode/tasks.json` and `extensions.json`**

Define build, watch, and clean tasks using:

```json
{
  "label": "LaTeX: Build paper",
  "type": "shell",
  "command": "powershell",
  "args": [
    "-NoProfile",
    "-ExecutionPolicy",
    "Bypass",
    "-File",
    "${workspaceFolder}\\tools\\latex_build.ps1"
  ],
  "group": {
    "kind": "build",
    "isDefault": true
  },
  "problemMatcher": []
}
```

The watch task adds `-Watch`; the clean task calls `latex_clean.ps1`.
`extensions.json` recommends `james-yu.latex-workshop`.

- [ ] **Step 5: Run the configuration acceptance test**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/tests/test_vscode_latex_config.ps1
```

Expected: exit code `0` and `VS Code LaTeX configuration test passed.`

### Task 3: End-to-end verification and usage guide

**Files:**
- Create: `tools/README_LaTeX工作流.md`
- Modify: `docs/superpowers/plans/2026-06-30-latex-synctex-workflow.md`

**Interfaces:**
- Consumes: completed scripts and workspace settings.
- Produces: verified commands and concise operating instructions.

- [ ] **Step 1: Run build and artifact verification**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/latex_build.ps1
```

Verify:

```powershell
Test-Path '02论文正文\001paper\01_主稿_LaTeX\paper.pdf'
Test-Path '02论文正文\001paper\01_主稿_LaTeX\paper.synctex.gz'
```

Expected: both return `True`.

- [ ] **Step 2: Verify failure reporting**

Run the build script with a nonexistent main file:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/latex_build.ps1 -Main missing.tex
```

Expected: non-zero exit and an absolute `LaTeX main file not found` path.

- [ ] **Step 3: Write the usage guide**

Document:

- `Ctrl+Shift+B`: build the paper;
- VS Code command `LaTeX Workshop: View LaTeX PDF file`: open the internal PDF;
- `Ctrl+Alt+J`: source-to-PDF forward SyncTeX;
- `Ctrl+单击`: PDF-to-source inverse SyncTeX;
- Terminal commands for build, watch, clean, and `-OpenPdf`;
- troubleshooting when `.synctex.gz` is stale or the wrong PDF is open.

- [ ] **Step 4: Run all acceptance tests**

Run both test scripts and expect exit code `0`.

- [ ] **Step 5: Mark completed plan checkboxes and review Git diff**

Confirm no manuscript source text changed during this implementation.
