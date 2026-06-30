$ErrorActionPreference = 'Stop'

$workspaceRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vscodeDirectory = Join-Path $workspaceRoot '.vscode'
$settingsPath = Join-Path $vscodeDirectory 'settings.json'
$tasksPath = Join-Path $vscodeDirectory 'tasks.json'
$extensionsPath = Join-Path $vscodeDirectory 'extensions.json'

foreach ($path in @($settingsPath, $tasksPath, $extensionsPath)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Missing VS Code configuration file: $path"
    }
}

$settings = Get-Content -LiteralPath $settingsPath -Raw | ConvertFrom-Json
$tasks = Get-Content -LiteralPath $tasksPath -Raw | ConvertFrom-Json
$extensions = Get-Content -LiteralPath $extensionsPath -Raw | ConvertFrom-Json

if ($settings.'latex-workshop.view.pdf.viewer' -ne 'tab') {
    throw 'LaTeX Workshop must use the internal tab PDF viewer.'
}
if ($settings.'latex-workshop.latex.autoBuild.run' -ne 'onSave') {
    throw 'LaTeX Workshop must build on save.'
}
if (
    $settings.'latex-workshop.view.pdf.internal.synctex.keybinding' -ne
    'ctrl-click'
) {
    throw 'Inverse SyncTeX must use Ctrl+click.'
}
if ($settings.'latex-workshop.synctex.afterBuild.enabled' -ne $true) {
    throw 'Forward SyncTeX after build must be enabled.'
}

$labels = @($tasks.tasks | ForEach-Object { $_.label })
foreach ($requiredLabel in @(
    'LaTeX: Build paper',
    'LaTeX: Watch paper',
    'LaTeX: Clean auxiliary files'
)) {
    if ($requiredLabel -notin $labels) {
        throw "Missing VS Code task: $requiredLabel"
    }
}

if ('james-yu.latex-workshop' -notin @($extensions.recommendations)) {
    throw 'LaTeX Workshop is not listed as a recommended extension.'
}

Write-Host 'VS Code LaTeX configuration test passed.'
