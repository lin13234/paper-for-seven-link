<#
.SYNOPSIS
Compiles a LaTeX main document with latexmk and SyncTeX support.

.DESCRIPTION
The default target is the paper in this workspace. Build products remain
beside the main .tex file so LaTeX Workshop can resolve SyncTeX paths.

.PARAMETER Main
Workspace-relative or absolute path to the LaTeX main document.

.PARAMETER Watch
Runs latexmk in continuous preview mode. Stop it from VS Code or with Ctrl+C.

.PARAMETER OpenPdf
After a successful one-shot build, opens the PDF in the current VS Code window.
#>
[CmdletBinding()]
param(
    [string]$Main = '',
    [switch]$Watch,
    [switch]$OpenPdf
)

$ErrorActionPreference = 'Stop'
$workspaceRoot = Split-Path -Parent $PSScriptRoot

# Accept both workspace-relative and absolute main-file paths.
$mainPath = if ([string]::IsNullOrWhiteSpace($Main)) {
    $candidates = @(
        Get-ChildItem -LiteralPath $workspaceRoot -Recurse -File -Filter 'paper.tex' |
            Where-Object {
                $_.FullName -match '[\\/]001paper[\\/].*_LaTeX[\\/]paper\.tex$'
            }
    )
    if ($candidates.Count -ne 1) {
        Write-Error (
            "Expected exactly one default paper.tex under 001paper/*_LaTeX; " +
            "found $($candidates.Count). Pass -Main explicitly."
        )
        exit 2
    }
    $candidates[0].FullName
}
elseif ([System.IO.Path]::IsPathRooted($Main)) {
    $Main
}
else {
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

# -synctex=1 generates paper.synctex.gz for forward and inverse navigation.
$latexmkArguments = @(
    '-pdf',
    '-synctex=1',
    '-interaction=nonstopmode',
    '-file-line-error',
    '-halt-on-error'
)
if ($Watch) {
    # LaTeX Workshop remains responsible for displaying the PDF.
    $latexmkArguments += @('-pvc', '-view=none')
}
$latexmkArguments += $mainName

Write-Host "LaTeX main file: $mainPath"
Write-Host "Command: latexmk $($latexmkArguments -join ' ')"

$exitCode = 1
Push-Location $mainDirectory
try {
    & $latexmk.Source @latexmkArguments
    $exitCode = $LASTEXITCODE
}
finally {
    Pop-Location
}

if ($exitCode -ne 0) {
    Write-Error "LaTeX build failed with exit code $exitCode."
    exit $exitCode
}
if (-not $Watch -and -not (Test-Path -LiteralPath $pdfPath -PathType Leaf)) {
    Write-Error "Build finished without producing PDF: $pdfPath"
    exit 4
}

if ($OpenPdf -and -not $Watch) {
    $code = Get-Command code -ErrorAction SilentlyContinue
    if ($null -eq $code) {
        Write-Warning 'VS Code command was not found; PDF was built but not opened.'
    }
    else {
        & $code.Source --reuse-window $pdfPath
    }
}

if (-not $Watch) {
    Write-Host "Build succeeded: $pdfPath"
}
exit 0
