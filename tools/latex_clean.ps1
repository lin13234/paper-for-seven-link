<#
.SYNOPSIS
Cleans LaTeX auxiliary files while retaining the generated PDF.

.PARAMETER Main
Workspace-relative or absolute path to the LaTeX main document.
#>
[CmdletBinding()]
param(
    [string]$Main = ''
)

$ErrorActionPreference = 'Stop'
$workspaceRoot = Split-Path -Parent $PSScriptRoot

$mainPath = if ([string]::IsNullOrWhiteSpace($Main)) {
    $candidates = @(
        Get-ChildItem -LiteralPath $workspaceRoot -Recurse -File -Filter 'paper.tex' |
            Where-Object {
                $_.FullName -match '[\\/]001paper[\\/].*_LaTeX[\\/]paper\.tex$' -and
                $_.FullName -notmatch '[\\/]\.worktrees[\\/]'
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
$exitCode = 1

Push-Location $mainDirectory
try {
    # latexmk -c removes auxiliary files but deliberately keeps the PDF.
    & $latexmk.Source -c $mainName
    $exitCode = $LASTEXITCODE
}
finally {
    Pop-Location
}

if ($exitCode -ne 0) {
    Write-Error "LaTeX cleanup failed with exit code $exitCode."
    exit $exitCode
}

Write-Host "Auxiliary files cleaned for: $mainPath"
exit 0
