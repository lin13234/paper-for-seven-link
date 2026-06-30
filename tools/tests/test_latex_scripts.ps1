$ErrorActionPreference = 'Stop'

# Resolve the isolated workspace root from tools/tests/.
$workspaceRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildScript = Join-Path $workspaceRoot 'tools\latex_build.ps1'
$cleanScript = Join-Path $workspaceRoot 'tools\latex_clean.ps1'

if (-not (Test-Path -LiteralPath $buildScript -PathType Leaf)) {
    throw "Missing build script: $buildScript"
}
if (-not (Test-Path -LiteralPath $cleanScript -PathType Leaf)) {
    throw "Missing clean script: $cleanScript"
}

# Run the real manuscript build. This also verifies non-ASCII path handling.
& powershell -NoProfile -ExecutionPolicy Bypass -File $buildScript
if ($LASTEXITCODE -ne 0) {
    throw "Build script failed with exit code $LASTEXITCODE"
}

$mainCandidates = @(
    Get-ChildItem -LiteralPath $workspaceRoot -Recurse -File -Filter 'paper.tex' |
        Where-Object {
            $_.FullName -match '[\\/]001paper[\\/].*_LaTeX[\\/]paper\.tex$'
        }
)
if ($mainCandidates.Count -ne 1) {
    throw "Expected one manuscript main file; found $($mainCandidates.Count)."
}
$outputDirectory = $mainCandidates[0].DirectoryName

foreach ($name in @('paper.pdf', 'paper.synctex.gz')) {
    $artifactPath = Join-Path $outputDirectory $name
    if (-not (Test-Path -LiteralPath $artifactPath -PathType Leaf)) {
        throw "Expected build artifact is missing: $artifactPath"
    }
}

Write-Host 'LaTeX script acceptance test passed.'