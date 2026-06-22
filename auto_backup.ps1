$ErrorActionPreference = 'Stop'

# 自动备份脚本：
# 1. 检查仓库是否有变更
# 2. 如果有变更，则自动提交
# 3. 然后自动推送到 GitHub

Set-Location -LiteralPath $PSScriptRoot

$status = git status --short
if ([string]::IsNullOrWhiteSpace($status)) {
    Write-Host "No changes detected. Nothing to do."
    exit 0
}

# 只处理已经在仓库里的变更，不主动删除文件
git add -A

$timestamp = Get-Date -Format 'yyyy-MM-dd HH:mm:ss'
$message = "auto backup $timestamp"

git commit -m $message
git push
