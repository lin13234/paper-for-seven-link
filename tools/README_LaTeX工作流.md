# LaTeX 编译与双向跳转

## 推荐操作

在 VS Code 中打开整个 `E:\hub` 文件夹，并打开论文主稿：

```text
02论文正文/001paper/01_主稿_LaTeX/paper.tex
```

LaTeX Workshop 已配置为保存 `.tex` 文件时自动编译，并在主稿目录
生成 `paper.pdf` 和 `paper.synctex.gz`。

## VS Code 操作

### 编译论文

按 `Ctrl+Shift+B`，运行默认任务：

```text
LaTeX: Build paper
```

也可以通过“终端 → 运行任务”选择：

- `LaTeX: Build paper`：单次完整编译；
- `LaTeX: Watch paper`：连续监听文件变化并自动重编译；
- `LaTeX: Clean auxiliary files`：清理辅助文件但保留 PDF。

### 打开内置 PDF

打开命令面板 `Ctrl+Shift+P`，执行：

```text
LaTeX Workshop: View LaTeX PDF file
```

PDF 会在 VS Code 标签页中打开。

### 源码跳转到 PDF

把光标放在目标 LaTeX 源码行，按：

```text
Ctrl+Alt+J
```

LaTeX Workshop 会跳转到 PDF 对应区域。

### PDF 跳转到源码

在内置 PDF 正文中：

```text
按住 Ctrl 并单击目标位置
```

VS Code 会跳转到对应 `.tex` 文件和行号。

## PowerShell 命令

从 `E:\hub` 根目录执行：

```powershell
# 编译默认论文
powershell -NoProfile -ExecutionPolicy Bypass -File tools/latex_build.ps1

# 编译并在 VS Code 中打开 PDF
powershell -NoProfile -ExecutionPolicy Bypass -File tools/latex_build.ps1 -OpenPdf

# 连续监听
powershell -NoProfile -ExecutionPolicy Bypass -File tools/latex_build.ps1 -Watch

# 清理辅助文件，保留 PDF
powershell -NoProfile -ExecutionPolicy Bypass -File tools/latex_clean.ps1

# 编译其他主文件
powershell -NoProfile -ExecutionPolicy Bypass -File tools/latex_build.ps1 `
  -Main "绝对路径或相对路径\main.tex"
```

## SyncTeX 无法跳转时

依次检查：

1. 主稿目录是否同时存在 `paper.pdf` 和 `paper.synctex.gz`；
2. 当前打开的 PDF 是否由当前 `paper.tex` 生成；
3. 重新运行 `LaTeX: Build paper`；
4. 关闭旧 PDF 标签页，再执行
   `LaTeX Workshop: View LaTeX PDF file`；
5. 不要手工移动 PDF 或 `.synctex.gz`。

当前清理任务使用 `latexmk -c`，会保留 PDF 和 `.synctex.gz`。如果手工
删除了 `.synctex.gz`，重新编译一次即可恢复双向跳转。
