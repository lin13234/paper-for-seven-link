# LaTeX 编译与 SyncTeX 双向跳转设计

## 目标

在 `E:\hub` 工作区内建立一套面向当前论文的 VS Code LaTeX 工作流，实现：

1. 使用 `latexmk` 编译主稿；
2. 通过 LaTeX Workshop 内置 PDF 查看器预览结果；
3. 从 LaTeX 源码正向跳转到 PDF 对应位置；
4. 从 PDF 反向跳转到 LaTeX 对应行；
5. 提供连续监听、手动编译和清理辅助文件的统一入口；
6. 编译失败时显示文件名、行号和明确错误信息。

## 已确认环境

- 编辑器：Visual Studio Code
- 扩展：`james-yu.latex-workshop`
- TeX 工具：TeX Live 2025
- 编译器：`latexmk`、`pdflatex`
- PDF 查看器：LaTeX Workshop 内置查看器
- 主稿：
  `02论文正文/001paper/01_主稿_LaTeX/paper.tex`

## 总体方案

### 1. PowerShell 工具脚本

在 `tools/` 下提供统一脚本：

- `latex_build.ps1`
  - 默认定位当前论文主稿；
  - 支持通过参数指定其他 `.tex` 主文件；
  - 调用 `latexmk -pdf -synctex=1`；
  - 使用 `-interaction=nonstopmode` 和 `-file-line-error`；
  - 返回 `latexmk` 的真实退出码；
  - 支持普通编译和连续监听模式；
  - 可选在成功后调用 VS Code 打开生成的 PDF。

- `latex_clean.ps1`
  - 调用 `latexmk -c` 清理辅助文件；
  - 默认保留最终 PDF；
  - 不删除源文件、图片、参考文献或手工生成的数据。

脚本不负责编辑器内的跳转逻辑。正向和反向同步由
LaTeX Workshop 读取 `.synctex.gz` 后完成。

### 2. VS Code 工作区配置

创建 `.vscode/settings.json`，配置：

- LaTeX Workshop 使用内置标签页 PDF 查看器；
- 默认 recipe 使用 `latexmk`；
- 编译命令始终带 `-synctex=1`；
- 保存 `.tex` 文件时自动编译；
- PDF 反向同步使用 `Ctrl+单击`；
- 构建产物保留在主稿目录，避免 SyncTeX 路径失配；
- 清理范围仅包含常见 LaTeX 辅助文件。

创建 `.vscode/tasks.json`，暴露三个任务：

- `LaTeX: Build paper`
- `LaTeX: Watch paper`
- `LaTeX: Clean auxiliary files`

任务从工作区根目录调用 `tools/` 中的脚本。

## 双向跳转交互

### 源码到 PDF

用户将光标放在 `.tex` 源码目标行，执行 LaTeX Workshop 的
“SyncTeX from cursor”命令。默认快捷键采用扩展提供的
`Ctrl+Alt+J`；不覆盖用户已有全局快捷键。

### PDF 到源码

在 LaTeX Workshop 内置 PDF 中按住 `Ctrl` 并单击正文位置，
编辑器跳转到对应 `.tex` 文件和行号。

这两种操作依赖最新编译生成的 `.synctex.gz`。如果同步失效，
先重新编译，再检查 PDF 是否由当前主稿生成。

## 错误处理

- 主文件不存在：脚本输出绝对路径并以非零状态退出；
- `latexmk` 不可用：输出安装/路径提示并终止；
- 编译失败：保留日志，向 VS Code 任务面板传递原始退出码；
- PDF 未生成：不尝试打开查看器；
- 连续监听：由 `latexmk -pvc` 管理增量编译，用户主动终止任务；
- 清理操作：仅使用 `latexmk -c`，不执行递归删除。

## 验证标准

1. 手动构建任务能生成 `paper.pdf` 和 `paper.synctex.gz`；
2. 修改并保存 `paper.tex` 后能够自动重新编译；
3. `Ctrl+Alt+J` 能从源码跳至 PDF 对应区域；
4. `Ctrl+单击` PDF 正文能返回正确源文件行；
5. 引入一个临时 LaTeX 语法错误时，问题面板显示文件和行号；
6. 清理任务移除辅助文件但保留 `paper.pdf`；
7. 路径包含中文时，三个任务仍可正常执行。

## 范围边界

- 本次不安装或配置 SumatraPDF；
- 不更换论文模板或 TeX 引擎；
- 不修改论文正文内容；
- 不把生成的 PDF 移到独立输出目录，以免破坏 SyncTeX 路径；
- 不覆盖用户级 VS Code 设置，只写当前工作区配置。
