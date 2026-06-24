# MMT 图形标注与 Inkscape 规范

本文档用于统一本论文所有机构图、拓扑图、仿真图和数据图的标注风格。目标期刊为 *Mechanism and Machine Theory*，图形风格应保持工程学术图的清晰、克制和可复现，不追求过度渲染效果。

## 1. 推荐制图流程

建议采用以下流程：

```text
KeyShot / MuJoCo / SolidWorks 渲染底图
-> Inkscape 添加学术标注
-> 导出 PDF / SVG / 600 dpi PNG
-> 插入 LaTeX 主稿
```

其中：

- KeyShot、MuJoCo 或 SolidWorks 负责机构几何和三维实体显示。
- Inkscape 负责关节编号、坐标轴、箭头、对称面、模式说明和子图编号。
- LaTeX caption 负责完整图题和学术解释，不建议把长句直接写进图中。

## 2. 字体

图内英文标注统一使用：

```text
Arial
```

不建议在同一张图里混用 Arial、Times New Roman、微软雅黑、宋体等字体。

数学符号建议：

- 图内短标签可先写为 `q1`, `q2`, `Jp`, `J1-J7`。
- 正文和 caption 中再使用 LaTeX 规范形式，例如 `$q_1$`, `$q_2$`, `$J_p$`。
- 如果必须在图中使用严格数学符号，可后期用 LaTeX 公式对象或矢量公式替换。

## 3. 页面尺寸

Inkscape 页面建议按论文实际图宽设置，而不是随便使用默认 A4 页面。

### 3.1 单栏图

```text
Width: 85 mm
Height: 根据内容确定，一般 55-70 mm
```

### 3.2 双栏图

```text
Width: 170 mm
Height: 根据内容确定，一般 100-120 mm
```

Figure 1 机构综合定义图建议使用双栏图：

```text
Width: 170 mm
Height: 110 mm
```

Inkscape 设置位置：

```text
File -> Document Properties -> Display units: mm
Custom size: 170 mm x 110 mm
```

## 4. Figure 1 推荐字号

Figure 1 建议作为双栏图使用，因此按照双栏图字号设置。

```text
关节编号 J1-J7: 10 pt, Arial Bold
坐标轴 X/Y/Z: 9 pt, Arial
坐标原点 O: 9 pt, Arial
Symmetry plane: 9 pt, Arial
Planar mode / Spatial mode: 9 pt, Arial
子图编号 (a), (b), (c): 11 pt, Arial Bold
普通短标签: 9 pt, Arial
辅助说明文字: 8-9 pt, Arial
```

如果某张图最终作为单栏图使用，可相应减小字号：

```text
关节编号 J1-J7: 9 pt
坐标轴 X/Y/Z: 8 pt
普通说明文字: 8 pt
子图编号 (a), (b): 10 pt, Arial Bold
箭头或轴线说明: 7.5-8 pt
```

## 5. 线宽

Inkscape 中建议统一使用以下线宽：

```text
普通标注线: 0.8 pt
关节轴线虚线: 1.0 pt
箭头线: 0.9 pt
坐标轴箭头: 1.0 pt
对称面边框: 0.7 pt
```

关节轴线虚线建议：

```text
Stroke width: 1.0 pt
Dash: 4 pt / 3 pt
Color: #1F5FFF
Opacity: 85%-100%
```

普通箭头线建议：

```text
Stroke width: 0.9 pt
Color: #111111 或对应强调色
```

## 6. 推荐颜色

全文图形颜色应统一，避免一张图一个配色。

```text
正文黑字: #111111
关节轴线蓝: #1F5FFF
平面主动关节 J1/J2: #2F5FB3
空间主动关节 J3/J7: #B34A55
被动关节标注: #333333
连杆浅灰: #D8DCE0
被动关节灰: #7C8794
对称面: #9EC7E8, opacity 15%-20%
```

注意：

- 主动关节可以用颜色强调，但不要过度饱和。
- 对称面必须半透明，不能遮挡机构本体。
- 文字优先用黑色或深灰色，不建议使用过多彩色文字。

## 7. 图层管理

Inkscape 中建议至少建立以下图层：

```text
01_base_render
02_symmetry_plane
03_joint_axes
04_labels
05_arrows_and_frames
```

操作建议：

- 将 KeyShot / MuJoCo / SolidWorks 渲染底图放在 `01_base_render`。
- 锁定底图图层，避免后续误移动。
- 所有关节编号和坐标轴放在可编辑标注图层中。
- 对称面单独放一层，便于调整透明度。

## 8. Figure 1 建议包含内容

Figure 1 机构综合定义图建议包含：

- 机构三维本体。
- 关节编号 `J1-J7`。
- 关节轴线虚线。
- 主动关节对：
  - 平面模式：`J1/J2`
  - 空间模式：`J3/J7`
- 对称面。
- 一个全局坐标系 `O-XYZ`。
- 必要时放 2-3 个代表性局部坐标系。

Figure 1 不建议包含：

- 所有 DH 坐标系。
- 完整螺旋坐标。
- 大段文字说明。
- 太多箭头和公式。
- 机器人外壳、电机、线缆等非机构学核心细节。

完整 DH 坐标、螺旋轴定义和闭链拓扑建议放在后续方法图或附图中。

## 9. KeyShot 渲染建议

KeyShot 中只负责输出干净底图，不建议直接在 KeyShot 内完成文字和坐标标注。

推荐设置：

```text
Camera: Orthographic
Background: White 或 very light gray
Ground plane: 关闭或仅保留极浅阴影
Lighting: Product / Studio / Softbox
Perspective: 关闭或保持很小
```

材料建议：

```text
连杆: matte light gray
关节套筒: satin silver / neutral gray
平面主动关节 J1/J2: muted blue
空间主动关节 J3/J7: muted red
其他被动关节: dark gray / blue-gray
```

## 10. 导出格式

推荐保留三类文件：

```text
SVG 源文件: 可继续编辑
PDF 文件: 优先用于 LaTeX 和投稿
600 dpi PNG: 备用位图版本
```

命名建议：

```text
figure1_mechanism_keyshot_base.png
figure1_mechanism_annotation.svg
figure1_mechanism_final.pdf
figure1_mechanism_final_600dpi.png
```

不要只保存 PNG，而不保存 SVG 源文件。否则后续改字号、线宽、编号会非常麻烦。

## 11. 当前 Figure 1 的推荐固定规格

当前论文第一张机构图建议固定为：

```text
Canvas: 170 mm x 110 mm
Font: Arial
J1-J7: 10 pt, Arial Bold
Normal labels: 9 pt, Arial
Coordinate labels: 9 pt, Arial
Subfigure labels: 11 pt, Arial Bold
Joint axis line: 1.0 pt, blue dashed line
Arrow line: 0.9 pt
Symmetry plane: #9EC7E8, opacity 15%-20%
Export: PDF + SVG + 600 dpi PNG
```

这套规格作为后续机构图和机器人图的基础规范。

---

## 12. KeyShot 推荐渲染参数

KeyShot 适合快速得到干净、稳定、视觉质量较高的机构底图。建议只在 KeyShot 中完成实体渲染，不在 KeyShot 中做关节编号、坐标轴和文字标注。

### 12.1 Camera

```text
Camera type: Orthographic
Perspective: Off
Depth of field: Off
View angle: 机构整体 3/4 视角
Object size in frame: 机构占画面宽度 75%-85%
```

说明：

- 机构学论文图优先保证几何关系清楚。
- 不建议使用强透视，否则平行轴线和对称关系会显得不可靠。
- 四周要留出标注空间，不要让机构铺满画布。

### 12.2 Environment / Lighting

```text
Environment: Studio / Product / Softbox
Brightness: 1.0-1.5
Contrast: 0.6-1.0
Background: White 或 very light gray
Ground shadows: Very light 或 Off
Ground reflections: Off
```

说明：

- 阴影只用于增加空间感，不能抢机构本体。
- 不建议使用深色 HDRI、复杂反射背景或强烈环境色。

### 12.3 Material

推荐材料风格：

```text
Links: matte light gray
Joint sleeves: satin silver / neutral gray
Passive joints: blue-gray / dark gray
Planar active joints J1/J2: muted blue
Spatial active joints J3/J7: muted red
Roughness: 0.35-0.55
Metallic: 0.0-0.3
Specular: low to medium
```

推荐颜色：

```text
Links: #D8DCE0
Passive joints: #7C8794
Planar active joints J1/J2: #2F5FB3
Spatial active joints J3/J7: #B34A55
```

### 12.4 Render

```text
Mode: Product
Resolution: 4000 px wide 或更高
Samples: 128-256
Denoise: On
Output: PNG
Color space: sRGB
Transparent background: 可选
```

建议导出两版：

```text
figure1_keyshot_white_bg.png
figure1_keyshot_transparent.png
```

白底版用于直接排版，透明版用于后续在 Inkscape 中叠加和调整。

## 13. Blender 推荐渲染参数

Blender 适合需要更强可控性时使用，例如统一相机、灯光、材质、透明背景和批量渲染。当前论文如果使用 Blender，应保持工程学术图风格，不建议做成产品宣传图。

### 13.1 Render Engine

优先使用：

```text
Render Engine: Cycles
Device: GPU Compute
Samples: 128-256
Denoise: On
Color Management: Standard
Look: Medium High Contrast 或 None
Exposure: 0
Gamma: 1
```

如果电脑较慢，可以使用：

```text
Render Engine: Eevee Next
Ambient Occlusion: On
Contact Shadows: On
Soft Shadows: On
```

说明：

- 论文主图不需要极高真实感，Cycles 128-256 samples 通常足够。
- Eevee 速度快，但阴影和边缘质感不如 Cycles 稳。

### 13.2 Camera

```text
Camera type: Orthographic
Orthographic scale: 根据模型大小调整
Object size in frame: 机构占画面宽度 75%-85%
Depth of field: Off
Resolution: 4000 x 2600 px 或 4000 x 3000 px
```

建议视角：

```text
View: 3/4 isometric-like view
Elevation: 15-25 deg
Azimuth: 35-55 deg 或 -35 至 -55 deg
```

说明：

- 正交相机是机构学论文首选。
- 机构不要铺满全图，四周留出标注空间。

### 13.3 Lighting

推荐简单三点光或大面积柔光：

```text
Key light: Area Light, size 4-6 m, power 300-600 W
Fill light: Area Light, size 5-8 m, power 80-200 W
Back light: optional, power 50-150 W
World background: White or very light gray
Ambient occlusion: light
```

不要使用：

```text
强烈彩色灯光
复杂 HDRI 背景
大面积硬阴影
镜面反射地面
```

### 13.4 Material

Blender 中建议使用 Principled BSDF。

```text
Links:
Base Color: #D8DCE0
Roughness: 0.45-0.60
Metallic: 0.0
Specular IOR Level: 0.3-0.5

Passive joints:
Base Color: #7C8794
Roughness: 0.35-0.50
Metallic: 0.0-0.2

Planar active joints J1/J2:
Base Color: #2F5FB3
Roughness: 0.40-0.55
Metallic: 0.0

Spatial active joints J3/J7:
Base Color: #B34A55
Roughness: 0.40-0.55
Metallic: 0.0
```

说明：

- 不建议使用高金属、高反射材质。
- 材质应服务结构辨识，不要让反光遮挡关节和轴线。

### 13.5 Output

```text
Resolution: 4000 px wide or higher
File format: PNG
Color: RGBA if transparent background is needed
Compression: 15%-30%
Transparent background: On for annotation workflow
```

推荐输出：

```text
figure1_blender_white_bg.png
figure1_blender_transparent.png
```

### 13.6 Freestyle / Outline

如果想要一点工程制图轮廓感，可以开启 Freestyle，但要克制：

```text
Freestyle: On
Line color: #333333
Line thickness: 0.4-0.7 px
Crease angle: 30-45 deg
```

注意：

- 轮廓线太粗会显得像卡通图。
- 如果 STL 本身三角面明显，不要用 Freestyle 强化网格边。

## 14. KeyShot 与 Blender 的选择建议

当前论文建议优先级：

```text
KeyShot: 最适合快速得到漂亮、稳定、干净的机构底图
Blender: 适合后续需要完全可控、批量渲染、透明背景和精细相机复现
MuJoCo/Python: 适合精确坐标、关节轴线、对称面和程序化标注
Inkscape: 负责最终学术标注
```

当前 Figure 1 推荐路线：

```text
KeyShot 渲染机构底图
-> Python/MuJoCo 辅助确认关节轴线和对称面
-> Inkscape 添加标注
-> 导出 PDF + SVG + 600 dpi PNG
```
