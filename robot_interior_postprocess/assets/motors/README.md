# 原库四种电机的 CAD

`sources.json` 保存下载地址、来源页面、源版本及 SHA256；STEP 文件保留原始内容。达妙来自厂商 dmBots 官方仓库，MG4005V2 来自 Robotopian 的对应型号产品资源。

- MG4005V2：供应商标注 MG4005-V2，下载模型轴向约 33.4 mm，原参数库本体高度为 34.2 mm，保留此差异，不缩放。
- DM6006：DM-J6006-2EC 厂商模型。
- DM8006：使用官方仓库的旧版 V1.0，轴向约 39.9 mm，与原参数库 40 mm 对应；不自动换成 V1.1。
- DM10010：DM-J10010-2EC，未使用 DM10010L；原始 CAD 横向最大约 120 mm，超过原库直径 111.2 mm，预览保留该差异。

公开下载不代表源 CAD 采用开源许可；来源未明确提供再分发条款，发布资产前应核实。颜色为预览配色。

在项目根目录执行：

```bash
OPENBLAS_NUM_THREADS=1 .conda-env/voxel_tool/bin/python robot_interior_postprocess/motor_cad.py
OPENBLAS_NUM_THREADS=1 .conda-env/voxel_tool/bin/python robot_interior_postprocess/cad_models.py --output robot_interior_postprocess/outputs/cactus
```

转换统一采用毫米，旋转轴对齐本地 Z 轴，居中且不缩放，交互网格最多 18000 个三角面。`models.json` 保留坐标转换、尺寸和来源。

后处理通过 `motor_model_mapping` 决定展示型号；没有明确映射时保留原包络。CAD 位于原电机中心并对齐原轴，因旧输出没有法兰基准及完整姿态，轴向正反、绕轴角度和安装面位置仍需装配核验。原圆柱包含 15 mm 连接余量，真实 CAD 不包含这段连接结构。此替换只用于可视化，不修改碰撞包络或制造结构。

`cad_preview.html` 和 `cad_assembly.glb` 使用 CAD；原 `layout_canvas.html`、`assembly.glb` 保留计算包络，便于对照。
