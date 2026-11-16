# Anything2robot 内部空间与电子布局后处理

新增[仙人掌壳体内壁减材走线试样](outputs/cactus_inner_groove_trial_01/README.md)：读取已有接线表和端口锚点，将 `REGULATOR.GND → U1.GND`、`REGULATOR.VOUT → S1.P` 两条静态线路沿承载半壳内壁重新安排，直接挖槽并凹刻接口去向。输出完整壳片、63 × 65 mm 真实曲面裁片、试线长度表和几何检查；接口仍是模型锚点，实物配准、固定压片和弯曲规格待校准。首次打印使用输出中的 ZIP，它只包含一件修改后的小样 STL。运行：

```bash
.conda-env/voxel_tool/bin/python robot_interior_postprocess/generate_shell_groove_trial.py \
  --source robot_interior_postprocess/outputs/cactus_restrict_schema_live_verified \
  --output robot_interior_postprocess/outputs/my_cactus_groove_trial
```

输出目录须为空；此入口用于这两个明确分支的试装，不替代整机布线流程。设计背景见[壳体走线方案](../docs/shell_integrated_wiring_plan.md)。

载板入口现已接入 **EPS 前的 VLM restrict schema**，与开口候选排序分开记录。schema 真实约束初始布局、每次 EPS 移动及最终选择；过程页面默认 16 步，并显示三份本地候选布局图。已验证示例：`outputs/cactus_restrict_schema_live_verified/process.html`。一键回放运行 `bash robot_interior_postprocess/run_generation_demo.sh --open`；追加 `--live-schema` 真实重跑 schema。详见 [载板装配工作流](CARRIER_ASSEMBLY_WORKFLOW.md)。

独立载板入口默认输出[逐步生成过程可视化](PROCESS_VISUALIZATION.md)：`process.html` 可播放真实阶段快照，`process_changes.md` 逐步写明变化，包含 EPS 接受的改进与候选拒绝原因。

独立载板入口现已增加[逐片外壳螺钉连接](SHELL_FASTENING.md)：通孔、头座、实体连接柱、嵌件盲孔、定位配合和合盖止挡进入最终 STL，并核对每一片外壳的连接覆盖。

新增独立载板装配入口 `assemble_robot.py`：从原机械结果开始，以“元件＋可拆载板”为布局单元，联合选择开口、枚举合法姿态、进行载板 EPS，生成固定连接支座、分壳、重新布线并输出局部装配依赖。运行、泛用输入契约和真实验收边界见 [载板装配工作流](CARRIER_ASSEMBLY_WORKFLOW.md)。此入口仍输出 `partial`；不把局部操作/静态线路通过称为完整机器人可制造。下面描述的原布局入口保持不变。

独立入口：读取已生成的仙人掌机械结果、实心榫头制造结果及 Tinkercad Eagle/BOM 文件，生成真实中空部件，固定原电机位置，仅安排剩余电子模块，并调用 voxel_tool 的候选位姿生成、联合布局搜索和严格端口 A* 路由。

初始可布通布局之后，现在继续调用 voxel_tool 的 Extended Pattern Search（EPS），在保持机械电机固定和可用内腔约束下，联合优化元件位置、真实路由线长、拥塞和布局跨度，再运行独立的等长减弯。记录见 `eps_report.json`、`routing_before_polish.json`、`routing_polish_report.json` 和 `optimization_summary.json`，初始结果保存在 `initial_placement.json` / `initial_routing.json`。

当前桥接允许平移、跳步和可行交换；上游 EPS 的体素形状旋转没有同时更新接口朝向，因此此适配器拒绝这类未认证旋转。没有提供额外软语义关系时，EPS 的语义代价为零。电机通道按初始机械场景构造，EPS 不重新切割外壳。

EPS 默认为最多 160 次评估；每条线减弯最多搜索 250000 个状态，分别可用 `eps_max_evaluations` 和 `polish_max_states_per_branch` 配置。减弯不会增加线路长度或弯折数，不保证有限预算内找到全局最少弯折。这里的直角转折统计不代表真实线缆最小弯曲半径已验证。

不修改原始模型、电路文件或两个上游项目的代码。默认配置为 `configs/cactus.json`，所有尺寸单位为毫米。

## 运行

默认仙人掌配置现在启用视觉 LLM restricted schema 阶段。先在本目录 `.env` 填写模型服务配置（模板 `.env.example`），不要把 Key 写入 JSON 配置或聊天；`.env` 已被 Git 忽略。配置缺失时明确停止，不会悄悄退回手写规则。

LLM 读取中空空间正交图、深度图、材料/禁入区切片、元件参考图、固定机械电机及电路连接上下文，只为可移动电子模块生成约束。原始响应尝试、最终原始 schema、规范化约束、资源与几何保护规则分别保存在 `layout_schema_attempt_*.json`、`layout_constraints_raw.json`、`layout_constraints.json`、`semantic_guardrails.json`。初始布局和 EPS 的约束检查保存在 `semantic_initial_validation.json` / `semantic_final_validation.json`。

几何编译、初始布局和 EPS 使用同一组受限约束；硬约束不能静默放宽，软关系代价进入 EPS。当前闭合内腔适配器拒绝要求外部操作空间或部分外置的 schema，并保留原始结果供修改。固定电机不会交给 LLM 重新布局。LLM 输出本身不是几何可行性证明。

可以显式回放保存的原始 schema，避免重复调用模型：

```bash
bash robot_interior_postprocess/run_layout_test.sh --real-models --open --schema-proposal /path/to/layout_constraints_raw.json
```

`tests/fixtures/layout_schema_replay.json` 是合成的离线集成测试输入，不是 LLM 输出。使用它跑通不能作为真实模型调用成功的证明。`schema_provenance.json` 明确记录 `live_llm` 或 `saved_schema_replay`。指定其他本地服务配置使用 `--llm-env /path/to/.env`。

### 一键测试元件布局、布线及可视化

完成相同空间排布流程，最后打开真实 CAD 外形的装配可视化：

```bash
bash /home/qinxuan/projects/anything2robot/robot_interior_postprocess/run_layout_test.sh --real-models --open
```

`--real-models` 只改变最终打开的查看器，搜索与布线仍依据原规划包络；CAD 保持实际尺寸，未重新验证候选 CAD 的碰撞和真实端口。三个电机仍显示原机械包络。测试会额外核对 CAD 视图和模型文件已导出。可叠加 `--rebuild-shell`。

```bash
bash /home/qinxuan/projects/anything2robot/robot_interior_postprocess/run_layout_test.sh --open
```

这会创建新的 `outputs/layout_test_时间戳/`，重新计算候选位置、电子元件摆放、电机接线通道、布线及交互可视化，然后打开浏览器。不会复制旧的 placement/routing 结果。默认只复用与输入和几何参数签名一致的中空外壳，缺少匹配缓存时自动重建。

终端显示 `TEST: PASS` 表示固定电机中心保持不变、所有元件已摆放、全部网络分支已布通、已实现的几何检查通过并生成三维预览。每一项的结果保存在 `test_result.json`，实时输出保存在 `run.log`。`index.html` 可在浏览器离线打开。

连外壳挖空也重新计算：

```bash
bash robot_interior_postprocess/run_layout_test.sh --rebuild-shell --open
```

同一个入口可指定不同配置，输入格式仍遵循 INPUTS.md：

```bash
bash robot_interior_postprocess/run_layout_test.sh --config /path/to/robot.json --open
```

默认使用项目的 `.conda-env/voxel_tool/bin/python`；需要其他环境时设置 `LAYOUT_PYTHON=/path/to/python`。脚本可从任意目录调用。不带 `--open` 时只生成文件；手动打开终端打印的 `VIEWER` 路径即可。

在 anything2robot 项目目录下：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .conda-env/voxel_tool/bin/python -u robot_interior_postprocess/run.py
```

修改配置里的 `voxel_tool_source` 可使用不同位置的 voxel_tool 源码。需要 Python 3.11+ 和 requirements.txt 中依赖。

仅生成机械场景和中空模型：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .conda-env/voxel_tool/bin/python -u robot_interior_postprocess/run.py --stage geometry
```

调试后续阶段时，`--reuse-geometry` 可复用已有 `space_masks.npz` 和中空 STL。仅限未改变机械输入、壁厚、分辨率及电机保护参数的情况；改变这些参数必须重建几何。

输出默认放在 `outputs/cactus/`。打开 `index.html` 查看综合报告，`layout_canvas.html` 是无需联网的交互三维预览，`assembly.glb` 可在其他三维查看器中打开。

查看器使用紧凑顶栏，不再在三维视图旁显示大段说明。详细审查信息保留在 JSON 和本说明中。

电源已包含在布局中：`BATTERY` 是 8 节 AA NiMH 与电池盒的安装总成，`REGULATOR` 是降压模块。原电路的 `P1` 台式电源由导入适配器替换，不会在机器人内再摆一台台式仪器。`installed_bom.csv` 和 `power_report.json` 列出实际规划对象及供电网络；这不代表原机械电机与该模拟电路已电气兼容。

CAD 视图顶栏的 `Power` 打开 `power_view.html`，或点击侧栏 `Locate power`，可隐藏外壳并定位电池和降压板。候选电池盒源 CAD 为约 127 × 71 × 19 mm；默认规划包络现已扩大为 132 × 76 × 32 mm，覆盖模型并留出余量。旧输出中的较小包络不因此自动有效，采用新尺寸需重跑布局。

更换输入的支持范围与适配要求见 [INPUTS.md](INPUTS.md)。已下载的元件 CAD、型号对应关系与来源见 [元件模型库](assets/electronics/README.md)，打开 [模型预览](assets/electronics/viewer.html) 可以直接旋转查看。存在已转换的模型库时，流程还会自动生成 `cad_preview.html` 和 `cad_models.html`；顶栏提供入口。CAD 装配预览保留真实尺寸，不替换现有概念包络或验证结论；候选电池盒的尺寸超过旧包络，采用它需要重新求解布局。

## 主要输出

- `mechanical_scene.json`：真实电机位姿、关节轴和打印到装配坐标变换。
- `circuit_audit.json`：原始接线、BOM 检查、电机映射与模拟电源替换记录。
- `meshes/`：原始制造部件恢复到装配坐标后的副本，中空部件，内腔减材工具，明确的电机接线通道及装配模型。
- `space_masks.npz` / `space_report.json`：材料、实际构造的内腔、安装保护区、电机禁入区与保守布局空间。
- `candidate_regions.json` / `placement.json`：仅对电池、Uno、降压板、开关做候选搜索；三个机械电机始终固定。
- `motor_access.json`：跨部件接线走廊及新增孔道的记录，区分需要柔性线束的段。
- `routing.json` / `routing_validation.json`：实际调用 voxel_tool 的布线路径、失败原因及逐段体素检查。
- `geometry_validation.json`：元件与最终外壳的实体布尔相交检查、路线在实际 STL 中的采样检查。
- `manifest.json`：当前阶段、成功/失败、未放置/未布通项与验证范围。

## 建模和范围

源适配器只接受闭合、单边界、实心制造部件。只在已知实心输入上使用体素填充；绝不能用此方法填充未知的已有空壳。内腔通过距离场内缩、保留电机附近的安装支撑和网格布尔差集生成，先形成真实空腔再做布局。布局边界额外留有离散化余量。

源电路的 5 V P1 电源被明确替换为电池与降压模块：电池 9.6 V 接降压输入及 Uno VIN；降压 5 V 经原 S1 控制舵机电源。导出的 SCH/BRD 决定电机信号对应关系，配置将 SERVO2/D9、SERVO1/D10、SERVO3/D11 对应到底座、左臂、右臂。

元件尺寸和端口为**明确标注的概念包络**，电池盒等尚未实测。原机械电机不是 Tinkercad 微型舵机；三线接口用于本轮连接/布局概念验证，不构成原电机可直接这样接线的证明。

本轮不声称以下已验证：真实硬件电气兼容、盖板与装入路径、外部开关操作、连续关节运动、线束最小弯曲半径和直径、强度与打印工艺。只输出静态布局及路径方案；存在未布通或实际几何碰撞时输出 partial，不把失败作为成功结果。

## 测试

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .conda-env/voxel_tool/bin/python -m pytest -q robot_interior_postprocess/tests
```
# 后处理前的电机型号映射

机械结果读取后、空腔生成及电子元件布局前，可以通过配置给每个电机指定硬件型号。例如在配置 JSON 中添加：

```json
"motor_model_mapping": {
  "motor_base": "MG4005V2",
  "motor_l_arm": "MG4005V2",
  "motor_r_arm": "MG4005V2"
}
```

内置型号标识为 `MG4005V2`、`DM6006`、`DM8006` 和 `DM10010`。这只是型号身份目录，没有附带经验证的性能、尺寸或协议参数。可用 `motor_model_catalog` 增加自定义型号，例如 `{"MY_SERVO": {"manufacturer": "Example", "description": "待验证候选型号"}}`。

映射键是机械电机 ID，不是电路里的 `SERVO1` 等 ID；电路对应关系继续由 `motor_mapping` 配置。单轴电机 ID 为 `motor_<link>`；双轴电机分别为 `motor_<link>_axis1`、`motor_<link>_axis2`，需要分别配置型号并同步更新电路映射。

输出目录新增 `motor_model_assignments.json`，`mechanical_scene.json` 的每个电机也包含 `hardware_assignment`。没有指定的电机保持未分配，系统不会根据圆柱尺寸猜测型号；拼错电机 ID 或使用未知型号会报错。

本阶段仅分配型号，保留上游位姿与圆柱几何。ST3215、SG90 等外形不同的硬件也可指定，但这不会替换 CAD、重做安装结构或验证扭矩、碰撞、电源和控制协议；输出明确标记兼容性未验证。真实矩形包络和安装孔位需要后续几何适配阶段处理。


## 原库电机的真实 CAD 预览

现支持 MG4005V2、DM6006、DM8006（旧版 V1.0）、DM10010 的下载模型。仙人掌配置已将三个电机明确映射为 MG4005V2；入口 `index.html` 默认显示 CAD 预览，并提供 Proxy layout 对照。`cad_models.html` 可查看四种电机，`cad_assembly.glb` 是 CAD 装配导出。来源、版本、转换命令及尺寸差异见 [电机 CAD 说明](assets/motors/README.md)。原碰撞与制造几何未修改，当前只对齐电机中心与轴线，完整安装位姿仍需核验。
