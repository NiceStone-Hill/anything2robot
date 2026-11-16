# 逐步生成过程可视化

2026-10-30 模型内 schema 提示：第 5 步在真实壳体视图内叠加当前元件的中心范围紫色线框、合法安装法向绿色箭头、重力软偏好的金色箭头和点色渐变。元件下拉框与硬约束表格行均可选择实例，选择后聚焦所属刚体并高亮对应行；旋转角和最小间隙作为视图内文字说明。叠加层可关闭，表格和完整 JSON 保留。视图放大至至少 340px 高度，便于观察。

点来自保存的 `space_masks.npz`，每个刚体最多抽样 1200 个真实 usable 体素中心，已逐点检查网格配准、usable 标记和刚体归属。点色沿显式重力方向表现高/低软偏好；没有软偏好时保持青色。紫框只是 bundle 中心的轴向范围，采样点只是空腔证据，两者都不能代替整个载板的空间、碰撞、间隙和工具路径检查。没有在 schema 阶段虚构元件位姿。所有提示均是标注为 X-ray 的显示叠加，不修改 CAD 或制造网格。

第 6 步标题改为 `Reject cuts that break the shell into fragments`，增加“期望两片连通壳体，实际 7/10 个材料区域”的对照表，说明这是载板摆放和 EPS 之前的开口几何筛选。未通过候选仍为诊断预览；完整失败原因保留。概览仍是 16 步，31 个真实事件及 20 个导出 STL 不变，没有调用模型。本轮 169 项回归通过；Canvas/WebGL API mock 检查了四个元件切换、世界范围坐标、叠加层开关、schema 不被修改以及筛选计数，没有声称完成实际浏览器/GPU 截图验证。

2026-10-30 候选与 schema 明细展示：`outputs/cactus_restrict_schema_live_verified/process.html` 的第 4 步现在直接列出完整开口目录，按刚体切换或选择 `All links` 查看全部 48 项。每项显示候选 ID、VLM 排名、实际 EPS 队列顺序、法向、平面坐标/偏移、切割比例以及运行结束后的检查结果。未执行、未入队与当前载板 EPS 不支持的斜面明确区分，不能当作已通过方案。点击表格行或缩略图仅生成对应平面的三维显示标记，不生成制造切片。模型排序理由、装配说明和未知项也直接列出。

第 5 步直接列出每个元件的硬约束表：归一化中心范围、按所属内腔 AABB 换算的世界毫米范围、合法法向、旋转和最小间隙；软偏好、权重、关系、依据、未知项与本地 guardrails 分开列出。完整原始 schema JSON 默认展开；若 guardrails 改变了原始值，另列完整编译后 JSON。页面可滚动查看明细，不依赖右侧折叠的原始证据区。既有实跑与回放页面已刷新，16 步和 31 个真实事件保持不变，未调用模型或修改布局/STL。169 项回归通过，Canvas/WebGL 接口 mock 验证了 48 项候选、斜面预览和完整 schema 展示；没有进行实际 GPU 截图验证。

2026-10-30 Restrict schema 与候选布局图：当前完整示例为 `outputs/cactus_restrict_schema_live_verified/process.html`，默认 **16 步概览、31 个完整事件**。第 4 步回放历史开口排序，第 5 步真实调用现有 `gpt-6-sol` 配置生成载板 restrict schema；两者的模型归属和跳转按钮分别显示。schema 阶段展示各实例的中心范围、合法旋转、间隙和重力偏好，原始 JSON 保留在证据区。载板初始候选和每次 EPS 移动实际执行这些约束，最终再独立检查全部实例；未修改旧 `run.py` 或上游项目。

第 8 步展示三份本地生成、满足 schema 与粗筛几何的布局种子，缩略图采用同一视角和尺度，点击后显示对应的实际三维快照。它们不是 VLM 直接输出的位姿，也不宣称已经通过支撑和工具路径检查；精确检查在之后的步骤完成。图片位于 `observations/layout_candidates/`，并以内嵌 PNG 保证离线页面可用。静态并排图为 `layout_candidates_comparison.png`。

本轮真实模型 schema 生成成功，4 个载板的最终 schema 硬约束通过，14 条线布通，12 个打印件、8 个壳片与 18 颗外壳螺钉的导出几何校核通过；物理配合/承重仍待验证。回归 **169 项通过**；Canvas 与 WebGL API mock 检查了模式切换、schema 展示、三候选图片/三维切换、全部章节和模型跳转，没有声称完成实际 GPU 截图测试。

`run_generation_demo.sh --open` 现在显式回放历史真实开口排序和本次真实 schema，然后重新计算种子、EPS、支撑、布线与紧固。追加 `--live-schema` 则使用同一 `.env` 重新请求 schema；普通运行 `assemble_robot.py` 时，仙人掌配置的 `layout_constraints.mode="llm"` 也会真实生成 schema。其他泛用配置省略该配置时仍可显式采用本地规则，不能把该结果标记为 VLM 输出。详细约束契约见 [载板工作流](CARRIER_ASSEMBLY_WORKFLOW.md)。

2026-10-30 Pipeline 展示精简：同一示例页面现在默认播放 **14 步概览**，可通过顶部 `Pipeline overview / All recorded events` 切换查看原始 29 个事件。两次早期开口失败合并为一次筛选，三次 EPS 改进合并为一次优化，无电子载板的壳体分割和其余失败尝试合并到最终选择。候选卡片与几何差异默认折叠。模型归属仍逐步显示，所有标题与说明为英文。

概览顺序为：机械输出导入、空腔准备、独立载板、VLM 排序、候选筛选、电子壳体开口、载板初始布局、EPS 优化、支撑与装配通道、布局选择与其余壳体分割、线路与通道、最终支撑和壳片复核、外壳螺钉与工具通道、导出与验收报告。旧第六步是单独处理上游名为 `BODY`、未分配电子载板的刚体；现在不单独占一步。新版第六步是电子元件所在壳体的候选开口，下一步才开始放入载板。

`process_trace.json/presentation/events` 保存概览，各章保留原事件索引及所用快照的索引，原 `events` 完整保留。每章只使用实际存在的快照，几何差异重新对照上一个展示章节计算，不把最后一次 EPS 移动当作全部优化过程。多个刚体的重复阶段以最后一个实际示例代表，统一组合结果在最终选择章节展示。未完成运行只展示已发生的阶段，不补造 14 步。本轮 **150 项测试通过**；既有仙人掌过程页面已刷新，29 个原始事件不变，没有重跑布局、修改 STL 或调用模型。

2026-10-30 候选比较与模型归属更新：当前示例为 `outputs/cactus_candidate_inspection/process.html`，共 29 个真实事件。每个候选先记录实际切割面；被几何检查拒绝的切割会重新生成只读诊断切片，蓝/橙区分两侧，红色标出额外脱离材料，明确不作为可制造结果。自动聚焦对应刚体并从侧面观察，统一尺度的对比卡显示法向、切割坐标、候选比例和最终结果。三维切片默认沿法向各分开 14 mm，仅影响显示。仙人掌 base 的 C1/C2 均沿 Y 切割，坐标分别为 −5.60/−41.19 mm，相差 35.59 mm；诊断分别得到 7/10 个材料区域，两者均被拒绝，最终选 C3。静态对比图为 `outputs/cactus_candidate_inspection/candidate_comparison.png`。

每一步新增 `ALG`、`VLM REPLAY` 或 `VLM LIVE` 标签与计算归属说明。本例读取历史真实 VLM 排序，原运行记录模型为 `gpt-6-sol`；本轮没有模型调用。模型只排序已提供的开口候选，几何、EPS、支撑、布线和螺钉检查均在本地计算。回放来源、原运行记录哈希以及逐步归属写入 trace；排序步骤保留模型原始理由作为证据。本轮全量回归 **141 项通过**；元件位姿与前轮相同，全部 20 个导出 STL（含 12 个打印件）与前轮逐字节一致，导出螺丝孔道检查通过。浏览器接口使用 WebGL mock 与 Canvas fallback 做交互冒烟检查；当前环境未完成实际 GPU 截图验证。

2026-10-29 外观与英文展示更新：当前示例为 `outputs/cactus_generation_english_cad_verified/process.html`。步骤、变化文字、过程控件、最终装配查看器和螺钉装入说明均为英文；四个电子元件使用来源可追踪的 CAD 显示模型。详见 [CAD appearance references](assets/electronics/presentation/README.md)。WebGL 提供深度测试和着色，未支持时回退到 Canvas。此更新全量回归 **136 项通过**，与前轮结果比较，12 个打印 STL 文件和元件位姿保持一致。以下早期示例目录保留作历史对照。

更新：2026-10-29。`assemble_robot.py` 默认记录真实运行中的阶段几何与变化说明，不改变布局、布线、分割或紧固参数。记录范围从 Anything2robot 输出导入开始；不重演上游图像生成、关节标注或机械优化。

## 当前示例

打开 `outputs/cactus_candidate_inspection/process.html`。这是一份可直接离线打开的页面，无服务器或在线资源依赖。

- 上一步、下一步、时间滑块和自动播放切换真实阶段快照，不插值制造不存在的中间求解状态。
- 左侧列出按执行顺序发生的事件，包括失败候选与 EPS 接受的真实改进。
- 右侧逐步写明变化、计算证据、显示对象的增加/移除、位姿变化和网格变化；网格变化同时记录源材料体积差。
- 可旋转和缩放，选择刚体，隐藏盖片或外壳，切换元件、载板、支撑、线路和螺钉。默认外壳线框便于观察内部变化。
- 元件安装档案阶段的载板在工作台排开展示，坐标系为 `bench_local_mm`。切回实际装配坐标时，不把排开展示计为求解器移动。
- 被拒绝的切割使用对应候选的只读诊断几何；若诊断也失败，则显示该刚体原壳和实际尝试的切割面，并记录诊断错误，不复用其他候选的壳片。
- 外壳源模型、材料体积和哈希取自真实阶段。三维显示使用简化副本，不能代替最终 STL 的孔位检查。

仙人掌本轮记录 29 步，包含七个候选开始检查事件、两个开口切割候选被拒绝、一个 EPS 初始种子及三次接受的改进、精确支座候选、一个没有可行布局的候选、最终布局、布线、通道后重建、外壳紧固和最终检查。对比卡总结完整运行的最终选择，步骤徽章保留事件发生当时的状态。

## 生成命令

仙人掌一键演示入口（创建新目录、实际重新生成、校核导出孔道，再打开过程页面）：

```bash
bash /home/qinxuan/projects/anything2robot/robot_interior_postprocess/run_generation_demo.sh --open
```

它将计算结果的 `partial` 与几何检查的 `passed` 分别显示。省略 `--open` 只生成文件；底层计算异常或 STL 校核失败会返回非零退出码。该演示脚本默认使用下面相同的空腔缓存和模型排序回放，仍是从 Anything2robot 机械输出开始的后处理。

等价的底层命令：

```bash
ROBOT_PROJECT=/home/qinxuan/projects/anything2robot
ROBOT_POST="$ROBOT_PROJECT/robot_interior_postprocess"
ROBOT_PY="$ROBOT_PROJECT/.conda-env/voxel_tool/bin/python"
ROBOT_PROCESS_OUT="$ROBOT_POST/outputs/process_$(date +%Y%m%d_%H%M%S)"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1

"$ROBOT_PY" -u "$ROBOT_POST/assemble_robot.py" \
  --config "$ROBOT_POST/configs/cactus_carrier_assembly.json" \
  --output "$ROBOT_PROCESS_OUT" \
  --geometry-cache "$ROBOT_POST/outputs/cactus_carrier_assembly_raw" \
  --proposal "$ROBOT_POST/outputs/cactus_carrier_assembly_verified_live/proposal_raw.json"

xdg-open "$ROBOT_PROCESS_OUT/process.html"
```

本命令重新计算布局、支座、线路、外壳分割与紧固，仅复用签名匹配的空腔缓存及已保存的真实 VLM 候选排序。模型来源标记为回放；不发生新模型调用。输出必须使用新目录。退出码 2 / `partial` 表示仍有实物验收待完成，过程页面仍已生成。

记录会在各阶段立即落盘；运行期间可刷新页面查看已完成的步骤。页面的“播放”是已记录状态的回放，不是计算进度模拟。发生异常时记录失败原因和最后可用状态，并继续抛出原错误。

原命令追加 `--no-process-trace` 可关闭记录。它只影响过程输出，不调整求解配置。`run.py` 和上游工作流没有接入此记录器。

## 文件与验证

- `process.html`：自包含过程查看器。
- `process_trace.json`：阶段、实例变换、真实几何哈希/体积与简化显示资源；同一几何只存一次。
- `process_changes.md`：每一步的英文变化说明、计算归属、快照差异与证据。
- 最终 `index.html` 顶部新增“查看生成过程”；`manifest.json/artifacts` 保存过程文件位置。

```bash
ROBOT_PROJECT=/home/qinxuan/projects/anything2robot
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
"$ROBOT_PROJECT/.conda-env/voxel_tool/bin/python" -m pytest -q \
  "$ROBOT_PROJECT/robot_interior_postprocess/tests"
"$ROBOT_PROJECT/.conda-env/voxel_tool/bin/python" \
  "$ROBOT_PROJECT/robot_interior_postprocess/verify_shell_fasteners.py" \
  --output "$ROBOT_PROJECT/robot_interior_postprocess/outputs/cactus_generation_process_verified"
```

本轮全量回归 132 项通过。测试覆盖不可变快照、几何复用、工作台坐标切换、失败记录、脚本安全转义、EPS 记录前后结果一致，以及规范化机器人记录开启/关闭后的输出几何相同。仙人掌还与上一轮无过程记录结果比较，元件位姿一致，12 个打印 STL 文件逐字节一致。
