# 独立的装配导向外壳分割

本模块读取已存在的布局结果，向全机每个 link 提出分壳候选，使用现有 restricted schema 阶段的模型配置进行策略排序，然后生成独立分件和离线预览。它不接入、不修改原来的 run.py、布局、EPS、布线、机械输入或 voxel_tool 源码。

## 运行

从 anything2robot 仓库根目录执行：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .conda-env/voxel_tool/bin/python \
  robot_interior_postprocess/partition_shells.py \
  --layout robot_interior_postprocess/outputs/cactus \
  --output robot_interior_postprocess/outputs/cactus_partition_new
```

输出目录必须是源目录之外的全新空目录。默认读取 `robot_interior_postprocess/.env`，与现有布局 restricted schema 使用同一个 `load_agent_config()` 和 `FunctionalAnnotationClient`，不复制密钥，也不固定模型名称。`--llm-env` 可以显式指定其他配置。模型请求失败会明确报错，不自动改用规则或合成响应。

架构提示词位于 `architecture_prompt.txt`。它要求覆盖所有 link、考虑电机两侧的装配、避免只按最高元件决定水平切割，并只引用程序生成的候选 ID。模型只能排列候选，不能修改电机、元件、线路和关节，也不能提交任意 CAD 操作。响应由本地受限 schema 校验，最多请求三次完整替换。这里的 restricted schema 是本地词汇和字段校验；复用的客户端发送 JSON object 请求，不声称服务端强制 JSON Schema。

保存实际模型结果后，可离线复现：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .conda-env/voxel_tool/bin/python \
  robot_interior_postprocess/partition_shells.py \
  --layout robot_interior_postprocess/outputs/cactus \
  --proposal robot_interior_postprocess/outputs/cactus_partition_live/proposal_raw.json \
  --output robot_interior_postprocess/outputs/cactus_partition_replay
```

回放仍重新计算几何；`model_provenance.json` 明确标识 `saved_schema_replay`，不会把它称为真实模型调用。`--prepare-only` 只导出观察图、上下文和候选，不请求模型。默认 `--gap-mm 0.4`，`--grip-radius-mm 12`；这些是概念参数，不是实物制造公差或已测量手部尺寸。

## 数据契约与低耦合边界

输入是只读文件：

- `manifest.json`：沿用原配置中的所属 link 和依赖源码路径。
- `mechanical_scene.json`：所有部件、关节轴、电机两侧所属 link 及优化圆柱包络。
- `placement.json`、电子元件的 `meshes/*_proxy_mm.stl`：已有世界坐标包络，不重新解释历史尺寸/旋转约定。
- 每个 link 的 `meshes/*_hollow_with_access_mm.stl`，不存在时使用 `*_hollow_assembly_mm.stl`。

整机每个 link 都参与，不依赖其是否有电子元件。电机在父子两侧分别作为操作目标；固定电机位置不变。每个输入保存 SHA256，结束时重新核对，没有匹配就拒绝沿用结果。

计算模块 `geometry.py` 只依赖 numpy/trimesh/Manifold；只有 `model.py` 的 live 分支依赖 voxel_tool 模型客户端。提示词、输入适配、几何和展示分别独立。现有工作流无需调用本模块；以后可以消费其输出文件而不直接依赖内部函数。

## 当前能力

1. 对世界坐标方向及不同的 PCA 主方向生成候选，在各投影跨度 35%、50%、65% 处尝试平面双片分割。不是根据最高元件生成切割高度。
2. VLM 获取整机和逐 link 的三视图、毫米范围、电机轴、任务目标和候选表。红色硬件是明确标记的 X-ray 叠加，不代表从外面真实可见。
3. 每个 link 排序 3–6 个候选。几何程序全部评估，优先选择局部任务可行数较多的方案，再比较 VLM 优先级。
4. 实际布尔切分已有壳体，要求每片闭合且只有一个连通体，拒绝过小碎片和过量接缝减材。
5. 同时考虑两个打开方向。凸硬件包络沿连续直线扫掠，与保留半壳检查装入干涉；利用相对运动，反向扫掠硬件包络，与移开半壳检查完整盖板平移。两片壳自身通过分离平面保证移开时互不穿透。
6. 额外检查从硬件包络外侧伸出的假设抓持通道，避免只用元件本体判断“可装配”。这不是实际安装螺钉的工具可达性。

所有几何检查使用完整原网格，只有观察图和交互展示简化。界面的展开滑块只用于查看，不代表装配顺序或已验证的整机运动。

## 输出与状态

- `index.html`：离线交互预览，按 link 查看、展开分件、显示硬件及任务检查。
- `meshes/part_N_half_M.stl`：世界坐标分件，映射见报告中的 `piece_files`。
- `partition_report.json`：各部件选择、两个打开方向的操作检查、失败与待验证项。
- `candidate_evaluations.json`：所有尝试与拒绝原因。
- `assembly_tasks.json`：局部任务草案；全机装配序列明确 pending。
- `proposal_raw.json`、`proposal_attempt_*.json`、`model_prompt.txt`、`model_provenance.json`：可追溯模型证据。
- `observations/`、`observation_context.json`、`candidate_catalog.json`、`input_hashes.json`。

退出码 `0`：所有 link 有有效分件且全部局部包络任务通过。状态为 `local_proxy_access_clear`，不表示整机可装配或制造可用。

退出码 `2`：生成了部分或几何上有效但仍有操作失败的候选，或某 link 尚未找到有效分件。状态为 `partial`，供审查和下一轮规划。

异常退出：输入、模型、布尔操作或完整性检查失败，manifest 保留错误。不会把失败输出标记为成功。

## 明确未完成的验证

默认仍是局部、未加接缝紧固结构的双片分壳原型。通过下面的 `--support-plan` 可以启用支撑归属和联合搜索；显式 PCB 接口现支持可拆载板，见文末。尚未生成接缝定位/紧固件、局部盖板或多片切割。真实电机 CAD 与安装坐标尚未接管几何判定；优化圆柱包含连接余量，可能与原安装区域重叠，报告会保留基线相交量，不能把它说成实际电机碰撞。

真实孔位、双侧安装座完整性、主载荷路径、工具操作、关节运动、打印方向、全机连接顺序与中间稳定性均待验证。局部可达性不包括相邻 link 或其他硬件之间的干涉，也不保证已连接的线束能随盖板移动。当前代码不会改变线路来消除这些问题。

下一步应使用失败报告扩展候选/任务模板，并补齐真实电机接口；不能把 partial 自动降级成“已装配验证”。

## 验证

```bash
.conda-env/voxel_tool/bin/python -m pytest -q robot_interior_postprocess/tests/test_shell_partition.py
```

测试覆盖中途干涉、穿过电机空腔的合理分壳、元件能进入但抓持通道受阻、碎片分壳，以及模型遗漏电机连杆/编造候选的拒绝。

## 可选：支撑归属与分割联合搜索

此模式对应开发计划的第三、第四步，入口与原布局/布线流程保持独立。默认不启用，原命令继续工作。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .conda-env/voxel_tool/bin/python \
  robot_interior_postprocess/partition_shells.py \
  --layout robot_interior_postprocess/outputs/cactus \
  --support-plan robot_interior_postprocess/configs/cactus_supports.json \
  --output robot_interior_postprocess/outputs/cactus_joint_new
```

默认调用现有 `.env` 中的同一个 VLM。支撑模板、不可拆已有安装座、线路保护等进入模型上下文；模型继续排序分割策略，程序搜索具体安装壳片、模板和根部。`--proposal` 仍允许显式回放；回放旧分割提议会重新做联合几何搜索，但不代表模型曾看过新支撑上下文。

### 支撑配置

```json
{
  "schema_version": 1,
  "units": "mm",
  "settings": {"beam_width": 2, "max_options_per_target": 3},
  "targets": [
    {"target_id": "U1_1", "owner_link": "base", "templates": ["bearing_pad", "tray"]}
  ],
  "existing_supports": []
}
```

`targets` 的 ID 来自已有布局实例，`owner_link` 必须属于该元件的刚体。这个检查阻止支撑被错误指定到另一个运动刚体。`bearing_pad` 为承托板加四根射线寻根支柱，`tray` 为带侧向限位壁的托盘加四根支柱，托盘尝试两种壁高。两者都与指定壳片一体打印，接触面来自概念包络，不推断 PCB 安装孔、舵机孔、重力或材料承载能力。

搜索会尝试：候选分割 × 两个安装壳片 × 包络的六个面 × 支撑模板/壁高 × 根部射线方向，并用有上限的束搜索保留组合。支撑先生成，再复核最终带支撑壳片的装入/假设抓持通道及盖板平移。新增支撑不能挡住其他已请求元件的这些路径。不能找到支撑的目标保留在 `missing_target_ids`，不会通过省略支撑变成成功。

每个生成支撑必须满足：

- 整体位于所属壳片一侧，不能穿过分割间隙；根部与缝保留可配置距离。
- 四根支柱都与承托板及所属材料有正体积连接，合并后仍是一个材料连通体。
- 不碰硬件、相邻刚体或先前支撑，不侵入固定线路管道。
- 元件能沿该候选打开方向装入，假设操作通道不被支撑挡住。

根部嵌入深度由实际壳片射线入/出交点限制，不使用无限延伸穿过壳体。当前没有力学优化；这些是几何候选，不是承载或固定验证。

### 导入已有、不可切断的安装座

有可信几何时，将下面的条目加入 `existing_supports`：

```json
{
  "id": "servo_mount_A",
  "target_id": "motor_l_arm",
  "owner_link": "base",
  "mesh": "../assets/my_mount_world_mm.stl",
  "attachment": "fused"
}
```

`mesh` 相对支撑配置文件解析，必须是世界装配毫米坐标的单材料连通实体，并与所属原壳体有实际根部重叠。系统只整体分配给某一片壳体；分割经过它或使其失去根部时拒绝该分割，不裁切安装座。它与所属壳片合并后再次检查连通；不允许连接到相邻运动刚体。当前仅支持 `fused`，独立托架/跨缝释放接口应在后续增加真实紧固与拆装任务后支持。

没有真实电机接口时不自动生成电机安装座。`support_ownership.json` 对电机的父子两侧分别记录 `pending_motor_interface`，不会将其从整机覆盖情况中隐藏。

### 线路与输出

源 `routing.json` 若存在，使用 `points_mm` 构造连续保护管道并纳入哈希。保护半径不小于原配置线半径；不改写、不删除、不重排线路。保护只证明新支撑不碰现有静态线路，不能证明运动线束、插拔或开盖后的线缆可行。缺失路由时明确输出 `pending_no_source_routing`。

新增输出：

- `support_plan_snapshot.json`：规范化模板、搜索参数与已有支撑输入。
- `support_ownership.json`：支撑—元件—刚体—制造壳片映射、根部、释放说明和各目标覆盖情况。
- `supports/*.stl`：独立支撑几何副本，便于检查；最终 `meshes/*.stl` 已包含融合支撑，制造时不要重复打印这两份。
- `partition_report.json` 的 `support_design`：必需/已生成/缺失目标、两种打开方向的模板搜索及拒绝原因。
- `index.html` 和 `preview.png` 的绿色支撑标注。支撑随所属壳片展开；取消绿色标注不从最终实体中移除支撑。

真实固定、接缝紧固、校准、结构强度与完整装配顺序仍为 pending。几何上生成了四个承托件，不代表四个元件已经被可靠固定。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .conda-env/voxel_tool/bin/python -m pytest -q \
  robot_interior_postprocess/tests/test_joint_supports.py
```

## 真实孔位支撑、带筋承托板与可拆载板

现已增加三种显式安装接口模板，旧 `bearing_pad` / `tray` 和原配置继续工作：

- `hole_standoffs`：按选定的真实孔生成螺柱及壳体根部。根部数量随孔数变化，不强制四根。
- `ribbed_shelf`：PCB 下面的对孔螺柱融合到承托板，再沿板平面的四个方向尝试伸到壳壁，用两个三角加强筋连接。元件安装方向与壳体连接方向可以不同。
- `removable_carrier`：PCB 螺柱与载板独立打印；载板边缘另设两个螺钉孔，固定到带嵌件孔的壳体安装座。检查 PCB/载板整体的装入路径，单独导出载板。

完整 opt-in 示例见 `configs/cactus_interface_supports.json`。该配置为 Uno 官方 Rev3e 和 Pololu 候选启用孔位支撑，电池盒与开关仍采用明确标记的旧概念模板。Uno 使用官方 H1/H2/H4 三个孔，H3 因相邻 ICSP/排针空间冲突而明确排除；没有使用社区 CAD 的不同孔阵列。安装数据及保守高度假设见 `hardware_catalog/README.md`。

每个请求增加 `mounting`：

```json
{
  "target_id": "REGULATOR_1",
  "owner_link": "base",
  "templates": ["hole_standoffs", "ribbed_shelf", "removable_carrier"],
  "mounting": {
    "definition": "../hardware_catalog/pololu_d24v50f5.json",
    "world_from_interface": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
    "registration": {"status": "explicit_candidate", "note": "填写实际基准和配准依据"},
    "fastener": "此处必须使用配置示例中的完整参数对象"
  }
}
```

上面的单位矩阵仅说明字段，不是仙人掌实例的实际配准。路径相对于支撑配置；器件定义内的源文件路径相对于定义。可选 `hole_ids` 明确选择至少两个已声明孔，其余孔仍保留在板体中。选择少量孔只代表几何候选，不自动证明抗转动或承载能力。孔位模板不能与旧概念模板混合回退；数据缺失、工具被挡或根部找不到时保留目标缺失状态。

紧固参数包括螺钉直径/长度/头部、嵌件型号/孔径/外径/长度、螺柱外径和高度、盲孔底厚、最小啮合长度、工具半径及制造校准状态。程序检查孔与螺钉兼容、螺柱壁厚、螺钉不顶底、最终孔未被壳体融合回填、螺钉和工具不碰硬件/支撑/线路。嵌件安装在 PCB 安装之前检查；可拆载板按“外部固定 PCB → 整体装入 → 拧紧载板”的有限顺序检查。新增支撑不能堵住先前支撑的螺钉或工具路径。合盖还检查已安装载板和螺钉的避让。

路径检查对板体和禁入区分别构造连续保守扫掠，保留板底台阶；对闭合源 CAD 在 PCB 上下表面切层并分离连通器件，再分别使用凸包连续扫掠，避免整个元件凸包填满底部空隙。该方法是保守检查，可能拒绝依赖局部凹槽的路径；不会把它称为精确非凸扫掠。原包络模板仍使用原有凸包方法。详细报告保留所用模型与证据。

### 离线生成一个有真实 PCB 孔位的示例

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .conda-env/voxel_tool/bin/python \
  robot_interior_postprocess/make_mounting_demo.py \
  --template ribbed_shelf --output robot_interior_postprocess/outputs/my_pcb_support_demo
```

`--template` 可改为 `hole_standoffs` 或 `removable_carrier`。示例使用官方 Pololu CAD/孔位和合成方形腔体，通过明确标记的合成提议回放，无需模型服务。生成 `result/index.html`、最终壳片与支撑。源 CAD 扫掠比旧包络检查耗时更长。

对现有仙人掌布局运行：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .conda-env/voxel_tool/bin/python \
  robot_interior_postprocess/partition_shells.py \
  --layout robot_interior_postprocess/outputs/cactus \
  --support-plan robot_interior_postprocess/configs/cactus_interface_supports.json \
  --output robot_interior_postprocess/outputs/my_cactus_interface_supports
```

可加 `--proposal /path/to/saved/proposal_raw.json` 离线回放。旧提议只负责候选排序，新支撑会重新计算；不代表旧模型看过新接口。`configs/cactus_uno_partition_replay.json` 是明确标记的确定性测试排序，增加了上部 z 分割候选以验证 Uno 沿 PCB 法向装入，不是一次新模型响应。

新增支撑可以沿保护线路开避线槽，余量由 `wire_relief_mm` 指定（默认 0.5 mm，位于原保护管之外）。只扣除新支撑的材料，不改变原壳体或线路；螺柱承托区和壳体根部禁止开槽，开槽后重查连通性及线路避让。剩余筋厚、强度与实际打印配合仍待验证。

可拆载板与固定支撑在装配阶段区别处理：固定材料始终检查所有请求目标的装入通道；可拆载板若挡住下层元件的装入或工具通道，会产生“先下层元件、后载板”的依赖。局部依赖图必须无环，线路在载板安装后沿原路径接入。最终载板、螺钉和支撑仍必须与线路及其他硬件无材料干涉；不得用阶段顺序来豁免终态碰撞。未找到全部安装结构时继续输出 `partial`。

新增输出：`hardware_interfaces.json` 保存型号/来源/配准/孔/紧固参数，`mounting_plan.json` 保存安装结构与局部步骤，`fasteners_bom.csv` 保存 PCB 与载板紧固件数量。`carriers/*.stl` 是需要独立打印的载板；`fasteners/*.stl` 和 `tools/*.stl` 仅用于几何检查与显示。`supports/*.stl` 仍是融合支撑副本，不要与最终壳片重复打印。工具在查看器中默认隐藏，可按项打开；载板为青色、螺钉为黄色、工具为紫色。

这些能力验证有限的安装几何，不生成外壳接缝紧固，不证明真实嵌件工艺、结构强度、全机装配顺序或实物通过。所有未核对型号和 `prototype_uncalibrated` 参数继续保留待验证状态。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .conda-env/voxel_tool/bin/python -m pytest -q \
  robot_interior_postprocess/tests/test_mounting_interfaces.py \
  robot_interior_postprocess/tests/test_joint_supports.py \
  robot_interior_postprocess/tests/test_shell_partition.py
```

### 四电子目标联合回归

`configs/cactus_interface_supports.json` 对保存的仙人掌布局同时请求电池、降压板、Uno、开关四个支撑，输出到 `outputs/cactus_all_supports_v1/`。电池 `open_tray` 在装入方向留开口，避线槽只切新增托盘；开关仍为包络承托，两者防脱接口尚未确认。两块 PCB 使用显式注册的安装孔，载板可向四侧延伸，固定根部仍严格避让全部装入路径。

嵌件先于硬件和布线安装。对其他电子部件遮挡的螺丝刀通道、载板完整装入扫掠，生成局部先后依赖并拒绝循环；永久支撑、最终螺钉、实体与线路的碰撞检查保持有效。`tests/test_cactus_support_coverage.py` 使用真实保存布局和线路要求四个目标全部生成，检查孔位、最终占用和装配顺序，不能通过省略失败目标满足验收。电机接口与整机制造验收继续单独报告 pending。
