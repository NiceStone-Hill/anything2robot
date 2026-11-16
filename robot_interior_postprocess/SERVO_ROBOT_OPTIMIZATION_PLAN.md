# 舵机驱动动物形与人形机器人的制造、装配与稳定性优化方案

2026-10-28 实施更新：电子元件位置与线路在新授权的载板流程中允许重新求解，固定的仍是上游舵机和关节。新增 [载板装配工作流](CARRIER_ASSEMBLY_WORKFLOW.md) 与 `assemble_robot.py`，实现独立载板、局部架构/位姿搜索、支座及操作验证；本研究方案中“默认冻结全部电子布局”的边界只适用于原分壳后处理，不适用于新入口。完整承载、盖板/电机装配和实物验证仍未完成。

调研日期：2026-10-26（北京时间）。本文是基于联网资料和当前代码的设计建议，没有实现下述新增能力，也没有改动现有工作流。工程建议与引用研究的实际贡献分别说明；不把已有平台可运动视为我们的系统已验证。

目标是给定机器人外形、关节关系和舵机配置，生成可制造、可安装、可接线、可校准、可维护的分件与支撑结构，并评价最终机器人的质量分布、关节负载和指定接触状态下的稳定性。仙人掌保留为一个几何回归样例，不承担通用性的证明。

## 1. 推荐的应用边界

### 1.1 按驱动与结构定义范围

第一阶段面向：**以位置伺服关节驱动、刚性连杆组成的动物形或人形机器人，使用打印的外壳、连接件和安装支撑。** 包括双足、四足、多足，以及具有关节头部、手臂、尾部的机器人；固定基座的动物形/人形上半身作为一种明确的支撑模式处理。排除轮式、连续旋转轮驱和推进器。PWM 舵机和总线智能舵机都可以纳入，不把“舵机”等同于某个品牌、方盒外形或通信协议。

首版进一步限定为可编译成刚体树的单输出转动关节及其组合，允许被动轴承、同轴辅助支承。舵机通过外部齿轮、平行四边形、差动、绳索驱动多个关节时，需要独立的传动模块，显式给出映射、约束与装配动作；不能因全部动力来自舵机就自动声称支持。公开平台可用作设计证据，但其中的复杂子机构不必在首版完整导入。

### 1.2 “机器人类别”与“验证等级”分开

| 等级 | 能给出的结论 | 不能自动推出的结论 |
| --- | --- | --- |
| 几何与装配 | 指定模型、工具及动作下，分件/支撑/安装顺序检查通过 | 承载和运动性能通过 |
| 静态/准静态 | 给定姿态、接触、摩擦与舵机能力下，有静态平衡解和裕量 | 能动态行走或跑跳 |
| 指定轨迹动力学 | 给定轨迹/控制器下，检查接触、负载、速度、供电与热约束 | 任意步态和地形可靠 |
| 实物验证 | 指定制造配置和试验任务完成 | 未试验型号和材料也等效 |

双足可以从第一阶段的数据模型中表达，但其动态行走结论必须等待相应验证。四足的对角小跑也不能用三足慢速行走的静态判据认证。重心并非舵机机器人独有；本范围的特殊之处在于接触集合、肢体姿态和执行器负载随任务变化。MIT 的腿式机器人课程强调接触、摩擦与动力学的共同作用。[R22](https://underactuated.mit.edu/humanoids.html)

## 2. 联网调研得到的主要启示

| 资料类别 | 直接证据 | 对本项目的推论 |
| --- | --- | --- |
| Poppy | 分步组装、舵盘安装、舵机配置和接线分别记录；型号版本会改变螺母/螺纹安装步骤。[R01](https://docs.poppy-project.org/en/assembly-guides/poppy-humanoid/) | 需要型号版本和装配任务，不应只有外形尺寸 |
| Stanford Pupper 2020 | 装配手册指定零位、工具、紧固件、嵌件、访问孔及翻转装配动作。[R03](https://pupper.readthedocs.io/en/latest/guide/assembly.html) | 访问孔与装配姿态是设计变量，分壳并非解决所有工具可达问题的唯一方法 |
| Petoi Bittle | 校准状态下装入肢体，再完成偏移校准和中心螺钉固定。[R04](https://bittle.petoi.com/6-calibration) | “配置—装舵盘—校准—锁紧”需要成为有先后关系的任务 |
| ToddlerBot | 提供装配、校准夹具、执行器辨识、打印配置及 CAD 到机器人描述的配套资料。[R06](https://hshi74.github.io/toddlerbot/hardware/04_assembly_manual.html)、[R07](https://hshi74.github.io/toddlerbot/tools/01_zero_point_calibration.html)、[R08](https://hshi74.github.io/toddlerbot/tools/02_sysID.html) | 机械设计必须保留校准与维修访问；仿真执行器模型不能只使用理想位置控制 |
| ROBOTIS X 系列 | 区分输出 horn、idler、垫圈、安装孔深和线缆通道，且明确区分堵转与持续能力。[R12](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/) | 舵机需要多接口描述；螺钉长度和辅助支承都参与几何验证 |
| ST3215 | 官方给出位置模式、中心设置、接线、输出花键以及不同电压版本的堵转与额定负载。[R13](https://docs.waveshare.com/ST3215_Servo) | 同名系列与不同版本不能共用未经核实的性能/安装数据；型号目录应包含工况 |
| 模块化设计研究 | Ha 等使用执行器、支架、连接件库搜索设计；RoboGrammar 用规则限制组合空间。[R16](https://studios.disneyresearch.com/2018/07/03/computational-design-of-robotic-devices-from-high-level-motion-specifications/)、[R17](https://people.csail.mit.edu/jiex/papers/robogrammar/) | 先建立可验证接口和有限模板，再扩大形态；两项研究不能直接证明外壳自动分割已经解决 |
| 分件与装配研究 | Chopper 同时考虑分件、接缝、连接与结构；ATA/ASAP 搜索拆装路径和顺序，ASAP 还考虑中间稳定性。[R18](https://collaborate.princeton.edu/en/publications/chopper-partitioning-models-into-3d-printable-parts/)、[R19](https://assembly.csail.mit.edu/)、[R20](https://asap.csail.mit.edu/) | 应联合规划，而不是分别完成后假定兼容；校准、柔性线缆和人工工具仍需额外建模 |
| 稳定性研究 | Feasible Region 说明仅处于支撑区域内，仍可能需要超过关节能力的力矩。[R21](https://iris.unitn.it/handle/11572/335381) | 重心、接触力和舵机能力必须同时检查 |

这些资料没有提供一套能直接处理任意自由曲面舵机机器人的现成算法。下面是结合它们与当前代码提出的系统设计，不是对单篇论文结论的复述。

## 3. 当前项目与目标之间的差距

### 3.1 已经能够复用的部分

- 已有关节树、机械位姿恢复、真实内腔和几何保护区。
- 已有电子布局、EPS 和路由，可作为固定布局输入。
- 已有同一 VLM 的 restricted schema 客户端、观察图和回放机制。
- 独立分割原型已经能生成平面双片候选、做局部凸包扫掠检查、导出文件并追踪输入哈希。

### 3.2 当前不能直接用于舵机腿式通用验证的部分

| 代码位置 | 实际情况 | 优化方向 |
| --- | --- | --- |
| `script/motor_param_lib.py` | 高度、半径、最大扭矩；高度加连接余量；独立质量仍是 TODO | 新增舵机硬件目录，拆开本体、安装件和规划包络 |
| `auto_design/modules/mesh_decomp.py::generate_constraints()` | 随机姿态采样，统一速度/加速度，按名称查找 foot 并给每个匹配关节施加总重的一半 | 独立建立显式接触状态、任务姿态、接触力分配和执行器约束验证；不用旧结果认证新模型 |
| 理想 URDF 生成 | 由原始体素分布与密度估计惯量 | 从挖空、分件、支撑及硬件全部确定后的装配重新计算 |
| `robot_interior_postprocess/urdf_export.py` | 明确删除旧惯量，仅作运动学展示 | 另导出带可信质量和惯量的新物理模型，保留原预览用途 |
| `shell_partition/io.py` | 读取已有 hollow 壳体、电子代理和电机圆柱；不读取支撑归属与校准接口 | 使用明确的结构、接口和任务契约 |
| 当前分割验证 | 单 link、无紧固结构、局部包络路径；整机顺序、真实工具和结构 pending | 验证最终带支撑的零件、子总成和操作序列 |

**尤其不能把当前圆柱与壳体的重叠直接当成真实舵机碰撞。** 优化包络包含连接余量。需要把实际本体、输出连接件和允许安装接触区拆开，再检查是否真有干涉。也不能简单把重叠材料挖掉，否则可能删掉原安装座。

## 4. 核心数据：舵机适配器、任务与状态

### 4.1 舵机适配器是未来换外形的主要扩展点

| 数据组 | 内容 |
| --- | --- |
| 身份 | 厂家、型号、硬件版本、资料来源、校验值、已实测/官方/估算状态 |
| 几何 | 本体 CAD、保守碰撞包络、case 坐标、输出轴坐标和完整配准 |
| 固定侧 | 安装耳/螺纹孔/定位面、孔轴、螺纹、有效深度、可承载接口 |
| 输出侧 | 舵盘/花键/法兰型号、连接孔、中心螺钉、装入方向和角度索引 |
| 辅助支承 | idler、对侧轴承、垫圈及轴向间隙；不可统一假设所有舵机都具备 |
| 电气 | 电压、控制与供电接口、线序、插头、出线方向、实物线缆约束 |
| 性能 | 质量、局部重心、惯量或估计区间；力矩—速度、额定/堵转、持续/瞬态工况与热条件 |
| 校准 | ID/通道、通信模式、零位、角度正方向、软限位、装舵盘姿态和夹具接口 |
| 安装方案 | 可兼容的支架与紧固模板、螺钉工具和装配前置条件 |

新外形但同样是单输出转动舵机时，以增加目录和安装模板为主。双输出、多轴共壳、差动驱动等应新增模块定义，不能把多个独立输出强塞进一个单轴接口。

模型内区分 `body_geometry`、`planning_envelope`、`mount_contact` 和 `operation_envelope`。允许安装接触只针对指定接口，不是把某个舵机从全部碰撞检查中排除。质量也只计一次：本体归固定侧刚体，舵盘/输出连接件按实际所属刚体计；更精细的内部质量分配有数据时再使用。

### 4.2 明确两类状态

- **工作状态**：关节姿态、接触脚/接触面、外载、任务时长、速度与加速度、环境摩擦。
- **装配状态**：已装零件、临时连接、当前零位、通电/失能要求、夹具、工具、装配朝向及线缆连接状态。

同一个舵机在装配时可以失能或受夹具固定，在运行时由控制器保持姿态。不能把运行中的保持力矩当作所有装配步骤天然具备的支承，也不能假设任何舵机都适合手动反扳输出轴。具体动作遵循型号与安装流程。[R03](https://pupper.readthedocs.io/en/latest/guide/assembly.html)、[R04](https://bittle.petoi.com/6-calibration)

## 5. 分件与支撑的联合设计

### 5.1 以关节安装单元组织，再处理覆盖件

每个关节安装单元包含舵机本体、固定侧支架、输出连接件、可选辅助支承，以及相关紧固与接线接口。先建立这些接口能否安装、锁紧和校准的任务，再生成周围覆盖件的候选。

系统允许三种架构，按区域选择：

1. 承力安装单元或局部骨架 + 独立外罩。
2. 单片承力壳 + 检修盖/另一半外罩。
3. 明确设计连接与定位面的两片承力壳。

不要求所有机器人额外增加完整骨架，避免不必要的重量。也不允许把承力接口任意切开，靠盖板螺钉自动承担原关节载荷。关键安装基准、轴承座、螺柱根部和已知载荷路径必须有保护/复核规则。

### 5.2 支撑必须拥有归属和释放方式

每个支撑记录被支撑对象、所属刚体、所属制造零件、连接接口、载荷、安装动作和拆卸前置条件。一体打印支撑检查材料连接与根部；可拆支架检查紧固接口；跨分件支撑必须有可执行的释放动作；跨运动关节只能通过明确的关节/传动连接。

对每个候选分割：选择架构 → 分配支撑归属 → 生成支撑/载板 → 生成接缝连接 → 验证最终零件。既有不可改接口保持保护；允许重建的电子支撑可改变根部和形式，但不改变被冻结的硬件位置。

支撑候选从有限模板开始：舵机双侧支承支架、固定侧安装座、板卡柱/载板、电池托盘、传感器安装座、可拆压条。重力向下托底只能是其中一种模板；腿、手臂、尾部在运动中会改变受力方向，不能通用套用竖直向下支柱。

### 5.3 分割候选从平面扩展为有边界的策略

保留双片平面候选，同时加入局部盖板、端盖、纵向对开、多片覆盖件和独立安装座。候选由安装界面、工具轴线和自然模块边界生成，不只取包围盒固定比例。可用规则表达“承力座保持完整，覆盖件从可维护方向打开”等组合；借鉴模块化设计与规则搜索，不先追求任意自由曲面切割。[R16](https://studios.disneyresearch.com/2018/07/03/computational-design-of-robotic-devices-from-high-level-motion-specifications/)、[R17](https://people.csail.mit.edu/jiex/papers/robogrammar/)

### 5.4 三种“支撑”使用不同对象

- 安装支撑：留在机器人内，计入载荷、质量与惯量。
- 装配夹具/临时支承：只在某些步骤存在，不计入运行质量。
- 打印支撑：制造后移除，需要检查能否取出，不应误计为机器人永久结构。

最终制造件需要对应打印方向、材料和工艺参数。不能仅通过实体连通判定打印安装座强度；ToddlerBot 的公开打印说明也强调方向和配置对零件可靠性的作用。[R09](https://hshi74.github.io/toddlerbot/hardware/03_3d_printing.html)

## 6. 装配与校准：验证动作，而不仅是终态

### 6.1 有条件的动作模板

至少包括：装入本体、装 idler/轴承、装舵盘、拧中心螺钉、固定支架、接插件插拔、布置关节线束、归零/校准、安装压条、关闭覆盖件。

每个动作包含移动对象或子总成、目标接口、允许接触、路径、工具实体、前置状态及结果。不能对所有舵机固定同一套校准顺序：有的需先设置中位再装舵盘，有的在机械对齐后存储偏移。动作模板由型号/模块说明选择。[R04](https://bittle.petoi.com/6-calibration)、[R07](https://hshi74.github.io/toddlerbot/tools/01_zero_point_calibration.html)

### 6.2 检查内容

1. 加支撑后，元件/载板/子总成的路径仍成立。
2. 舵盘中心螺钉、侧面固定螺钉及需要扶持的螺母都可操作。
3. 工具是刀头、杆、手柄、扳手转动空间的组合；既有径向圆柱探针仅作粗筛。
4. 校准夹具能够放入，标记或机械基准可使用，校准后仍能最终锁紧。
5. 最后一片壳可以装回并从外部紧固，连接器可释放，线缆不会被夹住。
6. 每个中间状态有足够约束；需要翻转、扶持和夹具的步骤明确输出。

移开带元件的盖板时，移动对象应是整个子总成；不得默认元件都留在另一半壳中。线束连接着的盖板可先断开插头或受限打开，动作与线长需匹配。可逆刚性动作可通过拆卸推导装配；接线、嵌件和预紧等仍需正向检查。ATA/ASAP 为路径和顺序搜索提供基础，但不是这些舵机特有任务的现成验证器。[R19](https://assembly.csail.mit.edu/)、[R20](https://asap.csail.mit.edu/)

## 7. 重心、关节负载和稳定性的正确接入

### 7.1 使用最终装配质量

重新计算每个 link 的最终质量、局部重心和惯量：中空壳体、支撑、接缝凸缘、螺柱、螺钉、嵌件、舵机、舵盘、电子与电池都在清单内。永久材料布尔合并后避免重复计量。打印件由实际切片材料分布/用料及试件修正估计，不把有填充率的零件当成均匀实心块；CAD 不含内部结构的硬件优先使用官方/实测质量。

对姿态 q，整机重心为：

`c(q) = sum_i m_i * (R_i(q) * c_i_local + t_i(q)) / sum_i m_i`

壳体切割本身若几乎不增减材料，重心变化可能很小；改变结果的通常是新增连接结构、支撑形式、执行器和电子位置。目标不是一味降低重心，还要控制关节所需力矩、远端肢体质量和任务可达性。独立质量模型可复用 Pinocchio 的运动学、重心和逆动力学能力。[R24](https://stack-of-tasks.github.io/pinocchio/index.html)

### 7.2 用“姿态 + 接触集合”替代 foot 名称规则

每个状态明确真实接触 link、接触面/接触点、接触法向与摩擦。不依赖 link 名字，不固定两只脚平均承重。双足支持单/双足状态，多足支持给定的支撑组合；非接触的摆动腿不能贡献地面支撑。

平整水平地面的指定静态状态，可先用实际足底区域的支撑凸包与重心投影做粗筛和边界裕量检查。单脚应使用足底接触区域而非足心；对角两点支撑等退化情况不能伪造有面积的支撑多边形。多面接触、坡面和动态状态交由接触力约束判断。

### 7.3 必须加入力矩受限的静态可行性

对给定 q 和接触，寻找接触力及执行器力矩，使广义静力平衡成立，同时满足法向非负、摩擦约束、关节/执行器限值、已知结构承载条件。浮动基座也需满足平衡；不能把躯干隐式固定来“站稳”。固定基座机器人则显式给出夹持/支座约束。

`g(q) = B * tau + sum_k J_k(q).T * lambda_k`

式中符号随所用库的重力与接触力约定统一。需要对多足接触进行可行力分配，不能先假定每脚承重固定。Feasible Region 的关键启示是舵机负载约束会缩小可静态支撑的区域；Drake 的静态平衡接口也把广义力平衡、摩擦与接触共同建模。[R21](https://iris.unitn.it/handle/11572/335381)、[R23](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_static_equilibrium_problem.html)

### 7.4 性能数据、任务与不确定性

不能用堵转力矩认证持续站立；记录电压、持续时间、温度、速度和供电条件。ROBOTIS 明确区分堵转和持续表现；ST3215 文档也区分堵转与额定负载。未知持续能力不统一猜成“堵转的某个固定百分比”，而是保留待辨识或实测状态。[R12](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)、[R13](https://docs.waveshare.com/ST3215_Servo)

首版任务包含中立站姿、抬臂/伸腿、下蹲和指定慢速支撑切换；具体姿态由模型与任务给出，随机姿态只能补充搜索。对临界姿态进行局部细化，不把有限采样称为连续全空间证明。

质量、重心、摩擦、装配误差和执行器能力带来源与范围，报告对误差敏感的方案。负载评估还需关注峰值/持续供电、压降和发热；关节线束验证弯曲半径、松弛与扫掠，不沿用静态中心线布通就认定运动布线成功。

动态验证是后续独立等级：需要有效的 q(t)、速度/加速度、接触计划和控制/执行器模型；浮动基座与接触一致。不能给所有关节随意相同速度/加速度，也不能以重心在凸包内替代完整动态结论。可辨识舵机响应再进入仿真，参考 ToddlerBot 的方法；输出显式惯量，避免由显示/碰撞代理自动推断错误质量。[R08](https://hshi74.github.io/toddlerbot/tools/02_sysID.html)、[R25](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-inertiafromgeom)

## 8. VLM 的职责与受限 schema

继续使用原 restricted schema 阶段的同一模型与环境配置。给模型的输入增加：舵机接口编号、支撑/承力区域、当前任务状态、零位与校准视图、姿态下的重心/接触/负载结果，以及已有失败证据。

模型输出选择与偏好：分件策略、安装单元模板、支撑归属、子总成组织、允许的装配姿态、建议的操作前后关系，以及需补充的数据。数值接口、材料强度、载荷能力和“已通过”状态不能由模型覆盖。已知硬约束由程序维护，模型新提出的要求区分待确认建议和已确认约束。

从“给几个平面排序”升级为“选择有接口的结构和装配策略”，但仍只引用允许的模板/对象 ID。架构提示词应要求：先处理承力和运动接口，再处理操作与校准，再处理维护和视觉；对固定侧和输出侧分别推理；优先尝试支撑/盖板归属变更，不擅自移动舵机和关节。

几何失败应给出对象与动作，例如 `center_screw_tool_blocked`、`support_root_crosses_split`、`calibration_fixture_unreachable`、`closure_traps_wire`；物理失败包括 `contact_force_infeasible`、`continuous_torque_unknown`、`mass_properties_incomplete`。模型可以重提策略，不能把这些失败改为软惩罚后通过。

JoinABLe 展示从 CAD 推断连接关系的可能，但它主要处理成对零件的 CAD 关节，且依赖 B-Rep，不能代替整机路径、紧固、校准与动力学验证。[R26](https://www.research.autodesk.com/publications/joinable-learning-bottom-up-assembly-of-parametric-cad-joints/)

## 9. 搜索与低耦合架构

建议新增独立的装配设计后处理入口，消费原工作流的文件快照与新增硬件目录，保留现有分割器和支撑生成器作为候选求解器。

```text
原机械/布局/布线结果 + 舵机目录 + 任务与接触状态
  → 数据完整性与坐标检查
  → 关节安装单元、保护接口、质量初估
  → VLM/规则给出架构、分件与支撑候选
  → 粗几何筛选与支撑归属检查
  → 精确生成支撑、接缝定位与紧固结构
  → 装配/校准/接线序列验证
  → 最终质量、力矩和接触可行性验证
  → 候选局部调整；失败证据回传；预算到达则保留 partial
  → 制造文件 + 装配步骤 + 质量/稳定性报告
```

离散变量是模板、支撑归属、分件拓扑与动作顺序；连续变量是分割偏移、支架尺寸、支撑根部和已允许的局部几何参数。先用有限候选、束搜索/局部搜索及缓存，不把全局最优作为第一版承诺。先淘汰硬约束不满足的方案，再对装配步骤、维护拆件数、质量、舵机利用率、接缝与打印成本比较，保留几种权衡方案。

原布局、舵机位姿、关节和线路默认冻结。新增质量和物理验证可以在独立模块中完成，不要求立即重写上游。只有无法通过分件和支撑解决时，输出结构化变更请求，明确需移动电池、换舵机或修改接口的原因；不自动改变原结果。未来若授权联动，才重算受影响的上游步骤。

新增输出建议为 `servo_catalog_snapshot.json`、`assembly_design.json`、`support_ownership.json`、`assembly_sequence.json`、`calibration_plan.json`、`mass_properties.json`、`contact_scenarios.json`、`static_feasibility.json`、`manufacturing_parts/` 和 `upstream_change_requests.json`。每类验证单独保存 passed/failed/pending、输入版本与证据。

## 10. 推荐实施顺序与验收

| 阶段 | 交付 | 验收依据 |
| --- | --- | --- |
| A：硬件与状态 | 两种包装/安装方式不同的具体舵机适配器；显式关节、接触、质量、校准数据 | 更换型号通过数据/模板接入；未知项不填造；与旧电机包络区分 |
| B：局部制造与装配 | 单关节安装单元、支撑归属、接缝紧固、舵盘/工具/校准动作 | 打印试件完成固定、校准、锁紧、关闭与拆出；没有隐式切断安装座 |
| C：整机联合后处理 | 子总成顺序、多候选分壳/支撑、保留线路检查、最终质量与静态可行性 | 至少四足和双足各一个模型；首版范围内的动作全部可追溯 |
| D：泛化测试 | 新外形、新舵机、跨类别留出集和失败集 | 冻结内核与模板版本后测试；不为测试模型添加专用 ID 判断 |
| E：动态任务扩展 | 接触计划、执行器辨识、任务轨迹、动态与热/供电验证 | 明确轨迹与工况下的仿真、实测对照；不把静态通过升级为会行走 |

首批基准建议由四足、六足/多足、双足、固定基座人形上半身以及仙人掌自由曲面例子组成。公开 Pupper、Poppy、ToddlerBot、Petoi、PLEN2 用于提取任务与可支持子机构；整机兼容性须逐例确认。两种舵机尽量同时覆盖传统单侧输出与带 idler/辅助支承的总线舵机，具体型号在数据完整性核验后确定，不按宣传扭矩选定。

必须包含失败回归：根部被切断、跨关节支撑、中心螺钉不可达、舵盘无法按零位装入、夹具放不进去、最后一颗螺钉被盖住、线路被压、质量重复计入、四足错误使用两足承重、重心粗筛通过但力矩受限不可行、换型号沿用旧孔距/缓存。

统计几何分件成功、完整任务完成、工具/校准可达、维护拆件数、最终质量与重心误差、最差任务静态裕量、执行器负载、搜索预算和实物试装结果。不给没有完整硬件资料的模型记作制造成功；有限姿态检查与动态验证分别记录。

**优先级建议：先完成舵机真实接口与质量数据，再完成单关节可装配试件和支撑归属，随后接入整机接触/静态负载验证，最后扩大分件策略与动态能力。** 当前几何分割原型保留为候选生成和对照基线。

## 11. 资料索引与使用边界

下表整理 26 份一手项目资料、官方文档或原始研究。它们不是 26 套完整解决方案；同一平台的不同文档分别提供装配、校准、打印或辨识证据。访问日期为 2026-10-26。网页、型号和硬件版本应在实际导入时再次固定；此处没有下载或重新分发外部 CAD。

| 编号 | 一手资料 | 本方案的使用方式与限制 |
| --- | --- | --- |
| R01 | [Poppy Humanoid assembly](https://docs.poppy-project.org/en/assembly-guides/poppy-humanoid/) | 检索可见分步装配与型号差异，直接重取遇 502；不从该页推断孔位 |
| R02 | [Poppy wiring arrangement](https://docs.poppy-project.org/en/assembly-guides/poppy-humanoid/wiring_arrangement) | 接线总线和最后关闭外罩的流程证据，不直接复制其供电架构 |
| R03 | [Stanford Pupper 2020 assembly](https://pupper.readthedocs.io/en/latest/guide/assembly.html) | 可读；零位、舵盘、工具、访问孔和嵌件；明确采用 2020 文档，不能与后续平台混用 |
| R04 | [Petoi Bittle calibration](https://bittle.petoi.com/6-calibration) | 校准状态、安装肢体和最终紧固；仅适用于对应版本的动作 |
| R05 | [ToddlerBot original paper](https://arxiv.org/abs/2502.00893) | 整机设计、辨识与可复现性的研究依据，不声称自动完成外壳设计 |
| R06 | [ToddlerBot assembly](https://hshi74.github.io/toddlerbot/hardware/04_assembly_manual.html) | 可读；文档同时涉及不同版本，导入时须固定版本 |
| R07 | [ToddlerBot zero-point calibration](https://hshi74.github.io/toddlerbot/tools/01_zero_point_calibration.html) | 可读；校准夹具与任务 |
| R08 | [ToddlerBot system identification](https://hshi74.github.io/toddlerbot/tools/02_sysID.html) | 可读；执行器辨识思路，参数不跨型号复制 |
| R09 | [ToddlerBot 3D printing](https://hshi74.github.io/toddlerbot/hardware/03_3d_printing.html) | 可读；方向与配置应随零件保存，不采用其打印机推荐作为本项目要求 |
| R10 | [ToddlerBot Onshape to robot descriptions](https://hshi74.github.io/toddlerbot/tools/04_onshape_to_robot.html) | 可读；CAD 到 URDF/MJCF 的数据链 |
| R11 | [ROBOTIS OP3 introduction](https://emanual.robotis.com/docs/en/platform/op3/introduction/) | 可读；集成智能舵机人形架构与安装特征；页面含不同硬件配置，本文不选取其价格/电池规格 |
| R12 | [ROBOTIS XL430-W250 e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/) | 可读；安装接口、idler、螺钉深度、线缆和性能定义；不套用到全部舵机 |
| R13 | [Waveshare ST3215 official documentation](https://docs.waveshare.com/ST3215_Servo) | 可读；不同版本、中心设置、接口和负载定义；具体性能需匹配型号与工况 |
| R14 | [PLEN2 official project](https://github.com/plenprojectcompany/PLEN2) | 官方仓库检索内容确认打印、装配和 home 配置；没有导入其 CAD 或验证整机 |
| R15 | [InMoov hand official page](https://inmoov.fr/inmoov-hand/) | 检索摘要确认不同舵机布置，全文 403；仅作为舵机布置多样性的补充索引，不作为尺寸/绳驱验证证据 |
| R16 | [Ha et al., Computational Design of Robotic Devices from High-Level Motion Specifications, 2018](https://studios.disneyresearch.com/2018/07/03/computational-design-of-robotic-devices-from-high-level-motion-specifications/) | 可读；模块库与搜索，不是任意外壳自动设计 |
| R17 | [RoboGrammar, SIGGRAPH Asia 2020](https://people.csail.mit.edu/jiex/papers/robogrammar/) | 作者页检索摘要与元数据可读，直接重取失败；借鉴图规则约束搜索，不声称其验证舵机装配 |
| R18 | [Chopper, SIGGRAPH Asia 2012 — Princeton record](https://collaborate.princeton.edu/en/publications/chopper-partitioning-models-into-3d-printable-parts/) | 大学原始成果记录可读；分件多目标，不能直接保证带硬件的装配 |
| R19 | [Assemble Them All, SIGGRAPH Asia 2022](https://assembly.csail.mit.edu/) | 可读；拆卸推导装配、路径搜索 |
| R20 | [ASAP, ICRA 2024](https://asap.csail.mit.edu/) | 可读；中间稳定性与执行约束，不代表人工接线/校准已经被覆盖 |
| R21 | [Orsolino et al., Feasible Region, IEEE T-RO 2020](https://iris.unitn.it/handle/11572/335381) | 作者机构成果记录可读；关节力矩受限的支撑可行域 |
| R22 | [MIT Underactuated Robotics — Highly-articulated Legged Robots](https://underactuated.mit.edu/humanoids.html) | 可读；接触、摩擦、ZMP 与动态建模；不是只看静态重心 |
| R23 | [Drake StaticEquilibriumProblem](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_static_equilibrium_problem.html) | 官方接口可读；参考约束构成，不要求替换现有动力学库 |
| R24 | [Pinocchio official project features](https://stack-of-tasks.github.io/pinocchio/index.html) | 官方特性页可读，较早文档；确认已有算法能力，不据此固定当前 API |
| R25 | [MuJoCo official XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-inertiafromgeom) | 可读；显式/自动质量惯量及重复显示/碰撞几何的处理 |
| R26 | [JoinABLe, Autodesk Research, CVPR 2022](https://www.research.autodesk.com/publications/joinable-learning-bottom-up-assembly-of-parametric-cad-joints/) | 原始研究页可读；成对 CAD 连接推断，与整机可装配性有明确差别 |
