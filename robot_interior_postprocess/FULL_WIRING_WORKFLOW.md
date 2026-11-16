# 从机械输入到全部线缆候选的集成测试

这个入口补上此前只测试“物理接口资料检查”、没有实际重新生成线路的问题。旧的载板、槽道和物理接口入口全部保留。

在项目根目录运行：

```bash
cd /home/qinxuan/projects/anything2robot
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .conda-env/voxel_tool/bin/python -u \
  robot_interior_postprocess/test_full_wiring_system.py --open
```

每次创建新目录。入口依次运行：

1. 软件回归测试。
2. 从保存的机械设计输入重新生成中空外壳、载板位置、支座、分壳、紧固与原电气适配数据。不复用旧中空缓存、载板位置或线路。
3. 生成候选电路及真实接口缺项表。
4. 根据明确写入配置的设计假设，为尚未选择的接口、电源分配等模块预留实体空间，为所有外部线束提出出线点。
5. 在实际装配的壳体、支撑、紧固工具空间、电机和部件包络之间搜索全部线束的路径。保持所提议的端点不动，检查弯曲半径、实体碰撞和导出后的几何；拥塞时重新排序布线。
6. 尝试将空间线路替换成内壁槽道，检查其他线路、新模块和壳体加工条件。失败时保留空间候选并说明需要线夹或独立走线通道，不丢弃该线路。
7. 输出三维查看器、全部线路与逐项结果、模块空间需求、接口假设、线长表和固定需求。

保存的开口和布局约束建议会明确回放。本入口的起点是机械设计结果，**没有重新运行更上游的外形标注、电机选型和机械遗传算法，也没有做实物测试**。

## 两种不同的完成状态

- `candidate_path_coverage_complete`：所有候选外部线束都得到路径。测试只有在这一项为真时才返回 0，不能用“至少一根线通过”代替全部线路完成。
- `physical_routing_complete`：实物接口、内部线束、安装与运动等验证完成。当前仍为 false。

`configs/harness_design.json` 明确记录 CAN 候选、模块占位尺寸、线径、圆角半径和其他假设。它不等于用户确认选型，也不是厂家规格。真实针脚到所提议出线点的插头、延长尾线、固定侧等仍须设计验证。总线线束内部主干和分支同样不能由外部路径覆盖率自动认定完成。

候选规划不生成可以绕过 `route_physical_wiring.py` 的真实硬件放行凭据。

## 通用性

路径搜索只依赖装配空间、拓扑、组件包络、端点和约束，没有仙人掌坐标或三电机硬编码。模块大小与线材要求来自独立配置。当前输入适配器仍是已有 Uno / Pololu / 总线电机候选组合；更换其他硬件需要提供相应接口资料和适配器。

不同外形可使用以下参数指向自己的机械装配配置及布局建议：

```bash
.conda-env/voxel_tool/bin/python robot_interior_postprocess/test_full_wiring_system.py \
  --assembly-config /absolute/path/to/assembly.json \
  --schema-proposal /absolute/path/to/layout_constraints.json \
  --opening-proposal /absolute/path/to/opening_proposal.json \
  --design-config /absolute/path/to/harness_assumptions.json
```

不保证任意外观一定布通。空间不足、缺乏端点候选、跨关节而没有运动定义等情况仍会明确失败并返回 2。

## 输出

`carrier_assembly/` 保存重新生成的装配；`physical_wiring/` 保存接口与电气检查；`harness_candidate/` 保存完整候选规划：

- `index.html` / `assembly_view.html`：三维装配与全部候选路径。
- `routes.json` / `routing_report.json`：路径、端点、导体映射、通过数量及全部失败项。
- `design_assumptions.json` / `terminal_proposals.json`：所有假设及尚未验证的实物接口连接。
- `component_proposals.json`：模块占位和拟安装位置；新模块的实际载板仍待生成。
- `routing_iterations.json`：拥塞重排过程。
- `channel_options.json`：每条空间线路的开槽尝试及回退原因。
- `wire_schedule.csv`：几何线长；接头、压接与服务余量未确定时不填制造裁线长度。
- `retention_plan.json`：固定位置和线槽/线夹需求，尚未作为可直接打印的固定件放行。
- `meshes/`：候选线缆实体、模块占位和通过检查的改槽壳片；并非所有文件都是打印件。

旧结果不覆盖。源装配在候选规划前后按文件校验；实际硬件未确认时，所有输出均保留明确状态，不把候选空间路径当作“整个机器人已能制造和运动”。

## 公共主线束与就近分支（可选新策略）

针对逐线规划绕行太多的问题，增加 `harness_bundle/`。新策略先为全部线缆预留足够粗的公共通道，把模块逐个接入已有主干，再合并距离过近的分叉点。各段根据实际包含的线缆计算截面，保留每条线的电气身份；共用套管不表示电气并接。

在现有装配上对比两种策略：

```bash
cd /home/qinxuan/projects/anything2robot
.conda-env/voxel_tool/bin/python robot_interior_postprocess/route_bundled_harness.py --open
```

可用 `--candidate /absolute/path/to/harness_candidate` 指定其他模型的候选结果，`--config` 指定自己的线束参数。默认用已保存的仙人掌结果。旧结果和原壳体保持不变，新策略暂不切槽。

从机械输入开始重跑，并追加新线束策略：

```bash
cd /home/qinxuan/projects/anything2robot
.conda-env/voxel_tool/bin/python -u robot_interior_postprocess/test_full_wiring_system.py --bundle --open
```

新三维页面默认显示整体共享线束，可选择单条线路，查看它所经过的共享段。`network.json` 保存每段成员、截面排布、尺寸、路径以及模块集中出口；`report.json` 保留碰撞和待完成项。

**范围区别：** 为减少模块周围散乱的线头，新策略提出每模块集中出线位置，并保留旧出口记录以便比较。没有把旧的58个分散候选出口当作已测量接口，也没有声称连接了真实插头。集中出口到真实接头的短接线、每根线在分叉及弯折中的具体走向、线夹/导线槽、强弱电兼容和运动仍待验证。新的通道覆盖不能与旧版端点覆盖直接等同，更不能替代实物完成状态。

该版本目前使用同一刚体内的静态通道；跨关节线束会明确拒绝，等待运动定义。粗通道放不下时也会失败，不会自动把线束画细。

### 电机线路、颜色与平滑

集中线束的新页面默认显示电机包络，并按真实保存的“电气实例 → 机械电机”映射核对电机线路，不能用名称猜测。仙人掌 CAN 候选中，每个电机对应供电、回线、通信线对，共9条候选线缆；通信线对内部包含两条导体连接。

- “每根线独立颜色”：各条候选线缆各有颜色，点击图例可高亮整条线路。
- “按功能分色”：供电红色、回线蓝色、通信绿色、控制紫色。
- “仅电机线路”或选择某个电机：检查它的全部连接；电机表也可点击。
- 白色虚线：集中出口与电机模型包络之间的**未规划距离参考**，不是接头位置、制造裁线长度或已通过碰撞检查的线路。

彩色分带用于解释共享线束中的成员，不代表各根线在截面内和分叉处的实际空间排布。

平滑会优先尝试更大的实际圆角，保留端点，并重新检查装配碰撞、非连接处分支重叠及导出几何；不能通过时保留上一条经过检查的路径。`smoothing_report.json` 记录采用的半径及拒绝原因，`motor_coverage.json` 记录逐电机接线与缺口。分叉连接处每根线的连续弯曲仍待设计。

## Pico 2 主控与真实来源模型

完整测试入口现在默认使用 `configs/cactus_pico2_wiring.json`：无排针 Pico 2 的新载板布局、Pico 接线与集中线束。运行：

```bash
cd /home/qinxuan/projects/anything2robot
.conda-env/voxel_tool/bin/python -u robot_interior_postprocess/test_full_wiring_system.py --controller pico2 --bundle --open
```

旧 Uno 入口保留，可显式使用 `--controller uno`。自定义装配配置仍可通过 `--assembly-config` 指定；使用自己的配置时必须配套检查 `--design-config`。

Pico 候选采用稳压器 5 V 输出 → 防倒灌保护模块 → VSYS（物理脚39）；GND脚38；通信接口使用3.3 V逻辑。当前 CAN 候选：MOSI=GP19/脚25、MISO=GP16/脚21、SCK=GP18/脚24、CS=GP17/脚22、IRQ=GP2/脚4。Pico不具备本方案所需的独立CAN接口，外部控制器及收发器仍待选型。接口3.3 V负载预算、固件和电机协议尚未验证，不能把候选接线当成通电指导。RS485候选另行输出，电机版本仍未选择。

模型来自官方 Pico 2 STEP、现有稳压板/电池盒/开关模型及配置中的候选电机 CAD。使用原毫米比例和装配变换，不根据占位框缩放。详细模型仅更新外观，碰撞规划仍使用明确的包络；模型安装方向、轴向定位和实物版本还需确认。

`appearance_report.json` 逐项记录来源、比例、坐标变换及缺项。尚未确定型号的通信模块、保险、主断电开关、分线器、保护模块和总线附件仍是预留空间；界面可勾选查看预留框，不能伪装成已选定的实体硬件。原有小开关模型也不代表新增主电源断路器已经选定。
