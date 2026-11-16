# 可恢复的壳体内壁线槽布线（第一版）

新入口 `route_shell_channels.py` 在载板装配结果之后运行。旧 `run.py`、
`assemble_robot.py`、`routing.py`、`carrier_layout/wiring.py` 和两线试样均不改动。
新算法在 `shell_channels/`，参数在 `configs/shell_channels.json`。

## 本版实际做什么

1. 读取最终分壳、载板包络、安装支座、电机包络、外壳螺钉/直杆工具空间、保护体素及显式端口。
2. 用各刚体的开口法向建立右手壁面坐标。射线范围、搜索范围来自壳体尺寸，无仙人掌名称、固定世界坐标或硬编码的两条线路。
3. 对同一刚体上的线路，沿可见内壁执行有预算的 A*；逐段采样，避开已有材料特征、障碍和已布线路。尝试扩大搜索净空给圆角腾出空间，不降低规定圆角半径。
4. 简化路径并在壁面坐标中生成相切圆角，再映射回真实曲面。保留输入端口锚点和初始出线方向；引线不移到“最近空气”。
5. 减材生成向开口开放的圆底槽；检查支座保护、剩余材料连通、线路实体、最终装配障碍、局部放线扫掠和采样剩余壁厚。
6. 用双精度 ASCII STL 导出并回读复查。只有通过的候选才替换新目录内的壳片，失败不会改变该壳片。
7. 导出全部输入线路的成功/失败/pending 记录、长度分解、前后对比图和离线可旋转预览。

## 恢复旧版本

- 输出目录必须全新或为空，且不能与输入相同、包含输入或处于输入之内。
- 原装配目录只读。开始时记录全部源文件 SHA256，将完整原结果复制到新目录 `baseline/`。
- 结束时再次核对原目录和 `baseline/` 的全部哈希。
- 恢复时直接使用原装配目录或 `baseline/index.html`，无需反向布尔、撤销代码或重新计算。
- 新根目录不继承旧的“已通过”验收结论；旧报告仅留在 `baseline/`。

## 测试入口

在项目根目录使用已有环境。快速测试会重新规划线槽，不复用新线路：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .conda-env/voxel_tool/bin/python -u \
  robot_interior_postprocess/test_shell_channel_system.py --open
```

从已有机械输入重跑载板搜索、分壳、原电气规划，再执行新线槽阶段：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .conda-env/voxel_tool/bin/python -u \
  robot_interior_postprocess/test_shell_channel_system.py --rebuild-assembly --open
```

两个命令都创建新的结果目录。第二个只复用签名匹配的基础空腔缓存，并明确回放已有模型约束与开口排序；不调用外部模型，不复用旧载板位置或旧线路。
它从原机械结果开始，不重跑 STL 标注、上游电机选型或原机械遗传算法。

`TEST: PASS` / 退出码 0 仅代表软件回归、源文件保全、至少一条静态新线路、STL 与输出完整性通过。每条未解决线路仍须查看 `channel_report.json`。该测试不证明全电路布通、可以制造或机器人能动。
`--min-accepted N` 可提高静态通过数量门槛，不能设置为 0。
独立的 `route_shell_channels.py` 候选导出完成后退出码为 2，表示整机仍是 partial。

## 换一个外观

先通过已有载板装配入口生成新机器人的结果，再使用同一个新入口：

```bash
.conda-env/voxel_tool/bin/python robot_interior_postprocess/route_shell_channels.py \
  --assembly /absolute/path/to/carrier_assembly \
  --config robot_interior_postprocess/configs/shell_channels.json \
  --connections /absolute/path/to/physical_cables.json \
  --output /absolute/path/to/new_channel_result
```

机械输入使用当前 carrier bundle 的毫米装配坐标、制造清单、壳体归属、基础中空/原始实体、保护掩码和开口信息。
支持任意刚体名和开口方向，但每个内壁区域目前必须是沿该开口可见的单层曲面；深倒扣、多层折叠或无法直接看见的区域拒绝。不是任意拓扑外观都保证布通。

规范化物理线束文件示例（坐标为格式示意，不能直接用于实物）：

```json
{
  "schema_version": 1,
  "units": "mm",
  "cables": [{
    "id": "wire_01",
    "net": "supply",
    "diameter_mm": 2.0,
    "evidence": "填写实测线径/线材来源；概念假设必须明确标注",
    "source": {
      "instance_id": "board_A", "port_id": "OUT", "owner_link": "housing_A",
      "position_mm": [10, 20, 30], "normal": [0, 0, 1],
      "evidence": "填写真实连接器配准依据"
    },
    "target": {
      "instance_id": "board_B", "port_id": "IN", "owner_link": "housing_A",
      "position_mm": [40, 20, 30], "normal": [0, 0, 1],
      "evidence": "填写真实连接器配准依据"
    }
  }]
}
```

省略 `--connections` 时，只能导入已有含显式锚点的旧版路由结果。导入全部分支，不按型号或电路名称选择两条样例。
载板所属刚体来自支座归属，电机端口缺少可靠归属时保持 pending；不根据电机名猜父侧/子侧。
多分支网络仍标注为规划分支，不自动变成接头。共享端点导致线缆重叠时拒绝，等待真实分线器/连接器设计。

默认测试使用有原端口记录的 Uno 仙人掌版本；Pico 2 版本尚无完整物理接线输入，不能移植 Uno 的接线结论。

## 输出

| 文件 | 含义 |
| --- | --- |
| `baseline/` | 完整原装配结果，精确复制 |
| `meshes/` | 新壳片及未改动的原载板/壳体文件；最终打印角色以制造清单为准 |
| `manufacturing_manifest.json` | 打印件、实际修改的壳片、制造未放行状态 |
| `channel_inputs.json` / `channel_config.json` | 本次线束、来源、全部几何参数 |
| `channel_routes.json` | 通过线路、原端口、壁面段/引线、检查证据 |
| `channel_report.json` | 每条输入的最终结果、失败原因和未解决项目 |
| `wire_lengths.csv` | 壁面段、引线和总几何长度；制造裁线长度保持 PENDING |
| `routing_comparison.png` | 相同成功分支在相同尺度下的新旧路径对比 |
| `index.html` | 离线装配查看器，可旋转、隐藏零件和线路 |
| `source_snapshot.json` | 所有原结果文件的 SHA256 |
| `test_result.json` | 集成测试结果（通过测试入口运行时） |

## 明确未完成的部分

- 这是静态线槽算法原型，尚未生成实际压片、线束应力释放件和打印标签；线路 ID 保存在报告中。
- 默认线径、槽深、间隙和圆角为实验参数，不是实测规格。只有壁面参数平面圆角；曲面空间弯曲半径、端口短引线转折仍须验证。
- 局部放线检查是规定距离的直线扫掠，不能代替整根线/插头的实际安装动作。
- 完整载板紧固/工具操作序列尚未在加入线缆后重验；原装配证据仅保留在 baseline，不能自动用于新布线。
- 壁厚通过移除表面顶点/三角形中心采样，不能当作连续最小壁厚或强度证明。
- 跨刚体线路不生成刚性槽桥；需要真实关节限位、两端固定点、动态线材规格及柔性线束算法。
- 布线失败还不会自动重排载板。暂时通过失败原因反馈给下一轮设计。
- 电气兼容、控制程序、整机动作、热和耐久试验均未由该入口完成。

下一步优先补全实际接线/连接器、压片与端口引线，再做一个关节的动态线束。

## 第二版：多入槽点、转弯搜索与电机映射

第二版使用相同入口，结果标记为 `shell_channels_v2`。第一版代码、配置、测试和说明已完整保存到
`backups/shell_channels_v1_before_multi_entry/`，其中 `snapshot.json` 记录文件哈希。
此前 `outputs/shell_channels_cactus_verified/` 的第一版结果继续保留。

本轮改变：

- 每个端口在多个距离环上寻找入槽点，默认搜索半径 40 mm、最多 6 个候选；候选需要同时通过壁面净空、端口引线弯曲和实体碰撞检查。端口位置和初始出线方向保持输入值。
- 引线可直接斜向入槽，也可先横向避开元件再进入内壁；中间转折使用规定半径圆角。引线与壁面线槽的连接处仍需要空间弯曲验证。
- 对最多 16 组入口组合搜索路径；某个候选在最终实体检查失败时，继续尝试其他候选，失败不会修改已交付壳片。
- 在壁面搜索走廊内选择路径简化点时，同时检查圆角、两侧切线长度和障碍。使用有预算的加权搜索，不能保证最短路径或一定找到所有可行路径。不会缩小规定半径来换取通过。
- 多候选复用壁面采样缓存；搜索节点数、转移检查数和候选数均有明确上限。
- 将电路实例通过 `circuit_audit.json`、`circuit_mapping.json` 关联到 `mechanical_scene.json` 中的电机和关节。父子关系不能决定电机本体固定侧，缺失时明确报告 `motor_terminal_registration_required`。
- 已有分支共用同一个端口坐标时，报告 `shared_terminal_requires_physical_junction`；不会把几个重叠的线实体当作独立接线完成。
- 测试报告分别给出软件测试结果、实际通过数量、`routing_coverage_status` 和整机状态，并对全部通过的线路额外复查配置规定的线间距。

电机端口可以通过 `--motor-terminals /absolute/path/to/motor_terminals.json` 提供，也可放在原装配包根目录。
格式如下，以下数值只用于说明格式，不能用于仙人掌实物：

```json
{
  "schema_version": 1,
  "units": "mm",
  "motors": [{
    "motor_id": "填写机械场景中的电机ID",
    "body_owner_link": "填写电机本体实际固定的相邻刚体ID",
    "evidence": "填写固定侧依据；设计假设必须明确注明",
    "ports": {
      "SIG": {
        "position_mm": [1, 2, 3],
        "normal": [0, 1, 0],
        "evidence": "填写装配坐标中的实际接口来源"
      }
    }
  }]
}
```

端口名必须与电路接口对应。新的端口数据会替换旧规划锚点，旧位置保存在来源记录中。
未提供的端口继续 pending；若两端属于不同刚体，继续报告需要动态柔性线束。
`--motor-terminals` 与完整的 `--connections` 二选一，避免混用两套端口定义。
`motor_registration_report.json` 输出映射关系和实际使用的端口记录。

第二版仙人掌复现命令将门槛明确提高到至少 3 条：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .conda-env/voxel_tool/bin/python -u \
  robot_interior_postprocess/test_shell_channel_system.py \
  --rebuild-assembly --min-accepted 3 --open
```

这是从已有机械结果重跑载板装配至新线槽的集成测试，不重跑上游电机选型。
通过数量门槛仍不代表整机全部接通或可以运动。

最终验证结果保存在 `outputs/shell_channels_v2_cactus_verified/`：28 项测试通过，
从机械结果重新装配后，14 条输入中 3 条通过静态布线检查；9 条待电机端口/固定侧登记，
2 条待实际分线接头。通过的线路为 `P1-:1`、`P1+:1`、`BATTERY_RAW:1`。
新增供电线的总几何长度约 364 mm 和 481 mm；绕行较长，尚未做最短线束或布局联动优化。
测试额外核对了配置的 0.5 mm 线间距、完整源结果备份和 STL 文件。
这些长度不包含已确认的接插件/装配余量，不能直接当作裁线长度。
