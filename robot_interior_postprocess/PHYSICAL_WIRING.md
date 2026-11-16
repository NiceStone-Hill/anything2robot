# 实际硬件接线准备（physical_wiring_v1）

这一步放在载板装配之后、实际线缆布线之前。电机尚未确定，因此系统同时输出 CAN 和 RS485 **候选方案**，不会把原配置中的 MG4005V2 自动认定为实际硬件，也不会生成缺乏依据的电机针脚坐标。

旧 `route_shell_channels.py`、v1 快照、v2 仙人掌结果保持原样。新功能使用独立入口和输出目录，已有非空目录不允许覆盖；输入装配文件逐个校验，生成过程只读。

## 运行测试

在项目根目录运行：

```bash
.conda-env/voxel_tool/bin/python robot_interior_postprocess/test_physical_wiring_system.py
```

这会运行接口与布线回归测试，并读取已保存的仙人掌载板装配，生成一个新的 `outputs/physical_wiring_cactus_<时间>/index.html`。软件测试成功返回 0；真实硬件状态仍为待确认。测试范围覆盖“已有装配 → 候选接线 → 接口登记 → 布线入口检查”，不重新运行上游外形生成、运动仿真，也不代表机器人已能制造或运动。

单独生成候选方案：

```bash
.conda-env/voxel_tool/bin/python robot_interior_postprocess/build_physical_wiring.py \
  --assembly robot_interior_postprocess/outputs/shell_channels_v2_cactus_verified/carrier_assembly \
  --output /tmp/my-physical-wiring-plan
```

若资料仍不完整，该命令返回 2，并留下完整检查结果。这是“待补充”，不是程序崩溃。

## 输出内容

- `interface_catalog.json`：带来源和文件校验值的候选接口库。Uno 官方 PCB 的 32 个排针焊盘分别记录，其中 3 个 GND 有不同物理位置。焊盘 XY 不作为插头出线点。
- `motor_candidates.json`：原装配中电机实例与机械关节的对应关系，实际选型保持未确认。
- `controller_pad_references.json`：焊盘在装配 PCB 基准平面上的参考坐标；不代表插头高度或出线方向。
- `legacy_compatibility_audit.json`：旧三线舵机接法与 MG4005V2 候选的差异。如果选择资料中的驱动版本，不能继续使用旧 5 V 电机电源或把 SIG 当成总线针脚。
- 每种总线目录的 `logical_plan.json`：候选模块、逻辑端子、外部连接和分线器内部导通关系。CAN 需要控制器和收发器，RS485 需要收发器；具体模块尚未选择。
- `carrier_requirements.json`：新增接口、供电、保护、分线及总线线束等部件的安装需求。尺寸和安装孔未知时保留为空，不会生成虚构载板。
- `registration_template.json`：实际型号、版本、安装归属、坐标变换、物理针脚、接头、出线点、线径、弯曲半径和证据的登记模板。
- `readiness.json`：逐条缺项及错误；`routing_contract.json` 绑定登记结果与该装配的文件指纹。
- 只有登记数据全部满足静态布线要求，才生成 `connections.json`。

仙人掌当前候选包含 CAN 29、RS485 28 条**外部线缆／线束连接**。总线差分对作为一束处理。这不是最终实物线束数量，也不覆盖待设计的总线主干内部走向；选定模块、自带终端、电缆接头或集成供电方式后需要调整拓扑。

## 资料来源与选型范围

`interface_catalog/candidate_profiles.json` 记录来源链接、范围及本地 PDF 校验值。MG4005-i10v2 / DF40V2 的候选电压和通信类型来自厂家目录第 51 页（经销商托管）；安装图只能支持机械参考，不能推出接线针序。Arduino 使用已保存的官方 Eagle 板文件，Pololu 使用官方产品资料。

目前提供 Uno Rev3e、D24V50F5、MG4005V2 CAN/RS485 的候选适配器。规划不依赖仙人掌名称、外壳坐标或固定电机数量；其他外形可使用同样的已完成载板装配输入。换用其他控制板或电机，需要添加有来源的接口资料及对应拓扑适配，不能仅修改显示名称。

电源方案是待评估候选：电机电源经保护、开关、分配到各电机；5 V 模块只给通信电子部分供电；Uno VIN 使用另一个符合其推荐范围的控制器电源。具体电源、保护器件、电流容量、压降、再生能量、USB 共存均未验证。

## 补齐真实硬件后

1. 确定电机及通信版本，取得与实际版本一致的手册、针序、插头料号、CAD 和线缆资料。选择不同型号时先更新接口适配器。
2. 选定接口模块、电源、分线器等实物；把真实尺寸、安装孔和位置纳入载板装配。`carrier_requirements.json` 是需求清单，当前不会自动给未选部件建模。
3. 复制对应方案的登记模板，填入型号版本及依据。`owner_link` 指实际固定的壳体；电机安装侧不能仅凭关节父子关系推断。局部坐标单位为毫米；坐标变换必须刚性且与装配一致。
4. 每个逻辑端子登记真实 `connector_id`、`physical_pin`，以及装配后插头外部的 `cable_exit_id`、位置、单位方向和依据。Uno 的 `physical_pin` 保留 `POWER:8` 等完整标识。差分对的两根导体必须分别占用针脚，但共享经登记的线束出线点、位置和方向。
5. 为每根外部线缆或线束填入总外径、最小弯曲半径、来源。现有模板只将差分对合束；其他多芯电缆需要在拓扑中明确合束，不能让多根独立电缆暗中复用同一出线口。
6. 用 `--registration <填写后的JSON>` 重新生成到新目录。通过后，运行新的物理布线入口：

```bash
.conda-env/voxel_tool/bin/python robot_interior_postprocess/route_physical_wiring.py \
  --assembly <对应的装配目录> \
  --physical-plan <新输出目录>/CAN \
  --output <新的布线输出目录> \
  --config <槽道配置JSON>
```

槽道配置的 `bend_radius_mm` 必须不小于所有登记线缆的最小弯曲半径。资料缺失、针脚复用、总线导体短接、版本或安装归属冲突、装配变动、文件校验不符都会阻止相应入口继续。

**验证边界：** 登记证据是用户提供、可追溯的数据，不是自动证明测量正确。静态端点登记通过也不代表电流、温升、EMI、通信协议、终端电阻或运动可靠性通过。新增部件、总线主干、接插件可插拔空间及跨关节活动线束仍需真实设计和检查；原槽道算法继续负责其覆盖范围内的静态几何判定。
