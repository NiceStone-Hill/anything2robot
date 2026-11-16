# 通用电气规划与机械布局的接口

电气能力由输入数据定义，不根据机器人名称、元件 ID 或控制板名称选择接线。仙人掌只是一组集成测试输入。

## 数据与流程

1. 建立元件档案：准确型号、版本、资料来源、端口、电压范围、协议、连接器和缺失项。机械库中的外形与最大扭矩不能代替采购与电气兼容性证明。
2. 定义逻辑网络：每个端点使用显式 `instance_id` / `port_id`；分别记录供电域和通信协议。开关、保护器件和转换模块应显式出现。
3. 执行 `electrical_review.py`：检查端点存在、重复接线、电压范围、供电源冲突、信号驱动冲突及协议标签。资料缺失保留为 pending。它不计算电流、浪涌、热、回生能量或固件兼容性，也不是完整 ERC。
4. 元件实物版本和电路确定后，准备真实局部端口坐标、插头及操作空间，生成规范化布线输入。逻辑 pin ID 不自动成为真实三维连接器锚点，不能移动到最近空气伪造端口。
5. 以“元件＋载板＋接口”重新搜索布局，执行几何布线与最终材料检查。新元件、开关开口或供电改变后不能复用旧线路成功结论。
6. 交付原理图、接线表、线束拓扑与制造候选。需要定制转接板时才进行真实封装放置、PCB 走线和设计规则检查；不能把逻辑网络文件改名为 BRD。

`electrical_review` 输入为 `schema_version=1`，包含 `components`、`nets` 和可选 `pending`。元件含 `id`、`model`、`evidence`、`ports` 和可选 `pending`。端口 `kind` 为 `power_in` / `power_out` / `passive` / `ground` / `signal_in` / `signal_out` / `bus`，可声明 `voltage_range_v`、`protocol`、`physical_pin` 和 `required`。网络通过 `terminals=[{"instance_id":...,"port_id":...}]` 连接端点，可声明电压范围或协议。

电压范围表示在该节点要求覆盖的完整工作范围，不是只有标称电压的兼容性声明。转换器、开关及保护器件两端是不同端口；当前检查器不会自动推导穿过这些器件的电压、状态或瞬态，必须在网络数据中声明并用独立计算验证。

## 运行

```bash
.conda-env/voxel_tool/bin/python robot_interior_postprocess/electrical_review.py \
  --plan /path/to/electrical_plan.json \
  --output /path/to/electrical_review.json \
  --html /path/to/wiring_review.html
```

退出码：1 为已知冲突，2 为仍有待补项，0 仅为上述有限检查通过。任何状态都不会得到 `complete_electrical_design=true`。

装配配置可以设置 `electrical.review_plan`，相对于配置文件解析。装配入口在机械计算前执行检查并记录输入哈希；已知冲突停止运行，缺失项进入装配验收报告。此字段可与 `none`、`normalized` 或已有电气适配器并用。旧适配器未指定该字段时保持原行为，不代表其电气兼容性已验证。

## 当前边界

- 新检查入口是通用的，但尚未实现电气自动选型或自动综合电路。
- `normalized` 路由器仍需要经过配准的端口和几何输入；本次没有把逻辑接线表自动接入物理路由。
- 布线/外壳紧固失败自动反馈到另一架构、跨刚体元件重新分配仍未实现。
- 开关必须最终验证外部操作可达性和实际安装接口，不能仅凭内部包络可放置就认定可用。
- 当前供电检查不自动验证 USB 等第二电源路径或反灌保护；这些必须显式建模并验证。
