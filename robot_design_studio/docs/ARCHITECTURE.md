# 数据、模块与状态契约

**2026-11-15 / 0.2.0 更新：机械扩展以 [MECHANICAL_CONTRACT.md](MECHANICAL_CONTRACT.md) 为准。** 该扩展已实际填充外观、刚体、关节、安装、几何和装配字段，并接入同一任务系统。下面保留第一阶段的电气契约和当时边界；第一阶段原文另存于 `history/PHASE1_ARCHITECTURE.md`。

## 模块边界

`catalog`（版本化部件事实）→ `project`（选择和设计版本）→ `providers/worker`（候选电路）→ `electrical`（确定性判断）→ `export`（审阅和下游契约）。`server/jobs` 管理本机请求、进程、取消、超时和记录；`web` 只负责交互，不定义电气真值。

新系统不导入旧应用的全局路径、UI 状态或启动脚本。旧证据由 `legacy.read_evidence` 只读导入；几何算法后续用独立环境的进程适配，不能在当前服务中混装依赖。

## 项目表示 v1.0

- `id`：稳定项目标识；`revision`：选择/意图修改后递增的设计版本；名称不是文件路径。
- `catalog_digest`：完整目录内容 SHA256；部件同时带 `catalog_id` 和 `catalog_revision`。目录变更不会让旧结果自动继续有效。
- `instances`：每件实体部件一个稳定实例 ID；`locked` 标记用户锁定；更换型号必须新建 ID。数量由同条目实例汇总。
- `units`：m/kg/s/rad、右手系。坐标变换工具要求 from_frame/to_frame、米制平移、正交且行列式为 +1 的旋转；CAD 单位显式转换，不依文件名猜测。
- `circuit`：通过结构/逻辑错误检查的当前候选。pending 可以预览，不能放行。错误候选只在 run 中保存。
- `validation`：errors/pending，电气、几何、装配、运动线路、仿真、实物六个分项；`manufacturing_released` 在当前阶段始终 false。
- `runs/events`：任务稳定 ID、真实状态事件、时间、输入版本和结果来源。
- `source_shapes/rigid_bodies/joints/mounts/geometry_assets/motion_requirements/assembly_dependencies/harnesses`：明确分开的后续模型边界，本阶段保持空数组，未实现其编辑/计算。导入非空数组会拒绝，避免静默丢失未来数据。不能将空接口声明为已完成几何、运动学或装配模型。

一个刚体将可包含多件打印件和电子元件；拆壳不能隐式增加关节。当前未建立刚体，因此实例 `owner_body_id` 必须为空；后续加入刚体后再做引用、环路和坐标一致性校核。

每次写入产生新的不可变 JSON 历史快照，再原子替换 `project.json`。设计版本冲突返回 HTTP 409；线程锁和文件锁防止并发更新丢失。历史快照不自动删除。导入创建副本，只恢复当前支持的选择数据，清除旧校核和结果，必须重新生成。目录版本不匹配时要求显式迁移；本阶段没有自动跨目录迁移器。

## 部件事实

目录的最低字段包括身份/来源、库存、几何与单位、安装、端口、电气、质量/惯量、校准、依赖和验证。未知物理量使用 `{value: null, evidence: unknown, reason: ...}`；部分接口字段为 null，由所属 pending/physical_evidence 解释。0 仅用于实际零电位、索引等明确量。

Pico 焊接状态分别建条目，实验室实际状态为 unknown。MPU6050 模块 VCC、电平、几何和板卡版本为空，不用芯片范围替代模块范围。SG90 Digital 是当前数据贯通候选，明确无对外实际角度反馈。其他电机/供电/配件可登记候选，但不等于采购或安装确认。

目录是受版本管理的研究者维护文件；本阶段不提供参与者修改物理事实的界面。编辑事实后须更改条目版本和目录版本，保存来源，再显式迁移项目。

## 电路输出

完整结构见 `circuit.schema.json`，由 `studio/contracts.py` 生成并与同一个验证器对应。严格字段、类型、数组限额、有限电压、重复 ID/端口检查；重复 JSON key、NaN、Infinity 和未知字段拒绝。schema 是格式限制，不替代电气校核。

- `nets`：网络 ID、种类、电平范围、实例/端口端点。
- `allocations`：MCU 实例、引脚、外设功能、对应网络。
- `addresses`：I²C 设备、SDA 网络、地址。校核同总线重复、SDA/SCL 外设一致和控制器存在。
- `missing_parts`：只能请求已登记条目，或 catalog_id=null 表示需要先建库；不参与当前连接。用户回第一层确认后才变成实例。
- `assumptions`：模型或规则的假设文字，不用它覆盖目录参数。

检查器按目录重新解析端口，不信任模型自报部件能力。检查电压/极性/共地、源冲突、输入输出角色、协议白名单、必需端口、MCU 分配、I²C 地址和已知峰值电流预算。CAN/RS485/TTL 尚无登记电机方案：不会因为网络填写了协议名就被认为适配完成。总线舵机 ID、物理线径、连接器承载和完整布局一致性尚未实现；对应能力不放行。

## 后台任务

任务有独立子进程。状态：queued → running → completed/rejected/failed/cancelled/timed_out/stale；服务重启后未完成任务标记 interrupted，可由用户重试。`completed` 只说明软件任务结束，电气/制造仍可能 pending。

绑定设计版本和 active_run_id，同设计版本的较旧任务也不能覆盖更新任务。保存选择使旧结果失效。用户取消与结果提交经同一任务锁串行化。在线模型每次 35 秒网络超时，最多两次，任务总预算 85 秒；取消直接结束进程。不自动执行模型代码、不传递 API key 给前端或模型提示词、不归档 HTTP Authorization。

任务目录含 input.json、run.json、result.json；result 保留每次候选与规则拒绝原因。连接失败只记录经过裁剪的错误类别，不写入可能包含密钥的网络异常正文。

## HTTP 边界

只绑定 127.0.0.1。响应无缓存、无跨域许可；写请求需要本次会话 token，拒绝不符 Origin/Host。静态文件用固定白名单，不把任意本地文件路径暴露给浏览器。当前是单机研究界面，不声称生产级公网服务。

| 接口 | 用途 |
|---|---|
| GET /api/session | 本机会话 token、在线配置是否齐全 |
| GET /api/catalog | 可审阅目录、摘要 |
| GET/POST /api/projects | 列表、创建 |
| GET /api/projects/{id} | 恢复保存的状态 |
| POST /api/projects/{id}/selection | 乐观版本校验，替换选择并使下游失效 |
| POST /api/projects/{id}/generate | 启动 reference/live 子进程 |
| POST /api/projects/{id}/review | 独立检查专家/模型候选，不自动写入 |
| GET /api/runs/{id} | 实际进度、来源和尝试记录 |
| POST /api/runs/{id}/cancel | 请求取消 |
| GET /api/projects/{id}/export | ZIP 审阅包 |
| POST /api/import | 导入 project.json 为副本并清除旧校核 |

## 下游与后续

路由契约分开 electrical_nets / harness_topology / spatial_routes。此阶段后两项为空，端口坐标/方向/接头为空，状态固定 blocked_pending_registration，不能调用物理布线制造器。下一步需要实物部件登记与完整关节模块契约，再适配旧载板/路由算法。AI 关节标注、3D、装配生成、动力学模型、控制协议仍属于后续阶段。
