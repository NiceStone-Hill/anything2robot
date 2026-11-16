# 0.2.0 机械项目与运行契约

这是 `ARCHITECTURE.md` 第一阶段契约的扩展，替代其中“几何字段永远保持空数组”的阶段性描述；旧文档已归档到 `history/`。

## 边界与坐标

`mechanical_service` 用标准库保存外观和受限配置 → `jobs` 启动独立 `mechanical_worker` → `mechanics/planning` 渲染/推荐 → `solid/modules/pipeline` 实体生成和检查 → 原项目保存通过的软件结果。新 worker 不导入旧应用全局状态。

- `source_shapes`：原始文件 ID、相对文件名、原文件名、显式单位、SHA256、字节数。不允许前端输入服务器路径。
- `mechanical_config`：长度 m、用户最大幅度（UI 传入 deg）、电机真实实例 ID、推荐模式、自然语言意图。任务内最终关节限位统一 rad。
- `rigid_bodies`：固定与运动两刚体、静止变换、所属资产 ID。可拆盖是零件，不另建关节。
- `joints`：稳定 ID、父子刚体、轴、原点 m、限位 rad、电机实例引用。所需力矩为 null，未声称载荷通过。
- `mounts`：固定电机座、运动输出桥、电子盒支撑的模板/资产/刚体引用和真实交叠接触体积 m³；电机座引用用户所选电机实例。板卡实际安装位置尚未登记。
- `geometry_assets`：模板版本、所有者、静止矩阵、m 制包围盒/体积、STL 文件名与摘要、打印候选标记、材料状态。硬件包络不是打印件。模板来源快照与生成代码摘要保存在 manifest。
- `assembly_dependencies`：有序操作及所引用零件 ID；检查报告另记录每次抽取前必须移除的零件。
- `mechanical_result`：当前采用的 run ID、几何状态、制造 false。失败任务可查看其输出证据，但不把失败刚体列表写入当前项目。

生成网格的顶点已经在统一关节静止坐标中定位。固定/运动刚体静止矩阵均为单位阵；运动只旋转子体一次。STL 带 mm 约定、GLB 为 m、预览为 m，源导入的仿射矩阵另列。不能对已有定位顶点再套一次源变换。

## 版本、失效与恢复

修改部件/外观/机械设置递增全局 revision，清除当前旧结果，保留不可变项目历史和每个 run 输入/输出。任务完成必须同时满足原 revision 和 active_run_id 匹配；取消、过期、超时、重启中断均不采用。

同一 revision 可以先生成机械、再生成电路，并一起导出；生成计算本身不递增 revision。后台最多两个任务，后启动任务的 active_run_id 防止前者覆盖。机械超时 180 秒，电路保持 85 秒。

部件目录 16 条目的旧摘要保持不变；机械模板是同一 `catalog/` 中独立版本化文件，没有通过更改旧目录摘要破坏已保存的阶段 1 项目。机械模板和部件目录快照都随包导出；候选紧固件只提出请求，不偷偷加入用户的采购/库存实例。

原项目工作区可重开。带非空机械数组的单文件 JSON 跨工作区导入仍拒绝，避免忽略缺失资源；原始外观文件可重新导入。资源完整导入/目录显式迁移尚待实现。

## 正式 HTTP 接口

- `POST /api/projects/{id}/appearance`：revision + 文件名、unit、base64，或 `builtin: box` 和尺寸；上传上限 5 MB。
- `POST /api/projects/{id}/mechanics`：revision + length_m、amplitude_deg、motor_instance_id、mode、intent。先核对已选择型号，再建立输入版本和任务。
- `GET /api/runs/{id}`：真实事件、进度、输出状态和校核。原取消接口继续适用。
- `GET /api/runs/{id}/artifacts/{name}`：只提供该运行 manifest 明确登记的文件；不支持任意路径读取。
- 原项目 export：同一设计版本时合并机械与电气审阅资料，再核对实体摘要。物理路由仍 blocked。

## 前端

三个正式页面：部件选择、电路接线、外观/关节/结构。按需加载 preview.json；WebGL 缓冲复用，包围盒载入时计算。Ubuntu 环境无法创建 WebGL 时，直接对同一实体顶点做软件投影/深度排序；动作变换和刚体归属相同。静止时只在交互/尺寸变化后绘制。软件深度排序是显示手段，不用于碰撞真值；碰撞仍由 worker 布尔实体检查。
