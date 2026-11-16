# 体积分区被反复拒绝的修复

用户项目“我的表达机器人”最近五次关节标注的位置/三视图校核均通过，但 partition_solids 失败。根因是二值体素场的精确 0.5 等值面存在点/边接触的鞍点歧义，默认 marching cubes 输出非流形表面。这是几何生成失败，不能解释为用户意图不可行或 AI 轴心错误。

## 实现

新增 mechanics/voxel_surface.py。优先维持 0.5 原等值面；仅当闭合/实体或 STL 重载失败时，重算 0.499 等值面。保留全部占用体素的标签和所有表面组件，不删除碎片、填洞或使用全局凸包。每次重算检查实体闭合与 STL 浮点重载，并测量双向顶点到对方三角面的距离，要求最大采样偏差 ≤0.4 格距。该采样检查不是连续 Hausdorff 证明。

0.001 阈值变化很小，但鞍点区域的三角化拓扑变化可能带来更大的局部表面位移，不能将它误报为 0.001 格距的整体几何误差。用户 500 mm 输入的格距为5 mm，实际最大采样偏差约1.7713 mm；源件不变，提取报告随分区保存。它是带明确精度界限的语义分区，不代表最终制造精度。正常闭合头部分区保持0.5，不改变边界。

标注与结构生成共享该提取器，避免下游重复生成同一种坏网格。下游运动切除、支承和打印件拓扑仍有此前记录的独立问题。

界面将关节位置校核、三视图一致性、体素归属、分区实体有效性逐项显示。失败记录刷新后仍可查看。增加“沿用关节，重新生成分区”按钮：只复用同项目、相同源摘要与标注配置下，已通过位置与投影检查的失败任务；重新校核原关节图并生成分区。恢复入口不接受客户端关节坐标，不能引用外项目任务，保留版本冲突与任务状态校核。新任务明确记录 reprocessed_annotation、原 run_id 和本次 live_model_called:false，不冒充重新调用 VLM。

## 实际验证

- 原五次失败任务 run_fd2dc081a437447a、run_ae2e1613257e4ed2、run_ed73ea62e12e478e、run_7d53e2a6da06422a、run_da1b36a362374b96，复用各自原图重算，分区均通过实体与 STL 重载检查。涵盖两部位和三部位图。见 examples/partition_fix_2026-11-15/recheck.json、evidence/partition-recheck.log。
- 正式服务恢复用户项目 project_de0ac637b0364432（我的表达机器人），任务 run_904ffbcdd70f43a4，约4.345秒，annotation_valid、joint_annotation_valid、partition_valid均为true。原失败任务保留。本次没有调用在线AI，后续电机选择已开放。
- 正式HTTP测试拒绝跨项目标注且没有改变版本；恢复任务与项目采用结果一致。见 evidence/partition-http.log 和 examples/partition_fix_2026-11-15/formal/。
- 独立Firefox：实际打开原项目成功结果；在复制工作区显示旧失败明细，点击恢复按钮，轮询完成后显示新标注和电机选择。见 evidence/partition-browser-qa.log、formal/failure-explained-browser.png、formal/recovered-button-browser.png。早先试图从浏览器测试环境直接调用页面模块函数因模块作用域失败，失败日志 partition-browser.log 保留，改为真实页面按钮操作。
- tests.partition_geometry：复现随机二值鞍点问题；改变格距后检查体积按尺度立方变化、原占用中心保持内部、闭合和STL重载；普通块不触发偏移；空分区拒绝。
- 共14项几何回归通过，64项核心回归通过。见 evidence/partition-geometry-tests.log、partition-core-tests.log。

这次修复仅解决关节分区这一阻碍。制造、全装配、线束及真实硬件校准状态不因此升级。旧系统、原始模型、原运行记录均保留；当前修复未推送GitHub。
