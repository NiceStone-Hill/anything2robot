# 所有分割外壳的螺钉连接

## 2026-10-28：相似结构共用孔位与贯通装入通道

当前结果为 `outputs/cactus_shell_access_consistent/`，替代下文较早的 22 组连接示例。本轮全量测试为 **128 项通过**；8/8 片外壳都有连接，共 **18 组**：主体 4、底座 6、左右手臂各 4。原四块载板与 14 条线路仍通过检查。复用前轮真实 VLM 排序，未新增模型调用。

- 使用实际分割截面的归一化轮廓、尺寸比例、双向轮廓距离，以及平面内旋转/镜像配准发现相似结构，无仙人掌名称规则。默认最大尺寸比 1.25、归一化距离 RMS 0.07、95 分位 0.20。`shell_fastening.pattern_policy` 可覆盖这些参数或 `enabled=false` 做对照；阈值只是形状分组启发式，所有候选仍在各自原尺寸几何中验证。
- 同组采用共同候选搜索；某位置和螺钉长度必须在所有成员上通过后才能接收。对应编号具有相同归一化位置、孔径、接头形式和完整螺钉规格。形状不完全相同的部件采用归一化对应，不声称孔位相隔毫米数相同或打印件可以互换。共同候选不足时明确失败，不静默改成各自孔位。
- 盖片采用默认 **Ø8 mm 贯通装入孔**，一直从外表面通至螺钉头座；头座距外表面最多 12 mm。承载半壳仍保留嵌件盲孔与孔底。整颗螺钉及直杆工具扫掠均检查；加入 75 mm 直杆、Ø6.4 mm 工具杆包络、Ø24 × 45 mm 手柄包络。实际持握不是已完成的人体工学证明。
- 螺钉采购规格为 **ISO 4762，M3，螺距 0.5 mm，圆柱头内六角，全牙，钢制 12.9 级**，头径 5.5 mm、头高 3 mm、2.5 mm 内六角；长度按头下长度计算。需要全牙版本以保证接近端部的嵌件啮合。尺寸依据：[Würth M3×25 全牙产品说明](https://eshop.wuerth.de/Hexagon-Socket-Head-Cap-Screw-full-thread-ISO-4762-DIN-912-steel-129-plain-with-full-thread-SCR-CYL-ISO4762-129-HS25-M3X25-25/008393%2025.sku/en/US/EUR/)。长度变体按输出采购规格核对，不把该 M3×25 商品编号用作所有长度的编号。
- 本轮数量：M3×10 两颗，M3×12 三颗，M3×16 九颗，M3×25、×30、×35、×40 各一颗。左右手臂对应长度顺序均为 16、10、12、16 mm。

新增 `shell_fasteners.csv`（逐孔坐标与型号）、`shell_fasteners_summary.csv`（型号数量和孔号汇总）、`shell_fastener_assembly.html`（从外部装入的步骤与剖面关系示意）。查看器标出入口，点击“对准螺钉入口”会合盖、隐藏螺钉和标注、朝向该孔轴，便于看到实际开口。

可直接运行：

```bash
cd /home/qinxuan/projects/anything2robot
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .conda-env/voxel_tool/bin/python -m pytest -q robot_interior_postprocess/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .conda-env/voxel_tool/bin/python robot_interior_postprocess/verify_shell_fasteners.py --output robot_interior_postprocess/outputs/cactus_shell_access_consistent
xdg-open robot_interior_postprocess/outputs/cactus_shell_access_consistent/index.html
```

预期：128 项测试通过；STL 校核返回 `status=passed`、`shell_pieces_connected=8`、`shell_screws=18`，输出 `shell_fastener_verification.json`。独立脚本重新加载最终打印 STL，检查全直径贯通孔、整颗螺钉与工具扫掠以及共同模板与孔轴坐标的一致性；本轮最大材料交叠仅约 `1.42e-6 mm³`，低于 `0.02 mm³` 几何容差。外壳仍是制造原型，实际嵌件、打印补偿、持握、拧紧和承载待验证，总体状态继续为 `partial`。

以下保留上一版本实现说明，数量以以上新回归为准。

实现更新：2026-10-28。此功能接入独立载板入口 `assemble_robot.py`，保持现有元件布局、舵机位置、载板与线路。原 `partition_shells.py` 仍为旧分壳对照入口，没有自动接管新紧固功能。

## 当前生成的结构

每个 link 的两片外壳共同生成：

- 承载半壳内侧的实体连接柱，融合到有正体积重叠的壳体材料上。
- 连接柱的嵌件盲孔，保留孔底材料；当前为需要选型和试片标定的 M3 原型配置。
- 盖片的同轴通孔和圆柱螺钉头座；螺钉从外侧装入。
- 环形定位凸台与留有间隙的对应定位孔。
- 跨越名义切割间隙的环形止挡。盖片在止挡上落座，避免拧紧时压缩间隙并改变最终装配尺寸。

不增加单独打印的外壳连接块：连接柱、定位结构、止挡和孔已进入最终外壳 STL。原来的四块电子载板仍独立打印。

## 如何选择位置

程序读取实际分割截面和原始实体外形，沿轮廓采样并向内寻找连接位置，尝试不同内缩距离、头座深度和给定螺钉长度。连接数量随截面周长调整，默认每个 link 至少 3 组、最多 6 组；位置必须分散且不能全部共线。

每个候选检查：

1. 两片上的连接材料都真正连壳；孔、定位配合与头座保留配置要求的材料空间。
2. 螺钉夹持长度、啮合长度和盲孔底厚相容；头座有可接触的平面材料，止挡有正的有效接触面积。
3. 新连接不碰元件、载板、已有支座、线路、邻近外壳或电机保护区。
4. 不挡住载板装入、载板固定螺钉与支座嵌件的原操作路径。
5. 外壳螺钉的杆和头分别进行连续直线扫掠，外侧螺丝刀通道可达。
6. 嵌件可在电子载板安装前装入；所有连接生成后再复核邻近外壳的新结构是否阻挡工具和螺钉路径。
7. 原始半壳保持分离平面约束；新肩部止挡终止于盖片内平面，只有声明的定位环继续伸出。定位环直径小于对应孔径，高度小于孔深，因此卸下全部螺钉后，盖片沿打开方向平移会减少配合深度。不能把这一证明用于任意倒扣、唇边或未知跨缝结构。

候选不足时明确失败并输出 `shell_connections.json` 的拒绝原因，不把无孔盖片标记为已连接，也不丢弃壳片。

## 配置与运行

当前仙人掌配置已启用：

```json
"shell_fastening": {"enabled": true}
```

同一入口的默认值也启用外壳连接。可配置参数及校验位于 `carrier_layout/closure.py` 的 `DEFAULTS` / `settings_for()`，未知字段、尺寸不相容或非有限数值会拒绝。`enabled=false` 仅用于对照，输出仍会保留未连接壳片的待完成状态。

继续使用之前的完整测试命令即可生成外壳连接，无需另跑打孔命令。调试时可以显式回放上轮真实 VLM 的候选排序，重新计算全部几何；`model_provenance.json` 保留 `saved_schema_replay`，不称作新的模型调用。

规范化机械输入如果含有贯通开口，必须额外提供 `meshes/<link>_solid_assembly_mm.stl` 原始实体外形。只有独立、闭合的正向外边界和负向内腔边界存在时，程序才允许自动取得外形包络，不把任意已有空壳填实。Anything2robot 原始输入直接提供恢复装配坐标后的原始实体。

## 输出与查看

- `shell_connections.json`：连接点、两端壳片、世界孔轴、螺钉长度、盲孔、定位间隙、止挡接触面积、工具检查和逐片覆盖。
- `shell_fastener_map.png`：实际截面轮廓上的孔轴位置图，标注螺钉编号与长度。
- `shell_disassembly.json`：逐 link 的卸螺钉、沿指定方向退出定位结构和打开盖片步骤；全机夹具/舵机装配仍单独待验。
- `fasteners_bom.csv`：新增各 link 的外壳螺钉和嵌件数量。
- `assembly_sequence.json`：每个 link 都有安装外壳嵌件、定位合盖、外部拧紧步骤，包括没有电子元件的肢体。
- `manufacturing_manifest.json`：实际要打印的 8 片外壳和 4 块载板；固定连接结构已融合，不重复打印。
- `index.html`：可按 link 查看，取消“显示螺钉”便于观察通孔、头座和定位孔；青色标记外壳连接结构。

验收要求是 `all_shell_pieces_fastened=true`，而且 `shell_connections.json/piece_coverage` 每一片壳体都有至少三组对应连接。这里表示连接几何已生成并通过所述检查，不代表实物已拧紧或结构强度通过。

当前仙人掌回归为 22 组外壳连接：`BODY` 4 组，`base`、`r_arm`、`l_arm` 各 6 组，8/8 壳片都有连接。原载板路径和 14 条线路保持可行。螺钉/嵌件厂家规格、打印补偿、定位实际配合、拧紧力矩和承载仍需要试片及实物校验；总体继续为 `partial`。

本轮结果目录：`outputs/cactus_shell_fasteners_final/`。完整测试集 123 项通过。独立重新加载 12 个打印 STL 后，均为闭合正体积实体；22 组螺杆及螺钉头所需空间与最终壳体的交叠体积为 0，逐片覆盖为 8/8，32 个装配任务无依赖环，27 项输入文件哈希保持一致。该轮复用上轮真实 VLM 的排序，没有新增模型调用。
