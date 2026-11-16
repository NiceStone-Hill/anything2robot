# 单关节机械生成：方法、来源与边界

2026-11-15，Ubuntu，0.2.0。该版本是实际执行的工程试件生成器，不是通用形态机器人系统或制造放行器。

## Anything2Robot 落地方式

| 来源 | 新实现 | 真实计算与边界 |
|---|---|---|
| `auto_design/modules/mesh_decomp.py`，`Mesh_Decomp.decompose` 最近连杆分配 | `studio/mechanics/solid.py::link_labels` + `pipeline.py` | 用有限线段距离分配源网格顶点；首个直线单关节问题的分区边界为平面，随后对源实体执行精确布尔切分。没有调用旧体素聚类全链，没有声称支持任意凹形分解。 |
| `auto_design/modules/motor_opt.py::General_GA.get_occupancy_cost` | `pipeline.py` 五个轴位置候选 | 从完整盒体、压盖、电机和舵盘假设包络计算外露体积，并计算两侧壳体/盒体余量；选择有效候选。不是原 GA，也不是裸圆柱拟合。 |
| 同文件 `InterferenceRemoval.rotate_around_axis/remove_interference` | `solid.py::rotate`、`pipeline.py` | 围绕同一关节轴旋转，记录初始分缝的碰撞；由截面与角度推导扩大分缝并实体去料；对最终装配在 17 个角度做布尔碰撞检查。独立测试用错开半步的 16 个角度复查。离散检查不保证连续无碰撞。 |
| `robot_interior_postprocess/shell_partition/geometry.py::overlap/convex_sweep` | `solid.py` 中独立适配实现 | AABB 预筛后求布尔交叠体积；以端点凸包保守检查直线装入/抽出空间。未直接加载旧程序状态、历史结果或应用入口。 |

保留仓库根目录 Non-Commercial Research License 的适用范围；上述是方法适配/重实现，不宣称原论文全算法已迁移。

## Text2Robot 核对结果与实现

作者仓库：<https://github.com/generalroboticslab/Text2Robot>，固定提交 `1203e8717a27fdc68928790746436ccd2e40cb8e`，Apache-2.0；作者 Ryan Ringel、Zachary Charlick、Jiaxun Liu、Boxi Xia、Boyuan Chen。

实际读取的资料：

- `README.md`：Fusion 360 中转换/切割、机器人模型和硬件模块之间的关系。
- `Fusion360_Scripts/Wrapper/utils/generate_robot.py` 与 `SliceBody2.py`：核对父子组件、关节创建、`createMotorBox` / `createKneeBox` 的连接区域填充和切除。特别记录：`SliceBody2.py` 中部分 `createMotorBox` 调用处于注释状态，不能以函数存在推断原作者的当前入口已执行这些步骤。
- `Assembly Instructions.pdf`：MotorCase、KneeConnector 与肢体的连接/装配顺序、电子盒内板卡固定、供电分支和舵机归零。手册指定 Hiwonder HTD-45H 八舵机，README 的 PyLX-16A 控制库名称不能当作实际硬件型号。
- 仓库列出的 `STL_Files/Electronics_Modules/{MotorCase5,KneeConnector,MotorHornforMetal,BatteryBox}.stl` 确认模块职责；本实现没有复制这些网格或缩放其孔位。

本地归档手册抽取文本 `sources/text2robot-assembly-extracted.txt` 和许可 `sources/Text2Robot-LICENSE`。新模块完全在 `studio/mechanics/modules.py` 生成：

1. **电机盒**：SG90 候选本体包络的开口盒、可拆压盖、线口、通孔、螺母窝、接到固定壳体的承力肋。连接完成后再钻孔，防止加强筋填回螺孔。
2. **输出连接模块**：配套原装舵盘假设包络、中心孔、可调槽、纵向连接臂与接到运动壳体的横肋；无打印花键。桥与壳体布尔合并后的连通性及原始接触体积均检查。
3. **电子盒**：底板、侧壁、可拆盖、固定耳、板卡扎带槽、壳体内支撑座及螺孔/螺母窝；可沿开盖方向取出。并不声称 Pico 2 / MPU6050 已完成孔位配准或固定。

这是将其模块、连接区域和装配方法转化为新的参数化生成器，不是移植 Fusion 360 API，也没有沿用四足/八电机/对称假设。为避免冒充测量值，`catalog/mechanical_templates.json` 把本体资料值、设计假设和实测未知分开；旧电子目录摘要不变。

## 一次任务的实际步骤

1. 导入用户上传的 STL/OBJ 原始字节或应用生成的简单 OBJ；保存摘要。只接受单个闭合凸形实体，最多 5 MB / 40000 三角面；不静默修补、不以凸包替换凹形输入。
2. 显式转换源单位，保留比例缩放外观，使最长坐标轴对应 X，建立右手坐标。不会缩放电机本体和孔位。当前长度 180–400 mm、截面 65–140 mm；最终空间不足会被拒绝，并非全部范围内的形状都保证成功。
3. 由真实源网格渲染三个正交视图，保存 `views.json` 的像素投影矩阵。在线模型接收视图、尺寸、意图，返回受约束的单关节位置比例、Z 轴、父/子区域、自由度、幅度、理由和不确定项。当前搜索范围只有长轴中部 45–55%，不声称通用三维语义标注。
4. 筛选完整模块轴位；按源外表面的半空间向内偏移 2.4 mm 生成真实等距凸形内壳，分出固定与运动两侧；按运动空间去料。为保证完整模块装入，外观可能明显改变，记录去料比例。
5. 生成连接、支撑、装配开口和可拆件。最终 7 个打印候选实体 + 2 个硬件假设包络；每个实体有模板、版本、所属刚体、变换和摘要。
6. 检查每个实体水密/单实体连通、三处实际承力接触、最终零件互碰、关节采样扫掠、装入/拆卸通道、工具空间、压盖孔道。
7. 保存单件 STL（mm）、装配 GLB（m）、预览网格（m）、项目/模板快照、原始外观、多视图、装配步骤和检查。固定与运动网格共用关节静止坐标，顶点已定位，因此静止装配矩阵为单位阵；运动只施加一次子刚体旋转。`source_to_joint_mm` 是带单位转换/缩放的导入仿射矩阵，不冒充纯刚性变换。

## 明确尚未证明的事项

- SG90 安装耳、轴偏置、轴高、配套舵盘孔位、螺钉长度/啮合、附加轴承、实物公差与承载。硬件包络是设计假设，**不是实测 CAD**。
- 真实电子件在盒内的布局、端口配准、分线实体、带插头穿线、动态线长/弯曲/夹线、重心/负载、连续碰撞、打印/装配/实机运动。
- URDF/MJCF 物理模型、MuJoCo、Pico 控制/校准、2–6 关节、Cactus/Dog/凹形泛化仍是后续实现工作。本轮没有用假质量或空字段声称完成它们。
- 拆卸扫掠是几何保守检查，不包含手指、柔性变形、螺钉拧紧过程或所有工具姿态。每个检查会记录先移除的部件和假设工具尺寸。

即使全部软件几何检查通过，产物仍为 `engineering_fixture`，`manufacturing_ready=false`。
