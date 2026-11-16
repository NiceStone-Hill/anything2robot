# 来源、改写与许可

此目录是 2026-11-15 开始的新实现，受仓库根目录 `../LICENSE` 的 Non-Commercial Research License 约束，不新增更宽松许可。

- `studio/electrical.py::voltage_range` 根据本仓库 `robot_interior_postprocess/electrical_review.py` 改写为独立纯函数。保留有限数、有序区间和拒绝布尔值的算法逻辑，异常及中文提示适配新契约。本轮重跑旧电气相关测试并增加新失败案例。
- 物理登记、受限模型输出、过程来源等设计借鉴同仓库 `physical_wiring/registration.py`、`semantic_bridge.py`；新服务没有直接加载其外部几何依赖。仍需在研究发表时依原仓库许可署名。
- 厂商资料只用于少量参数事实及链接，来源见 `docs/SOURCES.md`。库状态区分厂商资料、候选目标和实物未知。
- 未复制旧电机/Pico 的 CAD、STEP、网格及其第三方许可证。旧资产公开可下载不表示拥有任意再分发权。
- API 服务仍只用标准库；机械 worker 使用独立环境中的 NumPy、SciPy、Trimesh、Manifold3D、Pillow。实际版本见 requirements-geometry.txt 与 evidence/geometry-environment.json；未打包分发依赖。前端无 CDN 或外部字体。

## 0.2.0 机械方法

- Anything2Robot 的最近连杆分解、模块占用、旋转干涉方法，以及本仓库后处理的交叠体积和保守抽取扫掠，在 `studio/mechanics/` 独立适配/重实现。具体函数、限制和许可边界见 `docs/MECHANICAL_METHODS.md`。
- Text2Robot 作者 generalroboticslab，固定提交 `1203e8717a27fdc68928790746436ccd2e40cb8e`。核对作者的组件生成源码与装配手册后，重新实现 SG90 工程试件的电机盒、连接桥与电子盒方法；没有复制/缩放其 STL 或机械孔位。Apache-2.0 原文在 `docs/sources/Text2Robot-LICENSE`；手册文字抽取仅用于可复核研究记录。
- 原作者：Ryan Ringel、Zachary Charlick、Jiaxun Liu、Boxi Xia、Boyuan Chen，Text2Robot，2024。出处：https://github.com/generalroboticslab/Text2Robot 。本项目新代码仍受仓库原有研究许可约束，第三方原始资料保留各自许可。
