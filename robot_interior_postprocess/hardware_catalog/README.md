# 安装接口目录

这里的孔位定义与每次布局中的元件实例分开保存。孔位单位为毫米，局部 PCB 下表面为 z=0，板上方和螺丝刀进入方向为 +z。显式配准变换同时作用于板、孔、器件实体和工具，不允许缩放或镜像。

| 定义 | 数据来源与范围 |
| --- | --- |
| `pololu_d24v50f5.json` | 两个 M2 通孔与 PCB 厚度来自本地官方 STEP，孔径/孔距与官方产品页核对。已有闭合 CAD 网格用于静态碰撞和连续扫掠，恢复转换时的源中心，不按包络缩放。实际采用型号与装配位姿仍需确认。 |
| `arduino_uno_th_rev3e.json` | 从 Arduino 官方 Eagle `plain/hole` 提取四孔和板轮廓，保留原文件与 License。社区 FreeCAD 文件的孔阵列不完全相同，不能代替 Rev3e。当前顶底禁入区故意保守，尚不能据此声称 Uno 支撑可用；需匹配版本的器件实体/禁入数据后解除阻挡。1.6 mm 为名义板厚，实物应核对。 |
| `arduino_uno_th_rev3e_footprints.json` | 当前 Uno 安装模式：官方四孔、元件封装 XY、焊盘和逐个引脚禁入区。通过 `build_uno_interface.py` 从固定 Eagle 文件重建。元件高度上界、引脚伸出量、焊接和边界余量是明确记录的保守规划假设，不是原厂三维 CAD 或实测值。显示 PCB 轮廓及孔，碰撞仍使用全部禁入区。 |

原始资料的 SHA256 保存在定义中，读取时校验并纳入运行哈希。定义本身、引用的碰撞网格及实例配准配置也纳入哈希。来源 URL 用于追溯，运行不自动联网。Pololu 资产继续沿用原有来源记录中的许可状态。

官方 Eagle 的轮廓和孔可用下面的离线函数重新提取；它只读取板级孔，不把过孔、焊盘孔和封装内孔当成已确认安装孔：

```python
from hardware import eagle_board_geometry
geometry = eagle_board_geometry("hardware_catalog/sources/UNO-TH_Rev3e.brd")
```

当前仅支持单轮廓、平行于板法向的圆形通孔。曲线板边按不大于 5° 步长离散。槽孔、非平行孔、电机安装面需要独立接口定义，不能通过近似本 schema 自动转换。

紧固件参数属于实例中的 `mounting.fastener`。示例中的 M2 嵌件参数是 `prototype_uncalibrated` 候选，不是已选购的厂家料号，不能据此认定实际孔配合和强度通过。换具体嵌件时需填写其型号、外径、建议孔径、长度和试片结果，再设置制造配置。

`configs/cactus_interface_supports.json` 和 `configs/cactus_uno_mounts.json` 现为 Uno 选择 H1/H2/H4。H3 的候选螺柱与 IOH、ICSP1 引脚区相交，螺钉头及工具与 ICSP1 封装区相交，因此明确不使用；四个原始 PCB 孔仍保留。器件实例通过显式矩阵配准到原 U1 包络内，不使用预览的尺寸匹配算法。

```bash
.conda-env/voxel_tool/bin/python robot_interior_postprocess/build_uno_interface.py
```
