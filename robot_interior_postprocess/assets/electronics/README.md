# 元件 CAD 参考库

下载地址、上游提交号、许可证、文件大小和 SHA-256 见 [sources.json](sources.json)。原始 CAD 保留原坐标，没有替换现有碰撞包络。这个目录不代表物理电路已经可以装配运行。

打开 [viewer.html](viewer.html) 可以旋转、缩放真实 CAD 网格，并通过下拉框单独查看元件。页面无需联网。`converted/` 保留完整转换后的 PLY、STL、GLB，以及供交互显示使用的简化 PLY；颜色用于区分几何，不是原厂材质。

装配结果中的 `cad_preview.html` 将四个电子模块的 CAD 按原摆放位姿显示，保持真实尺寸。`cad_registration.json` 记录轴向配准和尺寸超出情况；这个视图未重新验证 CAD 碰撞或实物端口。原电机仍保留机械输出的包络，SG90 仅出现在单独模型库里。

Uno 的旧 FreeCAD 文件中，一个 LMV358 子对象无法被 CAD 转换器读取，预览省略了它；其他可见子对象已转换。详情记录在 `converted/models.json` 的 `omitted_source_objects`，没有用方盒伪造该子对象。

| 对应元件 | 文件 | 与当前方案的关系 |
| --- | --- | --- |
| U1 控制板 | `arduino_uno_r3/arduino_uno_r3.FCStd` | FreeCAD 社区 Uno 模型；应核对实际 R3 板版本、接口和接线空间。可用 FreeCAD 打开并导出 STEP。 |
| REGULATOR 降压板 | `regulator_d24v50f5/d24v50f5.step` | Pololu 官方 D24V50F5 模型；排针和线束空间还需核对。 |
| BATTERY 电池盒 | `battery_holder_8aa/battery_holder_8aa.step` | Adafruit 875 八节 AA 电池盒候选。官网尺寸 127 × 71 × 19 mm，大于旧包络的两个方向，不能直接沿用旧布局。不是已指定品牌的 NiMH 电池组模型；电芯及插头空间另行核对。 |
| S1 滑动开关 | `slide_switch/slide_switch.step` | Adafruit 805 SPDT 候选；不能视为 Eagle 导出封装的精确实物型号，也未验证它适合承载实际电机电流。 |
| Tinkercad 小舵机 | `sg90_servo/sg90_servo.step`、`.stl` | SG90 参考；只对应电路模拟中的舵机类别，**不能替代仙人掌原机械电机**。 |

原机械结果中的 MG4005V2 尚未找到可核实、可直接下载的精确 V2 CAD，继续保留优化器原始圆柱包络。没有以其他电机的模型冒充它。

实际采用某个模型时，需要确认毫米单位、方向和局部原点，定义端口坐标，并用其真实几何加安装/插拔余量更新包络，然后重新布局与布线。下载模型本身不会让旧布局自动变成实物验证结果。

来源：

- [FreeCAD Parts Library](https://github.com/FreeCAD/FreeCAD-library)，CC BY 3.0，保留 `licenses/FreeCAD_Parts_Library_CC-BY-3.0.txt`；具体文件路径及固定提交记录在清单。
- [Adafruit CAD Parts](https://github.com/adafruit/Adafruit_CAD_Parts)，MIT，保留 `licenses/Adafruit_CAD_Parts_MIT.txt`。
- [Pololu D24V50F5 官方资源](https://www.pololu.com/product/2851/resources)，厂家公开提供下载；未确认开放再分发许可证。
- [电池盒尺寸](https://www.adafruit.com/product/875)、[开关产品](https://www.adafruit.com/product/805)。

重新获取并校验：在仓库根目录运行 `.conda-env/voxel_tool/bin/python robot_interior_postprocess/download_models.py`，需要联网。

重新转换并生成预览：`.conda-env/voxel_tool/bin/python robot_interior_postprocess/cad_models.py --convert`；后续仅刷新页面可不带 `--convert`。需要 requirements.txt 中的 gmsh 和 fast-simplification。
