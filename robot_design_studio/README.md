# Robot Design Studio

从 STL/OBJ 模型生成关节、舵机安装结构、可拆壳体和电子载板的通用设计工作台。
所有规则按几何和元件接口运行，仙人掌只是测试输入。

当前保留每个模型/尺寸的最新项目。入口和状态见 [START_HERE.md](START_HERE.md)，
可移植项目包见 [examples/latest](examples/latest/README.md)。重复实验、录像和旧工作区已清理。

## 启动

需要 Python 3.11。首次安装：

```bash
python3.11 -m venv robot_design_studio/.venv
robot_design_studio/.venv/bin/python -m pip install -r robot_design_studio/requirements-geometry.txt
export ROBOT_GEOMETRY_PYTHON="$(pwd)/robot_design_studio/.venv/bin/python"
bash robot_design_studio/run.sh --port 8768 --workspace workspace/current
```

浏览器打开 http://127.0.0.1:8768 。已有本地项目位于
`workspace/internal_layout_demo_2026_11_15`；启动时将 `--workspace` 改为此路径即可。
工作区参数相对于 `robot_design_studio/`。端口被占用时复用已有服务。
新环境安装尚未重新验证；依赖版本见 requirements 文件。

可通过 `--ai-env /path/to/private.env` 显式加载在线 VLM 配置，变量为
`ROBOT_AI_BASE_URL`、`ROBOT_AI_MODEL`、`ROBOT_AI_API_KEY`。配置文件不提交到 Git。
未配置在线服务时仅使用明确标注的本地参考模式。

## 工作流

1. 导入模型，确认单位、尺寸和朝向；标注关节与刚体。
2. 按关节选舵机，生成安装座、输出连接、开盖切缝、紧固件孔和支撑。
3. 选电子元件、生成逻辑接线、填写完整实物包络。
4. 自动布局载板，检查连接、碰撞、螺丝工具空间及网格导出。
5. 空间不足时可按 5% 步长尝试放大副本，累计不超过原模型线性尺寸的 25%。
6. 下载通过软件检查的样机打印包；实物配合、线束与上电分别验证。

SG90 使用原配十字舵臂及打印连接片，不要求圆舵盘，不打印未经测量的轴花键。
壳件手动拆卸模式将直线取出列为提示，实际碰撞、连接及工具空间仍是硬检查。

## 当前边界

- 25 cm 仙人掌已生成 13 件样机打印包，结构与两板载板布局通过。
- 23 cm 的两板布局通过，但底部输出侧支撑连接失败；不是合格打印包。
- 20、22 cm 保留最新失败结果，不能据此断言尺寸绝对不可行。
- 当前样机无电池盒，使用单外部电源；尚未增加电源线出线缺口。
- 十字舵臂与板卡尺寸仍含工程假设，实物尺寸、负载、线束及电气未验证。
- 排线目前有同刚体限制；URDF/MJCF 导出不代表已运行仿真。

## 验证与维护

```bash
PYTHONPATH=robot_design_studio robot_design_studio/.venv/bin/python -m unittest discover -s robot_design_studio/tests -p 'test_*.py'
PYTHONPATH=robot_design_studio robot_design_studio/.venv/bin/python -m unittest robot_design_studio.tests.horn_interface_geometry
# 先停服务，再查看清理计划；加 --apply 才会删除旧结果。
python3 robot_design_studio/tools/prune_workspace.py robot_design_studio/workspace/current
```

测试夹具与运行时 CAD/许可证保留。默认新任务仍保存版本记录；手动清理只留下当前结果、
依赖任务及一份当前项目快照，不改变通过/失败结论。

实现说明：[壳体切分](docs/GENERIC_ASSEMBLY_SPLIT_DESIGN_2026-11-16.md)、
[紧固优化](docs/fastener-search.md)、[尺寸重试](docs/size-search.md)、
[舵臂接口](docs/STOCK_HORN_INTERFACE.md)、[打印包](docs/prototype-print-package.md)。
第三方来源与许可见 [THIRD_PARTY.md](THIRD_PARTY.md)。
