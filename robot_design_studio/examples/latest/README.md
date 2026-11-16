# Latest retained model snapshots

模型数据不进入 Git 历史；在 GitHub 的
[最新样机项目包](https://github.com/NiceStone-Hill/anything2robot/releases/tag/studio-snapshots-2026-11-17)
中下载所需项目的 `.tar.gz`。下载本目录的 `index.json` 放在同一个目录用于摘要校验。
每个模型/尺寸只保留当前项目；索引中的通过/失败状态保持真实，不把失败候选当成打印包。

| 项目 | 归档文件 |
| --- | --- |
| 01 · 500 mm 完整流程参考 | `project_95d1e73d971c4ba1.tar.gz` |
| 02 · 混合电机参考 | `project_5fa77e8916ab435c.tar.gz` |
| 03 · 仙人掌原尺寸 | `project_c08c662672884f95.tar.gz` |
| 04 · 仙人掌 200 mm | `project_8bcd3846a8074a41.tar.gz` |
| 05 · 仙人掌 250 mm | `project_140380e842284558.tar.gz` |
| 06 · 仙人掌 220 mm | `project_dd7369c54a73411a.tar.gz` |
| 06 · 仙人掌 230 mm | `project_6b1e78d10baa4c67.tar.gz` |

只需打印 25 cm 样机时，下载同一附件列表中的 `cactus-250mm-prototype-print.zip`。
其 13 件 STL 已验证闭合、连通、mm 单位并放到打印平面。十字舵臂的尺寸未实测，
还没有电源线出线缺口，线束与上电未验证；不是实物制造放行。

恢复到空目录（在仓库根目录运行）：

```bash
python3 robot_design_studio/tools/restore_latest.py robot_design_studio/workspace/current /path/to/downloads/project_*.tar.gz
bash robot_design_studio/run.sh --port 8768 --workspace workspace/current
```

每个包包含源模型、当前项目和其依赖的生成任务。重复的历史项目版本与试跑已删除。
测试夹具是回归测试输入，不属于要删除的重复实验历史。
