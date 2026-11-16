"""Collect evidence without interpreting incomplete outputs as success."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[2]
out=ROOT/'data/custom_dog'
def read(path):
    return json.loads(path.read_text()) if path.is_file() else None
annotation=read(out/'annotation.json')
selection=read(out/'motor_selection.json')
manufacturing=read(ROOT/'result/custom_dog_manufacturing/status.json')
assembly_dir=ROOT/'robot_interior_postprocess/outputs/custom_dog_assembly'
manifest=read(assembly_dir/'manifest.json')
summary={'input':annotation,'mechanical':{'status':'passed','exit_code':0,'round':1,'parts':6,'motors':selection,
    'source':'result/custom_dog_test/dog_20261101-204826/result_round1','fea':False,
    'static_plot_export':'failed; separate interactive preview generated'},
    'manufacturing':manufacturing,'assembly':manifest,
    'acceptance':read(out/'acceptance_summary.json'),
    'routing':read(assembly_dir/'routing_summary.json'),
    'schema_provenance':read(assembly_dir/'schema_provenance.json'),
    'opening_probe':read(out/'opening_probe.json'),
    'all_body_trial':read(ROOT/'robot_interior_postprocess/outputs/custom_dog_assembly_all_in_body_failed/manifest.json'),
    'display_registration':read(assembly_dir/'display_registration_report.json'),
    'regressions':read(out/'regression_checks.json'),
    'generality':{'automatic_arbitrary_input_support_proven':False,
                 'manual_inputs':['3D joint positions and axes','5-channel derivative of cactus concept circuit','robot-specific configuration and opening candidates'],
                 'changed_core':['headless annotation import','true X-extent scaling','joint connection row difference and A*','CLI failure exit status']}}
(out/'test_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
lines=['# 小狗模型跨实例测试结果','',
       '日期：2026-11-02（北京时间）。原模型、仙人掌原电路和既有结果保留。','',
       '模型按双臂展开宽 600 mm 测试，高约 759 mm、深约 358 mm。颈部和双肩/双髋共 5 个单轴关节；深度和转轴是本次明确的人工假设。','',
       '| 阶段 | 实际结果 |','| --- | --- |',
       '| 关节标注 | 5 个驱动关节，6 个刚体；额外端点仅供分解/接地 |',
       '| Anything2robot | 第一轮 exit_code=0；部件完整性和闭合检查通过；约 141 秒 |',
       '| 电机匹配 | MG4005V2 × 3，DM6006 × 2；从真实优化结果匹配库索引 |',
       '| 制造连接结构 | '+(manufacturing['status'] if manufacturing else '尚未执行')+' |',
       '| 载板/分壳/布线总流程 | '+(manifest.get('status','unknown') if manifest else '尚未完成')+' |',
       '| 局部装配 | 4 个载板、12 片分壳候选；开通道后载板装入/螺钉/工具局部检查通过 |',
       '| 布线 | 3 次尝试均为 6/20 条分支通过，14 条未布通 |',
       '| 头部外壳紧固 | 0/3 个必要连接，最终阶段失败；没有最终可打印装配交付 |','',
       '## 电机位置','', '| 关节 | 电机 | 优化中心 X/Y/Z（mm） | 转轴 |','| --- | --- | --- | --- |']
for m in selection:
    lines.append('| '+m['link']+' | '+m['model']+' | '+', '.join(f'{v:.1f}' for v in m['center_mm'])+' | '+str(m['axis'])+' |')
provenance=read(assembly_dir/'schema_provenance.json')
if provenance:
    lines += ['', '布局约束来源：'+json.dumps(provenance,ensure_ascii=False)+'。真实响应另存于 layout_constraints_raw.json。']
if manifest:
    lines += ['', '## 后处理证据', '', '```json',json.dumps(manifest,ensure_ascii=False,indent=2),'```']
lines += ['', '## 已定位的失败原因', '',
          '1. 布线：三个尝试结果一致。电池到机身的跨部件连接、电机端口出口和部分电机通道未通过严格检查；没有把端口移到最近空气来伪造成功。具体 14 项见 routing_summary.json。',
          '2. 外壳连接：机身已有 6 个几何检查通过的连接；头部共享连接搜索在 288 个候选中没有找到满足现有螺钉/头座与外侧入口要求的方案（287 个为 no_standard_length_or_counterbore_wall，1 个为 no_single_exterior_exit_along_screw_axis），所需至少 3 个，实际 0 个。',
          '3. 流程耦合：当前先选择载板/分壳候选，再布线和生成外壳紧固。后两者失败尚未反馈到另一分壳候选或跨刚体元件分配，因而前面的局部装配通过并不保证完整方案可行。', '',
          '## 泛用性判断边界','',
          '仙人掌的 4 个前后分壳位置在本模型机身上均产生不连通碎片；保留失败批次后，另对各轴及不同偏移做连通性筛查，作为新实例候选配置。这不是放宽连通性检查。', '',
          '本次是人工提供关节和新实例配置后的跨外形测试。机械阶段已经处理仙人掌之外的几何和五个关节；不能据此声称任意模型、电路或电机已可全自动导入。后处理以实际 manifest 和逐项验证报告为准。', '',
          '仙人掌原电路只有三路电机，本次在副本中增加 D5/D6 两路；SCH、BRD 和 BOM 同步更新。三线概念接口尚未验证实际 MG4005V2/DM6006 的电压、控制协议、电流能力和真实引脚。', '',
          '本次为运行测试修正了无界面导入、X 宽度缩放、关节连接内存/路径搜索与命令行失败状态。独立回归记录见 regression_checks.json。原静态 Plotly 图导出失败，已另存实际网格的离线交互预览。', '',
          '电池配置调整为 HEAD，控制板/降压板/开关留在 BODY；这是明确的新配置，并非系统自动跨刚体分配。电池规划尺寸不变，改为平放安装面。', '',
          '电池 CAD 的平放显示配准另存到 process_registered.html（若完整导出存在则另有 index_registered.html），原始求解记录和所有检查结果保留不动；配准副本仅修正 CAD 外观，不改变规划包络或位姿。', '',
          'FEA、动态平衡/行走、真实电气兼容、线束运动、紧固件公差和实物承载未验证。后处理 partial 不代表可制造整机。','',
          '## 查看文件','',
          f'- [关节标注]({out}/joint_annotation.png)',
          f'- [机械三维预览]({out}/mechanical_preview.html)',
          f'- [运行说明]({out}/README.md)',
          f'- [机器可读结果]({out}/test_summary.json)']
for name in ['process_registered.html','index_registered.html','display_registration_report.json','process.html','assembly_validation.json','routing.json','carrier_placement.json','shell_fastening_report.json','routing_summary.json','shell_connections.json','carrier_placement.json']:
    if (assembly_dir/name).exists():lines.append(f'- [{name}]({assembly_dir/name})')
(out/'test_report.md').write_text('\n'.join(lines)+'\n')
print(out/'test_report.md')
