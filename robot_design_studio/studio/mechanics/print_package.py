"""Version-bound prototype STL package, separate from electrical release."""
import hashlib,json,zipfile
from pathlib import Path
import numpy as np
import trimesh as tm
from .electronics_layout import read_mesh
from .graph_assembly import stl_deliverable


def generate(result,directory):
    directory=Path(directory)
    if not result.get('layout_valid') or not result.get('structure_verified'):
        return None
    if any(not c['passed'] for c in result['checks'] if c.get('blocking',True)):
        return None
    entries=[];files={}
    for asset in result['geometry_assets']:
        if not asset['printable_candidate']:continue
        mesh=read_mesh(directory,asset)
        # Largest actual planar facet is a generic starting orientation. It is
        # not a claim that no supports are required or that overhangs are safe.
        transform=np.eye(4)
        if len(mesh.facets_area):
            normal=mesh.facets_normal[int(np.argmax(mesh.facets_area))]
            transform=tm.geometry.align_vectors(normal,[0,0,-1])
        mesh.apply_transform(transform)
        shift=np.array([-mesh.bounds.mean(0)[0],-mesh.bounds.mean(0)[1],-mesh.bounds[0,2]])
        mesh.apply_translation(shift);transform[:3,3]+=shift
        mesh,repair=stl_deliverable(mesh)
        if not repair['passed']:return None
        raw=mesh.export(file_type='stl_ascii' if repair.get('stl_encoding')=='ascii' else 'stl')
        if isinstance(raw,str):raw=raw.encode()
        name=asset['id']+'.stl';files[name]=raw
        entries.append({'file':name,'name':asset['name'],'quantity':1,'units':'mm','source_sha256':asset['sha256'],
            'sha256':hashlib.sha256(raw).hexdigest(),'size_mm':mesh.extents.tolist(),'assembly_to_print_mm':transform.tolist(),
            'mesh_checks_passed':True,'orientation':'largest_planar_facet_down_review_in_slicer'})
    if not entries:return None
    manifest={'project_id':result['project_id'],'design_revision':result['design_revision'],
        'mechanical_run_id':result['mechanical_run_id'],'scope':'prototype_print_geometry',
        'print_geometry_valid':True,'manufacturing_ready':False,'parts':entries,
        'assembly_instructions':result['assembly_dependencies'],
        'advisories':[c for c in result['checks'] if not c['passed']],
        'pending':result.get('pending',[]),'material':'PLA','nozzle_mm':.4,
        'printer_build_volume_verified':False,'slicing_verified':False,'wiring_verified':False}
    with zipfile.ZipFile(directory/'prototype-print.zip','w',zipfile.ZIP_DEFLATED) as z:
        for name,raw in files.items():z.writestr('STL/'+name,raw)
        z.writestr('manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2))
        z.writestr('README.txt','原型打印几何包：只包含需打印的零件，不含电机、板卡或螺钉外形。\n单位 mm，每个零件已独立放到 Z=0；摆放方向是候选，请在切片器检查支撑、首层和打印机尺寸。\n材料 PLA，喷嘴 0.4 mm。未生成 G-code。\n装配位置与顺序见 manifest.json。直线拆卸提示仍保留，需要人工确认转动/倾斜拆装。\n几何检查通过不代表实物配合、线束或上电已验证；此包没有把制造放行状态改为通过。\n')
    (directory/'prototype-print-manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    return {'file':'prototype-print.zip','manifest':'prototype-print-manifest.json','part_count':len(entries),'scope':'prototype_print_geometry'}
