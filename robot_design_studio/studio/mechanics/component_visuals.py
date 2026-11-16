"""Traceable, unscaled reference CAD for display; planning envelopes stay authoritative."""
import hashlib,json,shutil
from pathlib import Path
import numpy as np
import trimesh as tm
from functools import lru_cache
from ..project import atomic

ROOT=Path(__file__).resolve().parents[2]/'assets/components'
REGISTRY=ROOT/'models.json'

def validate_mounting(entry,mesh):
    spec=entry.get('mounting')
    if not spec:return
    if spec['source_sha256']!=entry['source_sha256']:raise ValueError('PCB interface source hash mismatch')
    origins=[]
    for hole in spec['holes']:
        for angle in [0,np.pi/2,np.pi,3*np.pi/2]:
            offset=.85*hole['diameter_mm']/2*np.array([np.cos(angle),np.sin(angle)])
            origins.append([*(np.array(hole['xy_mm'])+offset),spec['pcb_bottom_mm']-.2])
    points,_,_=mesh.ray.intersects_location(origins,[[0,0,1]]*len(origins))
    if any(spec['pcb_bottom_mm']-.05<=p[2]<=spec['pcb_top_mm']+.01 for p in points):
        raise ValueError('Registered mounting bore intersects the reference PCB; check hole coordinates')

@lru_cache(maxsize=16)
def reference(catalog_id):
    entry=next((m for m in json.loads(REGISTRY.read_text())['models'] if catalog_id in m['catalog_ids']),None)
    if entry is None:return None
    path=verified_file(ROOT/entry['id'],entry['file'],entry['sha256'])
    verified_file(ROOT/entry['id'],entry['source_file'],entry['source_sha256'])
    mesh=tm.load(path,force='mesh');validate_mounting(entry,mesh)
    return entry,mesh

def registration(entry,mesh,dimensions):
    """Single source-to-envelope transform for CAD, PCB holes and mounts."""
    matrix=np.eye(4);matrix[:3,:3]=entry['source_rotation']
    rotated=mesh.copy().apply_transform(matrix)
    matrix[:3,3]=-rotated.bounds.mean(0)+[0,0,-dimensions[2]/2+rotated.extents[2]/2+entry.get('bottom_allowance_mm',1.)]
    return matrix

def verified_file(root,name,digest):
    if Path(name).name!=name:raise ValueError('Invalid component model asset path')
    file=Path(root)/name
    if hashlib.sha256(file.read_bytes()).hexdigest()!=digest:raise ValueError('Component model checksum mismatch: '+name)
    return file

def generate(placements,directory):
    """Return display assets, with full-size CAD inside the registered local envelope."""
    library=json.loads(REGISTRY.read_text());assets=[];reports=[];directory=Path(directory)
    for placement in placements:
        entry=next((m for m in library['models'] if placement['catalog_id'] in m['catalog_ids']),None)
        if entry is None:
            reports.append({'instance_id':placement['instance_id'],'status':'envelope_only','reason':'No specific hardware model selected.'});continue
        folder=ROOT/entry['id'];path=verified_file(folder,entry['file'],entry['sha256'])
        mesh=tm.load(path,force='mesh')
        dimensions=np.asarray(placement['envelope_dimensions_m'])*1000
        matrix=registration(entry,mesh,dimensions);mesh.apply_transform(matrix)
        fits=bool(np.all(mesh.bounds[0]>=-dimensions/2-1e-5) and np.all(mesh.bounds[1]<=dimensions/2+1e-5))
        if not fits:
            # Never shrink a model or silently enlarge a user's registered envelope.
            reports.append({'instance_id':placement['instance_id'],'model_id':entry['id'],'status':'model_outside_envelope','model_extents_mm':mesh.extents.tolist(),'envelope_mm':dimensions.tolist()});continue
        world=np.asarray(placement['transform_m'],float).copy();world[:3,3]*=1000;mesh.apply_transform(world)
        identity='cad_'+placement['instance_id'];filename=identity+'.ply';mesh.export(directory/filename)
        record={'id':identity,'file':filename,'sha256':hashlib.sha256((directory/filename).read_bytes()).hexdigest(),'instance_id':placement['instance_id'],
            'model_id':entry['id'],'name':entry['name'],'owner_body_id':placement['owner_body_id'],'replaces_envelope_id':placement['envelope_asset_id'],
            'length_unit':'mm','visual_only':True,'scale_factor':1.,'physical_variant_verified':False,'evidence':entry['evidence'],'source_url':entry['source_url'],
            'source_sha256':entry['source_sha256'],'source_to_world_mm':(world@matrix).tolist(),'reference_variant':entry['variant'],'bounds_m':(mesh.bounds/1000).tolist()}
        assets.append(record);reports.append({'instance_id':placement['instance_id'],'model_id':entry['id'],'status':'reference_cad','physical_variant_verified':False,'scale_factor':1.})
        for name in entry['attribution_files']:
            source=folder/name
            shutil.copy2(source,directory/(entry['id']+'-'+name))
    atomic(directory/'component-models.json',{'models':assets,'registration':reports,'note':'Reference CAD, not measured hardware. Envelopes remain collision and mass assumptions.'})
    return assets,reports

def load(directory,asset):
    return tm.load(verified_file(directory,asset['file'],asset['sha256']),force='mesh')

def preview_parts(directory,assets):
    """Split by source CAD face colour so both local renderers show real features."""
    result=[]
    for asset in assets:
        mesh=load(directory,asset)
        colors=np.asarray(mesh.visual.face_colors)
        for index,color in enumerate(np.unique(colors,axis=0)):
            selected=np.flatnonzero(np.all(colors==color,axis=1));part=mesh.submesh([selected],append=True)
            if len(part.faces)>7000:part=part.simplify_quadric_decimation(face_count=7000)
            result.append({'id':asset['id']+'_'+str(index),'name':asset['name'],'owner':asset['owner_body_id'],'vertices':(part.vertices/1000).tolist(),
                'faces':part.faces.tolist(),'color':'#'+''.join(format(int(v),'02x') for v in color[:3]),'hardware':True,'role':'reference_cad','instance_id':asset['instance_id']})
    return result

def scene(directory,geometry,visual_assets):
    """Replace matching display boxes only; leave collision geometry untouched."""
    replaced={a['replaces_envelope_id'] for a in visual_assets};result=tm.Scene()
    for item in geometry:
        if item['record']['id'] in replaced:continue
        mesh=item['mesh'].copy().apply_scale(.001)
        mesh.visual.face_colors=tm.visual.color.hex_to_rgba('#677581' if item['record'].get('role')=='pcb_fastener' else '#d2a052' if item['record']['id'].startswith('electronics_') else '#8da5a0')
        result.add_geometry(mesh,node_name=item['record']['id'])
    for asset in visual_assets:result.add_geometry(load(directory,asset).apply_scale(.001),node_name=asset['id'])
    return result


def copy_artifacts(source,destination,assets):
    source=Path(source);destination=Path(destination);files=[]
    for asset in assets:
        path=verified_file(source,asset['file'],asset['sha256']);shutil.copy2(path,destination/path.name);files.append(path.name)
    model_ids={a['model_id'] for a in assets}
    for file in source.iterdir():
        if file.name=='component-models.json' or any(file.name.startswith(identity+'-') for identity in model_ids):
            if file.is_file() and file.name not in files:shutil.copy2(file,destination/file.name);files.append(file.name)
    return files
