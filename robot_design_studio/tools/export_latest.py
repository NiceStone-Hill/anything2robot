"""Export retained projects as independent portable workspace archives."""
import argparse,hashlib,json,tarfile
from pathlib import Path


def export(root,destination):
    root=Path(root);destination=Path(destination);destination.mkdir(parents=True,exist_ok=True);records=[]
    for f in sorted(root.glob('project_*/project.json')):
        p=json.loads(f.read_text());name=p['id']+'.tar.gz';output=destination/name
        with tarfile.open(output,'w:gz',compresslevel=6) as archive:
            archive.add(f.parent,arcname=f.parent.name)
            for rid in p['runs']:archive.add(root/'runs'/rid,arcname='runs/'+rid)
        bounds=(p.get('articulation') or {}).get('analysis',{}).get('bounds_mm')
        records.append({'project_id':p['id'],'name':p['name'],'revision':p['revision'],
            'height_mm':bounds[1][2]-bounds[0][2] if bounds else None,'archive':name,
            'bytes':output.stat().st_size,'sha256':hashlib.sha256(output.read_bytes()).hexdigest(),
            'mechanical':p.get('mechanical_result'),'layout':p.get('layout_result')})
    (destination/'index.json').write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
    return records

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('workspace');parser.add_argument('destination')
    args=parser.parse_args()
    for row in export(args.workspace,args.destination):print(row['name'],round(row['bytes']/1e6,2),'MB',flush=True)
