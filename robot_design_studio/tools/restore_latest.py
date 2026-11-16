"""Restore verified latest-project archives into a new workspace."""
import argparse,hashlib,json,tarfile
from pathlib import Path

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('destination',type=Path)
    p.add_argument('archives',type=Path,nargs='+');args=p.parse_args()
    if args.destination.exists() and any(args.destination.iterdir()):raise SystemExit('Use an empty destination; existing projects are never overwritten')
    for archive in args.archives:
        index=json.loads((archive.parent/'index.json').read_text())
        row=next(r for r in index if r['archive']==archive.name)
        if hashlib.sha256(archive.read_bytes()).hexdigest()!=row['sha256']:raise SystemExit('Archive checksum mismatch')
        with tarfile.open(archive) as tar:
            for member in tar.getmembers():
                parts=Path(member.name).parts
                if not parts or parts[0] not in {row['project_id'],'runs'} or '..' in parts or member.name.startswith('/') or not (member.isfile() or member.isdir()):
                    raise SystemExit('Unsafe archive entry')
    args.destination.mkdir(parents=True,exist_ok=True)
    for archive in args.archives:
        with tarfile.open(archive) as tar:tar.extractall(args.destination)
        print('Restored',archive.name)
