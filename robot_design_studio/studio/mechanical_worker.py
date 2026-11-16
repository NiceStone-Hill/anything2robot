"""Geometry subprocess; errors are archived, never converted to success."""
import sys
from pathlib import Path
from .contracts import loads
from .project import atomic


def main():
    request=loads(sys.stdin.buffer.read(4_000_001));directory=Path(request['destination']).parent
    try:
        if request.get('kind')=='harness':
            from .mechanics.harness import generate
            generate(request['project'],request['layout_directory'],directory)
        elif request.get('kind')=='simulation':
            from .simulation import generate
            generate(request['project'],request['layout_directory'],directory)
        elif request.get('kind')=='layout':
            from .mechanics.electronics_layout import generate
            generate(request['project'],request['mechanical_directory'],directory)
        elif request.get('kind')=='articulation':
            from .mechanics.articulation import annotate
            annotate(request['project'],request['source_path'],directory,recovery=request.get('annotation_recovery'))
        elif request.get('kind')=='assembly':
            from .mechanics.graph_assembly import generate
            generate(request['project'],request['source_path'],directory)
        else:
            from .mechanics.pipeline import generate
            generate(request['project'],request['source_path'],directory)
    except Exception as exc:
        # Providers sanitize remote failures before they reach this boundary.
        reason=str(exc) if isinstance(exc,ValueError) else type(exc).__name__
        atomic(directory/'result.json',{'geometry_valid':False,'annotation_valid':False,'status':'geometry_rejected','manufacturing_ready':False,
               'error':reason,'error_kind':getattr(exc,'error_kind','geometry_or_input'),'checks':[{'name':'pipeline','passed':False,'reason':reason}],'artifacts':[]})


if __name__=='__main__':main()
