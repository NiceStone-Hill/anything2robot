"""Read-only historical evidence adapter, never upgrades historical validation."""
import hashlib
from pathlib import Path
from .contracts import loads


def read_evidence(path):
    path=Path(path).resolve();raw=path.read_bytes();data=loads(raw)
    return {'schema_version':'1.0','source':str(path),'sha256':hashlib.sha256(raw).hexdigest(),
            'historical_status':data.get('status'),'source_complete_assemblable':data.get('complete_assemblable'),
            'source_units':data.get('units'),'routing':data.get('routing'),
            'pending_count':data.get('pending_count'),'historical_error':data.get('error'),
            'current_validation':'pending','purpose':'read_only_regression_evidence',
            'manufacturing_released':False}
