import hashlib
from pathlib import Path
from .contracts import ContractError, loads, validate, ID

CATALOG_PATH = Path(__file__).resolve().parents[1] / 'catalog' / 'parts.json'

class Catalog:
    def __init__(self, path=CATALOG_PATH):
        raw = Path(path).read_bytes()
        self.digest = hashlib.sha256(raw).hexdigest()
        self.document = loads(raw)
        self.parts = {}
        for part in self.document['parts']:
            validate(part['id'], ID)
            if part['id'] in self.parts:
                raise ContractError('目录 ID 重复')
            self.parts[part['id']] = part
        self.versions={(p['id'],p['revision']):p for p in self.parts.values()}
        for snapshot in (Path(path).parent/'history').glob('*.json'):
            for old in loads(snapshot.read_text())['parts']:
                self.versions.setdefault((old['id'],old['revision']),old)
        for part in self.parts.values():
            for dep in part['dependencies']:
                if dep['catalog_id'] not in self.parts:
                    raise ContractError('目录依赖缺失')

    def resolve(self, instance):
        part = self.versions.get((instance['catalog_id'],instance['catalog_revision']))
        if not part:
            raise ContractError('部件目录版本不存在')
        return part

    def selection_issues(self, instances):
        issues, counts = [], {}
        for i in instances:
            p = self.resolve(i)
            counts[p['id']] = counts.get(p['id'], 0) + 1
        for identity, count in counts.items():
            p = self.parts[identity]
            stock = p['inventory']['quantity']
            if stock is not None and count > stock:
                issues.append({'code': 'stock_shortage', 'reason': p['name'] + ' 超出已登记库存', 'catalog_id': identity})
            for d in p['dependencies']:
                needed = d['quantity'] * count
                if counts.get(d['catalog_id'], 0) < needed:
                    issues.append({'code': 'missing_dependency', 'catalog_id': d['catalog_id'], 'quantity': needed-counts.get(d['catalog_id'], 0), 'reason': d['reason']})
        return issues
