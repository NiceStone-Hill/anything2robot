"""Run in geometry interpreter; independent solid reload and failure cases."""
import hashlib
import io
import json
import math
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
import numpy as np
import trimesh as tm
from studio.mechanics.solid import bevel_shape, normalize, inset_convex, regions, rotate, overlap, union
from studio.mechanics.pipeline import generate
from studio.mechanical_service import appearance, configure
from studio.catalog import Catalog
from studio.project import Store
from studio.contracts import ContractError
from studio.mechanics.planning import propose


class GeometryTests(unittest.TestCase):
    def test_invalid_model_proposal_is_not_executed(self):
        valid={'joint_fraction':.5,'amplitude_deg':10,'axis':[0,0,1],'parent_region':'negative_x','child_region':'positive_x','dof':1,'reason':'轻微摆动','uncertainties':['接口未测量']}
        with tempfile.TemporaryDirectory() as folder:
            directory=Path(folder)
            for change in [{'joint_fraction':.9},{'axis':[1,0,0]},{'amplitude_deg':True},{'parent_region':'invented_body'},{'dof':True}]:
                envelope={'choices':[{'message':{'content':json.dumps({**valid,**change})}}]}
                with patch.dict('os.environ',{'ROBOT_AI_BASE_URL':'https://example.invalid/v1','ROBOT_AI_MODEL':'test','ROBOT_AI_API_KEY':'not-a-real-key'}), patch('urllib.request.build_opener') as opener:
                    opener.return_value.open.return_value=io.BytesIO(json.dumps(envelope).encode())
                    with self.assertRaises(ContractError):propose(bevel_shape([240,90,80]),{'mode':'live','amplitude_deg':20,'intent':'测试'}, {'intent':'测试'},directory,lambda *args:None)
            views=json.loads((directory/'views.json').read_text())
            self.assertEqual(len(views),3)
            for camera in views:self.assertTrue(np.allclose(np.asarray(camera['mm_to_pixel_2x4'])@[0,0,0,1],[320,180]))

    def test_fresh_generation_and_independent_checks(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'input.stl';bevel_shape([240,90,80]).export(source)
            p={'id':'test','revision':1,'intent':'小幅左右摆动','mechanical_config':{'length_m':.24,'amplitude_deg':20,'mode':'reference','motor_instance_id':'servo'},
               'source_shapes':[{'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'format':'stl','unit':'mm'}]}
            result=generate(p,source,root/'run');self.assertTrue(result['geometry_valid']);self.assertFalse(result['manufacturing_ready'])
            fixed=[];moving=[]
            for a in result['geometry_assets']:
                mesh=tm.load(root/'run'/a['file']);self.assertTrue(mesh.is_volume);self.assertEqual(regions(mesh),1)
                self.assertTrue(np.allclose(mesh.bounds/1000,a['bounds_m'],atol=1e-7))
                (fixed if a['owner_body_id']=='fixed' else moving).append(mesh)
            a,b=union(*fixed),union(*moving)
            # Offset sample grid, not the generator's 17 sample points.
            for angle in np.arange(-18.75,20,2.5):self.assertLess(overlap(a,rotate(b,math.radians(angle))),.05)
            self.assertTrue(any(c['intersection_mm3']>1 for c in result['decomposition']['before_clearance_sweep']))
            self.assertTrue(any(not c['feasible'] for c in result['layout_candidates']))
            p['mechanical_config']['length_m']=.18
            with self.assertRaisesRegex(ValueError,'不足'):generate(p,source,root/'too_small')
            source.write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'摘要'):generate(p,source,root/'tampered')

    def test_invalid_geometry_and_units(self):
        mesh=tm.creation.box([240,90,80]);mesh.update_faces(np.arange(len(mesh.faces)-1))
        with self.assertRaisesRegex(ValueError,'闭合'):normalize(mesh,'mm',240)
        mesh=union(tm.creation.box([50,50,50]),tm.creation.box([50,50,50]).apply_translation([100,0,0]))
        with self.assertRaisesRegex(ValueError,'闭合'):normalize(mesh,'mm',240)
        a,t=normalize(bevel_shape([24,9,8]),'cm',240)
        b,_=normalize(bevel_shape([.24,.09,.08]),'m',240)
        self.assertTrue(np.allclose(a.extents,b.extents));self.assertGreater(np.linalg.det(t[:3,:3]),0)
        self.assertLess(inset_convex(a,2.4).volume,a.volume)

    def test_source_boundary(self):
        with tempfile.TemporaryDirectory() as folder:
            store=Store(folder,Catalog());p=store.create('中文 外观')
            with self.assertRaises(ContractError):appearance(store,p['id'],{'revision':1,'filename':'../bad.stl','unit':'mm','data_base64':'AAAA'})
            with self.assertRaises(ContractError):appearance(store,p['id'],{'revision':1,'builtin':'box','dimensions_mm':[float('nan'),90,80]})


if __name__=='__main__':unittest.main(verbosity=2)
