import json
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
from studio.mechanics.solid import box,cut
from studio.mechanics.assembly_split import split_shell
from studio.mechanics.parting_export import export


class ShellPartitionStageTests(unittest.TestCase):
    def test_empty_shell_export_is_not_complete_robot_validation(self):
        outer=box([-25,-20,-20],[25,20,20])
        shell=cut(outer,box([-22,-17,-17],[22,17,17]))
        base,cap,panel=split_shell(shell,outer,{'normal':[0,0,1]},[],[],3,[],shell_parts_only=True)
        self.assertEqual(panel['extraction_scope'],'empty_shell_parts')
        from studio.mechanics.cover_fasteners import finish
        from studio.mechanics.solid import overlap
        panel.update(owner_body_id='body',cover_asset_id='body_lid')
        reports=finish([{'record':{'id':'body_shell','owner_body_id':'body'},'mesh':base},
                        {'record':{'id':'body_lid','owner_body_id':'body'},'mesh':cap}],
                       {'lid_planes':[panel]},overlap)
        self.assertGreaterEqual(len(reports),3)
        self.assertTrue(all(r['passed'] and r['candidate_thread']=='M3' for r in reports))
        self.assertTrue(all(r['shaft_clearance_diameter_mm']==3.4 for r in reports))
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp);source=directory/'input.stl';outer.export(source)
            project={'id':'example','revision':1,'articulation':{'graph':{
                'bodies':[{'id':'body','name':'Body'}],'joints':[],'root_body_id':'body'}}}
            plate=box([-4,-4,-10],[4,4,-8])
            module={'id':'mount','joint_id':'joint','owner':'body','role':'fixed_mount',
                    'name':'Mount','template':'test','template_revision':'1','mesh':plate}
            result=export(project,directory,source,{'body':base},{'body':cap},{'body':panel},[module],
                          {'source_sha256':'test'},time.monotonic())
            self.assertTrue(result['shell_partition_valid'])
            self.assertFalse(result['geometry_valid'])
            self.assertFalse(result['manufacturing_ready'])
            self.assertEqual(len(result['geometry_assets']),2)
            reference=result['module_interfaces'][0]
            self.assertFalse(reference['manufacturing_part'])
            self.assertTrue(reference['requires_shell_attachment'])
            with np.load(directory/reference['geometry_file']) as stored:
                np.testing.assert_array_equal(stored['vertices'],plate.vertices)
            self.assertTrue((directory/'shell-partition-review.zip').is_file())
            preview=json.loads((directory/'preview.json').read_text())
            self.assertTrue(next(p for p in preview['parts'] if p['id']=='body_lid')['opening_offset_m'][1]>0)


if __name__=='__main__':unittest.main()
