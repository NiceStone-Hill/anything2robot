"""A clear screw bore alone must not establish screwdriver access."""
import unittest
import numpy as np
from studio.mechanics.fastening import interfaces, cylinder_path, check_access, placed_access
from studio.mechanics.joint_modules import module
from studio.mechanics.solid import box, cut, cylinder


class FastenerAccessTests(unittest.TestCase):
    def test_motor_templates_register_both_fixed_and_output_fasteners(self):
        for catalog in ['sg90_digital_towerpro','mg4005v2_candidate']:
            sites=module(catalog)['fastening_interfaces']
            self.assertEqual({s['side'] for s in sites},{'parent','child'})
            self.assertTrue(all(not s['physical_fit_verified'] for s in sites))
        sg=module('sg90_digital_towerpro')['fastening_interfaces']
        self.assertEqual(len(sg),5)
        self.assertEqual(len(sg[0]['tools']),2) # Driver and nut access.

    def record(self):
        return {'id':'screw','owner_body_id':'arm','open_cover_body_ids':['arm'],
                'detach_body_ids':[],'tools':[{'mesh':cylinder_path([0,0,0],[0,0,1],30,2.1)}]}

    def test_small_clear_bore_does_not_pass_larger_driver(self):
        wall=cut(box([-10,-10,10],[10,10,13]),cylinder(0,0,9,14,1))
        report=check_access([self.record()],{'arm':wall},{},[])[0]
        self.assertFalse(report['passed'])
        self.assertEqual(report['tools'][0]['blockers'][0]['asset_id'],'arm_shell')

    def test_cover_removed_at_correct_stage_and_other_bodies_still_block(self):
        empty=box([10,10,0],[11,11,1]);cover=box([-5,-5,10],[5,5,12])
        self.assertTrue(check_access([self.record()],{'arm':empty},{'arm':cover},[])[0]['passed'])
        self.assertFalse(check_access([self.record()],{'arm':empty,'other':cover},{},[])[0]['passed'])
        record=self.record();record['detach_body_ids']=['other']
        self.assertTrue(check_access([record],{'arm':empty,'other':cover},{},[])[0]['passed'])

    def test_rotated_tool_path_still_detects_obstruction(self):
        record=self.record();record['tools'][0]['mesh']=cylinder_path([0,0,0],[1,1,0],30,2.1)
        obstruction=box([9,9,-2],[12,12,2])
        self.assertFalse(check_access([record],{'arm':obstruction},{},[])[0]['passed'])

    def test_captive_nut_path_preserves_seat_roll_and_assembly_stage(self):
        from trimesh.transformations import rotation_matrix
        built=module('sg90_digital_towerpro')
        built.update(transform=rotation_matrix(.9,[1,2,3]),joint={'parent':'base','child':'arm'},
                     id='arbitrary_joint',graph={'joints':[{'parent':'base','child':'arm'}]})
        sites=placed_access(built,{'base':box([-70,-70,-70],[70,70,70])})
        parts={p[0]:p[3].copy().apply_transform(built['transform']) for p in built['parts']}
        hardware=[{'id':'arbitrary_joint_'+role,'mesh':parts[role],'owner':'base' if role in ['motor','retainer'] else 'arm',
                   'joint_id':'arbitrary_joint','role':role} for role in ['motor','retainer','rotor']]
        reports=check_access(sites,{'base':parts['fixed_mount'],'arm':parts['output_mount']},{},hardware)
        self.assertTrue(all(r['passed'] for r in reports),reports)

    def test_large_and_source_fragments_do_not_need_surface_distance(self):
        from unittest.mock import patch
        from studio.mechanics.shell_access import separate_voxel_artifacts
        from studio.mechanics.solid import union
        shell=union(box([0,0,0],[10,10,10]),box([20,0,0],[22,2,2]),box([30,0,0],[30.1,.1,.1]))
        source=box([-1,-1,-1],[32,12,12])
        with patch('trimesh.proximity.closest_point',side_effect=AssertionError('unnecessary expensive distance query')):
            kept,removed,_=separate_voxel_artifacts(shell,source,[],1.,1)
        self.assertFalse(removed)
        self.assertAlmostEqual(kept.volume,shell.volume,places=6)

    def test_substantial_cutting_fragment_skips_motion_surface_query(self):
        from unittest.mock import patch
        from studio.mechanics.shell_access import separate_cutting_remnants
        from studio.mechanics.solid import union
        shell=union(box([0,0,0],[10,10,10]),box([20,0,0],[22,2,2]))
        with patch('trimesh.proximity.closest_point',side_effect=AssertionError('large fragment must be preserved')):
            kept,removed,_=separate_cutting_remnants(shell,[],[box([19,-1,-1],[24,4,4])])
        self.assertFalse(removed)
        self.assertAlmostEqual(kept.volume,shell.volume,places=6)

    def test_search_budget_returns_unaccepted_candidate_without_removing_material(self):
        from unittest.mock import patch
        from studio.mechanics.shell_access import reconnect_local
        from studio.mechanics.solid import union
        shell=union(box([0,0,0],[10,10,10]),box([20,0,0],[22,2,2]))
        with patch('studio.mechanics.shell_access.time.monotonic',side_effect=[0.,31.]):
            kept,report=reconnect_local(shell,box([-1,-1,-1],[30,20,20]),[])
        self.assertFalse(report['passed'])
        self.assertEqual(report['error_kind'],'search_budget_exhausted')
        self.assertAlmostEqual(kept.volume,shell.volume,places=6)


if __name__=='__main__':unittest.main()
