"""Output origins and surviving mating surfaces, independent of source model."""
import unittest
import numpy as np
import trimesh as tm
from studio.mechanics.joint_modules import module
from studio.mechanics.joint_placement import registration_metrics, mounting_surface_checks
from studio.mechanics.solid import box, cut, union


class RegistrationTests(unittest.TestCase):
    def test_parallel_axis_is_insufficient(self):
        built=module('sg90_digital_towerpro')
        joint={'axis':[0,0,1],'position_mm':[11,22,33]}
        transform=np.eye(4);transform[:3,3]=joint['position_mm']
        self.assertTrue(registration_metrics(transform,joint,built)['output_registration_passed'])
        transform[2,3]+=42
        result=registration_metrics(transform,joint,built)
        self.assertEqual(result['axis_line_error_mm'],0.)
        self.assertFalse(result['output_registration_passed'])
        transform[:3,3]=joint['position_mm'];transform[0,3]+=1
        self.assertFalse(registration_metrics(transform,joint,built)['output_registration_passed'])

    def fixture(self):
        built=module('sg90_digital_towerpro')
        transform=tm.transformations.rotation_matrix(.8,[1,2,3]);transform[:3,3]=[17,-9,22]
        built.update(transform=transform,joint={'parent':'p','child':'c'},id='joint')
        pieces={side:next(p[3] for p in built['parts'] if p[0]==role).copy().apply_transform(transform)
                for side,role in [('p','fixed_mount'),('c','output_mount')]}
        return built,pieces

    def test_all_three_faces_on_correct_body_after_rotation(self):
        built,pieces=self.fixture()
        reports=mounting_surface_checks(built,pieces)
        self.assertEqual(len(reports),3)
        self.assertTrue(all(r['passed'] for r in reports),reports)
        self.assertFalse(any(r['physical_fit_verified'] for r in reports))

    def test_clear_holes_but_missing_seat_fails(self):
        built,pieces=self.fixture()
        witness=built['mounting_surfaces'][0]['mesh'].copy().apply_transform(built['transform'])
        pieces['p']=cut(pieces['p'],witness)
        self.assertFalse(mounting_surface_checks(built,pieces)[0]['passed'])

    def test_floating_seat_or_wrong_owner_fails(self):
        built,pieces=self.fixture()
        pieces['p']=union(pieces['p'],box([100,100,100],[102,102,102]))
        self.assertFalse(mounting_surface_checks(built,pieces)[0]['passed'])
        built,pieces=self.fixture();pieces['p'],pieces['c']=pieces['c'],pieces['p']
        self.assertFalse(any(r['passed'] for r in mounting_surface_checks(built,pieces)))


if __name__=='__main__':unittest.main()
