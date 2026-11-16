import unittest
import numpy as np
from studio.mechanics.parting_planes import plane_levels, longitudinal_normals


class PartingPlaneTests(unittest.TestCase):
    def test_vertical_seams_outrank_transverse_hint(self):
        for size in ([20,20,100],[80,30,25]):
            vertices=np.array([[x,y,z] for x in [-size[0],size[0]] for y in [-size[1],size[1]] for z in [-size[2],size[2]]])
            normals=longitudinal_normals(vertices,[0,0,1])
            self.assertTrue(all(abs(n[2])<1e-8 for n in normals[:4]))
            shifted=longitudinal_normals(vertices+[50,-30,10],[0,0,1])
            np.testing.assert_allclose(normals,shifted)

    def test_semantic_plane_is_used_and_mount_clearance_is_proposed(self):
        bounds=np.array([[-20,-20,-30],[20,20,30]])
        mount=np.array([[-5,-5,-10],[5,5,8]])
        levels=plane_levels(bounds,2,[mount],3,.4)
        self.assertEqual(levels[:2],[2.,9.])
        self.assertTrue(all(-27<v<27 for v in levels))

    def test_translating_model_translates_every_plane(self):
        bounds=np.array([[-20,-20,-30],[20,20,30]],float)
        mount=np.array([[-5,-5,-10],[5,5,8]],float)
        a=plane_levels(bounds,2,[mount],3,.4,[-4])
        delta=np.array([130,-70,93.])
        b=plane_levels(bounds+delta,95,[mount+delta],3,.4,[89])
        np.testing.assert_allclose(np.array(a)+93,b)

    def test_thin_body_does_not_invent_a_feasible_plane(self):
        self.assertEqual(plane_levels([[0,0,0],[10,10,4]],2,[],3,.4),[])


if __name__=='__main__':unittest.main()
