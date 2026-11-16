import unittest
import numpy as np
from studio.mechanics.harness import astar,tube
from studio.mechanics.electronics_layout import overlap
from studio.mechanics.solid import box
class HarnessGeometryTests(unittest.TestCase):
    def test_routes_around_wall_and_rejects_closed_barrier(self):
        free=np.ones((9,9,5),bool);free[4,:,:]=False;free[4,7,2]=True
        path=astar(free,(1,1,2),(7,1,2))
        self.assertIn((4,7,2),path);self.assertTrue(all(free[n] for n in path))
        free[4,7,2]=False
        with self.assertRaises(ValueError):astar(free,(1,1,2),(7,1,2))
    def test_wire_radius_and_cache_invalidation(self):
        wire=tube(np.array([0.,0.,0.]),np.array([10.,0.,0.]),1.)
        obstacle=box([4,-.5,-.5],[6,.5,.5])
        self.assertGreater(overlap(wire,obstacle),1)
        obstacle.apply_translation([0,10,0]);self.assertEqual(overlap(wire,obstacle),0)
        obstacle.apply_translation([0,-10,0]);self.assertGreater(overlap(wire,obstacle),1)
if __name__=='__main__':unittest.main()
