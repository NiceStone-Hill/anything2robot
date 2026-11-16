"""Open-shell repairs must preserve assembly space at any world position."""
import unittest
from studio.mechanics.solid import box, union, overlap, cut, regions
from studio.mechanics.shell_access import reconnect_local

class CorridorReconnectionTests(unittest.TestCase):
    def test_repair_routes_around_assembly_corridor(self):
        for offset in ([0,0,0],[31,-19,7]):
            with self.subTest(offset=offset):
                shell=union(box([-8,-5,-3],[-3,5,3]),box([3,-5,-3],[8,5,3])).apply_translation(offset)
                corridor=box([-2,-1,-10],[2,1,10]).apply_translation(offset)
                domain=box([-10,-10,-5],[10,10,5]).apply_translation(offset)
                repaired,report=reconnect_local(shell,domain,[corridor],radius=2,budget_s=5)
                self.assertTrue(report['passed'],report)
                self.assertEqual(regions(repaired),1)
                self.assertLess(overlap(repaired,corridor),.05)
                self.assertLess(cut(shell,repaired).volume,.01)

if __name__=='__main__':unittest.main()
