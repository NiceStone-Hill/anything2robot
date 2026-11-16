import unittest
from studio.jobs import job_time_budget
class JobBudgetTests(unittest.TestCase):
 def project(self,count):return {'articulation':{'graph':{'joints':[{}]*count}}}
 def test_head_and_electrical_limits_unchanged(self):
  self.assertEqual(job_time_budget(self.project(1),'assembly',85),180)
  self.assertEqual(job_time_budget(self.project(5),'wiring',85),85)
  self.assertEqual(job_time_budget(self.project(5),'harness',85),180)
 def test_larger_graph_has_bounded_time_and_explicit_override(self):
  self.assertEqual(job_time_budget(self.project(5),'assembly',85),540)
  self.assertEqual(job_time_budget(self.project(5),'layout',85),540)
  self.assertEqual(job_time_budget(self.project(100),'assembly',85),630)
  self.assertEqual(job_time_budget(self.project(5),'assembly',700),700)
 def test_removable_shell_search_budget_is_bounded_by_body_count(self):
  p=self.project(3);p['articulation']['graph']['bodies']=[{}]*4
  self.assertEqual(job_time_budget(p,'assembly',85),600)
  self.assertEqual(job_time_budget(p,'layout',85),360)
  p['articulation']['graph']['bodies']=[{}]*100
  self.assertEqual(job_time_budget(p,'assembly',85),1080)
if __name__=='__main__':unittest.main()
