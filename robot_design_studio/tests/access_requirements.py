"""Intact output shells must not hide required assembly/service openings."""
import unittest
from studio.mechanics.shell_access import access_requirement
class AccessRequirements(unittest.TestCase):
 def graph(self):return {'root_body_id':'base','joints':[{'id':'a','parent':'base','child':'arm'}]}
 def test_output_only_arm_can_stay_intact(self):
  modules=[{'owner':'arm','role':r} for r in ['rotor','output_mount']]
  self.assertFalse(access_requirement('arm',self.graph(),modules,{})['required'])
 def test_external_root_requires_real_access_checks_instead_of_oversized_hatch(self):
  modules=[{'owner':'base','role':'motor','joint_id':'a','external_service':True}]
  self.assertFalse(access_requirement('base',self.graph(),modules,{})['required'])
  self.assertTrue(access_requirement('base',self.graph(),modules,{'service_opening_requirements':[{'owner_body_id':'base'}]})['required'])
 def test_root_and_internal_motor_still_require_access(self):
  self.assertTrue(access_requirement('base',self.graph(),[],{})['required'])
  self.assertTrue(access_requirement('arm',self.graph(),[{'owner':'arm','role':'motor'}],{})['required'])
 def test_child_joint_requires_access(self):
  g=self.graph();g['joints'].append({'id':'b','parent':'arm','child':'tip'})
  self.assertTrue(access_requirement('arm',g,[],{})['required'])
 def test_assigned_electronics_require_access(self):
  for p in [{'component_placements':[{'owner_body_id':'arm'}]}, {'layout_config':{'components':[{'owner_body_id':'arm'}]}}]:
   self.assertTrue(access_requirement('arm',self.graph(),[],p)['required'])
if __name__=='__main__':unittest.main()
