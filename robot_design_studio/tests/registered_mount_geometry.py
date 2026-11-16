"""Opt-in geometry tests of CAD-derived PCB mount holes and rigid poses."""
import copy,unittest
import numpy as np,trimesh as tm
from studio.mechanics import component_visuals as visuals,pcb_mounts
from studio.mechanics.electronics_layout import overlap
from studio.mechanics.solid import cylinder
class RegisteredMounts(unittest.TestCase):
 def test_native_holes_are_void_in_actual_cad(self):
  for catalog in ['pico2_unknown','mpu6050_lab']:
   entry,cad=visuals.reference(catalog);visuals.validate_mounting(entry,cad)
   wrong=copy.deepcopy(entry);wrong['mounting']['holes'][0]['xy_mm']=[0,0]
   with self.assertRaisesRegex(ValueError,'intersects'):visuals.validate_mounting(wrong,cad)
 def test_flat_upright_rotated_holes_and_solid_bearing(self):
  for catalog,dims in [('pico2_unknown',[55,25,12]),('mpu6050_lab',[25,20,10])]:
   for order in [[0,1,2],[1,2,0],[2,0,1]]:
    orientation=np.eye(4);orientation[:3,:3]=np.eye(3)[order,:];effective=np.array(dims)[order]
    for angle in [0,np.pi/2]:
     w,d=effective[:2] if angle==0 else effective[:2][::-1]
     tray,hardware,mount=pcb_mounts.attach(catalog,dims,orientation,angle,effective[2],w,d)
     self.assertTrue(tray.is_volume);self.assertLess(overlap(tray,hardware),.02)
     matrix=np.array(mount['source_to_tray_mm']);self.assertAlmostEqual(np.linalg.det(matrix[:3,:3]),1.)
     for hole in mount['holes']:
      x,y,z=hole['source_center_mm'];probe=cylinder(x,y,z-5.9,z+.01,mount['fastener']['shaft_diameter_mm']/2-.05).apply_transform(matrix)
      self.assertLess(overlap(probe,tray),.02)
     frame=tm.transformations.rotation_matrix(.8,[1,1,0]);frame[:3,3]=[31,-17,22]
     world=pcb_mounts.world_record(mount,frame,[11,9,-43],-1)
     expected=tm.transform_points([h['source_center_mm'] for h in mount['holes']],np.array(world['source_to_world_mm']))
     self.assertTrue(np.allclose(expected,[h['center_world_mm'] for h in world['holes']]))
 def test_cover_head_entry_and_tool_obstruction(self):
  from studio.mechanics.cover_fasteners import finish
  from studio.mechanics.solid import box,cut
  panel={'owner_body_id':'body','transform_mm':np.eye(4).tolist(),'plane_z_mm':0,'top_local_mm':8,
         'fasteners':[{'position_mm':[x,y,0]} for x,y in [(-6,0),(6,0),(0,6),(0,-6)]]}
  def fixture():
   base=box([-15,-15,-5],[15,15,0]);cap=box([-15,-15,0],[15,15,8])
   base=cut(base,*[cylinder(*p['position_mm'][:2],-3.1,-1,2.5,6) for p in panel['fasteners']])
   return [{'record':{'id':'body_shell'},'mesh':base},{'record':{'id':'body_lid'},'mesh':cap}]
  parts=fixture();rows=finish(parts,{'lid_planes':[panel]},overlap)
  self.assertTrue(all(r['passed'] for r in rows))
  self.assertTrue(all(r['shaft_obstruction_mm3']<.02 and r['head_obstruction_mm3']<.02 for r in rows))
  blocked=fixture();blocked.append({'record':{'id':'foreign_component'},'mesh':box([5,-1,9],[7,1,15])})
  rows=finish(blocked,{'lid_planes':[panel]},overlap)
  self.assertTrue(any(not r['passed'] and r['tool_blocked_assets']==['foreign_component'] for r in rows))
 def test_small_height_is_rejected_without_shrinking_cad(self):
  with self.assertRaisesRegex(ValueError,'envelope'):
   pcb_mounts.attach('pico2_unknown',[55,25,4],np.eye(4),0,4,55,25)
if __name__=='__main__':unittest.main()
