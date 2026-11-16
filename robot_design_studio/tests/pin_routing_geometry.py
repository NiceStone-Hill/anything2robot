"""Independent CAD-pad alignment and truthful guide/connector evidence checks."""
import re
import unittest
import numpy as np
import trimesh as tm
from studio.mechanics import component_visuals
from studio.mechanics.harness_connections import reference_pad,wire_color,segment_lane,guide_mesh,endpoint_record,envelope_exit
from studio.mechanics.bundle_network import pack
from studio.catalog import Catalog


class PinRoutingTests(unittest.TestCase):
    def test_diagonal_exit_stops_at_first_envelope_face(self):
        normal=np.array([1.,2.,0.]);normal/=np.linalg.norm(normal)
        point=envelope_exit(np.zeros(3),normal,np.array([[-2,-3,-4],[2,3,4]]),0.)
        np.testing.assert_allclose(point,[1.5,3,0])

    def test_pico_pads_are_real_step_header_holes_and_follow_cad_pose(self):
        entry,mesh=component_visuals.reference('pico2_unknown')
        matrix=tm.transformations.rotation_matrix(.61,[0,1,0]);matrix[:3,3]=[32,41,-83]
        asset={**entry,'model_id':entry['id'],'source_to_world_mm':matrix.tolist()}
        text=(component_visuals.ROOT/entry['id']/entry['source_file']).read_text()
        rows=dict(re.findall(r'#(\d+)\s*=\s*(.*?);',text,re.S));holes=[]
        for value in rows.values():
            match=re.match(r"CIRCLE\('',#(\d+),([^,)]+)\)",value)
            if not match or abs(float(match[2])-.475)>1e-5:continue
            axis=re.search(r"AXIS2_PLACEMENT_3D\([^,]*,#(\d+)",rows[match[1]])
            if not axis:continue
            coords=re.search(r'\(([-\d.Ee+, ]+)\)',rows[axis[1]])
            if coords:holes.append([float(v) for v in coords[1].split(',')])
        for pin in [1,2,3,4,36,39,40]:
            record=reference_pad(asset,'port',str(pin));native=np.array(record['source_position_mm'])
            self.assertLess(np.linalg.norm(np.array(holes)-native,axis=1).min(),1e-5)
            expected=tm.transform_points([native],matrix)[0]/1000
            np.testing.assert_allclose(record['position_m'],expected,atol=1e-12)
            np.testing.assert_allclose(record['exit_direction'],matrix[:3,2])
            self.assertFalse(record['physical_verified'])
        # USB +Y: numbered header order must not accidentally be mirrored.
        self.assertEqual(reference_pad(asset,'GP0','1')['source_position_mm'][:2],[-8.89,24.13])
        self.assertEqual(reference_pad(asset,'VSYS','39')['source_position_mm'][:2],[8.89,21.59])

    def test_mpu_reference_does_not_claim_verified_label_or_connector(self):
        asset={'model_id':'mpu6050','source_to_world_mm':np.eye(4).tolist(),'source_sha256':'test'}
        records=[reference_pad(asset,p,None) for p in ['VCC','GND','SCL','SDA']]
        for record in records:
            self.assertFalse(record['pin_label_verified']);self.assertFalse(record['physical_verified'])
            self.assertIn('assumed',record['evidence'])
        np.testing.assert_allclose(np.diff([r['source_position_mm'][0] for r in records]),[-18/7]*3)

    def test_unknown_hardware_has_distinct_explicit_placeholders(self):
        project={'instances':[{'id':'supply','catalog_id':'supply_5_candidate','catalog_revision':Catalog().parts['supply_5_candidate']['revision'],'label':'5V supply'}]}
        mesh=tm.creation.box([20,12,8]);records=[]
        for index,port in enumerate(['OUT','GND']):
            records.append(endpoint_record({'instance_id':'supply','port_id':port},project,{'visual_assets':[],'component_placements':[]},{'supply':mesh},np.array([30,0,0]),index,2,Catalog()))
        self.assertNotEqual(records[0]['position_m'],records[1]['position_m'])
        for r in records:
            self.assertIn('placeholder',r['evidence']);self.assertFalse(r['physical_verified'])

    def test_distinct_colours_and_full_diameter_packed_lanes(self):
        self.assertEqual(len({wire_color(i) for i in range(50)}),50)
        radius,circles=pack((1.2,)*11)
        segment={'points_m':[[0,0,0],[.04,0,0]],'packing_wire_order':['w'+str(i) for i in range(11)],'packing_circles_xy_radius_mm':circles}
        lanes=[segment_lane(segment,key) for key in segment['packing_wire_order']]
        for i,lane in enumerate(lanes):
            self.assertLessEqual(np.linalg.norm(lane[0][1:])+.6,radius-.5+1e-8)
            for other in lanes[:i]:self.assertGreaterEqual(np.linalg.norm(lane[0]-other[0]),1.4-1e-8)
        mesh=guide_mesh([[0,0,0],[.04,0,0]],.6)
        np.testing.assert_allclose(mesh.extents,[40,1.2,1.2],atol=1e-8)
        self.assertGreater(len(guide_mesh([[0,0,0],[.04,0,0]],.6,True).split()),1)


if __name__=='__main__':unittest.main()
