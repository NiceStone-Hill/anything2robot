import unittest
from studio.catalog import Catalog
from studio.providers import reference
from studio.electrical import review
from studio.contracts import ContractError
class PwmCatalogTests(unittest.TestCase):
 def test_pca_reference_has_separate_servo_supply_and_official_assets(self):
  from pathlib import Path
  p=Catalog().parts['pca9685_adafruit_815']
  self.assertEqual(len([k for k in p['ports'] if k.startswith('PWM')]),16)
  self.assertFalse(p['electrical']['servo_supply_regulated'])
  self.assertIn('VPLUS',p['ports'])
  self.assertEqual(p['electrical']['address_options'],[64])
  self.assertIsNone(p['geometry']['dimensions_m']['value'])
  for key in ('cad_asset','display_asset'):
   self.assertTrue((Path(__file__).resolve().parents[1]/p['geometry'][key]).is_file())
 def test_pca_routes_servos_from_driver_without_imu(self):
  c,p=self.project('2')
  p['instances']=[i for i in p['instances'] if i['catalog_id']!='mpu6050_lab']
  p['instances'].append({'id':'driver','catalog_id':'pca9685_adafruit_815','catalog_revision':'1','label':'Driver','locked':False,'owner_body_id':None})
  proposal=reference(p,c)
  self.assertFalse(review(p,proposal,c)['errors'])
  self.assertEqual([n['endpoints'][0]['instance_id'] for n in proposal['nets'] if n['kind']=='pwm'],['driver']*3)
  self.assertEqual([a['function'] for a in proposal['allocations']],['I2C0_SDA','I2C0_SCL'])
 def test_single_external_feed_has_no_internal_power_or_splitter_requests(self):
  c,p=self.project('2')
  removed={'mpu6050_lab','supply_5_candidate','supply_48_candidate','splitter_candidate'}
  p['instances']=[i for i in p['instances'] if i['catalog_id'] not in removed]
  p['instances'].append({'id':'driver','catalog_id':'pca9685_adafruit_815','catalog_revision':'1','label':'Driver','locked':False,'owner_body_id':None})
  p['power_config']={'mode':'single_external','voltage_v':None,'distribution':'wire_splice'}
  proposal=reference(p,c);checked=review(p,proposal,c)
  self.assertTrue(checked['logical_valid'],checked['errors'])
  self.assertEqual(proposal['missing_parts'],[])
  self.assertIn('external_power_unverified',[x['code'] for x in checked['pending']])
  self.assertFalse(checked['manufacturing_released'])
  rail=next(n for n in proposal['nets'] if n['id']=='external_power')
  self.assertEqual(len(rail['endpoints']),5)
  self.assertNotIn('3V3',[e['port_id'] for e in rail['endpoints']])
  rail['voltage_v']=[6,6]
  self.assertFalse(review(p,proposal,c)['logical_valid'])
  del p['power_config'];proposal=reference(p,c)
  self.assertTrue({'supply_5_candidate','supply_48_candidate','splitter_candidate'}<={r['catalog_id'] for r in proposal['missing_parts']})
 def project(self,revision):
  c=Catalog();ids=['pico2_unknown','mpu6050_lab']+['sg90_digital_towerpro']*3+['supply_5_candidate','supply_48_candidate','wire_candidate','splitter_candidate']
  p={'id':'project_test','revision':1,'intent':'three servos','instances':[{'id':'part_'+str(n),'catalog_id':k,'catalog_revision':revision if k=='pico2_unknown' else c.parts[k]['revision'],'label':k,'locked':False,'owner_body_id':None} for n,k in enumerate(ids)]};return c,p
 def test_three_registered_pwm_channels_are_reviewed(self):
  c,p=self.project('2');proposal=reference(p,c)
  self.assertEqual([(a['port_id'],a['function']) for a in proposal['allocations'] if a['function'].startswith('PWM')],[('GP2','PWM1_A'),('GP3','PWM1_B'),('GP4','PWM2_A')])
  self.assertFalse(review(p,proposal,c)['errors'])
 def test_old_catalog_revision_remains_unchanged(self):
  c,p=self.project('1');self.assertNotIn('GP4',c.resolve(p['instances'][0])['ports'])
  with self.assertRaises(ContractError):reference(p,c)
if __name__=='__main__':unittest.main()
