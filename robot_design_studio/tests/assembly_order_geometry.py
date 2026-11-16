"""Deferred actuators need a real, acyclic and enforced assembly sequence."""
import unittest
import numpy as np
from studio.mechanics.fastening import placed_access, check_access, order_assembly
from studio.mechanics.joint_modules import module
from studio.mechanics.solid import box


class AssemblyOrderTests(unittest.TestCase):
    def setUp(self):
        self.graph={'joints':[{'id':'upstream','parent':'base','child':'middle'},
                              {'id':'downstream','parent':'middle','child':'tip'}]}

    def records(self,identity):
        joint=next(j for j in self.graph['joints'] if j['id']==identity)
        built=module('sg90_digital_towerpro')
        built.update(transform=np.eye(4),joint=joint,id=identity,graph=self.graph)
        return placed_access(built,{'base':box([-30,-30,-30],[30,30,30])})

    def test_only_registered_later_hardware_may_be_absent(self):
        site=next(s for s in self.records('upstream') if s['id'].endswith('horn_center'))
        self.assertEqual(site['deferred_joint_ids'],['downstream'])
        motor={'id':'later_motor','joint_id':'downstream','owner':'middle','role':'motor','mesh':box([-3,-3,10],[3,3,15])}
        pieces={'middle':box([20,20,0],[22,22,2])}
        self.assertTrue(check_access([site],pieces,{},[motor])[0]['passed'])
        motor['joint_id']='upstream'
        self.assertFalse(check_access([site],pieces,{},[motor])[0]['passed'])
        motor['joint_id']='downstream';pieces['middle']=motor['mesh']
        self.assertFalse(check_access([site],pieces,{},[motor])[0]['passed'])

    def steps_and_reports(self,count=3):
        reports=[s for identity in ['upstream','downstream'] for s in self.records(identity)]
        steps=[{'id':identity+'_step_'+str(i)} for identity in ['upstream','downstream'] for i in range(count)]
        steps.extend({'id':r['id']+'_fasten'} for r in reports)
        return steps,reports

    def test_parent_fastening_precedes_every_deferred_motor(self):
        steps,reports=self.steps_and_reports()
        ordered=order_assembly(steps,reports,self.graph);indices={s['id']:i for i,s in enumerate(ordered)}
        for report in reports:
            for later in report['deferred_joint_ids']:
                self.assertLess(indices[report['id']+'_fasten'],indices[later+'_step_0'])
        for step in ordered:
            self.assertTrue(all(indices[d]<indices[step['id']] for d in step['depends_on']))

    def test_two_step_templates_and_cycles(self):
        steps,reports=self.steps_and_reports(2)
        self.assertEqual(len(order_assembly(steps,reports,self.graph)),len(steps))
        steps[0]['depends_on']=['downstream_step_0']
        with self.assertRaises(ValueError):order_assembly(steps,reports,self.graph)

    def test_missing_deferred_installation_is_rejected(self):
        steps,reports=self.steps_and_reports()
        reports[0]['deferred_joint_ids']=['unregistered']
        with self.assertRaises(ValueError):order_assembly(steps,reports,self.graph)


if __name__=='__main__':unittest.main()
