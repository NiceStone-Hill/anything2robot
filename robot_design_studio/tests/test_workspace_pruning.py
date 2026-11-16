import importlib.util,json,tempfile,unittest
from pathlib import Path
spec=importlib.util.spec_from_file_location('pruner',Path(__file__).resolve().parents[1]/'tools/prune_workspace.py')
pruner=importlib.util.module_from_spec(spec);spec.loader.exec_module(pruner)

class PruningTests(unittest.TestCase):
    def test_retains_failed_current_and_dependency_not_previous_success(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for rid,result in [('run_old',{'geometry_valid':True}),('run_latest',{'geometry_valid':False,'mechanical_run_id':'run_support'}),('run_support',{})]:
                folder=root/'runs'/rid;folder.mkdir(parents=True)
                (folder/'run.json').write_text(json.dumps({'kind':'assembly'}));(folder/'result.json').write_text(json.dumps(result))
            folder=root/'project_a';folder.mkdir()
            p={'id':'project_a','revision':8,'updated_at':'2026-11-17','active_run_id':None,'events':[],
               'runs':['run_old','run_support','run_latest'],'mechanical_result':{'run_id':'run_latest','geometry_valid':False}}
            f=folder/'project.json';f.write_text(json.dumps(p));original=f.read_bytes()
            pruner.prune(root);self.assertEqual(f.read_bytes(),original)
            report=pruner.prune(root,True)
            self.assertEqual(set(report['retained_runs']),{'run_support','run_latest'})
            self.assertFalse((root/'runs/run_old').exists())
            self.assertFalse(json.loads(f.read_text())['mechanical_result']['geometry_valid'])
            self.assertEqual(len(list((folder/'history').iterdir())),1)
    def test_missing_dependency_aborts_before_deleting(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);f=root/'project_a/project.json';f.parent.mkdir()
            f.write_text(json.dumps({'id':'project_a','mechanical_result':{'run_id':'run_missing'}}))
            old=root/'runs/run_old';old.mkdir(parents=True)
            with self.assertRaises(ValueError):pruner.prune(root,True)
            self.assertTrue(old.exists())
