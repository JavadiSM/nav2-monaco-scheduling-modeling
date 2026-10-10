import hashlib,json,os,tempfile,unittest
from unittest.mock import patch
from pathlib import Path
from scripts.run_full_course_comparison import has_valid_completion,active_identity,assert_no_active_campaign
class CampaignResumeTests(unittest.TestCase):
    def fixture(self,root,passed=True,targets=20):
        trial=root/'trial';trial.mkdir();run=dict(completed=True,simulation_completed=True,navigation_outcomes={'red':dict(passed=passed,ordered_targets_passed=targets)})
        (trial/'run.json').write_text(json.dumps(run));(trial/'validation.json').write_text(json.dumps({'passed':True}));return trial,dict(trial='trial',status='complete',exit_code=0)
    def test_exit_success_does_not_accept_failed_navigation(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);trial,row=self.fixture(root,False);self.assertFalse(has_valid_completion(root,row))
    def test_partial_target_sequence_cannot_be_resumed_as_complete(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);trial,row=self.fixture(root,targets=19);self.assertFalse(has_valid_completion(root,row))
    def test_recovered_presentation_preserves_original_physics_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);trial,row=self.fixture(root);run=json.loads((trial/'run.json').read_text());run.update(completed=False,error='Presentation timeout');(trial/'run.json').write_text(json.dumps(run));row['exit_code']=1
            self.assertFalse(has_valid_completion(root,row))
            recovery=dict(kind='recording_postprocessing',validated_full_mission=True,original_evidence_sha256={'run.json':hashlib.sha256((trial/'run.json').read_bytes()).hexdigest()})
            (trial/'recording-recovery.json').write_text(json.dumps(recovery));row['postprocessing_recovery']='recording-recovery.json'
            self.assertTrue(has_valid_completion(root,row))
            (trial/'run.json').write_text('{}');self.assertFalse(has_valid_completion(root,row))
    def test_independent_trial_completion_keeps_unknown_exit_status(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);trial,row=self.fixture(root);row['exit_code']=None
            record=dict(kind='runner_interruption',validated_full_mission=True,original_evidence_sha256={'run.json':hashlib.sha256((trial/'run.json').read_bytes()).hexdigest()})
            (trial/'runner-recovery.json').write_text(json.dumps(record));row['manager_recovery']='runner-recovery.json'
            self.assertTrue(has_valid_completion(root,row));self.assertIsNone(row['exit_code'])
            (trial/'run.json').write_text('{}');self.assertFalse(has_valid_completion(root,row))
    def test_manager_recovery_does_not_accept_incomplete_presentation(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);trial,row=self.fixture(root);run=json.loads((trial/'run.json').read_text());run['completed']=False;(trial/'run.json').write_text(json.dumps(run));row['exit_code']=None
            record=dict(kind='runner_interruption',validated_full_mission=True,original_evidence_sha256={'run.json':hashlib.sha256((trial/'run.json').read_bytes()).hexdigest()})
            (trial/'runner-recovery.json').write_text(json.dumps(record));row['manager_recovery']='runner-recovery.json';self.assertFalse(has_valid_completion(root,row))
    def test_resume_refuses_live_trial_after_manager_exit(self):
        with patch('scripts.run_full_course_comparison.active_identity',side_effect=[False,True]):
            with self.assertRaisesRegex(ValueError,'previous trial is still active'):assert_no_active_campaign({'runner':{},'active_trial_pid':123,'active_trial_start_ticks':456})
    def test_process_identity_checks_ticks_and_command(self):
        ticks=int(Path('/proc/self/stat').read_text().rsplit(') ',1)[1].split()[19])
        self.assertTrue(active_identity(os.getpid(),ticks,'python'))
        self.assertFalse(active_identity(os.getpid(),ticks+1,'python'))
        with self.assertRaisesRegex(ValueError,'command did not'):active_identity(os.getpid(),ticks,'nonexistent-command-marker')
if __name__=='__main__':unittest.main()
