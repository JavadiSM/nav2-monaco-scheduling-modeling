import csv,json,math,tempfile,unittest
from pathlib import Path
from scripts.report_full_course_comparison import aggregate_policy,deadline_floor_diagnostics,export_calibration_statistics,METRICS
class FullCourseStatisticsTests(unittest.TestCase):
    def test_equal_run_means_are_distinct_from_pooled_job_means(self):
        rows=[]
        for response,completed,met,missed,completion in [(1.,1,2,2,10.),(3.,9,9,1,20.)]:
            row={key:0 for key,_,_ in METRICS}
            row.update(placement='local',average_response_s=response,completed=completed,jobs=completed+1,unfinished=1,deadline_met=met,deadline_missed=missed,deadline_meet_rate=met/(met+missed),completion_sim_s=completion)
            rows.append(row)
        cooling=[dict(policy='local',core_types='Cortex-A7',cooling_entries='3',cooling_duration_s='4')]
        result=aggregate_policy(rows,cooling,'local')
        self.assertEqual(result['n_runs'],2);self.assertEqual(result['average_response_s_mean'],2.)
        self.assertAlmostEqual(result['pooled_response_s'],2.8)
        self.assertAlmostEqual(result['pooled_DMR'],11/14)
        self.assertEqual(result['completion_sim_s_mean'],15.)
        self.assertAlmostEqual(result['completion_sim_s_sample_sd'],math.sqrt(50))
        self.assertEqual(result['Cortex-A7_cooling_entries_mean_per_run'],1.5)
        self.assertEqual(result['Cortex-A7_cooling_seconds_mean_per_run'],2.)
    def test_deadline_lower_bounds_round_service_and_include_upload(self):
        def job(device,budget,deadline,upload=0.,status='missed'):
            return dict(execution_device=device,selected_budget_s=budget,D_i_s=deadline,
                        release_s='2',upload_arrival_s=2+upload,deadline_status=status)
        jobs=[job(0,.001,.0009),job(1,.001,.003,.003),
              job(1,.001,.001),job(1,.001,.0001,status='pending_not_due')]
        result=deadline_floor_diagnostics(jobs,.001)
        self.assertEqual(result['evaluated_jobs'],3)
        self.assertEqual(result['sub_step_deadline_jobs'],1)
        self.assertEqual(result['optimistic_service_impossible'],1)
        self.assertEqual(result['upload_plus_service_impossible'],2)

    def test_upload_bound_applies_before_a_budget_is_sealed(self):
        base=dict(execution_device='1',selected_budget_s='',D_i_s='.002',
                  release_s='1',upload_arrival_s='1.003',deadline_status='missed')
        rows=[base,dict(base,D_i_s='.003'),dict(base,deadline_status='pending_not_due')]
        result=deadline_floor_diagnostics(rows,.001)
        self.assertEqual(result['evaluated_jobs'],2)
        self.assertEqual(result['upload_only_impossible'],1)
        self.assertEqual(result['sealed_budget_jobs'],0)
        self.assertEqual(result['upload_plus_service_impossible'],0)

    def test_hi_bound_includes_middle_frequency_before_overrun(self):
        job=dict(execution_device='1',selected_budget_s='.008',C_LO_s='.0038',D_i_s='.0028',
                 release_s='0',upload_arrival_s='0',deadline_status='missed')
        result=deadline_floor_diagnostics([job],.001)
        self.assertEqual(result['optimistic_service_impossible'],1)
        self.assertEqual(result['reaction_service_impossible'],1)
        # At 2.7 ms the continuous maximum service is feasible, but its
        # 1 ms lattice minimum is 3 ms; no rounding of D is permitted.
        job['D_i_s']='.0031';job['selected_budget_s']='.010'
        result=deadline_floor_diagnostics([job],.001)
        self.assertEqual(result['optimistic_service_impossible'],0)
        self.assertEqual(result['reaction_service_impossible'],1)

    def test_calibration_statistics_never_connect_separate_lap_clocks(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name)
            (root/'frozen-task-parameters.json').write_text(json.dumps({'tasks':{'task':{'C_LO_s':8/3,'C_HI_s':4,'class':'P','T_nominal_s':.5}}}))
            with (root/'calibration-samples.csv').open('w',newline='') as stream:
                writer=csv.writer(stream);writer.writerow(['run','task','actual_CPU_s','release_sim_s'])
                writer.writerows([['first','task',1,10],['first','task',3,11],['second','task',4,10]])
            row=export_calibration_statistics(root,root)[0]
            self.assertEqual(row['CPU_sample_count'],3)
            self.assertAlmostEqual(row['CPU_mean_pooled_s'],8/3)
            self.assertEqual(row['CPU_mean_of_run_means_s'],3)
            self.assertEqual(row['observed_SIM_inter_arrival_mean_s'],1)
            self.assertEqual(row['observed_SIM_inter_arrival_min_s'],1)

    def test_missing_policy_has_no_manufactured_zero_metrics(self):
        result=aggregate_policy([],[],'greedy')
        self.assertEqual(result['n_runs'],0);self.assertIsNone(result['completion_sim_s_mean']);self.assertIsNone(result['pooled_DMR'])
if __name__=='__main__':unittest.main()
