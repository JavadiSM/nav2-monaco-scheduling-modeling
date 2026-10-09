#!/usr/bin/env python3
"""Extract mean/max HI-task budgets from the sealed characterization campaign."""
import csv,hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    evidence=ROOT/'docs/evidence'
    source=evidence/'task-characterization-summary.json'
    summary=json.loads(source.read_text())
    if summary['status']!='FINAL':raise ValueError('A sealed campaign is required')
    sys.path.insert(0,str(ROOT))
    from tools.abstract_compute.hardware import load_platform
    reference=json.loads((ROOT/'config/task_execution.json').read_text())['reference']
    cores=load_platform()[0].cores
    historical=json.loads((evidence/'extracted-parameters.json').read_text())
    rows={}
    for name,row in historical['tasks'].items():
        m=summary['tasks'][name]['metrics']['cpu_inclusive_ms']
        if m['n']!=row['jobs']:raise ValueError('Sample count mismatch')
        rows[name]={k:row[k] for k in ('task','task_id','class','T_nominal_s','observed_mean_inter_entry_s','granularity')}
        rows[name].update({'budget_status':'USER_ASSUMED_MEAN_LO_OBSERVED_MAX_HI','criticality':'HI','C_LO_s':m['mean']/1000.,'C_HI_s':m['max']/1000.,'sample_count':m['n'],'historical_Q95_s':row['C_model_s']})
    for row in rows.values():
        for mode in ('LO','HI'):
            row[f'W_{mode}_mcycles']=row[f'C_{mode}_s']*reference['frequency_mhz']*reference['performance_eta']
            for kind,cid in (('A7',0),('A15',1)):
                core=cores[cid];row[f'{kind}_C_{mode}_s']=row[f'W_{mode}_mcycles']/(core.dvfs_level().frequency_mhz*core.core_type.performance_eta)
    data={'schema_version':1,'source':source.name,'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'successful_laps':summary['successful_analyzed_laps'],'reference':reference,'cpu_clock':'HOST_THREAD_CPU','unit':'s','budget_policy':'actual_CPU_le_mean_selects_LO_else_observed_max_HI','bounds_status':'empirical_model_budgets_not_certified_WCET','tasks':rows}
    (evidence/'dual-budget-parameters.json').write_text(json.dumps(data,indent=2)+'\n')
    with (evidence/'dual-budget-parameters.csv').open('w',newline='') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['task','criticality','samples','C_LO_reference_s','C_HI_reference_s','historical_Q95_s','T_nominal_s','W_LO_mcycles','W_HI_mcycles','A7_C_LO_s','A7_C_HI_s','A15_C_LO_s','A15_C_HI_s'])
        for name,t in rows.items():w.writerow([name,'HI',t['sample_count'],t['C_LO_s'],t['C_HI_s'],t['historical_Q95_s'],t['T_nominal_s'],t['W_LO_mcycles'],t['W_HI_mcycles'],t['A7_C_LO_s'],t['A7_C_HI_s'],t['A15_C_LO_s'],t['A15_C_HI_s']])
    print(f"Extracted {len(rows)} HI tasks from {data['successful_laps']} complete missions")
if __name__=='__main__':main()
