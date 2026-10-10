"""Sample relative deadlines from HI work and the ideal fastest A15 service."""
import random
from tools.abstract_compute.hardware import load_platform


def generate_deadlines(tasks,platform_path=None):
    # Seconds are converted through the declared work normalization, not a CPU label.
    devices=load_platform(platform_path)
    a15=[c.core_type for d in devices.values() for c in d.cores.values() if c.core_type.name=='Cortex-A15']
    if not a15:raise ValueError('An A15 deadline reference is required')
    reference=max(a15,key=lambda t:t.max_frequency_mhz*t.performance_eta)
    rate=reference.max_frequency_mhz*reference.performance_eta
    rng=random.SystemRandom();rows={}
    for name,spec in sorted(tasks.items()):
        c=spec.budget_hi_s if spec.budget_hi_s is not None else spec.budget_s
        work=spec.reference_work(c);ideal=work/rate;factor=rng.uniform(1.1,1.3)
        rows[name]=dict(C_HI_reference_s=c,W_HI_mcycles=work,
                       reference_frequency_mhz=spec.reference_frequency_mhz,reference_eta=spec.reference_eta,
                       T_i_s=spec.period_s,release_rate_hz=spec.activation_rate_hz,
                       ideal_A15_max_HI_s=ideal,uniform_factor=factor,D_i_s=ideal*factor)
    return dict(schema_version=2,units='s',work_units='normalized Mcycles',
                C_basis='HI observed maximum at the declared reference normalization',
                deadline_reference=dict(core='Cortex-A15',frequency_mhz=reference.max_frequency_mhz,eta=reference.performance_eta),
                formula='D_i = uniform(1.1,1.3) * W_HI / (f_A15_max * eta_A15)',
                excluded_costs=['queueing','communication','thermal effects','cooling'],
                relative_to='actual callback arrival',period_policy='unchanged activation period in seconds',tasks=rows)


def validate_deadlines(table,tasks,devices):
    """Audit the unit conversion independently of queueing and thermal state."""
    import math
    errors=[]
    types=[c.core_type for d in devices.values() for c in d.cores.values() if c.core_type.name=='Cortex-A15']
    if not types:return ['No A15 deadline reference']
    rate=max(t.max_frequency_mhz*t.performance_eta for t in types)
    if set(table.get('tasks',{}))!=set(tasks):errors.append('Incomplete deadline families')
    for name,spec in tasks.items():
        row=table.get('tasks',{}).get(name,{})
        c=spec.budget_hi_s if spec.budget_hi_s is not None else spec.budget_s
        work=c*spec.reference_frequency_mhz*spec.reference_eta
        factor=row.get('uniform_factor',float('nan'))
        expected=dict(C_HI_reference_s=c,W_HI_mcycles=work,ideal_A15_max_HI_s=work/rate,D_i_s=work/rate*factor)
        for key,value in expected.items():
            if not math.isclose(row.get(key,float('nan')),value,rel_tol=1e-12,abs_tol=1e-15):errors.append(name+': invalid '+key)
        if not 1.1<=factor<=1.3:errors.append(name+': invalid deadline factor')
        if row.get('T_i_s')!=spec.period_s:errors.append(name+': altered activation period')
    return errors
