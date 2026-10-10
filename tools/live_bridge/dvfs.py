"""Paired operating-point selection and independent execution-accounting checks."""
import math


def configured_selector(config):
    """Build the live policy; custom schedulers may supply an engine callback."""
    dvfs=config.get('dvfs',{})
    dt=config['step_ns']/1e9
    if dvfs.get('decision_epoch_s',dt)!=dt:
        raise ValueError('DVFS decisions must use the common physics/hardware lattice')
    if dvfs.get('switch_latency_s',0.)!=0.:
        raise ValueError('Nonzero DVFS switching latency is not modeled yet')
    if dvfs.get('overrun_policy','remaining_work_at_maximum_point') not in ('remaining_work_at_maximum_point','continue_selected_point','device_maximum_until_overruns_complete'):
        raise ValueError('Unsupported per-job overrun policy')
    policy=dvfs.get('policy','maximum')
    if policy=='maximum':return None
    if policy=='fixed':
        level=dvfs['level_id']
        if isinstance(level,bool) or not isinstance(level,int):raise ValueError('Invalid operating-point ID')
        return lambda engine,job,core:level
    raise ValueError('Unknown DVFS policy')


def validate_job_execution(job,spec,core,dt,cooling_mask,end_tick,*,overrun_policy='remaining_work_at_maximum_point',device_boost_mask=None):
    """Reconstruct delivered work from recorded f/V intervals, including pauses."""
    errors=[];jid=job['job_id'];eps=1e-8
    def error(message):errors.append(f'DVFS job {jid}: {message}')
    spans=job['execution_segments'];start=job['start_tick'];finish=job['budget_end_tick']
    if start is None:
        if spans:error('execution before dispatch')
        return errors
    stop=end_tick if finish is None else finish
    remaining=spec.reference_work(job['selected_budget_s']);consumed=0.;ticks=0;previous=start;crossing=None
    threshold=spec.reference_work(spec.budget_lo_s) if job.get('budget_mode')=='HI' else None
    for span in spans:
        a,b=span['start_tick'],span['end_tick'];level=core.core_type.dvfs_by_id.get(span['level_id'])
        if not(start<=a<b<=stop) or a<previous:
            error('overlapping or out-of-range execution segment');continue
        previous=b;ticks+=b-a
        if any(cooling_mask[a:b]):error('work consumed during whole-device cooling')
        if level is None:
            error('invalid operating-point ID');continue
        if (span['frequency_mhz'],span['voltage_v'])!=(level.frequency_mhz,level.voltage_v):error('unpaired frequency/voltage')
        rate=level.frequency_mhz*core.core_type.performance_eta;per_tick=rate*dt;capacity=(b-a)*per_tick;work=min(remaining,capacity)
        if not math.isclose(span['work_mcycles'],work,rel_tol=1e-9,abs_tol=eps):error('incorrect work integral')
        if remaining<=capacity+eps and remaining<=capacity-per_tick+eps:error('execution continued beyond the first completion tick')
        if threshold is not None and crossing is None and consumed+work>=threshold-eps:
            crossing=a+max(0,math.ceil((threshold-consumed)/per_tick-1e-9))
        if overrun_policy=='remaining_work_at_maximum_point' and job.get('overrun_tick') is not None and b>job['overrun_tick'] and level.level_id!=core.core_type.default_dvfs_level_id:
            error('nonmaximum point after overrun boundary')
        if overrun_policy=='device_maximum_until_overruns_complete':
            if device_boost_mask is None:error('missing device boost history')
            elif level.level_id!=core.core_type.default_dvfs_level_id and any(device_boost_mask[a:b]):error('nonmaximum point during device boost')
        consumed+=work;remaining=max(0.,remaining-work)
    active_ticks=stop-start-int(sum(cooling_mask[start:stop]))
    if ticks!=active_ticks:error('missing or extra active execution ticks')
    if not math.isclose(job['consumed_work_mcycles'],consumed,rel_tol=1e-9,abs_tol=eps):error('incorrect consumed-work total')
    if not math.isclose(job['remaining'],remaining,rel_tol=1e-9,abs_tol=eps):error('incorrect remaining work')
    if finish is not None and remaining>eps:error('completion before selected work was consumed')
    if finish is None and spans and remaining<=eps:error('missing completion boundary')
    if job.get('overrun_tick')!=crossing:error('incorrect LO-budget crossing tick')
    return errors
