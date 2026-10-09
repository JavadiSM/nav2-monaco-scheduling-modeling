"""Lockstep broker for real callback arrivals, staged outputs and local devices."""
import argparse,csv,json,mmap,os,selectors,socket,struct,subprocess,time,traceback,hashlib
from dataclasses import asdict
from pathlib import Path
from .engine import LiveEngine,TASK_NAMES
PACKET=struct.Struct('<7Q')
ENTER,DEP,STAGE,END,WAIT,RESUME,SLEEP,PUBLISH,LOOKUP,DEVICE_GATE,START_READY,START_ACK,TRIGGER,RESULT_OWNER,RESULT_GATE,PUBLISH_BEGIN=range(1,17)
def runtime_paths(out):
    tag=hashlib.sha256(str(Path(out).resolve()).encode()).hexdigest()[:16]
    return '/tmp/nav2-live-'+tag+'.sock',Path('/tmp/nav2-live-'+tag+'.clock')

class Broker:
    def __init__(self,out,seconds,vehicles=None,bootstrap_s=None):
        self.out=Path(out).resolve();self.out.mkdir(parents=True,exist_ok=True)
        config_file=self.out/'trial-config.json'
        self.config=json.loads((config_file if config_file.exists() else Path(__file__).resolve().parents[2]/'config/live_bridge.json').read_text())
        vehicles=vehicles or self.config['vehicle_count'];self.seconds=seconds;self.engine=LiveEngine(vehicles,self.config['step_ns'],self.out/'hardware-config.json' if (self.out/'hardware-config.json').exists() else None,dual_budget=self.config.get('budget_policy')=='actual_CPU_mean_LO_observed_max_HI',task_parameters=self.out/'task-parameters.json' if (self.out/'task-parameters.json').exists() else None);self.dt=self.engine.dt;self.limit=round((seconds+self.config['bootstrap_sim_s'])/self.dt);self.bootstrap=round((self.config['bootstrap_sim_s'] if bootstrap_s is None else bootstrap_s)/self.dt)
        self.sockpath,self.clockpath=runtime_paths(self.out)
        # Keep an already mapped clock file nonempty while attaching the broker.
        self.clockfile=self.clockpath.open('r+b' if self.clockpath.exists() else 'w+b')
        self.clockfile.truncate(32);self.clock=mmap.mmap(self.clockfile.fileno(),32)
        self.clock[:]=struct.pack('<4Q',0,0,0,0)
        self.server=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);self.server.bind(self.sockpath);self.server.listen(256);self.server.setblocking(False)
        self.selector=selectors.DefaultSelector();self.selector.register(self.server,selectors.EVENT_READ)
        self.clients={};self.producers={};self.expected_commands=[0]*vehicles;self.last_activity=time.monotonic();self.armed=False;self.stop=False
        self.stop_reason='simulation-time limit';self.stop_request=None
        self.rows=(self.out/'protocol.jsonl').open('w',buffering=1);self.failure=None
        self.live=(self.out/'live-events.jsonl').open('w',buffering=1)
        self.engine.event_sink=self.live_event
        self.start_ready=set();self.start_acked=set();self.origin_tick=None;self.dispatch_started=False;self.barrier_host_start=None
        self.stepfile=(self.out/'clock-pairs.csv').open('w',newline='');self.stepwriter=csv.writer(self.stepfile);self.stepwriter.writerow(['tick','physics_ns','hardware_ns','real_callbacks_busy'])
        self.stats={'packets':0,'physics_steps':0,'outputs':0,'blocked_outputs':0,'lookup_misses':0,'phase_completion_before_callback_end':0}
        root=Path(__file__).resolve().parents[2]
        self.stepper=subprocess.Popen([str(root/'build/live-bridge/stepper')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(self.out/'stepper.log').open('w'),text=True,bufsize=1)
    def live_event(self,row):self.live.write(json.dumps(row)+'\n')
    def start_barrier(self):
        if not self.armed and self.engine.tick>=self.bootstrap:
            if self.start_ready!=set(self.engine.devices) or not (self.out/'nav2-ready').exists():return
            if (self.out/'views-required').exists() and not (self.out/'views-ready').exists():return
            self.armed=True;self.origin_tick=self.engine.tick;self.limit=self.origin_tick+round(self.seconds/self.engine.dt)
            self.clock[:]=struct.pack('<4Q',self.engine.tick*self.engine.step_ns,1,0,self.engine.tick)
            self.engine.event('common_start',origin_tick=self.origin_tick)
            (self.out/'armed').write_text(str(self.origin_tick*self.engine.step_ns)+'\n')
            self.barrier_host_start=time.monotonic()
        if self.armed and not self.dispatch_started:
            first={j.device for j in self.engine.jobs.values() if j.task=='bt_tick' and j.staged_tick is not None}
            if self.start_acked==set(self.engine.devices) and first==set(self.engine.devices):
                self.dispatch_started=True;self.engine.event('common_dispatch_begin')
            elif time.monotonic()-self.barrier_host_start>60:raise RuntimeError('Synchronized goal/first-BT barrier timed out')
    def reply(self,c,a=0):
        c.send(PACKET.pack(0,a,self.engine.tick*self.engine.step_ns,0,0,0,0))
    def log(self,op,c,p):
        self.rows.write(json.dumps({'tick':self.engine.tick,'op':op,'connection':c.fileno(),'args':list(p)})+'\n')
    def packet(self,c,p):
        op,a,b,d,e,f,g=p;s=self.clients[c];self.stats['packets']+=1;self.log(op,c,p)
        if op==ENTER:
            jid=self.engine.arrive(a,TASK_NAMES[b],(d,) if d else ());s.update(job=jid,busy=True,wait=None);self.reply(c,jid)
        elif op==DEP:self.engine.add_parent(a,b);self.reply(c)
        elif op==STAGE:
            j=self.engine.jobs[a]
            if self.engine.dual_budget and d!=1:raise RuntimeError('Unsealed computation cannot release a result')
            self.engine.event('actual_computation_sealed',job_id=a,actual_cpu_ns=b)
            s['busy']=False;s['wait']=('stage',a)
            if j.status!='completed':self.engine.stage(a,b if self.engine.dual_budget else None,dispatch=False)
            self.stats['blocked_outputs']+=1
        elif op==RESULT_GATE:
            if a not in self.engine.jobs:raise RuntimeError('Unknown action-result producer')
            s['busy']=False;s['wait']=('result',a)
        elif op in (TRIGGER,RESULT_OWNER):
            if not a or a not in self.engine.jobs:raise RuntimeError('Unknown triggering job')
            device=self.engine.jobs[a].device
            self.producers.setdefault((device,b),[]).append((a,0,0))
            self.engine.event('request_trigger' if op==TRIGGER else 'result_owner',job_id=a,signature=b,device_id=device);self.reply(c)
        elif op==END:
            j=self.engine.jobs[a]
            if j.status!='completed':raise RuntimeError('Actual callback ended before its modeled completion gate')
            if self.engine.dual_budget and b!=j.measured_cpu_ns:raise RuntimeError('Actual CPU measurement changed after budget selection')
            j.outputs_committed=True;j.real_cpu_ns=b;s.update(job=None,busy=False,wait=None);self.engine.event('actual_end',job_id=a,real_cpu_ns=b);self.reply(c)
        elif op==WAIT:s['busy']=False;self.reply(c)
        elif op==RESUME:s['busy']=bool(s['job']);self.reply(c)
        elif op==SLEEP:s['busy']=False;s['wait']=('sleep',a)
        elif op==PUBLISH_BEGIN:
            if a and self.engine.jobs[a].status!='completed':raise RuntimeError('Publication opened before selected completion')
            device=self.engine.jobs[a].device if a else g
            self.producers.setdefault((device,b),[]).append((a,d,None));self.reply(c)
        elif op==PUBLISH:
            if a and self.engine.jobs[a].status!='completed':raise RuntimeError('Output before model completion')
            device=self.engine.jobs[a].device if a else g
            values=self.producers.setdefault((device,b),[])
            pending=next((i for i,v in enumerate(values) if v==(a,d,None)),None)
            if pending is None:values.append((a,d,e))
            else:values[pending]=(a,d,e)
            if len(values)>128:del values[:-128]
            if f:self.expected_commands[device]=b
            if s['job'] is None:s['busy']=False
            self.stats['outputs']+=1;self.engine.event('publication',job_id=a,device_id=device,final_command=bool(f),signature=b,source_begin_ns=d,source_end_ns=e);self.reply(c)
        elif op==DEVICE_GATE:s['busy']=False;s['wait']=('device',a)
        elif op==START_READY:
            if a not in self.engine.devices:raise RuntimeError('Unknown mission device')
            self.start_ready.add(a);s['wait']=('start',a)
        elif op==START_ACK:
            if not self.armed or a not in self.start_ready:raise RuntimeError('Mission acknowledgement before common start')
            self.start_acked.add(a);self.engine.event('mission_accepted',device_id=a);self.reply(c)
        elif op==LOOKUP:
            values=self.producers.get((b,a),[])
            if d and any(v[2] is None and v[1]<=d for v in values):
                s['busy']=False;s['wait']=('lookup',(a,b,d));return
            parent=self.lookup_parent(a,b,d);self.record_binding(a,b,d,parent);self.reply(c,parent)
        else:raise RuntimeError('Unknown bridge opcode')
    def lookup_parent(self,key,device,source):
        values=self.producers.get((device,key),[])
        matches=[v[0] for v in values if source and v[2] is not None and v[1]<=source<=v[2]]
        return (matches[0] if len(matches)==1 else 0) if source else (values[-1][0] if values else 0)
    def record_binding(self,key,device,source,parent):
        self.stats['lookup_misses']+=not bool(parent)
        self.engine.event('input_binding',signature=key,device_id=device,parent_id=parent,source_ns=source,method='DDS_source_interval' if source else 'equivalent_payload_or_goal_UUID')
    def release(self):
        for c,s in list(self.clients.items()):
            if not s['wait']:continue
            kind,value=s['wait'];allow=False;reply_value=0
            if kind=='start':allow=self.armed
            elif kind=='sleep':allow=self.engine.tick*self.engine.step_ns>=value
            elif kind=='device':allow=not self.engine.cooling[value]
            elif kind=='lookup':
                key,device,source=value
                allow=not any(v[2] is None and v[1]<=source for v in self.producers.get((device,key),[]))
                if allow:reply_value=self.lookup_parent(key,device,source);self.record_binding(key,device,source,reply_value)
            elif kind=='result':allow=self.engine.jobs[value].outputs_committed and not self.engine.cooling[self.engine.jobs[value].device]
            else:
                j=self.engine.jobs[value]
                allow=(j.status=='completed' or self.engine.deliverable(value)) and not self.engine.cooling[j.device]
                if allow and j.status!='completed':
                    self.engine.finish(value,dispatch=False);self.stats['phase_completion_before_callback_end']+=1
            if allow:s['wait']=None;s['busy']=bool(s['job']) or kind in ('device','result');self.reply(c,reply_value)
    def step(self):
        self.stepper.stdin.write('step '+' '.join(map(str,self.expected_commands))+'\n');self.stepper.stdin.flush()
        line=self.stepper.stdout.readline()
        if not line:raise RuntimeError('Gazebo stepper exited: '+str(self.stepper.poll()))
        actual=int(line);expected=(self.engine.tick+1)*self.engine.step_ns
        if actual!=expected:raise RuntimeError(f'Physics/model clock mismatch: {actual} != {expected}')
        if self.armed:self.engine.advance()
        else:self.engine.tick+=1
        self.stats['physics_steps']+=1
        self.stepwriter.writerow([self.engine.tick,actual,self.engine.tick*self.engine.step_ns,0])
        self.clock[:]=struct.pack('<4Q',actual,int(self.armed),0,self.engine.tick)
        if self.engine.tick%max(1,round(.01/self.engine.dt))==0:self.live_event({'kind':'clock','tick':self.engine.tick})
        self.release()
    def run(self):
        start=time.monotonic();last_progress=start;last_report=-1
        try:
            while (not self.armed or self.engine.tick<self.limit or any(s['busy'] for s in self.clients.values()) or time.monotonic()-self.last_activity<self.config['settle_quiet_host_s']) and not self.stop:
                if (self.out/'stop-requested.json').exists():
                    self.stop_request=json.loads((self.out/'stop-requested.json').read_text());self.stop_reason=self.stop_request['reason'];break
                events=self.selector.select(.0005)
                for key,_ in events:
                    if key.fileobj is self.server:
                        c,_=self.server.accept();c.setblocking(True);self.clients[c]={'job':None,'busy':False,'wait':None};self.selector.register(c,selectors.EVENT_READ)
                    else:
                        c=key.fileobj;raw=c.recv(PACKET.size)
                        if not raw:
                            s=self.clients.pop(c);self.selector.unregister(c);c.close()
                            if s['job'] and not self.stop:raise RuntimeError('Live callback disconnected without completion')
                        else:
                            if len(raw)!=PACKET.size:raise RuntimeError('Truncated live packet')
                            self.packet(c,PACKET.unpack(raw))
                    self.last_activity=time.monotonic()
                self.start_barrier();self.release()
                busy=any(s['busy'] for s in self.clients.values())
                now=time.monotonic()
                can_step=(self.dispatch_started and self.engine.tick<self.limit) if self.armed else True
                if can_step and not busy and now-self.last_activity>=self.config['settle_quiet_host_s']:
                    self.engine.dispatch();self.release()
                    if any(s['busy'] for s in self.clients.values()):continue
                    self.step();last_progress=time.monotonic()
                elif busy and now-last_progress>self.config['host_computation_timeout_s']:raise RuntimeError('Real computation failed to quiesce within 30 host seconds')
                if self.engine.tick%round(1/self.engine.dt)==0 and self.engine.tick!=last_report:
                    last_report=self.engine.tick
                    (self.out/'progress.json').write_text(json.dumps({'sim_s':self.engine.tick*self.engine.dt,'jobs':len(self.engine.jobs),'host_elapsed_s':now-start})+'\n')
        except BaseException as ex:
            self.failure=f'{type(ex).__name__}: {ex}';traceback.print_exc();raise
        finally:
            self.clock[16:24]=struct.pack('<Q',1)
            for c in self.clients:
                try:c.shutdown(socket.SHUT_RDWR)
                except OSError:pass
                c.close()
            try:self.stepper.stdin.write('quit\n');self.stepper.stdin.flush()
            except (BrokenPipeError,OSError):pass
            try:self.stepper.wait(timeout=5)
            except subprocess.TimeoutExpired:self.stepper.kill();self.stepper.wait()
            self.live_event({'kind':'trial_end','tick':self.engine.tick});self.live.close();self.rows.close();self.stepfile.close();self.save(time.monotonic()-start)
            self.clock.close();self.clockfile.close();self.server.close()
    def save(self,host_s):
        jobs=[]
        for j in self.engine.jobs.values():
            row=asdict(j);row['parents']=sorted(j.parents);row['assumed_wcet_s']=j.selected_budget_s
            jobs.append(row)
        report={'schema':2 if self.engine.dual_budget else 1,'task_parameters':{name:asdict(spec) for name,spec in self.engine.tasks.items()},'step_s':self.engine.dt,'clock':'SIM','policy':'ready FIFO / oldest idle core / local only','job_source':'actual callback entry','completion':'sealed actual computation then selected LO/HI budget then buffered output release' if self.engine.dual_budget else 'historical Q95 phase gate','origin_tick':self.origin_tick,'race_seconds':(self.engine.tick-self.origin_tick)*self.engine.dt if self.origin_tick is not None else 0.,'sim_seconds':self.engine.tick*self.engine.dt,'host_seconds':host_s,'failure':self.failure,'stop_reason':self.stop_reason,'stop_request':self.stop_request,'hardware_configuration':self.engine.platform_config,'stats':self.stats,'jobs':jobs,'events':self.engine.events,'thermal':self.engine.samples,'config':self.config,'hardware':{d:[{'core_id':c,'type':v.core_type.name,'frequency_mhz':v.dvfs_level().frequency_mhz,'voltage_v':v.dvfs_level().voltage_v,'performance_eta':v.core_type.performance_eta} for c,v in p.cores.items()] for d,p in self.engine.devices.items()}}
        (self.out/'trace.json').write_text(json.dumps(report,indent=2)+'\n')
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=30);p.add_argument('--vehicles',type=int);p.add_argument('--bootstrap',type=float);a=p.parse_args()
    Broker(a.output,a.seconds,a.vehicles,a.bootstrap).run()
if __name__=='__main__':main()
