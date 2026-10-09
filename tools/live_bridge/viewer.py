"""A read-only live Gantt driven by the broker's actual scheduling events."""
import argparse,json,math,time
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont,ImageTk
import tkinter as tk
from .engine import TASK_NAMES

TITLE='Live FIFO - Single vehicle'
COLORS=['#2c78b8','#9fc6e6','#f28b22','#ffbf79','#379d36','#97d881','#d73536','#f19094','#8b63b4','#c6a9dc','#8b614f']
LABELS=['Control','Local map','Global map','Smoother','BT','Planner','AMCL','Noise','Command input','Collision','Path install']
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
class LiveGantt:
    def __init__(self,config=None):
        cfg=config or {};self.dt=cfg.get('step_ns',1_000_000)/1e9;self.vehicles=cfg.get('vehicle_count',1);self.window_s=cfg.get('gantt_window_s',2.);self.color=cfg.get('vehicle_color','red')
        self.thermal=cfg.get('thermal',{});self.temperatures={d:[self.thermal.get('initial_temperature_c',self.thermal.get('ambient_temperature_c',45.))]*2 for d in range(self.vehicles)}
        self.jobs={};self.segments=[];self.active={};self.cooling={};self.cooling_spans=[];self.tick=0;self.origin=None;self.ended=False
    def consume(self,e):
        self.tick=e['tick'];kind=e['kind'];jid=e.get('job_id')
        if kind=='thermal':self.temperatures[e['device_id']]=e['temperature_c']
        elif kind=='common_start':self.origin=e['origin_tick']
        elif kind=='arrival':self.jobs[jid]=dict(task=e['task'],device=e['device_id'])
        elif kind=='start':
            self.jobs[jid]['core']=e['core_id'];self.active[jid]=self.tick
        elif kind=='budget_complete':self.close(jid);self.jobs[jid]['budget_done']=True
        elif kind=='cooling_start':
            d=e['device_id'];self.cooling[d]=self.tick
            for j in list(self.active):
                if self.jobs[j]['device']==d:self.close(j)
        elif kind=='cooling_end':
            d=e['device_id'];a=self.cooling.pop(d,None)
            if a is not None:self.cooling_spans.append((d,a,self.tick))
            for j,row in self.jobs.items():
                if row.get('core') is not None and not row.get('budget_done') and not row.get('finished') and row['device']==d:self.active[j]=self.tick
        elif kind=='finish':self.jobs[jid]['finished']=True
        elif kind=='trial_end':self.ended=True
    def close(self,jid):
        a=self.active.pop(jid,None)
        if a is not None:self.segments.append((jid,a,self.tick))
        # A cooling boundary closes only the current segment, not its remaining budget.
    def image(self,window=None):
        im=Image.new('RGB',(1280,720),'#f8fafc');g=ImageDraw.Draw(im)
        font=lambda n:ImageFont.truetype(FONT,n)
        elapsed=0. if self.origin is None else max(0.,(self.tick-self.origin)*self.dt)
        if window is None:window=self.window_s*math.floor(max(0.,elapsed-(1e-9 if self.ended else 0.))/self.window_s)
        g.text((28,18),'LIVE LOCAL FIFO  |  SINGLE VEHICLE  |  A7 + A15',fill='#172b43',font=font(27))
        g.text((28,59),f'Window {window:.0f}–{window+self.window_s:.0f} s   |   Actual arrivals · Q95 budgets · {self.dt:.3f} s steps',fill='#506078',font=font(18))
        g.text((28,84),f'Modeled core temperatures  |  Ambient {self.thermal.get("ambient_temperature_c",45.):g} °C  |  Initial {self.thermal.get("initial_temperature_c",self.thermal.get("ambient_temperature_c",45.)):g} °C  |  Tmax {self.thermal.get("max_temperature_c",46.2):g} °C',fill='#506078',font=font(14))
        lanes=2*self.vehicles;x0,x1,y0,h=155,1248,115,376/(2*self.vehicles)
        base=0 if self.origin is None else self.origin
        x=lambda tick:x0+((tick-base)*self.dt-window)/self.window_s*(x1-x0)
        for lane in range(lanes):
            y=y0+lane*h;g.rectangle((x0,y+2,x1,y+h-2),fill='#ffffff')
            car=self.color.title() if self.vehicles==1 else ('Red','Blue','White','Green')[lane//2]
            g.text((20,y+h/2-12),car+' / '+('A7' if lane%2==0 else 'A15'),fill=('#b52b32','#1a65b4','#606b76','#24874a')[lane//2],font=font(23))
            temperature=self.temperatures[lane//2][lane%2]
            g.text((20,y+h/2+18),f'{temperature:.2f} °C',fill='#33445a',font=font(20))
        for n in range(11):
            px=x0+n*(x1-x0)/10;g.line((px,y0-10,px,y0+lanes*h),fill='#dfe5ec');g.text((px-12,y0+lanes*h+10),f'{window+n*self.window_s/10:.1f}',fill='#566277',font=font(16))
        def span(a,b,lane,color,height=1):
            left=max(x0,x(a));right=min(x1,x(b))
            if right<=x0 or left>=x1 or b<a:return
            right=max(left+1,right);y=y0+lane*h
            margin=5 if height==2 else h*.28
            g.rectangle((left,y+margin,right,y+height*h-margin),fill=color)
        for d,a,b in self.cooling_spans+[(d,a,self.tick) for d,a in self.cooling.items()]:span(a,b,2*d,'#f7dede',2)
        for jid,a,b in self.segments+[(j,a,self.tick) for j,a in self.active.items()]:
            row=self.jobs[jid];span(a,b,2*row['device']+row['core'],COLORS[TASK_NAMES.index(row['task'])])
        for i,(name,c) in enumerate(zip(LABELS,COLORS)):
            col,row=i%4,i//4;px=25+col*310;py=552+row*28;g.rectangle((px,py+3,px+17,py+20),fill=c);g.text((px+25,py),name,fill='#33445a',font=font(17))
        g.rectangle((955,611,972,628),fill='#f7dede');g.text((980,608),'Device cooling',fill='#33445a',font=font(17))
        state='Preparing / waiting for common start' if self.origin is None else ('Trial complete' if self.ended else 'Running on common simulation clock')
        g.text((28,656),state,fill='#506078',font=font(18));g.text((806,650),f't = {elapsed:8.3f} s',fill='#102c4c',font=font(32))
        return im

def main():
    p=argparse.ArgumentParser();p.add_argument('--trial',type=Path,required=True);a=p.parse_args();a.trial.mkdir(parents=True,exist_ok=True)
    root=tk.Tk();root.title(TITLE);root.geometry('1280x720');root.resizable(False,False);label=tk.Label(root,borderwidth=0);label.pack();cfg=json.loads((a.trial/'trial-config.json').read_text()) if (a.trial/'trial-config.json').exists() else {};thermal_file=a.trial/'hardware-config.json';cfg['thermal']=json.loads(thermal_file.read_text())['thermal'] if thermal_file.exists() else {};model=LiveGantt(cfg);stream=[None];last=[None];last_window=[0]
    def draw():
        if stream[0] is None:
            try:stream[0]=(a.trial/'live-events.jsonl').open()
            except FileNotFoundError:pass
        if stream[0]:
            while True:
                pos=stream[0].tell();line=stream[0].readline()
                if not line:break
                if not line.endswith('\n'):stream[0].seek(pos);break
                e=json.loads(line)
                model.consume(e)
        key=(model.tick,model.origin,model.ended)
        if key!=last[0]:
            window=0 if model.origin is None else model.window_s*math.floor(max(0.,(model.tick-model.origin)*model.dt-(1e-9 if model.ended else 0.))/model.window_s)
            if window!=last_window[0] and not model.ended:
                previous=model.image(last_window[0]);previous.save(a.trial/'gantt-previous-window.png')
                if last_window[0]==0:previous.save(a.trial/'gantt-first-window.png')
            im=model.image();photo=ImageTk.PhotoImage(im);label.configure(image=photo);label.image=photo;last[0]=key;last_window[0]=window
            if model.ended:im.save(a.trial/'live-gantt.png')
        root.after(100,draw)
    root.after(0,draw);root.after(300,lambda:(a.trial/'viewer-ready').touch());root.mainloop()
if __name__=='__main__':main()
