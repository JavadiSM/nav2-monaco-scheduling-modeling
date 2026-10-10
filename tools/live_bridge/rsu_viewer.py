"""Render actual modeled schedules of the four geographically nearest RSUs."""
import json,math
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
from .viewer import LiveGantt,FONT,COLORS,LABELS
from .engine import TASK_NAMES
POLICY_COLORS={'local':'#c62828','random':'#1565c0','offload':'#238b45','greedy':'#e6a000'}
class NearestRSUGantt(LiveGantt):
    def __init__(self,config=None):
        super().__init__(config)
        scene=json.loads((Path(__file__).resolve().parents[2]/'scenarios/monaco/scenario.json').read_text())
        self.endpoints={self.vehicles+i:(e['x'],e['y']) for i,e in enumerate(scene['servers'])};self.position=(scene['start']['x'],scene['start']['y']);self.frequency={}
    def consume(self,event):
        super().consume(event)
        if event.get('position_xy') is not None:self.position=event['position_xy']
        if event['kind']=='dvfs_change':self.frequency[event['device_id']]=event['frequency_mhz']
    def image(self,window=None):
        image=Image.new('RGB',(1280,720),'#f4f7fb');g=ImageDraw.Draw(image);font=lambda n:ImageFont.truetype(FONT,n)
        elapsed=0. if self.origin is None else max(0.,(self.tick-self.origin)*self.dt)
        if window is None:window=self.window_s*math.floor(max(0.,elapsed-(1e-9 if self.ended else 0.))/self.window_s)
        color=POLICY_COLORS[self.placement];g.rectangle((0,0,1280,7),fill=color)
        g.text((22,15),f'{self.placement.upper()} | FOUR NEAREST RSUs',font=font(25),fill=color)
        g.text((850,17),f't = {elapsed:8.3f} s | SIM',font=font(24),fill='#172b43')
        base=self.origin or 0;nearest=sorted(self.endpoints,key=lambda d:(math.dist(self.position,self.endpoints[d]),d))[:4]
        for rank,d in enumerate(nearest):
            left=20+(rank%2)*635;top=65+(rank//2)*280;right=left+605;bottom=top+255
            g.rounded_rectangle((left,top,right,bottom),radius=12,fill='white',outline='#d3dce8',width=2)
            distance=math.dist(self.position,self.endpoints[d]);covered=distance<=5.
            g.text((left+15,top+12),f'{rank+1}. {self.rsu_names[d]}',font=font(22),fill='#153954')
            g.text((left+320,top+15),f'{distance:.2f} m | '+('IN RANGE' if covered else 'OUT OF RANGE'),font=font(16),fill='#238b45' if covered else '#6a7788')
            temp=self.temperatures.get(d,[45.])[0];state='COOLING' if d in self.cooling else 'AVAILABLE'
            frequency='No execution yet' if d not in self.frequency else f'Last f: {self.frequency[d]:g} MHz'
            g.text((left+15,top+49),f'A15 core 0 | {temp:.3f} °C | {frequency} | {state}',font=font(17),fill='#a62222' if d in self.cooling else '#34465c')
            x0=left+30;x1=right-25;y0=top+100;y1=top+190
            def x(t):return x0+((t-base)*self.dt-window)/self.window_s*(x1-x0)
            g.rectangle((x0,y0,x1,y1),fill='#f5f8fb')
            for a,b in [(a,b) for dev,a,b in self.cooling_spans if dev==d]+([(self.cooling[d],self.tick)] if d in self.cooling else []):
                if x(b)>x0 and x(a)<x1:g.rectangle((max(x0,x(a)),y0,min(x1,x(b)),y1),fill='#f7dede')
            for jid,a,b in self.segments+[(j,a,self.tick) for j,a in self.active.items()]:
                row=self.jobs[jid]
                if row['device']!=d or x(b)<=x0 or x(a)>=x1:continue
                start=max(x0,x(a));stop=min(x1,max(start+1,x(b)))
                g.rectangle((start,y0+25,stop,y1-25),fill=COLORS[TASK_NAMES.index(row['task'])])
            for n in range(5):
                px=x0+n*(x1-x0)/4;g.line((px,y0,px,y1),fill='#d8e0eb');g.text((px-12,y1+9),f'{window+n*self.window_s/4:.1f}',font=font(15),fill='#52647a')
            g.text((left+250,top+225),'Simulation time (s)',font=font(14),fill='#52647a')
        for i,(label,color) in enumerate(zip(LABELS,COLORS)):
            px=20+(i%6)*210;py=642+(i//6)*28;g.rectangle((px,py+3,px+13,py+16),fill=color);g.text((px+19,py),label,font=font(14),fill='#34465c')
        g.text((22,699),'Nearest is geometric; coverage is checked at task send. Bars replay actual broker scheduling events.',font=font(12),fill='#52647a')
        return image
