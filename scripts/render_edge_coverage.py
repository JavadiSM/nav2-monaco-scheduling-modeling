#!/usr/bin/env python3
"""Draw the chosen 5 m coverage on the frozen scene, without editing its map."""
import json
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tools.abstract_compute.communication import reachable_endpoints

def main():
    scene=json.loads((ROOT/'scenarios/monaco/scenario.json').read_text())
    servers=scene['servers'] if 'servers' in scene else scene['edge_servers']
    route=scene['centreline']
    fig,ax=plt.subplots(figsize=(13,6));fig.patch.set_facecolor('#f6f8fc')
    for e in servers:
        ax.add_patch(Circle((e['x'],e['y']),5,facecolor='#09a5ae',edgecolor='#007f88',alpha=.10,lw=.8))
        ax.plot(e['x'],e['y'],'s',color='#007f88',ms=4)
        ax.annotate(e['id'].replace('edge_','E'),(e['x'],e['y']),xytext=(3,4),textcoords='offset points',fontsize=8)
    ax.plot([p[0] for p in route],[p[1] for p in route],color='#223449',lw=3)
    for name,c in [('start','#18a674'),('finish','#d94760')]:
        p=scene[name];ax.plot(p['x'],p['y'],'o',color=c,ms=8);ax.annotate(name.upper(),(p['x'],p['y']),xytext=(-45,-20),textcoords='offset points',fontsize=9)
    ax.set_aspect('equal');ax.grid(alpha=.15);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
    ax.set_title('Frozen circuit | 19 edge locations | 5 m ideal coverage | communication cost = 0 s',loc='left',fontweight='bold')
    fig.tight_layout();out=ROOT/'docs/figures/metric-map';fig.savefig(out/'edge-coverage-5m.png',dpi=170);fig.savefig(out/'edge-coverage-5m.svg')
    counts=[len(reachable_endpoints(p,servers)) for p in route]
    (ROOT/'docs/evidence/edge-coverage.json').write_text(json.dumps(dict(radius_m=5,communication_cost_s=0,active_offloading=False,
       evaluated_centreline_points=len(route),uncovered_centreline_points=counts.count(0),minimum_visible_servers=min(counts),maximum_visible_servers=max(counts)),indent=2)+'\n')
if __name__=='__main__':main()
