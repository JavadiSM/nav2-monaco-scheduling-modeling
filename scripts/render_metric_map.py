#!/usr/bin/env python3
"""Draw the existing circuit, checkpoints and parked endpoints in metres."""
from pathlib import Path
import csv
import json
import numpy as np
from PIL import Image
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

ROOT = Path(__file__).resolve().parents[1]


def main():
    scene = json.loads((ROOT / 'scenarios/monaco/scenario.json').read_text())
    metadata = yaml.safe_load((ROOT / 'scenarios/monaco/map.yaml').read_text())
    pixels = np.asarray(Image.open(ROOT / 'scenarios/monaco/map.pgm'))
    h, w = pixels.shape
    res = metadata['resolution']
    x0, y0, _ = metadata['origin']
    extent = [x0, x0 + w * res, y0, y0 + h * res]
    route = np.array(scene['centreline'])
    output = ROOT / 'docs/figures/metric-map'
    output.mkdir(parents=True, exist_ok=True)
    report = {
        'units': 'metres', 'frame': 'ROS map', 'map_resolution_m_per_pixel': res,
        'map_pixels': [w, h], 'map_extent_m': dict(zip(['xmin','xmax','ymin','ymax'],extent)),
        'map_width_m': w * res, 'map_height_m': h * res,
        'centreline_bounds_m': {'xmin': float(route[:,0].min()), 'xmax': float(route[:,0].max()),
                                'ymin': float(route[:,1].min()), 'ymax': float(route[:,1].max())},
        'centreline_span_x_m': float(np.ptp(route[:,0])),
        'centreline_span_y_m': float(np.ptp(route[:,1])),
        'road_width_m': scene['road_width_m'],
        'centreline_length_m': scene['centreline_length_m'],
        'shortest_drivable_path_m': scene['shortest_drivable_path_m'],
        'start_finish_direct_distance_m': scene['start_finish_direct_distance_m'],
        'start': scene['start'], 'finish': scene['finish'],
        'parked_endpoint_count': len(scene['servers']), 'coverage_radius_assigned': True, 'coverage_radius_m': 5.0,
    }
    evidence = ROOT / 'docs/evidence'
    (evidence / 'metric-map-dimensions.json').write_text(json.dumps(report,indent=2)+'\n')
    with (evidence / 'metric-map-locations.csv').open('w',newline='') as f:
        writer = csv.writer(f,lineterminator='\n');writer.writerow(['kind','id','x_m','y_m','yaw_rad'])
        for k in ('start','finish'):
            p=scene[k];writer.writerow([k,k,p['x'],p['y'],p['yaw']])
        for k, rows in (('checkpoint',scene['checkpoints']),('parked_endpoint',scene['servers'])):
            for p in rows:writer.writerow([k,p['id'],p['x'],p['y'],p['yaw']])
    fig,ax=plt.subplots(figsize=(16,9))
    # Occupancy pixels are already the navigation map; no geometry is regenerated.
    ax.imshow(pixels,cmap='gray',vmin=0,vmax=255,extent=extent,origin='upper',alpha=.62,zorder=0)
    ax.plot(route[:,0],route[:,1],color='#586772',lw=.8,ls='--',label='Route centreline')
    for cp in scene['checkpoints']:
        ax.scatter(cp['x'],cp['y'],s=24,c='#cf9400',zorder=5)
        ax.annotate(f"CP{cp['id']:02}",(cp['x'],cp['y']),xytext=(5,-11),textcoords='offset points',fontsize=7,color='#775200')
    for endpoint in scene['servers']:
        ax.scatter(endpoint['x'],endpoint['y'],s=38,marker='s',c='#0089bc',edgecolors='white',linewidths=.6,zorder=6)
        ax.annotate(endpoint['id'],(endpoint['x'],endpoint['y']),xytext=(7,7),textcoords='offset points',fontsize=11,color='#005077',weight='bold',bbox={'facecolor':'white','edgecolor':'none','alpha':.85,'pad':1.5},zorder=8)
    for name,color in [('start','#009956'),('finish','#d43241')]:
        p=scene[name];ax.scatter(p['x'],p['y'],s=65,c=color,zorder=7)
        ax.annotate(name.upper(),(p['x'],p['y']),xytext=(-60,8),textcoords='offset points',fontsize=9,color=color,weight='bold')
    xmin,xmax,ymin,ymax=extent
    ydim=ymin-1.3
    ax.annotate('',xy=(xmin,ydim),xytext=(xmax,ydim),arrowprops={'arrowstyle':'<->','lw':1})
    ax.text((xmin+xmax)/2,ydim-.25,f'Map width = {w*res:.2f} m',ha='center',va='top',fontsize=10)
    xdim=xmax+1.2
    ax.annotate('',xy=(xdim,ymin),xytext=(xdim,ymax),arrowprops={'arrowstyle':'<->','lw':1})
    ax.text(xdim+.3,(ymin+ymax)/2,f'Map height = {h*res:.2f} m',ha='left',va='center',rotation=90,fontsize=10)
    bx,by=xmin+.8,ymin+.8
    ax.plot([bx,bx+5],[by,by],color='black',lw=4)
    ax.text(bx+2.5,by+.45,'5 m',ha='center',fontsize=9)
    ax.text(xmax-.5,ymax-.3,f"Road width: {scene['road_width_m']:.2f} m\nRoute length: {scene['centreline_length_m']:.2f} m\n{len(scene['servers'])} roadside endpoints; coverage radius 5 m",ha='right',va='top',fontsize=9,bbox={'facecolor':'white','alpha':.9,'edgecolor':'#a0a0a0'})
    ax.set_xlim(xmin-1,xmax+3);ax.set_ylim(ymin-2.5,ymax+1)
    ax.set_aspect('equal');ax.set_xlabel('x (m) — ROS map frame');ax.set_ylabel('y (m) — ROS map frame')
    ax.xaxis.set_major_locator(MultipleLocator(5));ax.yaxis.set_major_locator(MultipleLocator(5))
    ax.xaxis.set_minor_locator(MultipleLocator(1));ax.yaxis.set_minor_locator(MultipleLocator(1))
    ax.grid(which='major',alpha=.3,lw=.7);ax.grid(which='minor',alpha=.12,lw=.4)
    ax.set_title('Existing Monaco circuit — metric map and parked endpoint coordinates',fontsize=15,pad=15)
    fig.tight_layout();fig.savefig(output/'circuit-dimensions.png',dpi=180);fig.savefig(output/'circuit-dimensions.svg');plt.close(fig)
    svg=output/'circuit-dimensions.svg';svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
