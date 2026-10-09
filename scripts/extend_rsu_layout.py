#!/usr/bin/env python3
"""Add roadside endpoints while preserving the accepted route and existing models."""
from copy import deepcopy
import hashlib,json,math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[1]


def canonical_without_endpoints(root):
    root=deepcopy(root);world=root.find('world')
    for model in list(world.findall('model')):
        if model.get('name','').startswith('edge_'):world.remove(model)
    return ET.canonicalize(ET.tostring(root,encoding='unicode'),strip_text=True)


def main():
    scene_path=ROOT/'scenarios/monaco/scenario.json';world_path=ROOT/'scenarios/monaco/world.sdf'
    scene=json.loads(scene_path.read_text());before_scene=deepcopy(scene)
    root=ET.parse(world_path).getroot();world=root.find('world');before_world=canonical_without_endpoints(root)
    models={m.get('name'):m for m in world.findall('model') if m.get('name','').startswith('edge_')}
    original_models={n:ET.canonicalize(ET.tostring(m,encoding='unicode'),strip_text=True) for n,m in models.items()}
    original={e['id']:deepcopy(e) for e in scene['servers']}
    route=np.asarray(scene['centreline']);dist=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(route,axis=0),axis=1))]
    checkpoints={c['id']:c for c in scene['checkpoints']}
    endpoints=deepcopy(scene['servers']);new=[]
    for e in endpoints:
        if 'route_s_m' not in e:e['route_s_m']=checkpoints[e['checkpoint_id']]['s'];e['placement_role']='checkpoint'

    def pose_at(s):
        i=min(len(route)-2,max(0,int(np.searchsorted(dist,s,side='right')-1)))
        fraction=(s-dist[i])/(dist[i+1]-dist[i]);p=route[i]+fraction*(route[i+1]-route[i]);delta=route[i+1]-route[i]
        return p,math.atan2(delta[1],delta[0])

    def place(name,s,role,left=None,right=None):
        p,yaw=pose_at(s);normal=np.array([-math.sin(yaw),math.cos(yaw)]);choices=[]
        for offset in (1.3,-1.3,1.6,-1.6,1.9,-1.9,2.2,-2.2,2.5,-2.5,3.,-3.):
            q=p+offset*normal;clear=float(np.linalg.norm(route-q,axis=1).min())
            if clear<scene['road_width_m']/2+.48:continue
            if any(math.dist(q,[e['x'],e['y']])<1. for e in endpoints):continue
            score=abs(offset)
            if left is not None:score+=max(math.dist(q,[e['x'],e['y']]) for e in (left,right))
            choices.append((score,q,offset,clear))
        if not choices:raise RuntimeError('No clear roadside position at route station '+str(s))
        _,q,offset,clear=min(choices,key=lambda c:c[0])
        e=dict(id=name,checkpoint_id=None,x=float(q[0]),y=float(q[1]),yaw=yaw,static=True,worker_enabled=False,route_s_m=float(s),placement_role=role,roadside_offset_m=float(offset),minimum_sampled_centreline_clearance_m=clear)
        endpoints.append(e);new.append(e);return e

    for name in ('start','finish'):
        if not any(e.get('placement_role')==name for e in endpoints):place('edge_'+name,scene[name]['s'],name)
    count=1
    for _ in range(60):
        ordered=sorted(endpoints,key=lambda e:e['route_s_m'])
        gaps=[(a,b) for a,b in zip(ordered,ordered[1:]) if math.dist([a['x'],a['y']],[b['x'],b['y']])>10.+1e-9]
        if not gaps:break
        for a,b in gaps:
            while any(e['id']==f'edge_fill_{count:02d}' for e in endpoints):count+=1
            place(f'edge_fill_{count:02d}',(a['route_s_m']+b['route_s_m'])/2.,'infill',a,b);count+=1
    else:raise RuntimeError('RSU gap refinement did not converge')
    endpoints.sort(key=lambda e:e['route_s_m']);scene['servers']=endpoints
    for e in new:
        model=deepcopy(models['edge_01']);model.set('name',e['id']);model.find('pose').text=f"{e['x']:.12g} {e['y']:.12g} 0 0 0 {e['yaw']:.12g}";world.append(model)
    assert canonical_without_endpoints(root)==before_world,'A non-RSU world element changed'
    for n,value in original_models.items():assert ET.canonicalize(ET.tostring(models[n],encoding='unicode'),strip_text=True)==value
    for e in endpoints:
        if e['id'] in original:
            for key,value in original[e['id']].items():assert e[key]==value,'Existing endpoint changed'
    check=deepcopy(scene);check['servers']=before_scene['servers'];assert check==before_scene,'Route or navigation scenario changed'
    scene_path.write_text(json.dumps(scene,indent=2)+'\n');ET.indent(root);ET.ElementTree(root).write(world_path,encoding='unicode',xml_declaration=True)
    manifest_path=ROOT/'docs/evidence/frozen-scene-manifest.json';manifest=json.loads(manifest_path.read_text())
    manifest['policy']='Freeze route, occupancy map, vehicle, navigation and physics. Explicitly authorized RSU additions update only server layout and world models.'
    for p in (scene_path,world_path):manifest['files'][str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    distances=[dict(from_id=a['id'],to_id=b['id'],distance_m=math.dist([a['x'],a['y']],[b['x'],b['y']])) for a,b in zip(endpoints,endpoints[1:])]
    evidence=dict(server_count=len(endpoints),new_server_count=len(new),new_endpoint_ids=[e['id'] for e in new],maximum_consecutive_distance_m=max(g['distance_m'] for g in distances),distance_definition='Euclidean XY distance, consecutive in route-station order; open start-to-finish route',start_finish_roadside=True,route_unchanged=True,existing_endpoint_poses_unchanged=True,non_RSU_world_unchanged=True,gaps=distances)
    evidence_path=ROOT/'docs/evidence/rsu-layout.json'
    if new or not evidence_path.exists():evidence_path.write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps({k:v for k,v in evidence.items() if k!='gaps'},indent=2))
if __name__=='__main__':main()
