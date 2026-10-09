"""Derive a locally widened starting apron, preserving the rest of the circuit."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2
import numpy as np
from PIL import Image
import yaml


def pose_at(scene, at):
    remaining = at
    for a, b in zip(scene['centreline'], scene['centreline'][1:]):
        length = math.dist(a, b)
        if length and remaining <= length:
            fraction = remaining / length
            return dict(x=a[0]+fraction*(b[0]-a[0]), y=a[1]+fraction*(b[1]-a[1]),
                        yaw=math.atan2(b[1]-a[1], b[0]-a[0]), s=at)
        remaining -= length
    raise ValueError('Pose lies beyond route')


def grid_pose(scene, color, config):
    p = pose_at(scene, config['row_s_m'])
    lateral = config['lateral_offsets_m'][color]
    p['x'] -= lateral*math.sin(p['yaw'])
    p['y'] += lateral*math.cos(p['yaw'])
    return p


def cross(a, b):
    return a[0]*b[1]-a[1]*b[0]


def outside_parts(a, b, polygon):
    """Clip a barrier centreline against the apron; retain its exterior pieces."""
    delta = b-a
    cuts = [0.,1.]
    for p, q in zip(polygon, np.roll(polygon,-1,axis=0)):
        edge = q-p
        denominator = cross(delta,edge)
        if abs(denominator) < 1e-10:continue
        t, u = cross(p-a,edge)/denominator, cross(p-a,delta)/denominator
        if 0 <= t <= 1 and 0 <= u <= 1:cuts.append(t)
    cuts = sorted(set(cuts))
    contour = polygon.astype(np.float32)
    return [(a+u*delta,a+v*delta) for u,v in zip(cuts,cuts[1:])
            if v-u>1e-8 and cv2.pointPolygonTest(contour,tuple(a+(u+v)*.5*delta),False)<0]


def prepare_start_grid(project, scene, out, config):
    from scripts.generate_monaco import box, static_model
    project, out = Path(project), Path(out)
    source = project/'scenarios/monaco'
    end = config['merge_at_s_m']
    samples = np.r_[[-config['rear_extension_m']],np.linspace(0,end,31)]
    left, right, centres, widths = [], [], [], []
    for at in samples:
        p = pose_at(scene,float(at))
        fraction = np.clip((at-config['wide_until_s_m'])/(end-config['wide_until_s_m']),0,1)
        width = config['width_m']*(1-fraction)+scene['road_width_m']*fraction
        point = np.array([p['x'],p['y']]); normal=np.array([-math.sin(p['yaw']),math.cos(p['yaw'])])
        left.append(point+normal*width/2); right.append(point-normal*width/2)
        centres.append(point); widths.append(width)
    left,right=np.array(left),np.array(right)
    polygon=np.vstack([left,right[::-1]])
    tree=ET.parse(source/'world.sdf'); world=tree.getroot().find('world')
    link=world.find("model[@name='continuous_track_barriers']/link")
    removed,split=0,0
    for visual in list(link.findall('visual')):
        pose=np.array(list(map(float,visual.findtext('pose').split())))
        size=np.array(list(map(float,visual.findtext('geometry/box/size').split())))
        tangent=np.array([math.cos(pose[5]),math.sin(pose[5])])
        a,b=pose[:2]-tangent*size[0]/2,pose[:2]+tangent*size[0]/2
        pieces=outside_parts(a,b,polygon)
        if len(pieces)==1 and np.allclose(pieces[0][0],a) and np.allclose(pieces[0][1],b):continue
        collision=link.find("collision[@name='"+visual.get('name')+"_collision']")
        link.remove(visual)
        if collision is not None:link.remove(collision)
        removed+=1
        for i,(p,q) in enumerate(pieces):
            length=float(np.linalg.norm(q-p))
            if length<.015:continue
            midpoint=(p+q)/2
            for item in (visual,collision):
                if item is None:continue
                part=deepcopy(item)
                part.set('name',item.get('name')+'_retained_'+str(i))
                part.find('pose').text=' '.join(map(str,[*midpoint,*pose[2:]]))
                part.find('geometry/box/size').text=' '.join(map(str,[length,*size[1:]]))
                link.append(part)
            split+=1
    apron=static_model(world,'fleet_start_apron')
    for i,(a,b) in enumerate(zip(centres,centres[1:])):
        midpoint=(a+b)/2; delta=b-a
        box(apron,'asphalt_'+str(i),[*midpoint,.0045,0,0,math.atan2(delta[1],delta[0])],
            [float(np.linalg.norm(delta))+.035,max(widths[i],widths[i+1]),.008],'0.12 0.14 0.17 1')
    boundary=[*zip(left,left[1:]),*zip(right,right[1:]),(left[0],right[0])]
    for i,(a,b) in enumerate(boundary):
        delta=b-a; midpoint=(a+b)/2
        color='0.92 0.93 0.94 1' if i%2 else '0.80 0.12 0.16 1'
        box(apron,'barrier_'+str(i),[*midpoint,.2,0,0,math.atan2(delta[1],delta[0])],
            [float(np.linalg.norm(delta))+.025,.055,.4],color,True)
    gate=world.find("model[@name='start']")
    row=pose_at(scene,config['row_s_m'])
    gate.find('pose').text=f"{row['x']} {row['y']} 0 0 0 {row['yaw']}"
    for item in gate.findall('link/visual'):
        name=item.get('name')
        if name.startswith('post_'):
            side=-1 if name.endswith('-1') else 1
            values=list(map(float,item.findtext('pose').split())); values[1]=side*(config['width_m']/2+.12)
            item.find('pose').text=' '.join(map(str,values))
        elif name in ('stripe','header'):
            values=list(map(float,item.findtext('geometry/box/size').split()))
            values[1]=config['width_m']+(.35 if name=='header' else -.1)
            item.find('geometry/box/size').text=' '.join(map(str,values))
    tree.write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
    metadata=yaml.safe_load((source/'map.yaml').read_text())
    original=np.array(Image.open(source/'map.pgm'))
    image=original.copy(); resolution=metadata['resolution']; origin=metadata['origin']
    def pixel(p):
        return (round((p[0]-origin[0])/resolution), image.shape[0]-1-round((p[1]-origin[1])/resolution))
    pixels=np.array([pixel(p) for p in polygon],dtype=np.int32)
    cv2.fillPoly(image,[pixels],254)
    for a,b in boundary:cv2.line(image,pixel(a),pixel(b),0,2)
    Image.fromarray(image).save(out/'map.pgm')
    metadata['image']='map.pgm'
    (out/'map.yaml').write_text(yaml.safe_dump(metadata,sort_keys=False))
    # Every model except the existing barriers and start gate stays byte-equivalent.
    before=ET.parse(source/'world.sdf').getroot().find('world')
    for model in before.findall('model'):
        if model.get('name') not in ('continuous_track_barriers','start'):
            assert ET.tostring(model)==ET.tostring(world.find("model[@name='"+model.get('name')+"']"))
    changed=np.column_stack(np.where(image!=original))
    evidence=dict(width_m=config['width_m'],merge_at_s_m=end,row_s_m=config['row_s_m'],
        affected_original_barriers=removed,retained_barrier_pieces=split,
        modified_map_pixels=len(changed), original_world_sha256=hashlib.sha256((source/'world.sdf').read_bytes()).hexdigest(),
        fleet_world_sha256=hashlib.sha256((out/'world.sdf').read_bytes()).hexdigest(),
        checkpoint_and_server_models_unchanged=True, original_scene_files_unchanged=True,
        grid={c:grid_pose(scene,c,config) for c in config['lateral_offsets_m']})
    (out/'start-grid.json').write_text(json.dumps(evidence,indent=2)+'\n')
    return evidence
