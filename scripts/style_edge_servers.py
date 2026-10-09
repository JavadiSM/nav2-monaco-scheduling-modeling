#!/usr/bin/env python3
"""Replace only edge-server visuals with minimal blue cabinets and antennas."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]

def add_visual(link,name,pose,kind,size,colour):
    visual=ET.SubElement(link,'visual',name=name)
    ET.SubElement(visual,'pose').text=pose
    geometry=ET.SubElement(visual,'geometry');shape=ET.SubElement(geometry,kind)
    if kind=='box':ET.SubElement(shape,'size').text=size
    else:
        radius,length=size
        ET.SubElement(shape,'radius').text=str(radius)
        ET.SubElement(shape,'length').text=str(length)
    material=ET.SubElement(visual,'material')
    for field in ('ambient','diffuse'):ET.SubElement(material,field).text=colour

def without_server_visuals(root):
    root=deepcopy(root)
    for model in root.findall('./world/model'):
        if model.get('name','').startswith('edge_'):
            for link in model.findall('link'):
                for visual in link.findall('visual'):link.remove(visual)
    return ET.canonicalize(ET.tostring(root,encoding='unicode'),strip_text=True)

def main():
    path=ROOT/'scenarios/monaco/world.sdf';old=path.read_bytes();root=ET.fromstring(old)
    before=without_server_visuals(root)
    models=[m for m in root.findall('./world/model') if m.get('name','').startswith('edge_')]
    expected=len(json.loads((ROOT/'scenarios/monaco/scenario.json').read_text())['servers'])
    if len(models)!=expected:raise RuntimeError('World endpoint count differs from scene')
    for model in models:
        link=model.find('link')
        for v in link.findall('visual'):link.remove(v)
        add_visual(link,'parking_bay','0 0 0.012 0 0 0','box','0.86 0.60 0.025','0.04 0.40 0.60 1')
        add_visual(link,'server_cabinet','0 0 0.65 0 0 0','box','0.66 0.44 1.25','0.035 0.24 0.72 1')
        add_visual(link,'cabinet_top','0 0 1.285 0 0 0','box','0.68 0.46 0.035','0.08 0.46 0.90 1')
        add_visual(link,'front_panel','0.337 0 0.73 0 0 0','box','0.014 0.34 0.82','0.025 0.075 0.14 1')
        for i,z in enumerate((.47,.64,.81,.98)):
            add_visual(link,f'rack_slot_{i}',f'0.347 0 {z} 0 0 0','box','0.012 0.29 0.045','0.11 0.38 0.62 1')
        add_visual(link,'status_light','0.347 -0.10 1.08 0 0 0','box','0.015 0.035 0.035','0.10 0.95 0.80 1')
        add_visual(link,'antenna','-0.17 0 1.55 0 0 0','cylinder',(.018,.50),'0.05 0.10 0.17 1')
        add_visual(link,'antenna_tip','-0.17 0 1.81 0 0 0','cylinder',(.027,.035),'0.12 0.90 0.96 1')
    assert without_server_visuals(root)==before,'Non-visual world content changed'
    ET.indent(root);ET.ElementTree(root).write(path,encoding='unicode',xml_declaration=True)
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    manifest_path=ROOT/'docs/evidence/frozen-scene-manifest.json'
    manifest=json.loads(manifest_path.read_text())
    manifest['policy']='Freeze geometry, navigation and physics. User-authorized server visual restyling is the sole world-content exception.'
    manifest['files']['scenarios/monaco/world.sdf']=digest
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    evidence=dict(server_count=len(models),visual_only=True,collision_geometry_unchanged=True,
        all_non_visual_world_content_unchanged=True,positions_unchanged=True,occupancy_map_unchanged=True,
        cabinet_dimensions_m=[.66,.44,1.25],antenna_top_m=1.8275,
        original_world_sha256=hashlib.sha256(old).hexdigest(),styled_world_sha256=digest)
    (ROOT/'docs/evidence/server-visual-style.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps(evidence,indent=2))
if __name__=='__main__':main()
