#!/usr/bin/env python3
"""Check fleet isolation, starting clearance and preservation of the long route."""
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.generate_monaco import shortest_grid_path
from tools.monaco_fleet import prepare_fleet
from tools.monaco_start_grid import grid_pose, pose_at


def main():
    source=ROOT/'scenarios/monaco'
    originals={p.name:p.read_bytes() for p in source.iterdir() if p.is_file()}
    cars=prepare_fleet(ROOT)
    scene=json.loads((source/'scenario.json').read_text())
    config=yaml.safe_load((ROOT/'config/monaco_fleet.yaml').read_text())
    grid=config['start_grid']; assets=ROOT/'artifacts/fleet/assets'
    centre=pose_at(scene,grid['row_s_m'])
    tangent=np.array([math.cos(centre['yaw']),math.sin(centre['yaw'])])
    normal=np.array([-tangent[1],tangent[0]])
    metadata=yaml.safe_load((assets/'map.yaml').read_text())
    image=np.array(Image.open(assets/'map.pgm'))
    res=metadata['resolution'];origin=metadata['origin']
    clearance=distance_transform_edt(image==254)*res
    def cell(p):
        return (image.shape[0]-1-int((p['y']-origin[1])/res),int((p['x']-origin[0])/res))
    lengths={}
    for color,lateral in grid['lateral_offsets_m'].items():
        p=grid_pose(scene,color,grid)
        offset=np.array([p['x']-centre['x'],p['y']-centre['y']])
        assert abs(offset@tangent)<1e-9, 'Vehicles do not share a transverse row'
        assert abs(offset@normal-lateral)<1e-9
        assert clearance[cell(p)]>.255, 'Insufficient initial footprint clearance'
        length=shortest_grid_path(clearance>.255,cell(p),cell(scene['finish']),res)
        assert length>.78*scene['centreline_length_m'], 'Apron introduced a shortcut'
        lengths[color]=length
    base=ET.parse(source/'racecar.sdf').getroot().find('model')
    original_params=yaml.safe_load((source/'nav2_params.yaml').read_text())
    red_params=yaml.safe_load((assets/'red-nav2.yaml').read_text())
    assert red_params['bt_navigator']['ros__parameters'].pop('default_server_timeout')==config['nav2']['action_ack_timeout_ms']
    original_params['bt_navigator']['ros__parameters'].pop('default_server_timeout')
    assert red_params==original_params, 'Fleet changed navigation tuning'
    for car in cars:
        model=ET.parse(car['sdf']).getroot().find('model')
        for link in base.findall('link'):
            counterpart=model.find("link[@name='"+link.get('name')+"']")
            for tag in ('collision','inertial'):
                assert [ET.tostring(e) for e in link.findall(tag)]==[ET.tostring(e) for e in counterpart.findall(tag)]
        bridge=yaml.safe_load(Path(car['bridge']).read_text())
        assert len(bridge)==6
        for endpoint in bridge:
            for key in ('ros_topic_name','gz_topic_name'):
                assert endpoint[key].startswith('/'+car['color']+'/')
                assert not endpoint[key].endswith('/clock'), 'Duplicate clock publisher'
        def unprefix(value):
            if isinstance(value,dict):return {k:unprefix(v) for k,v in value.items()}
            if isinstance(value,list):return [unprefix(v) for v in value]
            prefix='/'+car['color']
            return value[len(prefix):] if isinstance(value,str) and value.startswith(prefix+'/') else value
        params=unprefix(yaml.safe_load(Path(car['params']).read_text()))
        assert params==yaml.safe_load((assets/'red-nav2.yaml').read_text())
    assert originals=={p.name:p.read_bytes() for p in source.iterdir() if p.is_file()}
    result=dict(passed=True,starting_row_verified=True,footprints_clear=True,
        baseline_files_unchanged=True,physical_vehicle_parameters_unchanged=True,
        namespaced_bridges_verified=True,navigation_tuning_preserved=True,
        shortest_drivable_paths_m=lengths)
    (assets/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
