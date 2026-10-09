"""Bind each roadside location to an independent server processor template."""
from dataclasses import dataclass
import json
from pathlib import Path
from .hardware import load_platform,Processor
from .thermal import ThermalModel


@dataclass
class EdgeResource:
    endpoint_id:str
    position_xy:tuple[float,float]
    processor:Processor


def load_edge_resources(scene=None,*,platform_path=None,vehicle_count=1):
    if scene is None:
        scene=json.loads((Path(__file__).resolve().parents[2]/'scenarios/monaco/scenario.json').read_text())
    if isinstance(vehicle_count,bool) or not isinstance(vehicle_count,int) or vehicle_count<0:raise ValueError('Invalid vehicle count')
    endpoints=scene['servers']
    devices=load_platform(platform_path,device_classes=('vehicle',)*vehicle_count+('server',)*len(endpoints))
    result={}
    for index,endpoint in enumerate(endpoints):
        if endpoint['id'] in result:raise ValueError('Duplicate roadside endpoint ID')
        processor=devices[vehicle_count+index]
        ThermalModel(processor).validate_idle_recovery()
        result[endpoint['id']]=EdgeResource(endpoint['id'],(endpoint['x'],endpoint['y']),processor)
    return result
