"""Explicit multicore fixtures for concurrency and coupling unit cases."""
import json,tempfile
from pathlib import Path
from tools.abstract_compute.hardware import load_platform as production_platform
from tools.live_bridge.engine import LiveEngine as ProductionEngine
ROOT=Path(__file__).resolve().parents[1]

def configuration():
    cfg=json.loads((ROOT/'config/abstract_compute.json').read_text())
    cfg['device_classes']['vehicle'].update(lp_cores=1,hp_cores=1)
    cfg['device_classes']['server'].update(lp_cores=2,hp_cores=2)
    return cfg

def load_platform(path=None,**kwargs):
    if path is not None:return production_platform(path,**kwargs)
    with tempfile.TemporaryDirectory() as folder:
        f=Path(folder)/'hardware.json';f.write_text(json.dumps(configuration()))
        return production_platform(f,**kwargs)

class LiveEngine(ProductionEngine):
    def __init__(self,*args,**kwargs):
        if len(args)>2 or kwargs.get('platform_path') is not None:
            super().__init__(*args,**kwargs);return
        with tempfile.TemporaryDirectory() as folder:
            f=Path(folder)/'hardware.json';f.write_text(json.dumps(configuration()));kwargs['platform_path']=f
            super().__init__(*args,**kwargs)
