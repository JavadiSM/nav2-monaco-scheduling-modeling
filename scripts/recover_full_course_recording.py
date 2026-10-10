#!/usr/bin/env python3
"""Recover presentation postprocessing after an independently validated full lap."""
import argparse,hashlib,io,json,shutil,time
from pathlib import Path
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,indent=2)+'\n');tmp.replace(path)
def recover(trial):
    trial=Path(trial).resolve();campaign=trial.parent
    control=json.loads((campaign/'control.json').read_text());old=control.get('runner',{});folder=Path('/proc')/str(old.get('pid',0))
    if folder.exists() and int((folder/'stat').read_text().split()[21])==old.get('start_ticks'):raise ValueError('Stop the campaign runner before changing its manifest')
    run=json.loads((trial/'run.json').read_text());mission=json.loads((trial/'red-mission.json').read_text());audit=json.loads((trial/'validation.json').read_text());metrics=json.loads((trial/'metrics.json').read_text())
    if not (run.get('simulation_completed') and mission.get('passed') and mission.get('full_course') and mission.get('ordered_targets_passed')==20 and audit.get('passed') and metrics.get('full_course') and metrics.get('validation_errors')==0):raise ValueError('Only an audited successful full mission may be recovered')
    presentation=json.loads((trial/'presentation.json').read_text())
    if presentation.get('completed'):raise ValueError('Presentation is already complete')
    archive=ROOT/'.private/media-backups/live-bridge'/(trial.name+'.gif');raw=trial/'views/live-three-view.gif';source=raw if raw.exists() else archive
    data=source.read_bytes();sha=hashlib.sha256(data).hexdigest()
    with Image.open(io.BytesIO(data)) as picture:
        size=list(picture.size);frames=picture.n_frames
        if frames<2 or size!=[1440,540]:raise ValueError('Invalid full-course camera recording')
        picture.seek(frames//2);picture.convert('RGB').save(trial/'three-view-preview.png')
    archive.parent.mkdir(parents=True,exist_ok=True)
    if archive.exists():
        if digest(archive)!=sha:raise ValueError('A different recording archive already exists')
    else:shutil.move(raw,archive)
    if digest(archive)!=sha:raise ValueError('Recording archive differs from the original')
    recovered=dict(presentation,completed=True,size=size,frames=frames,raw_gif_archive=str(archive.relative_to(ROOT)),encoding_deferred=True,postprocessing_recovered=True)
    save(trial/'presentation-recovered.json',recovered)
    evidence={name:digest(trial/name) for name in ('run.json','red-mission.json','validation.json','presentation.json')}
    record=dict(kind='recording_postprocessing',validated_full_mission=True,recovered_unix_s=time.time(),original_error=run.get('error'),original_evidence_sha256=evidence,raw_gif_sha256=sha,frames=frames,size=size,original_files_preserved=True)
    save(trial/'recording-recovery.json',record)
    manifest_path=campaign/'manifest.json';manifest=json.loads(manifest_path.read_text());backup=campaign/'manifest-before-recording-recovery.json'
    if not backup.exists():shutil.copy2(manifest_path,backup)
    row=next(r for r in manifest['trials'] if r['trial']==trial.name)
    row.setdefault('original_status',row['status']);row.update(status='complete',postprocessing_recovery='recording-recovery.json')
    manifest['interruption_reason']='Recording postprocessing recovery; physical mission and original evidence retained'
    save(manifest_path,manifest)
    return dict(trial=trial.name,frames=frames,size=size,original_physics_preserved=True,recording_sha256=sha)
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path);args=parser.parse_args();print(json.dumps(recover(args.trial)))
