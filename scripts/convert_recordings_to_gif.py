#!/usr/bin/env python3
"""Convert local recordings, validate GIFs, then optionally remove their MP4s."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]

def convert(source, output, *, fps=5, width=960, start=None, duration=None):
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.stem + '.partial.gif')
    command = ['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error', '-threads', '2']
    if start is not None:
        command += ['-ss', str(start)]
    command += ['-i', str(source)]
    if duration is not None:
        command += ['-t', str(duration)]
    command += ['-filter_complex_threads', '1', '-filter_complex',
        f'fps={fps},scale={width}:-1:flags=lanczos,split[a][b];'
        '[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3',
        '-loop', '0', str(temporary)]
    subprocess.run(command, check=True)
    with Image.open(temporary) as im:
        frames, milliseconds = im.n_frames, 0
        for frame in range(frames):
            im.seek(frame)
            im.load()
            milliseconds += im.info.get('duration', 0)
        dimensions = list(im.size)
    if frames < 2 or milliseconds <= 0:
        raise RuntimeError('GIF validation failed; original recording retained')
    temporary.replace(output)
    return dict(frames=frames, duration_s=milliseconds / 1000, dimensions=dimensions,
                bytes=output.stat().st_size)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--remove-mp4', action='store_true')
    args = parser.parse_args()
    manifest_path = ROOT / 'artifacts/gif-conversion-manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else []
    for source in sorted((ROOT / 'artifacts').rglob('*.mp4')):
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        output = source.with_suffix('.gif')
        info = convert(source, output)
        entry = dict(source=str(source.relative_to(ROOT)), source_sha256=digest,
                     gif=str(output.relative_to(ROOT)), **info, source_removed=False)
        if args.remove_mp4:
            source.unlink()
            entry['source_removed'] = True
        manifest.append(entry)
        manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
        print(json.dumps(entry), flush=True)

if __name__ == '__main__':
    main()
