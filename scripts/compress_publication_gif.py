#!/usr/bin/env python3
"""Create a compact full-interval GIF for the project README."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
from PIL import Image
from make_gif_speed_variants import delays, validate

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', nargs='?', type=Path,
                        default=ROOT / 'artifacts/dual-view/dual-view.gif')
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'docs/media/dual-view-4x.gif')
    parser.add_argument('--speed', type=float, default=4.)
    parser.add_argument('--width', type=int, default=896)
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--colors', type=int, default=80)
    parser.add_argument('--dither', choices=('none', 'bayer'), default='none')
    args = parser.parse_args()
    if args.speed <= 0 or args.width < 2 or args.fps < 1 or not 4 <= args.colors <= 256:
        parser.error('Invalid playback or compression parameters')
    source, output = args.source.resolve(), args.output.resolve()
    if source == output:
        parser.error('Source and output must differ')
    original = source.read_bytes()
    offsets = delays(original)
    source_duration = sum(int.from_bytes(original[o:o+2], 'little') for o in offsets) / 100
    with Image.open(source) as image:
        source_dimensions = list(image.size)
    output.parent.mkdir(parents=True, exist_ok=True)
    workspace = ROOT / 'artifacts/media-compression'
    workspace.mkdir(parents=True, exist_ok=True)
    filters = f'setpts=(PTS-STARTPTS)/{args.speed},fps={args.fps},scale={args.width}:-2:flags=lanczos'
    with tempfile.TemporaryDirectory(prefix='gif-', dir=workspace) as temporary:
        palette, encoded = Path(temporary)/'palette.png', Path(temporary)/'encoded.gif'
        common = ['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error']
        subprocess.run(common + ['-i', str(source), '-vf', filters + f',palettegen=max_colors={args.colors}:stats_mode=diff',
                       '-frames:v', '1', str(palette)], check=True)
        subprocess.run(common + ['-i', str(source), '-i', str(palette), '-filter_complex_threads', '1',
                       '-filter_complex', f'[0:v]{filters}[v];[v][1:v]paletteuse=dither={args.dither}:bayer_scale=5:diff_mode=rectangle',
                       '-gifflags', '+transdiff', '-loop', '0', str(encoded)], check=True)
        # Match the final display interval despite the GIF centisecond clock.
        encoded_data = bytearray(encoded.read_bytes())
        encoded_offsets = delays(encoded_data)
        total_ticks = sum(int.from_bytes(encoded_data[o:o+2], 'little') for o in encoded_offsets)
        expected = source_duration / args.speed
        correction = round(expected * 100) - total_ticks
        final_offset = encoded_offsets[-1]
        final_delay = int.from_bytes(encoded_data[final_offset:final_offset+2], 'little') + correction
        if correction and 2 <= final_delay <= 65535:
            encoded_data[final_offset:final_offset+2] = final_delay.to_bytes(2, 'little')
            encoded.write_bytes(encoded_data)
        stats = validate(encoded)
        if abs(stats['duration_s'] - expected) > max(.1, 1 / args.fps):
            raise RuntimeError('Compression changed the full playback interval')
        if encoded.stat().st_size >= len(original):
            raise RuntimeError('Output is not smaller than the source')
        output.write_bytes(encoded.read_bytes())
    assert source.read_bytes() == original, 'Source recording changed'
    evidence = dict(source=str(source.relative_to(ROOT)) if source.is_relative_to(ROOT) else source.name,
        source_sha256=hashlib.sha256(original).hexdigest(), source_bytes=len(original),
        source_frames=len(offsets), source_dimensions=source_dimensions, source_duration_s=source_duration,
        publication_file=str(output.relative_to(ROOT)) if output.is_relative_to(ROOT) else output.name,
        publication_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        publication_bytes=output.stat().st_size, playback_speed=args.speed,
        width=args.width, sampled_fps=args.fps, palette_colors=args.colors,
        dither=args.dither, full_interval_preserved=True, **stats)
    evidence['size_reduction_percent'] = 100 * (1 - evidence['publication_bytes'] / evidence['source_bytes'])
    (ROOT / 'docs/evidence/gif-compression.json').write_text(json.dumps(evidence, indent=2)+'\n')
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == '__main__':
    main()
