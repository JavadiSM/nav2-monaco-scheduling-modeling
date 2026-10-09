#!/usr/bin/env python3
"""Retime final GIFs without changing or dropping their encoded image frames."""
from fractions import Fraction
import hashlib
import json
from pathlib import Path
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]

def delays(data):
    if data[:6] not in (b'GIF87a', b'GIF89a'):
        raise ValueError('Not a GIF')
    position = 13
    if data[10] & 128:
        position += 3 * (2 ** ((data[10] & 7) + 1))
    pending = None
    offsets = []
    def skip_blocks(position):
        while data[position]:
            position += 1 + data[position]
        return position + 1
    while position < len(data):
        marker = data[position]
        if marker == 0x3b:
            break
        if marker == 0x21:
            label = data[position + 1]
            if label == 0xf9:
                if data[position + 2] != 4:
                    raise ValueError('Unexpected graphics control size')
                pending = position + 4
            position = skip_blocks(position + 2)
        elif marker == 0x2c:
            if pending is None:
                raise ValueError('Frame has no explicit timing')
            offsets.append(pending)
            pending = None
            packed = data[position + 9]
            position += 10
            if packed & 128:
                position += 3 * (2 ** ((packed & 7) + 1))
            position = skip_blocks(position + 1)
        else:
            raise ValueError(f'Unexpected GIF block at {position}')
    return offsets

def validate(path):
    with Image.open(path) as im:
        duration = 0
        for n in range(im.n_frames):
            im.seek(n)
            im.load()
            duration += im.info.get('duration', 0)
        return dict(frames=im.n_frames, duration_s=duration / 1000, dimensions=list(im.size))

def main():
    evidence = []
    for stem in ('full-course', 'dual-view'):
        source = ROOT / 'docs/media' / (stem + '.gif')
        original = source.read_bytes()
        offsets = delays(original)
        source_stats = validate(source)
        assert len(offsets) == source_stats['frames']
        variants = []
        for speed in (Fraction(1), Fraction(2), Fraction(3), Fraction(4)):
            target = source if speed == 1 else source.with_name(f'{stem}-{float(speed):g}x.gif')
            altered = bytearray(original)
            elapsed_original = elapsed_new = 0
            for offset in offsets:
                elapsed_original += int.from_bytes(original[offset:offset+2], 'little')
                ticks = round(Fraction(elapsed_original) / speed) - elapsed_new
                if not 2 <= ticks <= 65535:
                    raise ValueError('Requested speed exceeds supported GIF delay range')
                altered[offset:offset+2] = ticks.to_bytes(2, 'little')
                elapsed_new += ticks
            if speed != 1:
                target.write_bytes(altered)
            stats = validate(target)
            assert stats['frames'] == source_stats['frames']
            assert stats['dimensions'] == source_stats['dimensions']
            assert abs(stats['duration_s'] - source_stats['duration_s']/float(speed)) <= .011
            restored = bytearray(target.read_bytes())
            for offset in offsets:
                restored[offset:offset+2] = original[offset:offset+2]
            assert bytes(restored) == original, 'Encoded visual data changed'
            variants.append(dict(speed=float(speed), file=str(target.relative_to(ROOT)), **stats,
                actual_speed=source_stats['duration_s']/stats['duration_s'],
                encoded_images_unchanged=True, sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
            print(f'{target.name}: {stats["frames"]} frames, {stats["duration_s"]:.2f} s', flush=True)
        assert source.read_bytes() == original, 'Original modified'
        evidence.append(dict(source=str(source.relative_to(ROOT)), variants=variants))
    (ROOT/'docs/evidence/gif-speed-variants.json').write_text(json.dumps(evidence, indent=2)+'\n')

if __name__ == '__main__':
    main()
