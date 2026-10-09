#!/usr/bin/env python3
"""Check that the accepted circuit assets retain their exact frozen content."""
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    manifest=json.loads((ROOT/'docs/evidence/frozen-scene-manifest.json').read_text())
    changed=[p for p,digest in manifest['files'].items() if not (ROOT/p).exists() or hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=digest]
    if changed:raise SystemExit('Frozen scene differs: '+', '.join(changed))
    print(f"Frozen scene verified: {len(manifest['files'])} files unchanged")
if __name__=='__main__':main()
