"""Fetch bounded ReadyMap elevation tiles with hashes; preserve the original bytes and source datum."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET

from shapely.geometry import box
from shapely.ops import unary_union
import shapely

import compile as compiler
from terrain import URL
from scenes import SCENES


def download(url):
    """Read one public elevation resource with a descriptive client identity and bounded network timeout."""
    request = urllib.request.Request(url, headers={'User-Agent': 'PrestigeRoads-osgEarth-research/0.1'})
    with urllib.request.urlopen(request, timeout=45) as response:
        return response.read()


def fetch(scene):
    """Retain native elevation covering full selected roads and the terrain halo; cap requests at sixteen tiles."""
    compiler.configure_scene(scene)
    root = Path(__file__).parent/'source'
    output = root/(scene+'-readymap')
    if (output/'manifest.json').exists():
        print(f'{scene}: retained elevation manifest exists; leaving source evidence unchanged', flush=True)
        return
    roads, _, _, _ = compiler.read_source(root/(SCENES[scene].get('source', scene)+'.osm.gz'))
    extent = unary_union([r['line'] for r in roads]).union(compiler.REGION).buffer(450).bounds
    west, south, east, north = shapely.transform(box(*extent), compiler.TO_GEO.transform, interleaved=False).bounds
    metadata = download(URL)
    xml = ET.fromstring(metadata)
    if xml.findtext('SRS') != 'EPSG:4326' or xml.find('TileSets').get('profile') != 'global-geodetic':
        raise ValueError('ReadyMap elevation profile changed; inspect before downloading tiles')
    level = max(int(t.get('order')) for t in xml.findall('TileSets/TileSet'))
    step = 180/2**level
    keys = [(x, y) for x in range(math.floor((west+180)/step), math.floor((east+180)/step)+1)
            for y in range(math.floor((south+90)/step), math.floor((north+90)/step)+1)]
    if len(keys) > 16:
        raise ValueError('Elevation request exceeds the bounded sixteen-tile experiment budget')
    output.mkdir(parents=True, exist_ok=True)
    (output/'tilemapresource.xml').write_bytes(metadata)
    tiles = []
    for x, y in keys:
        url = f'{URL}{level}/{x}/{y}.tif'
        path = output/f'{level}-{x}-{y}.tif'
        if not path.exists():
            path.write_bytes(download(url))
        payload = path.read_bytes()
        tiles.append({'x': x, 'y': y, 'url': url, 'file': path.name, 'bytes': len(payload),
                      'sha256': hashlib.sha256(payload).hexdigest()})
    record = {'scene': scene, 'source_url': URL, 'retrieved_utc': datetime.now(timezone.utc).isoformat(),
              'title': xml.findtext('Title'), 'description': xml.findtext('Abstract'), 'level': level,
              'datum': 'EGM96 orthometric meters, as configured in tests/readymap_elevation.xml',
              'coverage_request': [west, south, east, north], 'tiles': tiles,
              'metadata_sha256': hashlib.sha256(metadata).hexdigest(),
              'attribution': 'ReadyMap / CGIAR SRTM Elevation Composite; raw source samples retained'}
    (output/'manifest.json').write_text(json.dumps(record, indent=2)+'\n', encoding='utf-8')
    print(f'{scene}: retained {len(tiles)} native LOD {level} elevation tiles', flush=True)


def main():
    """Fetch selected scenes sequentially, reusing already retained public source snapshots."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenes', nargs='+', choices=SCENES)
    for scene in parser.parse_args().scenes:
        fetch(scene)


if __name__ == '__main__':
    main()
