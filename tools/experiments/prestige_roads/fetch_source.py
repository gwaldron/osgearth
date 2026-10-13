"""Fetch small OSM map-API snapshots with context and provenance; never replace retained evidence implicitly."""

import argparse
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET

from scenes import SCENES


def download_region(bbox, grid):
    """Merge bounded map-API cells by source identity/version; preserve complete returned ways and request provenance."""
    if not 1 <= grid <= 4:
        raise ValueError('Source grid must contain at most sixteen requests')
    west, south, east, north = bbox
    merged, requests = {}, []
    for row in range(grid):
        for column in range(grid):
            bounds = (west+(east-west)*column/grid, south+(north-south)*row/grid,
                      west+(east-west)*(column+1)/grid, south+(north-south)*(row+1)/grid)
            url = 'https://api.openstreetmap.org/api/0.6/map?bbox='+','.join(f'{v:.7f}' for v in bounds)
            request = urllib.request.Request(url, headers={'User-Agent': 'PrestigeRoads-osgEarth-research/0.1'})
            with urllib.request.urlopen(request, timeout=60) as response:
                data = response.read()
            root = ET.fromstring(data)
            if root.tag != 'osm':
                raise ValueError('Map API did not return an OSM document')
            requests.append({'url': url, 'retrieved_utc': datetime.now(timezone.utc).isoformat(),
                             'response_sha256': hashlib.sha256(data).hexdigest()})
            for element in root:
                if element.tag not in ('node', 'way', 'relation'):
                    continue
                key = element.tag, element.get('id')
                previous = merged.get(key)
                if previous is None or int(element.get('version', '0')) > int(previous.get('version', '0')):
                    merged[key] = element
    root = ET.Element('osm', version='0.6', generator='Prestige Roads bounded map-API merge')
    for kind in ('node', 'way', 'relation'):
        for key in sorted((k for k in merged if k[0] == kind), key=lambda k: int(k[1])):
            root.append(merged[key])
    return root, requests


def fetch(scene, output):
    """Download one bounded extract, strip contributor identifiers, and retain deterministic gzip plus provenance."""
    settings = SCENES[scene]
    path = output/(settings.get('source', scene)+'.osm.gz')
    if path.exists():
        print(f'{scene}: retained snapshot already exists; leaving it unchanged', flush=True)
        return
    west, south, east, north = settings['bbox']
    bbox = (west-.0015, south-.001, east+.0015, north+.001)
    root, requests = download_region(bbox, settings.get('source_grid', 1))
    for element in root:
        for key in ('user', 'uid', 'changeset'):
            element.attrib.pop(key, None)
    payload = gzip.compress(ET.tostring(root, encoding='utf-8', xml_declaration=True), mtime=0)
    output.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)
    provenance = {'scene': scene, 'requests': requests, 'retrieved_utc': datetime.now(timezone.utc).isoformat(),
                  'source_bbox': bbox, 'render_bbox': settings['bbox'], 'bytes': len(payload),
                  'sha256': hashlib.sha256(payload).hexdigest(),
                  'nodes': len(root.findall('node')), 'ways': len(root.findall('way')),
                  'relations': len(root.findall('relation')),
                  'source_policy': ('Complete returned way/node references; user, uid, changeset attributes removed. '
                                    'Multiple cells merge newest object versions; this is not an atomic snapshot.'),
                  'attribution': 'OpenStreetMap contributors', 'license': 'https://www.openstreetmap.org/copyright'}
    (output/(scene+'-source.json')).write_text(json.dumps(provenance, indent=2)+'\n', encoding='utf-8')
    print(f'{scene}: {len(payload)} bytes, {provenance["ways"]} ways, SHA256 {provenance["sha256"]}', flush=True)


def main():
    """Fetch named scenes sequentially so the public map API receives bounded, serialized requests."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenes', nargs='+', choices=SCENES)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent/'source')
    args = parser.parse_args()
    for scene in args.scenes:
        fetch(scene, args.output)


if __name__ == '__main__':
    main()
