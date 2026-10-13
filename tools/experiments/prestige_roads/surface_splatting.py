"""Create comparable Prestige asphalt and plain coverage scenes for the three fixed city fixtures."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from city_fixtures import FIXTURES, ROOT
from fetch_elevation import download
from terrain import ReadyMap, URL


# Shared starting coverage level for the road demos; this is a paging threshold, not a camera altitude.
ROAD_MIN_LOD = 15


def camera_terrain(city, cameras):
    """Retain native ReadyMap evidence for camera targets only; leave the rendered elevation layer unchanged."""
    directory = ROOT/'source'/('render-fixture-'+city+'-camera-elevation')
    if not (directory/'manifest.json').is_file():
        metadata = download(URL)
        xml = ET.fromstring(metadata)
        if xml.findtext('SRS') != 'EPSG:4326' or xml.find('TileSets').get('profile') != 'global-geodetic':
            raise ValueError('Camera elevation requires the retained global-geodetic ReadyMap profile')
        level = max(int(tile.get('order')) for tile in xml.findall('TileSets/TileSet'))
        step = 180/2**level
        keys = sorted({(math.floor((pose['longitude']+180)/step), math.floor((pose['latitude']+90)/step))
                       for pose in cameras.values()})
        if len(keys) > 4:
            raise ValueError('Camera elevation request exceeds four native tiles')
        directory.mkdir(parents=True, exist_ok=True)
        (directory/'tilemapresource.xml').write_bytes(metadata)
        tiles = []
        for x, y in keys:
            url = f'{URL}{level}/{x}/{y}.tif'
            path = directory/f'{level}-{x}-{y}.tif'
            if not path.is_file():
                path.write_bytes(download(url))
            tiles.append({'x': x, 'y': y, 'url': url, 'file': path.name,
                          'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
        record = {'scene': city, 'purpose': 'Camera placement only; no terrain alteration',
                  'source_url': URL, 'retrieved_utc': datetime.now(timezone.utc).isoformat(),
                  'level': level, 'datum': 'EGM96 orthometric meters', 'tiles': tiles,
                  'metadata_sha256': hashlib.sha256(metadata).hexdigest()}
        (directory/'manifest.json').write_text(json.dumps(record, indent=2)+'\n', encoding='utf-8')
    return ReadyMap(directory, None)


def scenes(city):
    """Use attributed asphalt only; preserve the current elevation stack and keep unresolved structures separate."""
    source = ROOT/'output'/('render-fixture-'+city)
    contract = source/'render-contract'/'contract.json'
    if not contract.is_file():
        raise ValueError('Generate render_contract.py output first')
    manifest = json.loads((source/'manifest.json').read_text(encoding='utf-8'))
    fixture = FIXTURES[city]
    terrain = camera_terrain(city, manifest['cameras'])
    x, y = fixture['origin']
    index = source/'render-contract'/'asphalt.gpkg'
    stamp = source/'render-contract'/'runtime-index.json'
    vector_input = 'render-contract/asphalt.geojson'
    if index.is_file() and stamp.is_file():
        record = json.loads(stamp.read_text(encoding='utf-8-sig'))
        source_hash = hashlib.sha256((source/vector_input).read_bytes()).hexdigest()
        index_hash = hashlib.sha256(index.read_bytes()).hexdigest()
        if record['source_sha256'].lower() == source_hash and record['index_sha256'].lower() == index_hash:
            vector_input = 'render-contract/asphalt.gpkg'
    texture = (ROOT.parents[2]/'data'/'asphalt.jpg').as_posix()
    template = '''<map name="{title} / {mode} / {view}" xmlns:prestige="osgEarthPrestige">
    <options><terrain color="#bcc0b7" tile_size="17" screen_space_error="{sse}" max_lod="22"/></options>
    <TMSElevation name="ReadyMap elevation / unchanged">
        <url>https://readymap.org/readymap/tiles/1.0.0/116/</url><vdatum>egm96</vdatum>
    </TMSElevation>
    <OGRFeatures name="compiled-asphalt"><url>{vector_input}</url>
        <attribution>OpenStreetMap contributors / ODbL; inferred road dimensions</attribution>
    </OGRFeatures>
    {layer}
    <prestige:sky preset="flat" hours="{hours}" ambient="0.18" sun_intensity="4" environment_intensity="0"/>
    <viewpoints home="0"><viewpoint name="{view}" long="{lon}" lat="{lat}" height="{height}"
        heading="0" pitch="{pitch}" range="{distance}"/></viewpoints>
</map>
'''
    splat = f'''<prestige:surface_splatting name="Prestige asphalt" features="compiled-asphalt">
        <anchor>{x},{y},0</anchor><texture_period>1.0</texture_period><normal_strength>1</normal_strength>
        <material color="{texture}"/>
        <min_level>{ROAD_MIN_LOD}</min_level><tile_size>512</tile_size>
        <max_data_level>19</max_data_level><buffer_width>2m</buffer_width>
        <cache_policy usage="no_cache"/>
    </prestige:surface_splatting>'''
    plain = f'''<FeatureImage name="Plain pavement coverage" features="compiled-asphalt">
        <min_level>{ROAD_MIN_LOD}</min_level><tile_size>512</tile_size>
        <max_data_level>19</max_data_level><buffer_width>2m</buffer_width>
        <cache_policy usage="no_cache"/>
        <styles><style type="text/css">default {{ fill: #414447; }}</style></styles>
    </FeatureImage>'''
    records = []
    for mode, layer in (('splat', splat), ('coverage', plain)):
        for view in ('overview', 'intersection', 'detail', 'grazing'):
            pose = manifest['cameras']['overview' if view == 'overview' else 'intersection']
            height = float(terrain.geographic(pose['longitude'], pose['latitude']))+0.1
            distance = 3200 if view == 'overview' else 200 if view == 'intersection' else 18 if view == 'detail' else 75
            for sse in (32, 8):
                content = template.format(title=fixture['title'], mode=mode, view=view, layer=layer,
                    sse=sse, vector_input=vector_input, hours=13 if city == 'london' else 17,
                    lon=pose['longitude'], lat=pose['latitude'], height=height,
                    pitch=-20 if view == 'grazing' else -90, distance=distance)
                path = source/f'{mode}-{view}-{sse}.earth'
                path.write_text(content, encoding='utf-8', newline='\r\n')
                records.append({'file': path.name, 'target_height_ellipsoid_m': height,
                                'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    record = {'city': city, 'vector_input': vector_input, 'texture_period_m': 1,
              'coverage': {'min_lod': ROAD_MIN_LOD, 'max_data_lod': 19, 'tile_pixels': 512},
              'elevation': terrain.metadata, 'scenes': records}
    (source/'surface-scenes.json').write_text(json.dumps(record, indent=2)+'\n', encoding='utf-8')


def main():
    """Generate scene descriptions only; vectors, terrain, and renderer builds are not modified."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cities', nargs='+', choices=FIXTURES)
    for city in parser.parse_args().cities:
        scenes(city)


if __name__ == '__main__':
    main()
