"""Feed one authoritative Boston junction polygon to the integrated Prestige GradingLayer."""

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import shapely
from pyproj import Transformer
from shapely.geometry import shape

from city_fixtures import ROOT


SOURCE = ROOT/'output/render-fixture-boston'
OUTPUT = ROOT/'output/grading-boston'
OWNER = 'junction/61341764'


def digest(path):
    """Hash an input for reproducible provenance without modifying it."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """Create paired scenes and a complete, unclipped junction pad; leave every source vector unchanged."""
    OUTPUT.mkdir(parents=True, exist_ok=True)
    source = SOURCE/'render-contract/asphalt.geojson'
    collection = json.loads(source.read_text(encoding='utf-8'))
    features = [f for f in collection['features'] if f['properties']['surface_id'] == OWNER]
    if len(features) != 1 or features[0]['properties']['receiver_id'] != 'terrain/boston':
        raise ValueError('Expected one authoritative terrain-owned junction polygon')
    geometry = shape(features[0]['geometry'])
    if not geometry.is_valid or geometry.geom_type != 'Polygon':
        raise ValueError('Junction pad requires a valid complete polygon')
    (OUTPUT/'junction-pad.geojson').write_text(json.dumps({'type': 'FeatureCollection', 'features': features},
        indent=2)+'\n', encoding='utf-8')
    fixture = json.loads((SOURCE/'manifest.json').read_text(encoding='utf-8'))
    local = Transformer.from_crs(4326, fixture['metric_crs'], always_xy=True)
    metric = shapely.transform(geometry, local.transform, interleaved=False)
    base = ET.fromstring((SOURCE/'marking-comparison/mask-intersection-8.earth').read_text(encoding='utf-8'))
    for node in base.findall('OGRFeatures/url'):
        node.text = '../render-fixture-boston/render-contract/'+Path(node.text).name
    for layer in base.findall('{osgEarthPrestige}surface_splatting'):
        material = 'asphalt' if layer.get('name') == 'Prestige asphalt' else layer.get('name').split()[-1]
        values = layer.find('material')
        values.attrib.clear()
        values.attrib.update(color=f'../bridge-boston-congress/materials/{material}-color.png',
            packed=f'../bridge-boston-congress/materials/{material}-orm.png', layout='orm')
        for key in ('color', 'packed'):
            if not (OUTPUT/values.get(key)).is_file():
                raise ValueError('Generate the shared bridge materials before building this scene')
    terrain = base.find('options/terrain')
    terrain.set('max_lod', '22')
    terrain.set('min_tile_range_factor', '7')
    terrain.set('screen_space_error', '8')
    ET.register_namespace('prestige', 'osgEarthPrestige')
    cameras = {'overview': (-65, 150, 0), 'close': (-16, 65, 45), 'top': (-90, 100, 0)}
    for enabled in (False, True):
        for view, (pitch, distance, heading) in cameras.items():
            tree = ET.fromstring(ET.tostring(base))
            title = ('Flattened' if enabled else 'Original')+' Boston junction / polygon pad'
            tree.set('name', title)
            source_layer = ET.SubElement(tree, 'OGRFeatures', name='Compiled Boston junction pad')
            ET.SubElement(source_layer, 'url').text = 'junction-pad.geojson'
            flatten = ET.SubElement(tree, '{osgEarthPrestige}grading', name='Compiled road polygon grading',
                features='Compiled Boston junction pad', open='true' if enabled else 'false')
            for name, value in {'min_level': 15, 'max_data_level': 20, 'tile_size': 257,
                'line_width': 0, 'buffer_width': 5, 'max_line_width': 1, 'max_buffer_width': 5,
                'elevation_resolution': 5}.items():
                ET.SubElement(flatten, name).text = str(value)
            camera = tree.find('viewpoints/viewpoint')
            camera.set('name', title)
            camera.set('pitch', str(pitch))
            camera.set('range', str(distance))
            camera.set('heading', str(heading))
            mode = 'after' if enabled else 'before'
            (OUTPUT/f'{mode}-{view}.earth').write_text(ET.tostring(tree, encoding='unicode'), encoding='utf-8')
    provenance = {'owner': OWNER, 'source_sha256': digest(source), 'pad_area_m2': metric.area,
        'pad_bounds_m': list(metric.bounds), 'pad_sha256': digest(OUTPUT/'junction-pad.geojson'),
        'policy': 'One complete compiled junction footprint, level polygon mode; not district-wide grading',
        'geometry_unchanged': shape(features[0]['geometry']).equals_exact(geometry, 0),
        'shoulder_m': 5, 'elevation_resolution_m': 5, 'datum': 'Ellipsoid output from preceding ReadyMap elevation',
        'exclusions': 'No structure vectors, elevated ramps, tunnels, bridge landings or water features are inputs',
        'limitations': ['Level pad, not longitudinal road grading', 'Shoulder also modifies nearby non-road terrain',
            'Separate neighboring pads can disagree; merged network pads would erase longitudinal grades',
            'No automatic external feature-file invalidation'], 'generator_sha256': digest(Path(__file__))}
    (OUTPUT/'manifest.json').write_text(json.dumps(provenance, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(provenance, indent=2))


if __name__ == '__main__':
    main()
