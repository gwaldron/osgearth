"""Compare compiler paint as terrain coverage and depth-clamped mesh decals on the fixed city fixtures."""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from pyproj import Transformer
import shapely
from shapely.geometry import Point, shape

from city_fixtures import FIXTURES, ROOT
from raster_roads import PALETTE
from surface_splatting import ROAD_MIN_LOD, camera_terrain


DECAL_SHADER = '''
#pragma vp_function oe_road_paint_position, vertex_view, 0.95
out vec3 oe_road_paint_viewPosition;
// Capture the mesh after the depth-clamping vertex stage.
void oe_road_paint_position(inout vec4 vertex)
{
    oe_road_paint_viewPosition = vertex.xyz;
}
[break]
#pragma vp_function oe_road_paint_material, fragment_coloring, 0.9
#pragma import_defines(OE_USE_PBR)
in vec3 oe_road_paint_viewPosition;
in vec3 vp_Normal;
#ifdef OE_USE_PBR
struct OE_PBR { float displacement, roughness, ao, metal; } oe_pbr;
#endif
// Derive the decal's geometric normal from clamped positions; keep paint nonmetallic.
void oe_road_paint_material(inout vec4 color)
{
    vec3 normal = cross(dFdx(oe_road_paint_viewPosition), dFdy(oe_road_paint_viewPosition));
    if (dot(normal,normal) > 1e-20)
        vp_Normal = normalize(normal) * (dot(normal,vp_Normal) < 0.0 ? -1.0 : 1.0);
#ifdef OE_USE_PBR
    oe_pbr.roughness = 0.6;
    oe_pbr.ao = 1.0;
    oe_pbr.metal = 0.0;
#endif
}
'''


def paint_camera(source, manifest):
    """Choose an existing white polygon near the junction; prefer compact paint without inventing arrow semantics."""
    local = Transformer.from_crs(4326, manifest['metric_crs'], always_xy=True)
    geographic = Transformer.from_crs(manifest['metric_crs'], 4326, always_xy=True)
    target = manifest['cameras']['intersection']
    center = Point(*local.transform(target['longitude'], target['latitude']))
    features = json.loads((source/'render-contract/white.geojson').read_text(encoding='utf-8'))['features']
    candidates = []
    for feature in features:
        polygon = shapely.transform(shape(feature['geometry']), local.transform, interleaved=False)
        if not 0.05 < polygon.area < 20 or polygon.distance(center) > 90:
            continue
        rectangle = np.asarray(polygon.minimum_rotated_rectangle.exterior.coords)
        lengths = np.linalg.norm(np.diff(rectangle, axis=0), axis=1)
        if lengths.min() < 0.12 or lengths.max() > 14:
            continue
        candidates.append((0 if lengths.min() > 0.9 else 1,
                           polygon.centroid.distance(center), feature['id'], polygon))
    if not candidates:
        raise ValueError('No compact marking near the fixed junction camera')
    _, _, identity, polygon = min(candidates, key=lambda item: item[:2])
    longitude, latitude = geographic.transform(polygon.centroid.x, polygon.centroid.y)
    return {'longitude': longitude, 'latitude': latitude, 'feature_id': identity}


def build(city):
    """Generate comparable scenes and tiny constant paint assets; leave compiler geometry and evidence unchanged."""
    source = ROOT/'output'/('render-fixture-'+city)
    contract = json.loads((source/'render-contract/contract.json').read_text(encoding='utf-8'))
    manifest = json.loads((source/'manifest.json').read_text(encoding='utf-8'))
    output = source/'marking-comparison'
    output.mkdir(exist_ok=True)
    cameras = dict(manifest['cameras'])
    cameras['paint'] = paint_camera(source, manifest)
    terrain = camera_terrain(city, cameras)
    records = []
    materials = [m for m in ('yellow', 'white') if contract['material_counts'][m]]
    hashes = {m: hashlib.sha256((source/'render-contract'/(m+'.geojson')).read_bytes()).hexdigest()
              for m in materials}
    for material in materials:
        Image.new('RGB', (4, 4), PALETTE[material]).save(output/(material+'.png'))
    Image.new('RGB', (4, 4), (153, 153, 153)).save(output/'roughness.png')
    for sse in (32, 8):
        template = (source/f'splat-intersection-{sse}.earth').read_text(encoding='utf-8')
        # The derived scene lives one folder below the existing asphalt scene.
        template = template.replace('<url>render-contract/', '<url>../render-contract/')
        for backend in ('mask', 'decal'):
            blocks = []
            for material in materials:
                blocks.append(f'''<OGRFeatures name="paint-{material}">
        <url>../render-contract/{material}.geojson</url>
    </OGRFeatures>''')
                if backend == 'mask':
                    x, y = FIXTURES[city]['origin']
                    blocks.append(f'''<prestige:surface_splatting name="Paint {material}" features="paint-{material}">
        <anchor>{x},{y},0</anchor><texture_period>1</texture_period>
        <coverage_outline>false</coverage_outline>
        <material color="{material}.png" roughness="roughness.png"/>
        <min_level>{ROAD_MIN_LOD}</min_level><tile_size>512</tile_size>
        <max_data_level>21</max_data_level><buffer_width>2m</buffer_width>
        <cache_policy usage="no_cache"/>
    </prestige:surface_splatting>''')
                else:
                    color = '#'+''.join(f'{c:02x}' for c in PALETTE[material])
                    blocks.append(f'''<FeatureModel name="Decal {material}" features="paint-{material}">
        <merge_geometry>true</merge_geometry><max_granularity>0.00005</max_granularity>
        <styles><style type="text/css">default {{
            fill: {color}; altitude-clamping: terrain; altitude-technique: gpu; altitude-binding: vertex;
            altitude-offset: 0m; render-decal: true; render-depth-test: true; render-lighting: true;
        }}</style></styles>
        <shader><![CDATA[{DECAL_SHADER}]]></shader>
    </FeatureModel>''')
            for view, key, distance, pitch in (('intersection', 'intersection', 200, -90),
                                               ('detail', 'paint', 18, -90), ('grazing', 'paint', 55, -15),
                                               ('overview', 'overview', 3200, -90)):
                pose = cameras[key]
                height = float(terrain.geographic(pose['longitude'], pose['latitude']))+0.1
                text = template.replace('    <prestige:sky', '\n'.join(blocks)+'\n    <prestige:sky')
                start, end = text.index('    <viewpoints'), text.index('</viewpoints>')+len('</viewpoints>')
                text = text[:start]+f'''    <viewpoints home="0"><viewpoint name="{backend} {view}"
        long="{pose['longitude']}" lat="{pose['latitude']}" height="{height}"
        heading="0" pitch="{pitch}" range="{distance}"/></viewpoints>'''+text[end:]
                ET.fromstring(text)
                path = output/f'{backend}-{view}-{sse}.earth'
                path.write_text(text, encoding='utf-8', newline='\r\n')
                records.append({'file': path.name, 'height_ellipsoid_m': height,
                                'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    record = {'city': city, 'input_sha256': hashes, 'cameras': cameras, 'scenes': records,
              'ground_receiver': contract['ground_receiver_id'], 'paint_roughness': 0.6,
              'mask': {'tile_pixels': 512, 'min_lod': ROAD_MIN_LOD, 'max_lod': 21, 'coverage_outline': False,
                       'terrain_elevation_unchanged': True},
              'decal': {'clamping': 'GPU terrain depth', 'vertical_offset_m': 0,
                        'polygon_offset': [-1, -1], 'max_triangle_edge_angle_degrees': 0.00005},
              'limitations': ['Depth clamping is a comparison baseline, not exact grid attachment',
                              'No elevated structure paint is included; deck heights remain unknown',
                              'No distance handoff between the two separate comparison scenes']}
    (output/'manifest.json').write_text(json.dumps(record, indent=2)+'\n', encoding='utf-8')
    print(city, len(records), 'comparison scenes', cameras['paint'], flush=True)


def main():
    """Prepare bounded comparisons for selected existing fixtures; missing inputs fail explicitly."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cities', nargs='+', choices=FIXTURES)
    for city in parser.parse_args().cities:
        build(city)


if __name__ == '__main__':
    main()
