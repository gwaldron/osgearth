"""Pair frozen road footprints with source centerlines and show single-LOD Prestige road grading."""

import argparse
import faulthandler
import hashlib
import json
import math
import pickle
import xml.etree.ElementTree as ET
from pathlib import Path

import shapely
from shapely.geometry import GeometryCollection, mapping, shape
from shapely.ops import unary_union

import city_fixtures
import compile as compiler
import raster_roads as planar


def digest(path):
    """Identify immutable compiler inputs without changing them."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    """Write a reproducible UTF-8 artifact with a terminal newline."""
    path.write_text(json.dumps(value, indent=2)+'\n', encoding='utf-8')


def publish(city):
    """Associate profiles with compiled masks; inferred fill associations never replace source ownership."""
    fixture = city_fixtures.configure(city)
    source = city_fixtures.ROOT/'output'/('render-fixture-'+city)
    output = city_fixtures.ROOT/'output'/('road-grading-'+city)
    output.mkdir(exist_ok=True)
    contract = json.loads((source/'render-contract/contract.json').read_text(encoding='utf-8'))
    for name, expected in contract['input_sha256'].items():
        if digest(source/name) != expected:
            raise ValueError('Frozen fixture changed: '+name)
    inputs = {name: digest(source/name) for name in ['resolved.pkl', 'render-contract/asphalt.geojson',
        'render-contract/ownership-domains.geojson']}
    with (source/'resolved.pkl').open('rb') as stream:
        roads, _, _, _, _, _ = pickle.load(stream)
    roads = {str(r['id']): r for r in roads if not r['bridge'] and not r['tunnel']}
    owners = {o['surface_id']: o for o in contract['owners']}
    pavement = json.loads((source/'render-contract/asphalt.geojson').read_text(encoding='utf-8'))
    by_owner = {}
    for feature in pavement['features']:
        polygon = shapely.transform(shape(feature['geometry']), planar.TO_LOCAL.transform, interleaved=False)
        by_owner.setdefault(feature['properties']['surface_id'], []).append(shapely.make_valid(polygon))
    authoritative = unary_union([p for polygons in by_owner.values() for p in polygons])
    print(city, 'loaded authoritative pavement', flush=True)
    features, masks = [], []

    def add(mask, road_ids, surface_id, association):
        """Bind complete XY alignments to a valid local mask, retaining the association evidence separately."""
        polygons = [p for p in compiler.parts(shapely.make_valid(mask)) if p.area > 1e-8]
        if not polygons:
            return
        lines = [roads[r]['line'] for r in sorted(set(road_ids))]
        if not lines:
            raise ValueError('Pavement has no known centerline association: '+surface_id)
        mask = unary_union(polygons)
        alignment = unary_union(lines)
        points = shapely.points(shapely.get_coordinates(shapely.segmentize(mask, 2.0)))
        radius = max(20.0, float(shapely.distance(points, alignment).max())+12.0)
        if radius > 250:
            raise ValueError('Unexpectedly distant profile: '+surface_id)
        geometry = GeometryCollection([*lines, *polygons])
        features.append({'type': 'Feature', 'id': len(features)+1, 'geometry': mapping(shapely.transform(
            geometry, planar.TO_GEO.transform, interleaved=False)), 'properties': {
                'surface_id': surface_id, 'receiver_id': 'terrain/'+city,
                'source_way_ids': ','.join(sorted(set(road_ids))), 'profile_search_width_m': math.ceil(2*radius),
                'grading_junction': surface_id.startswith('junction/'),
                'association': association, 'elevation_policy': 'Sample preceding elevation; source geometry Z ignored',
                'support_width_is_not_pavement_width': True}})
        masks.extend(polygons)

    # Accepted junction ownership supplies its actual approach IDs, including compound junctions.
    for sid, polygons in sorted(by_owner.items()):
        if sid.startswith('junction/'):
            add(unary_union(polygons), [i for i in owners[sid]['source_way_ids'] if i in roads], sid,
                'Compiler junction approach membership; shared fitted plane with connected-profile transitions')
    remainder = unary_union(by_owner.get('network/'+city, []))
    print(city, 'paired junctions', len(features), flush=True)
    corridor_masks = []
    for index, (rid, road) in enumerate(sorted(roads.items())):
        x0, y0, x1, y1 = road['surface'].bounds
        local_remainder = shapely.make_valid(shapely.clip_by_rect(remainder, x0-1, y0-1, x1+1, y1+1))
        mask = road['surface'].intersection(local_remainder)
        if not mask.is_empty:
            add(mask, [rid], 'network/'+city, 'Compiler source-way corridor footprint')
            corridor_masks.append(mask)
        if index % 250 == 0:
            print(city, 'paired source corridors', index, flush=True)
    # Rounded compiler fill has shared network ownership. Only its height association is inferred.
    residual = remainder.difference(unary_union(corridor_masks))
    print(city, 'residual fill area', residual.area, flush=True)
    road_ids = list(roads)
    line_tree = shapely.STRtree([roads[i]['line'] for i in road_ids])
    fill_count = 0
    for part in compiler.parts(residual):
        if part.area > 1e-8:
            nearest = int(line_tree.nearest(part.representative_point()))
            add(part, [road_ids[nearest]], 'network/'+city,
                'Inferred nearest ground centerline for rounded fill; shared network ownership unchanged')
            fill_count += 1
    error = authoritative.symmetric_difference(unary_union(masks)).area
    if error > .001:
        raise ValueError(f'Grading footprint mismatch {error} m2')
    write_json(output/'road-profiles.geojson', {'type': 'FeatureCollection', 'features': features})
    write_scenes(city, source, output)
    manifest = {'city': city, 'input_sha256': inputs, 'grading_source_sha256': digest(output/'road-profiles.geojson'),
        'native_lod': 16, 'tile_size': 257, 'min_level': 16, 'max_data_level': 16, 'max_level': None,
        'profile_sample_spacing_m': 10, 'profile_smoothing_m': 30, 'base_elevation_resolution_m': 5,
        'shoulder_m': 5, 'paired_features': len(features), 'inferred_fill_features': fill_count,
        'junction_transition_m': 30, 'junction_policy': 'Common plane fitted to centerline terrain samples',
        'footprint_area_m2': authoritative.area, 'footprint_symmetric_difference_m2': error,
        'elevation_source': 'ReadyMap TMS 116 / EGM96, converted by osgEarth to ellipsoid heights',
        'policy': 'Absolute sparse elevations at one native LOD; finer terrain uses parent grading samples',
        'limitations': ['Terrain-derived junction planes are inferred, not surveyed engineering grades',
            'Roads only; bridge, elevated ramp, tunnel, building and water grading deferred',
            'Shoulders can overlap nearby roads; finite native grids approximate footprint boundaries',
            'Rounded fill height association is inferred and retained separately from road ownership'],
        'generator_sha256': digest(Path(__file__))}
    if inputs != {name: digest(source/name) for name in inputs}:
        raise ValueError('Source changed during export')
    write_json(output/'manifest.json', manifest)
    print(json.dumps(manifest), flush=True)


def write_scenes(city, source, output):
    """Keep existing asphalt and paint, adding sparse road grading after the unmodified ReadyMap source."""
    base = ET.fromstring((source/'marking-comparison/mask-intersection-8.earth').read_text(encoding='utf-8'))
    for node in base.findall('OGRFeatures/url'):
        node.text = f'../render-fixture-{city}/render-contract/'+Path(node.text).name
    for layer in base.findall('{osgEarthPrestige}surface_splatting'):
        material = 'asphalt' if layer.get('name') == 'Prestige asphalt' else layer.get('name').split()[-1]
        values = layer.find('material')
        values.attrib.clear()
        values.attrib.update(color=f'../bridge-boston-congress/materials/{material}-color.png',
            packed=f'../bridge-boston-congress/materials/{material}-orm.png', layout='orm')
        for key in ('color', 'packed'):
            if not (output/values.get(key)).is_file():
                raise ValueError('Missing shared material')
    terrain = base.find('options/terrain')
    terrain.set('max_lod', '22')
    terrain.set('min_tile_range_factor', '7')
    terrain.set('screen_space_error', '8')
    ET.register_namespace('prestige', 'osgEarthPrestige')
    for enabled in (False, True):
        for view, pitch, distance, heading in [('overview', -65, 600, 0), ('close', -18, 100, 40),
            ('top', -90, 170, 0)]:
            tree = ET.fromstring(ET.tostring(base))
            title = ('Graded' if enabled else 'Original')+' '+city+' roads / centerline profiles'
            tree.set('name', title)
            source_layer = ET.SubElement(tree, 'OGRFeatures', name='Compiled road profiles')
            ET.SubElement(source_layer, 'url').text = 'road-profiles.geojson'
            grading = ET.SubElement(tree, '{osgEarthPrestige}grading', name='Single-LOD road grading',
                features='Compiled road profiles', open='true' if enabled else 'false')
            settings = {'road_footprints': 'true', 'min_level': 16, 'max_data_level': 16, 'tile_size': 257,
                'junction': '[grading_junction]', 'profile_id_attribute': 'source_way_ids', 'junction_transition': 30,
                'line_width': '[profile_search_width_m]', 'buffer_width': 5, 'max_line_width': 500,
                'max_buffer_width': 5, 'sample_spacing': 10, 'smoothing_distance': 30, 'elevation_resolution': 5}
            for name, value in settings.items():
                ET.SubElement(grading, name).text = str(value)
            camera = tree.find('viewpoints/viewpoint')
            camera.set('name', title)
            camera.set('pitch', str(pitch))
            camera.set('range', str(distance))
            camera.set('heading', str(heading))
            mode = 'after' if enabled else 'before'
            (output/f'{mode}-{view}.earth').write_text(ET.tostring(tree, encoding='unicode'), encoding='utf-8')


def main():
    """Publish selected fixed fixtures without recompiling or modifying any retained geometry."""
    faulthandler.dump_traceback_later(120, repeat=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cities', nargs='+', choices=city_fixtures.FIXTURES)
    for city in parser.parse_args().cities:
        publish(city)


if __name__ == '__main__':
    main()
