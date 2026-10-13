"""Build three metric road-rendering fixtures from retained source OSM, including separate bridge vectors."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from pyproj import CRS, Transformer
import shapely
from shapely.geometry import Point, box, mapping, shape
from shapely.ops import unary_union

import compile as compiler
from fetch_source import download_region
import raster_roads as planar
from structure_vectors import export_structures
from vector_patch import fingerprint, fresh_planar_compile, ring_path, write_views


ROOT = Path(__file__).resolve().parent
FIXTURES = {
    'boston': {'title': 'Central Boston', 'origin': (-71.0565, 42.3515), 'locale': 'boston-region',
               'source': 'boston-metro.osm.gz', 'preferred_bridge': 'Congress',
               'purpose': 'Downtown junctions, Fort Point bridges, South Station elevated highway ramps'},
    'nyc': {'title': 'Central NYC - Lower Manhattan', 'origin': (-73.995, 40.7095), 'locale': 'manhattan',
            'source': 'nyc-render-fixture.osm.gz', 'preferred_bridge': 'Brooklyn Bridge',
            'purpose': 'Lower Manhattan and Brooklyn approaches, East River bridge decks and stacked roads'},
    'london': {'title': 'Central London', 'origin': (-.113, 51.506), 'locale': 'london',
               'source': 'london-render-fixture.osm.gz', 'preferred_bridge': 'Waterloo Bridge',
               'purpose': 'Westminster, Waterloo and Blackfriars bridges; left-hand urban junctions'}
}
SIZE_M = 2000.0
SOURCE_HALO_M = 500.0
REGION = box(-SIZE_M/2, -SIZE_M/2, SIZE_M/2, SIZE_M/2)


def configure(name):
    """Select one local metric frame and locale, clearing process-local caches before compiling a fixture."""
    fixture = FIXTURES[name]
    planar.ORIGIN = fixture['origin']
    planar.LOCAL = CRS.from_proj4(
        f'+proj=aeqd +lat_0={planar.ORIGIN[1]} +lon_0={planar.ORIGIN[0]} +datum=WGS84 +units=m')
    planar.TO_LOCAL = Transformer.from_crs('EPSG:4326', planar.LOCAL, always_xy=True)
    planar.TO_GEO = Transformer.from_crs(planar.LOCAL, 'EPSG:4326', always_xy=True)
    planar.REGION = REGION
    planar.GEO_REGION = geographic(REGION)
    planar.SOURCE = ROOT/'source'/fixture['source']
    planar.SCENE_TEMPLATE = fixture['locale']
    planar.SCENE_NAME = name+'-render-fixture'
    planar.PAINT_NETWORK = planar.MATERIAL_CONTEXT = None
    compiler.SCENES[planar.SCENE_NAME] = dict(compiler.SCENES[fixture['locale']])
    # The same supported road classes participate in each fixture; country marking defaults remain distinct.
    compiler.SCENES[planar.SCENE_NAME]['additional_classes'] = ['service', 'tertiary_link', 'living_street']
    planar.SCENE_TEMPLATE = planar.SCENE_NAME
    planar.configure_compiler(REGION.buffer(250, join_style=2))
    return fixture


def geographic(geometry):
    """Densify metric boundaries before geographic export so a two-kilometer square stays accurately represented."""
    return shapely.transform(shapely.segmentize(geometry, 25), planar.TO_GEO.transform, interleaved=False)


def fetch(name):
    """Retain bounded original map-API data once; existing snapshots are never silently refreshed or replaced."""
    fixture = configure(name)
    path = planar.SOURCE
    if path.exists():
        print(f'{name}: using retained source OSM {path.name}', flush=True)
        return
    bounds = geographic(REGION.buffer(SOURCE_HALO_M, join_style=2)).bounds
    root, requests = download_region(bounds, 3)
    for element in root:
        for key in ('user', 'uid', 'changeset'):
            element.attrib.pop(key, None)
    nodes = {element.get('id') for element in root.findall('node')}
    missing = [(way.get('id'), node.get('ref')) for way in root.findall('way')
               for node in way.findall('nd') if node.get('ref') not in nodes]
    if missing:
        raise ValueError(f'Map-API snapshot has missing way nodes: {missing[:10]}')
    payload = gzip.compress(ET.tostring(root, encoding='utf-8', xml_declaration=True), mtime=0)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)
    compiler.write_json(path.with_suffix('.json'), {
        'fixture': name, 'title': fixture['title'], 'requests': requests,
        'source_bbox': bounds, 'render_origin': fixture['origin'], 'render_size_m': [SIZE_M, SIZE_M],
        'context_m': SOURCE_HALO_M, 'sha256': hashlib.sha256(payload).hexdigest(),
        'nodes': len(nodes), 'ways': len(root.findall('way')), 'relations': len(root.findall('relation')),
        'missing_way_nodes': 0, 'retrieved_utc': datetime.now(timezone.utc).isoformat(),
        'attribution': 'OpenStreetMap contributors', 'license': 'https://www.openstreetmap.org/copyright',
        'policy': ('Nine bounded map-API requests, complete returned ways. Newest object versions merged; '
                   'not an atomic snapshot. Parent relations retained; members outside the source may be absent.')})
    print(f'{name}: retained {len(nodes):,} nodes and {len(root.findall("way")):,} ways', flush=True)


def source_inventory(output):
    """Catalog actual in-AOI bridges and incomplete relation evidence before emitting any fixture success marker."""
    roads, xy, _, relations = compiler.read_source(planar.SOURCE)
    bridges = []
    for road in roads:
        clipped = road['line'].intersection(REGION)
        if not road['bridge'] or road['tunnel'] or clipped.length < .01:
            continue
        midpoint = clipped.interpolate(.5, normalized=True)
        bridges.append({'road_id': str(road['id']), 'name': road['tags'].get('name'),
                        'bridge_name': road['tags'].get('bridge:name'), 'highway': road['tags']['highway'],
                        'layer': road['tags'].get('layer'), 'bridge': road['tags']['bridge'],
                        'length_in_aoi_m': clipped.length,
                        'source_way_fully_inside': REGION.covers(road['line']),
                        'midpoint': list(planar.TO_GEO.transform(midpoint.x, midpoint.y))})
    if not bridges:
        raise ValueError('A rendering fixture must contain a source-tagged road bridge')
    with gzip.open(planar.SOURCE, 'rb') as stream:
        root = ET.parse(stream).getroot()
    available = {(element.tag, element.get('id')) for element in root}
    missing = [{'relation_id': relation['id'], 'missing_members': [m for m in relation['members']
                if (m['type'], m['ref']) not in available]} for relation in relations]
    missing = [record for record in missing if record['missing_members']]
    compiler.write_json(output/'source-inventory.json', {
        'bridges': bridges, 'incomplete_relations': missing,
        'policy': 'Inventory is source evidence; bridge heights and legal elevated turn movements are unresolved.'})
    return bridges, missing


def cameras(fixture, layouts, bridges):
    """Choose repeatable source-based plan views; record all camera poses rather than relying on viewer state."""
    candidates = [j for j in layouts if REGION.buffer(-120).covers(Point(j['position']))]
    if not candidates:
        raise ValueError('Fixture has no resolved interior junction for the marking comparison')
    junction = max(candidates, key=lambda j: (j.get('arm_count', 0) == 4, j.get('arrow_count', 0), j['name']))
    named = [b for b in bridges if fixture['preferred_bridge'].casefold() in
             ((b['name'] or '')+' '+(b['bridge_name'] or '')).casefold()]
    bridge = max(named or bridges, key=lambda b: b['length_in_aoi_m'])
    poses = {'overview': (*fixture['origin'], 3600),
             'intersection': (*planar.TO_GEO.transform(*junction['position']), 200),
             'bridge': (*bridge['midpoint'], min(1800, max(350, bridge['length_in_aoi_m']*1.5)))}
    return poses, {'junction': junction['name'], 'bridge_road_id': bridge['road_id'],
                   'bridge_name': bridge['bridge_name'] or bridge['name']}


def structure_preview(output, poses):
    """Add distinctly colored structure footprints to flat diagnostic views without assigning physical heights."""
    package = json.loads((output/'structures.geojson').read_text(encoding='utf-8'))
    models, paths = [], []
    for index, (material, color) in enumerate((('asphalt', '#496879'), ('yellow', '#e7bc2e'), ('white', '#e5e3d8'))):
        features = [feature for feature in package['features'] if feature['properties']['material'] == material]
        filename = f'preview-structure-{material}.geojson'
        compiler.write_json(output/filename, {'type': 'FeatureCollection', 'features': features})
        paths.append(f'<g fill="{color}" fill-rule="evenodd">')
        for feature in features:
            polygon = shapely.transform(shape(feature['geometry']), planar.TO_LOCAL.transform, interleaved=False)
            paths.append(f'<path d="{ring_path(polygon, np.zeros(2))}"/>')
        paths.append('</g>')
        models.append(f'''
    <OGRFeatures name="structure-{material}-source"><url>{filename}</url></OGRFeatures>
    <FeatureModel name="Structure {material} / 2D diagnostic" features="structure-{material}-source">
        <styles><style type="text/css">default {{
            fill: {color}; altitude-offset: {2+index*.02}; render-lighting: false;
            render-depth-test: false; render-order: {10+index};
        }}</style></styles>
    </FeatureModel>''')
    for view in poses:
        source = (output/f'{view}.earth').read_text(encoding='utf-8')
        source = source.replace('road vectors /', 'road vectors / blue structures are 2D footprints /')
        source = source.replace('    <viewpoints', ''.join(models)+'\n    <viewpoints')
        (output/f'{view}.earth').write_text(source, encoding='utf-8', newline='\r\n')
        svg = (output/f'{view}.svg').read_text(encoding='utf-8').replace('</svg>', ''.join(paths)+'</svg>')
        (output/f'{view}.svg').write_text(svg, encoding='utf-8')


def build(name):
    """Compile all vectors afresh, validate the crop/materials, and publish a manifest only after successful checks."""
    fixture = configure(name)
    if not planar.SOURCE.is_file():
        raise ValueError(f'Missing {planar.SOURCE.name}; run the fetch command first')
    output = ROOT/'output'/('render-fixture-'+name)
    output.mkdir(parents=True, exist_ok=True)
    pending = output/'build-in-progress.json'
    compiler.write_json(pending, {'status': 'incomplete'})
    started = time.perf_counter()
    source_hashes = fingerprint()
    source_hash = hashlib.sha256(planar.SOURCE.read_bytes()).hexdigest()
    bridges, incomplete = source_inventory(output)
    compiler.write_json(output/'aoi.geojson', {'type': 'FeatureCollection', 'features': [
        {'type': 'Feature', 'properties': {'fixture': name, 'width_m': SIZE_M, 'height_m': SIZE_M,
                                         'crs_definition': planar.LOCAL.to_string()},
         'geometry': mapping(geographic(REGION))}]})
    layers, roads, layouts, cells = fresh_planar_compile(REGION, output)
    for material, geometries in layers.items():
        if not shapely.is_valid(geometries).all() or (material in ('asphalt', 'white') and not geometries):
            raise ValueError(f'Missing required or invalid {material} polygons')
    pavement = unary_union(layers['asphalt'])
    outside = {material: unary_union(layers[material]).difference(pavement).area
               for material in ('white', 'yellow', 'gutter')}
    crop_error = {material: unary_union(geometries).difference(REGION).area for material, geometries in layers.items()}
    if max([*outside.values(), *crop_error.values()]) > .001:
        raise ValueError(f'Material containment failure: {outside}, {crop_error}')
    poses, selected = cameras(fixture, layouts, bridges)
    exports = write_views(layers, np.zeros(2), SIZE_M, output, fixture['title'], poses)
    planar.configure_compiler(REGION.buffer(250, join_style=2))
    structures = export_structures(REGION, output, planar.SOURCE)
    structure_preview(output, poses)
    records = [json.loads((output/f'marking-evidence-{index:02d}.json').read_text()) for index in cells]
    discovery = json.loads((output/'layout-discovery.json').read_text())
    unresolved = Counter(d.get('reason', 'unspecified') for d in discovery['junctions'] if d['status'] != 'resolved')
    manifest = {
        'fixture_schema': 'prestige-roads-render-fixture/1', 'fixture': name, 'title': fixture['title'],
        'purpose': fixture['purpose'], 'finished_utc': datetime.now(timezone.utc).isoformat(),
        'compile_seconds': time.perf_counter()-started, 'source_osm': str(planar.SOURCE),
        'source_sha256': source_hash, 'compiler_sha256': source_hashes, 'reused_geometry_checkpoints': False,
        'origin': fixture['origin'], 'size_m': [SIZE_M, SIZE_M], 'area_m2': REGION.area,
        'metric_crs': planar.LOCAL.to_string(), 'geographic_bbox': planar.GEO_REGION.bounds,
        'compile_context_m': 250, 'driving_side': compiler.SCENES[compiler.SCENE]['driving_side'],
        'center_line': compiler.SCENES[compiler.SCENE]['center_line'], 'surface_ways_in_context': len(roads),
        'junction_layouts': len(layouts), 'lane_connections': sum(len(j['connectors']) for j in layouts),
        'materials': exports, 'structures': structures, 'source_bridge_ways_in_aoi': len(bridges),
        'crosswalks': sum(c['rendered'] for r in records for c in r['crossings']),
        'stop_bars': sum(len(c.get('bars', [])) for r in records for c in r['controls']),
        'outside_pavement_m2': outside, 'outside_crop_m2': crop_error,
        'guidance_intrusion_m2': max(r['guidance_intrusion_m2'] for r in records),
        'unresolved_junction_reasons': dict(unresolved), 'incomplete_source_relations': len(incomplete),
        'cameras': {view: {'longitude': p[0], 'latitude': p[1], 'range_m': p[2], 'heading': 0, 'pitch': -90}
                    for view, p in poses.items()}, 'camera_selection': selected,
        'preview': {'terrain': 'flat reference globe; no elevation or grading', 'lighting': False,
                    'structure_color': 'blue gray; overlapping levels are diagnostic only'},
        'limitations': ['Structure heights and elevated junctions unresolved; no tunnel surfaces',
                        'Ground polygons retain material-only attribution; final per-surface ownership is pending',
                        'Markings and widths include prototype inferences, not an exhaustive local traffic-rule model',
                        'Source graph incidence is not a legal turn graph; boundary context is finite']}
    if source_hashes != fingerprint() or source_hash != hashlib.sha256(planar.SOURCE.read_bytes()).hexdigest():
        raise ValueError('Source or compiler changed during compilation; rerun before viewing')
    compiler.write_json(output/'manifest.json', manifest)
    pending.unlink()
    print(json.dumps({k: manifest[k] for k in ('fixture', 'compile_seconds', 'junction_layouts', 'lane_connections',
                                              'source_bridge_ways_in_aoi', 'structures', 'crosswalks', 'stop_bars')},
                     indent=2), flush=True)


def main():
    """Fetch serialized bounded snapshots or build each requested fixture using the current compiler."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('fetch', 'compile', 'all'))
    parser.add_argument('cities', nargs='+', choices=FIXTURES)
    args = parser.parse_args()
    for name in args.cities:
        if args.operation in ('fetch', 'all'):
            fetch(name)
        if args.operation in ('compile', 'all'):
            build(name)


if __name__ == '__main__':
    main()
