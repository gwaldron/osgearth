"""Publish render ownership from retained compiler construction domains without changing the vector baseline."""

import argparse
import hashlib
import json
from pathlib import Path
import pickle

import shapely
from shapely.geometry import box, mapping, shape
from shapely.ops import unary_union

import city_fixtures
import compile as compiler
import raster_roads as planar


def digest(path):
    """Identify immutable source artifacts independently of file timestamps."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def identifier(geometry):
    """Identify one normalized metric shape within a specific compiler revision; editing shape changes its ID."""
    return hashlib.sha256(shapely.normalize(geometry).wkb).hexdigest()[:24]


def publish(city):
    """Partition existing material polygons by compiler-owned junction domains, retaining explicit network remainder."""
    fixture = city_fixtures.configure(city)
    source = city_fixtures.ROOT/'output'/('render-fixture-'+city)
    output = source/'render-contract'
    output.mkdir(exist_ok=True)
    if (source/'build-in-progress.json').exists():
        raise ValueError('Fixture compilation is incomplete')
    manifest = json.loads((source/'manifest.json').read_text(encoding='utf-8'))
    if digest(Path(manifest['source_osm'])) != manifest['source_sha256']:
        raise ValueError('Fixture source has changed; rebuild its vectors first')
    inputs = ['manifest.json', 'resolved.pkl', 'source-road-evidence.json', 'structures.geojson',
              'road-topology-edges.geojson', 'road-topology-nodes.geojson', 'lane-connectivity.geojson',
              *[name+'.geojson' for name in planar.PALETTE]]
    hashes = {name: digest(source/name) for name in inputs}
    with (source/'resolved.pkl').open('rb') as stream:
        roads, _, _, _, transitions, layouts = pickle.load(stream)
    region, receiver = box(-1000, -1000, 1000, 1000), 'terrain/'+city
    domains, owners = [], []
    occupied = shapely.Polygon()
    for layout in sorted(layouts, key=lambda j: j['name']):
        nodes = sorted(set(layout.get('cluster_nodes', [layout['node']])))
        sid = 'junction/'+'-'.join(str(n) for n in nodes)
        domain = layout['domain'].intersection(region).difference(occupied)
        if domain.is_empty:
            continue
        domains.append(domain)
        occupied = occupied.union(domain)
        owners.append({'surface_id': sid, 'receiver_id': receiver, 'kind': 'resolved_ground_junction',
                       'source_way_ids': sorted({str(a['road']['id']) for a in layout['approaches']}),
                       'source_node_ids': [str(n) for n in nodes], 'layout_name': layout['name'],
                       'source_restrictions': layout['restrictions'], 'vertical_profile': None,
                       'ownership_basis': 'Accepted compiler junction domain; shared by all its approaches'})
    # The compiler's rounded corridor union includes fill outside individual source way buffers.
    # Keep that shared network as one explicit owner; nearest-way assignment would invent provenance.
    domains.append(region.difference(occupied))
    owners.append({'surface_id': 'network/'+city, 'receiver_id': receiver, 'kind': 'shared_ground_network',
                   'source_way_ids': sorted(str(r['id']) for r in roads), 'vertical_profile': None,
                   'ownership_basis': 'Compiler corridor union and rounded fill outside accepted junction domains',
                   'limitation': 'Individual corridor, unresolved junction, and transition ownership not subdivided',
                   'transition_count': len(transitions)})
    tree = shapely.STRtree(domains)
    counts, errors = {}, {}
    for material in planar.PALETTE:
        original = json.loads((source/(material+'.geojson')).read_text(encoding='utf-8'))
        features, baseline, actual = [], [], []
        for feature in original['features']:
            polygon = shapely.transform(shape(feature['geometry']), planar.TO_LOCAL.transform, interleaved=False)
            baseline.append(polygon)
            parent = material+'/'+identifier(polygon)
            for index in tree.query(polygon, predicate='intersects'):
                for fragment in compiler.parts(polygon.intersection(domains[index])):
                    if fragment.area <= 1e-12:
                        continue
                    owner = owners[index]
                    sid = owner['surface_id']
                    geographic = shapely.transform(fragment, planar.TO_GEO.transform, interleaved=False)
                    if not geographic.is_valid:
                        geographic = shapely.make_valid(geographic)
                    for part in compiler.parts(geographic):
                        fid = sid+'/'+material+'/'+identifier(part)
                        features.append({'type': 'Feature', 'id': fid, 'geometry': mapping(part), 'properties': {
                            'feature_id': fid, 'parent_feature_id': parent, 'surface_id': sid,
                            'receiver_id': receiver, 'material': material,
                            'vertical_profile': None, 'elevation_m': None,
                            'inference': 'Compiled shape; material identity does not imply a mapped marking',
                            'evidence_ref': '../source-road-evidence.json'}})
                        actual.append(shapely.make_valid(shapely.transform(
                            part, planar.TO_LOCAL.transform, interleaved=False)))
        error = unary_union(baseline).symmetric_difference(unary_union(actual)).area
        if error > .001 or len({f['id'] for f in features}) != len(features):
            raise ValueError(f'{city} {material}: coverage error {error} or duplicate feature IDs')
        errors[material], counts[material] = error, len(features)
        compiler.write_json(output/(material+'.geojson'), {'type': 'FeatureCollection', 'features': features})
    compiler.write_json(output/'ownership-domains.geojson', {'type': 'FeatureCollection', 'features': [
        {'type': 'Feature', 'id': owner['surface_id'], 'properties': owner,
         'geometry': mapping(shapely.transform(domain, planar.TO_GEO.transform, interleaved=False))}
        for owner, domain in zip(owners, domains)]})
    structure_package = json.loads((source/'structures.geojson').read_text(encoding='utf-8'))
    structure_ids = sorted({f['properties']['surface_id'] for f in structure_package['features']})
    contract = {'schema': 'prestige-road-render-contract/1', 'fixture': city,
                'input_sha256': hashes, 'publisher_sha256': digest(Path(__file__)),
                'geometry_policy': 'Baseline retained verbatim; attributed copies split at compiler ownership domains',
                'coordinates': 'WGS84 longitude,latitude degrees; all render geometry is 2D',
                'construction_frame': manifest['metric_crs'],
                'material_frame': {'kind': 'WGS84 east-north-up', 'anchor': [*fixture['origin'], 0], 'units': 'meters'},
                'vertical_policy': 'Ground uses terrain receiver; structures retain unknown heights; layer is not meters',
                'ground_receiver_id': receiver, 'owners': owners, 'structure_surface_ids': structure_ids,
                'structure_file': '../structures.geojson', 'source_topology_edges': '../road-topology-edges.geojson',
                'source_topology_nodes': '../road-topology-nodes.geojson',
                'resolved_movements': '../lane-connectivity.geojson',
                'connectivity_policy': 'Only source node identity creates incidence; only resolved layouts assert movements',
                'marking_evidence_files': ['../'+p.name for p in sorted(source.glob('marking-evidence-*.json'))],
                'feature_identity': 'Surface IDs use source node sets or fixed fixture namespace; shape IDs are revision-local',
                'material_counts': counts, 'symmetric_difference_m2': errors,
                'limitations': ['Shared network owner is not a solved corridor or lane graph',
                                'Per-paint source/inference links remain in evidence files, not one-to-one feature links',
                                'Structure profiles, tunnel rendering, and grading integration remain deferred']}
    if hashes != {name: digest(source/name) for name in inputs}:
        raise ValueError('Fixture changed during ownership export')
    compiler.write_json(output/'contract.json', contract)
    print(city, len(owners), counts, errors, flush=True)


def main():
    """Generate attributed copies of the three fixed fixtures without rebuilding or altering their baseline geometry."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cities', nargs='+', choices=city_fixtures.FIXTURES)
    for city in parser.parse_args().cities:
        publish(city)


if __name__ == '__main__':
    main()
