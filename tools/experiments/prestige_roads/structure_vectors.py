"""Export road structures and their source topology without requiring or inventing elevations."""

import argparse
import collections
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path

import numpy as np
import shapely
from shapely.geometry import LineString, Point, box, mapping
from shapely.ops import unary_union

import compile as compiler
from semantics import oneway_value


def nonzero_level(value):
    """Recognize nonzero or uninterpreted level evidence conservatively; absence is not a measured zero."""
    if value is None:
        return False
    try:
        values = [float(part.strip()) for part in value.split(';')]
        return any(not math.isfinite(v) or v != 0 for v in values)
    except ValueError:
        return True


def separate_structure(road):
    """Keep bridges and non-ground level evidence out of planar unions without assigning a support type or height."""
    tags = road['tags']
    return road['bridge'] or nonzero_level(tags.get('layer')) or nonzero_level(tags.get('level'))


def attributes(road):
    """Retain source tags separately from inferred dimensions; unknown heights and grades remain JSON null."""
    tags = road['tags']
    ramp = tags['highway'].endswith('_link')
    kind = 'elevated_ramp' if road['bridge'] and ramp else 'bridge' if road['bridge'] else 'layered_road'
    return {'road_id': str(road['id']), 'surface_id': f'way/{road["id"]}', 'source_version': road['version'],
            'structure_kind': kind, 'is_link_road': ramp, 'bridge': tags.get('bridge'), 'tunnel': tags.get('tunnel'),
            'layer': tags.get('layer'), 'level': tags.get('level'), 'incline': tags.get('incline'),
            'source_tags': tags, 'source_node_ids': [str(n) for n in road['refs']],
            'width_m': road['width'], 'width_basis': road['width_reason'],
            'lanes': road['lanes'], 'lanes_basis': road['lane_count_reason'],
            'height_status': 'unresolved', 'elevation_m': None, 'grade_percent': None,
            'vertical_profile': None, 'vertical_datum': None,
            'level_policy': 'Source layer/level retained verbatim; neither is converted into meters',
            'geometry_status': 'per-way footprint; elevated junction layout not resolved'}


def source_topology(roads, xy, to_geo):
    """Split source ways at shared nodes, retaining directed incidence without claiming legal turn movements."""
    owners = collections.defaultdict(set)
    for road in roads:
        for node in road['refs']:
            owners[node].add(road['id'])
    edges, nodes, by_road = [], {}, collections.defaultdict(list)
    for road in roads:
        refs = road['refs']
        cuts = [i for i, n in enumerate(refs) if i in (0, len(refs)-1) or len(owners[n]) > 1]
        direction = oneway_value(road['tags'])
        supported = direction in ('yes', '1', 'true', '-1', 'no', '0', 'false')
        forward = supported and direction != '-1'
        backward = supported and direction in ('no', '0', 'false', '-1')
        for start, end in zip(cuts, cuts[1:]):
            coordinates = [xy[n] for n in refs[start:end+1]]
            if LineString(coordinates).length < 1e-8:
                continue
            identifier = f'way/{road["id"]}/{start}-{end}'
            by_road[road['id']].append(identifier)
            a, b = str(refs[start]), str(refs[end])
            props = {'edge_id': identifier, 'road_id': str(road['id']), 'from_node': a, 'to_node': b,
                     'forward': forward, 'backward': backward, 'direction_status': 'decoded' if supported else 'unknown',
                     'layer': road['tags'].get('layer'), 'level': road['tags'].get('level'),
                     'bridge': road['tags'].get('bridge'), 'tunnel': road['tags'].get('tunnel'),
                     'source_node_ids': [str(n) for n in refs[start:end+1]],
                     'access_tags': {k: v for k, v in road['tags'].items() if any(
                         k == key or k.startswith(key+':') for key in
                         ('access', 'vehicle', 'motor_vehicle', 'motorcar', 'bus', 'bicycle', 'foot', 'oneway'))}}
            geometry = shapely.transform(LineString(coordinates), to_geo.transform, interleaved=False)
            edges.append({'type': 'Feature', 'id': identifier, 'properties': props, 'geometry': mapping(geometry)})
            for node, incoming, outgoing in ((a, backward, forward), (b, forward, backward)):
                entry = nodes.setdefault(node, {'node_id': node, 'incident_edges': [],
                                               'incoming_edges': [], 'outgoing_edges': []})
                entry['incident_edges'].append(identifier)
                if incoming:
                    entry['incoming_edges'].append(identifier)
                if outgoing:
                    entry['outgoing_edges'].append(identifier)
    features = []
    for node, props in nodes.items():
        props['connectivity_basis'] = 'Shared OSM node identity; incidence does not authorize a turn or access'
        props['context_complete'] = False
        features.append({'type': 'Feature', 'id': node, 'properties': props,
                         'geometry': {'type': 'Point', 'coordinates': list(to_geo.transform(*xy[int(node)]))}})
    return edges, features, by_road


def export_structures(region, output, source):
    """Re-read bounded OSM, export separate 2D structure materials/topology, and fail on invalid geometry."""
    from urban import street_paint
    roads, xy, flags, relations = compiler.read_source(source)
    edges, nodes, by_road = source_topology(roads, xy, compiler.TO_GEO)
    selected = [r for r in roads if not r['tunnel'] and separate_structure(r)]
    # Reuse existing cross-sections/width handoffs. Flat temporary samples produce XY only, never exported heights.
    original_terrain = compiler.TERRAIN
    try:
        compiler.TERRAIN = lambda x, y: np.zeros(np.broadcast_arrays(x, y)[0].shape)
        compiler.solve_profiles(roads, xy)
    finally:
        compiler.TERRAIN = original_terrain
    branch_nodes = [Point(xy[int(f['id'])]).buffer(12) for f in nodes if len(f['properties']['incident_edges']) > 2]
    exclusions = unary_union(branch_nodes)
    features, road_count, outside = [], 0, 0.0
    for road in selected:
        pavement = road['surface'].intersection(region)
        if pavement.is_empty or pavement.area < 1e-8:
            continue
        road_count += 1
        props = attributes(road)
        props['topology_edge_ids'] = by_road[road['id']]
        # The stripe generator normally skips bridges; this copy only enables its existing planar lane policy.
        paint_road = dict(road, bridge=False)
        locale = compiler.SCENES[compiler.SCENE]
        white, yellow = street_paint([paint_road], exclusions, road['surface'],
                                    locale['driving_side'], locale['center_line'])
        details = {'asphalt': pavement, 'white': white.intersection(pavement), 'yellow': yellow.intersection(pavement)}
        for material, geometry in details.items():
            if not geometry.is_valid:
                raise ValueError(f'Invalid structure {road["id"]} {material}')
            if material != 'asphalt':
                outside += geometry.difference(pavement).area
            for index, polygon in enumerate(compiler.parts(shapely.segmentize(geometry, 5))):
                geographic = shapely.transform(polygon, compiler.TO_GEO.transform, interleaved=False)
                if not geographic.is_valid:
                    raise ValueError(f'Invalid projected structure {road["id"]} {material}')
                features.append({'type': 'Feature', 'id': f'way/{road["id"]}/{material}/{index}',
                                 'properties': dict(props, material=material,
                                     marking_basis='Existing lane schema; dimensions inferred; branch areas suppressed'),
                                 'geometry': mapping(geographic)})
    if outside > .001:
        raise ValueError('Structure paint lies outside its own pavement')
    for name, content in [('structures', features), ('road-topology-edges', edges), ('road-topology-nodes', nodes)]:
        compiler.write_json(output/f'{name}.geojson', {'type': 'FeatureCollection', 'features': content})
    compiler.write_json(output/'structure-source-evidence.json', {
        'roads': [attributes(r) for r in selected], 'width_flags': flags, 'relations': relations,
        'tunnels': [{'road_id': str(r['id']), 'source_tags': r['tags'], 'geometry_status': 'not_exported',
                     'topology_edge_ids': by_road[r['id']]} for r in roads if r['tunnel']],
        'topology_scope': 'Complete selected ways within context; ways outside selection may be absent',
        'inference_policy': 'No height or grade inference in this export; source incline/ele tags remain evidence',
        'movement_policy': 'Source restrictions/connectivity relations retained; elevated legal turns not solved'})
    return {'roads': road_count, 'features': len(features), 'topology_edges': len(edges), 'topology_nodes': len(nodes),
            'materials': dict(collections.Counter(f['properties']['material'] for f in features)),
            'paint_outside_pavement_m2': outside, 'height_status': 'unresolved', 'coordinates': '2D WGS84 lon/lat'}


def main():
    """Export a bounded structure-only Boston sample from source, recording provenance without terrain or meshes."""
    import raster_roads as planar
    from vector_patch import fingerprint
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--center', nargs=2, type=float, default=[-71.058, 42.3455], metavar=('LON', 'LAT'))
    parser.add_argument('--size', type=float, default=1200)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent/'output/boston-structure-ramps')
    args = parser.parse_args()
    if not math.isfinite(args.size) or not 10 <= args.size <= 3000:
        raise ValueError('Choose a bounded sample between 10 and 3000 meters across')
    center = np.array(planar.TO_LOCAL.transform(*args.center))
    region = box(*(center-args.size/2), *(center+args.size/2))
    if not planar.REGION.covers(region.buffer(250)):
        raise ValueError('Requested sample exceeds the retained Boston source context')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    compiler.write_json(output/'build-in-progress.json', {'status': 'incomplete'})
    sources = fingerprint()
    planar.configure_compiler(region.buffer(250, join_style=2))
    report = export_structures(region, output, planar.SOURCE)
    if sources != fingerprint():
        raise ValueError('Compiler changed during export')
    compiler.write_json(output/'manifest.json', dict(report, center=args.center, size_m=args.size,
        finished_utc=datetime.now(timezone.utc).isoformat(), source_osm=str(planar.SOURCE),
        source_sha256=hashlib.sha256(planar.SOURCE.read_bytes()).hexdigest(), compiler_sha256=sources,
        policy='Fresh separate structure vectors and source topology; no elevation inference or tunnel surfaces'))
    (output/'build-in-progress.json').unlink()
    print(report, flush=True)


if __name__ == '__main__':
    main()
