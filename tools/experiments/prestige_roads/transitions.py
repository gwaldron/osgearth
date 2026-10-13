"""Compile source-indicated single outer-lane drops as continuous longitudinal pavement and lane geometry."""

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon, mapping
from shapely.ops import substring, unary_union

from intersections import LayoutError, dashed, frame, straight_arrow


def ease(value):
    """Return a clamped quintic blend with zero endpoint first and second derivatives."""
    t = np.clip(value, 0, 1)
    return t*t*t*(10-15*t+6*t*t)


def sample_frame(line, stations):
    """Sample source centers and consistent normals at metric stations; reject repeated/degenerate tangents."""
    frames = [frame(line, s) for s in stations]
    return np.array([f[0] for f in frames]), np.array([f[2] for f in frames])


def build_drop(source, target, chain, roads, region):
    """Build one evidenced lane-count decrease; return proposed footprints without mutating source roads."""
    count = source['lanes']
    turns = source['tags']['turn:lanes'].split('|')
    right = turns[-1] == 'merge_to_left' and all(t in ('', 'through') for t in turns[:-1])
    left = turns[0] == 'merge_to_right' and all(t in ('', 'through') for t in turns[1:])
    if len(turns) != count or not (right ^ left) or target['lanes'] != count-1:
        raise LayoutError('Only a single explicitly indicated outer-lane drop with matching counts is supported')
    if target['width'] >= source['width'] or target['width']/(count-1) < 2.5:
        raise LayoutError('Lane-drop widths contradict the supported narrowing carriageway policy')
    if any(r['bridge'] or r['tunnel'] or r['tags'].get('layer', '0') != '0' for r in (source, target)):
        raise LayoutError('Taper crossing a structure/level boundary needs deck ownership')
    if any('conditional' in k or k.startswith(('access', 'vehicle', 'change:lanes'))
           for r in (source, target) for k in r['tags']):
        raise LayoutError('Qualified lane-change/access policy is unresolved')
    length = min(180.0, source['line'].length*.75)
    if length < 50 or target['line'].length < 30:
        raise LayoutError('Insufficient longitudinal extent for this inferred taper policy')
    start = source['line'].length-length
    route = LineString([*substring(source['line'], start, source['line'].length).coords,
                        *list(substring(target['line'], 0, 20).coords)[1:]])
    if not region.buffer(-1).covers(route.buffer(max(source['width'], target['width'])/2)):
        raise LayoutError('Complete taper and exit collar are not inside the render crop')
    nodes = set(source['refs'][1:])
    if any(nodes.intersection(r['refs']) for r in roads if r['id'] not in (source['id'], target['id'])
           and r['tags']['highway'] != 'cycleway'):
        raise LayoutError('Branch inside taper requires joint junction/transition ownership')
    stations = np.linspace(0, route.length, max(257, int(route.length*2)+1))
    centers, normals = sample_frame(route, stations)
    blend = ease(stations/length)
    widths = source['width']+(target['width']-source['width'])*blend
    pitches = source['width']/count+(target['width']/(count-1)-source['width']/count)*blend
    side = 1 if right else -1
    outside = widths/2
    lanes, separators = [], []
    # Number geometrically from the continuing outside edge. Reflect this construction for a left-lane drop.
    for index in range(count-1):
        lateral = side*(outside-pitches*(index+.5))
        points = centers+normals*lateral[:, None]
        source_lane = index+1 if right else count-index
        target_lane = index+1 if right else count-1-index
        lanes.append({'from_lane': source_lane, 'to_lane': target_lane, 'points': points,
                      'line': LineString(points), 'role': 'continuing'})
        if index < count-2:
            boundary = centers+normals*(side*(outside-pitches*(index+1)))[:, None]
            separators.append(dashed(LineString(boundary), 3, 9))
    merge = ease((stations/length-.12)/.68)
    lateral = side*(outside-pitches*(count-1.5)-pitches*(1-merge))
    points = centers+normals*lateral[:, None]
    lanes.append({'from_lane': count if right else 1, 'to_lane': count-1 if right else 1,
                  'points': points, 'line': LineString(points), 'role': 'merging'})
    boundary = centers+normals*(side*(outside-pitches*(count-1)))[:, None]
    # End guidance before the residual lane is too narrow to occupy; taper geometry continues to its zero width.
    remaining = widths-pitches*(count-1)
    last = np.flatnonzero(remaining >= 1.8)
    if len(last) > 1:
        separators.append(dashed(LineString(boundary[:last[-1]+1]), 1.5, 4.5))
    arrow_station = length*.12
    index = int(np.argmin(abs(stations-arrow_station)))
    direction = centers[index+1]-centers[index-1]
    direction /= np.linalg.norm(direction)
    direction = direction+normals[index]*side*.45
    direction /= np.linalg.norm(direction)
    separators.append(straight_arrow(points[index], direction))
    left_edge = centers+normals*widths[:, None]/2
    right_edge = centers-normals*widths[:, None]/2
    footprint = Polygon(np.vstack([left_edge, right_edge[::-1]]))
    if not footprint.is_valid:
        raise LayoutError('Taper footprint self-intersects')
    updates = []
    for road in (source, target):
        s = np.linspace(0, road['line'].length, max(2, int(road['line'].length*2)+1))
        c, normal = sample_frame(road['line'], s)
        b = ease((s-start)/length) if road is source else np.ones(len(s))
        w = source['width']+(target['width']-source['width'])*b
        a, z = c+normal*w[:, None]/2, c-normal*w[:, None]/2
        updates.append((road, Polygon(np.vstack([a, z[::-1]])).intersection(region), LineString(a), LineString(z)))
    paint = unary_union(separators).intersection(footprint.buffer(-.15))
    return {'name': f'lane-drop-{source["id"]}', 'source': source['id'], 'target': target['id'],
            'chain': chain, 'start_station_m': start, 'taper_length_m': length, 'route': route,
            'domain': footprint.buffer(.1), 'footprint': footprint, 'lanes': lanes, 'paint': paint,
            'left_edge': LineString(left_edge), 'right_edge': LineString(right_edge), 'side': 'right' if right else 'left',
            'source_lanes': count, 'target_lanes': count-1, 'road_updates': updates,
            'assumptions': ['Source lane counts and merge direction are evidence; exact taper placement is inferred.',
                            'Use up to the last 180 m of the final ground approach, preserving its first quarter.',
                            'Center alignment follows OSM; pavement width and lane pitches ease continuously.',
                            'Merge path reaches the adjacent lane before taper completion; priority is not assigned.',
                            'Paint shape, shoulder allocation, and lane-change trajectory are illustrative.',
                            'No gore is invented for a lane ending without a mapped diverging branch.']}


def discover_transitions(roads, region):
    """Trace indicated one-way merge chains to a count decrease; report missing/ambiguous continuations explicitly."""
    layouts, diagnostics = [], []
    for road in sorted(roads, key=lambda r: r['id']):
        if 'merge_to_' not in road['tags'].get('turn:lanes', ''):
            continue
        record = {'source_way': road['id']}
        try:
            if road['tags'].get('oneway') != 'yes':
                raise LayoutError('Merge tracing currently requires source-order one-way travel')
            following = [r for r in roads if r['refs'][0] == road['refs'][-1]
                         and r['tags'].get('oneway') == 'yes' and r['id'] != road['id']]
            if len(following) != 1:
                raise LayoutError('Merge continuation is missing or branching in the selected source context')
            target = following[0]
            if target['lanes'] == road['lanes']:
                record.update(status='continued', next_way=target['id'])
            else:
                chain, current = [road['id']], road
                while True:
                    previous = [r for r in roads if r['refs'][-1] == current['refs'][0]
                                and r['tags'].get('oneway') == 'yes' and r['lanes'] == road['lanes']
                                and r['tags'].get('turn:lanes') == road['tags'].get('turn:lanes')]
                    if len(previous) != 1:
                        break
                    current = previous[0]
                    if current['id'] in chain:
                        raise LayoutError('Cyclic merge indication chain')
                    chain.insert(0, current['id'])
                layout = build_drop(road, target, chain, roads, region)
                if any(layout['domain'].intersects(other['domain']) for other in layouts):
                    raise LayoutError('Lane transition ownership domains overlap')
                # Commit proposed footprints only after ownership checks; rejected candidates leave no geometry behind.
                for updated, surface, left_edge, right_edge in layout.pop('road_updates'):
                    updated['surface'], updated['left_edge'], updated['right_edge'] = surface, left_edge, right_edge
                layouts.append(layout)
                record.update(status='resolved', next_way=target['id'], name=layout['name'], indication_chain=chain)
        except LayoutError as error:
            record.update(status='unresolved', reason=str(error))
        diagnostics.append(record)
    return layouts, diagnostics


def export_transition(layout, output, ground, to_geo, write_json):
    """Audit taper support and endpoint lane identity, then export its source evidence and actual rendered paths."""
    outside = [lane['line'].buffer(.9, cap_style=2).difference(ground.buffer(.003)).area for lane in layout['lanes']]
    if max(outside) > .01:
        raise LayoutError(f'Lane-drop car-width envelope leaves pavement: {max(outside)}')
    merging = next(lane for lane in layout['lanes'] if lane['role'] == 'merging')
    continuing = next(lane for lane in layout['lanes']
                      if lane['role'] == 'continuing' and lane['to_lane'] == merging['to_lane'])
    endpoint_error = float(np.linalg.norm(merging['points'][-1]-continuing['points'][-1]))
    if endpoint_error > 1e-6 or any(not lane['line'].is_simple for lane in layout['lanes']):
        raise LayoutError('Invalid lane-drop endpoint or self-intersecting path')
    records, features = [], []
    for lane in layout['lanes']:
        record = {k: lane[k] for k in ('from_lane', 'to_lane', 'role')}
        records.append(dict(record, xy_m=lane['points'].tolist()))
        geometry = shapely.transform(lane['line'], to_geo.transform, interleaved=False)
        features.append({'type': 'Feature', 'properties': record, 'geometry': mapping(geometry)})
    audit = {'lane_paths': len(records), 'max_car_width_envelope_outside_pavement_m2': max(outside),
             'merge_endpoint_error_m': endpoint_error, 'taper_length_m': layout['taper_length_m']}
    report = {k: layout[k] for k in ('name', 'source', 'target', 'chain', 'side', 'source_lanes', 'target_lanes',
                                    'start_station_m', 'taper_length_m', 'assumptions')}
    write_json(output/f'{layout["name"]}.json', dict(report, lane_paths=records, audit=audit))
    write_json(output/f'{layout["name"]}.geojson', {'type': 'FeatureCollection', 'features': features})
    return audit
