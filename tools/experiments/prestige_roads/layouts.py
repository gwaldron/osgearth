"""Discover bounded junction layouts from topology; retain unsupported candidates with explicit reasons."""

import itertools
import math
from collections import defaultdict

import numpy as np
from shapely.geometry import LineString, Point
from shapely.ops import unary_union

from intersections import LayoutError, audit_layout, build_layout, frame


def physical_arms(roads, xy, node, junction_nodes):
    """Split incident ways locally at a shared node, stopping each arm at the next topological junction."""
    arms = []
    for road in sorted(roads, key=lambda r: r['id']):
        if node not in road['refs'] or road['tags']['highway'] == 'cycleway':
            continue
        if road['refs'].count(node) != 1:
            raise LayoutError('Repeated junction node requires roundabout/loop handling')
        index = road['refs'].index(node)
        for side, refs in [('before', road['refs'][:index+1][::-1]), ('after', road['refs'][index:])]:
            if len(refs) < 2:
                continue
            stop = next((i for i, n in enumerate(refs[1:], 1) if n in junction_nodes), len(refs)-1)
            refs = refs[:stop+1]
            arms.append({'road': road, 'key': f'{road["id"]}:{side}', 'side': side,
                         'outward': LineString([xy[n] for n in refs]), 'refs': refs})
    return arms


def two_way_approaches(arms, xy, node):
    """Build separate incoming/outgoing ports on right-hand-traffic two-lane arms; reject ambiguous lane schemas."""
    approaches = []
    for arm in arms:
        road, line = arm['road'], arm['outward']
        tags = road['tags']
        if road['bridge'] or road['tunnel'] or tags.get('layer', '0') != '0':
            raise LayoutError('Two-way layout needs one ground level')
        if tags.get('oneway', 'no') not in ('no', '0', 'false') or road['lanes'] != 2:
            raise LayoutError('Two-way case supports one lane each direction; divided transitions need another policy')
        if any('conditional' in k or k.startswith(('access', 'vehicle', 'motor_vehicle', 'lanes:both')) for k in tags):
            raise LayoutError('Qualified approach or shared turn lane requires explicit access/lane policy')
        if any(tags.get('lanes:'+d, '1') != '1' for d in ('forward', 'backward')):
            raise LayoutError('Directional lane counts do not agree with one lane each way')
        station = min(max(14, road['width']*2), line.length*.42)
        if station < 8:
            raise LayoutError('Two-way junction too close to the next junction for independent ownership')
        center, tangent, normal = frame(line, station)
        angle = math.atan2(center[1]-xy[node][1], center[0]-xy[node][0])
        mouth = {'center': center, 'normal': normal, 'width': road['width'], 'angle': angle}
        # Leave excess mapped width as unallocated shoulder/parking pavement, not an oversized driving lane.
        pitch = min(road['width']/2, 3.7)
        for incoming in (True, False):
            direction = -tangent if incoming else tangent
            left = np.array([-direction[1], direction[0]])
            position = center-left*pitch/2
            suffix = 'forward' if (arm['side'] == 'before') == incoming else 'backward'
            approaches.append({'way': arm['key']+(':'+suffix), 'arm': arm['key'], 'road': road, 'mouth': mouth,
                               'outward': line, 'incoming': incoming, 'center': position, 'tangent': tangent,
                               'normal': normal, 'direction': direction, 'left': left, 'lane_centers': [position],
                               'pitch': pitch, 'width': pitch, 'count': 1, 'station': station, 'angle': angle,
                               'travel': suffix})
    return approaches


def two_way_movements(approaches, relations, node):
    """Infer local turn opportunities while honoring simple restrictions; fail on unresolved qualified relations."""
    incoming = [a for a in approaches if a['incoming']]
    outgoing = [a for a in approaches if not a['incoming']]
    pairs = {(a['way'], b['way']) for a in incoming for b in outgoing if a['arm'] != b['arm']}
    original, evidence = set(pairs), []
    by_id = {a['way']: a for a in approaches}
    for relation in relations:
        members, tags = relation['members'], relation['tags']
        relevant = any(m['type'] == 'node' and m['role'] == 'via' and int(m['ref']) == node for m in members)
        if not relevant:
            ids = {a['road']['id'] for a in approaches}
            if any(m['type'] == 'way' and int(m['ref']) in ids for m in members):
                if any(m['role'] == 'via' and m['type'] == 'way' for m in members):
                    raise LayoutError('Relevant via-way restriction cannot be resolved locally')
            continue
        restriction = tags.get('restriction', '')
        if any('conditional' in k or k == 'except' or k.startswith('restriction:') for k in tags):
            raise LayoutError('Conditional/qualified restriction retained; no unconditional two-way layout generated')
        if restriction not in ('no_left_turn', 'no_right_turn', 'no_straight_on', 'no_u_turn',
                               'only_left_turn', 'only_right_turn', 'only_straight_on') or len(members) != 3:
            raise LayoutError('Unsupported restriction form at two-way junction')
        start = [int(m['ref']) for m in members if m['role'] == 'from' and m['type'] == 'way']
        end = [int(m['ref']) for m in members if m['role'] == 'to' and m['type'] == 'way']
        if len(start) != 1 or len(end) != 1:
            raise LayoutError('Restriction needs one from and one to way')
        a = [p for p in incoming if p['road']['id'] == start[0]]
        b = [p for p in outgoing if p['road']['id'] == end[0]]
        if len(a) != 1 or len(b) != 1:
            raise LayoutError('Restriction ambiguous after splitting an interior-node approach')
        pair = (a[0]['way'], b[0]['way'])
        if restriction.startswith('only_'):
            pairs = {p for p in pairs if p[0] != pair[0] or p == pair}
        else:
            pairs.discard(pair)
        evidence.append({'relation': relation['id'], 'from': start[0], 'to': end[0], 'restriction': restriction})
    assignments = []
    for start, end in sorted(pairs):
        source, target = by_id[start], by_id[end]
        a, b = source['direction'], target['direction']
        cross = float(a[0]*b[1]-a[1]*b[0])
        dot = float(source['direction'] @ target['direction'])
        angle = math.degrees(math.atan2(cross, dot))
        turn = 'through' if abs(angle) < 35 else ('left' if angle > 0 else 'right')
        tag = source['road']['tags'].get('turn:lanes:'+source['travel'])
        if tag is not None:
            if '|' in tag or any(v not in ('left', 'right', 'through', '') for v in tag.split(';')):
                raise LayoutError('Unsupported directional turn indication')
            if tag and turn not in tag.split(';'):
                continue
        assignments.append((source, 0, target, 0))
    permitted = {(a['way'], b['way']) for a, _, b, _ in assignments}
    return assignments, evidence, sorted(original-permitted)


def mouth_stations(arms, controls, kind):
    """Derive one-way mouth distances from width, nearby signals, and available source-way length."""
    stations = {}
    for arm in arms:
        road, line = arm['road'], arm['outward']
        incoming = (arm['side'] == 'after') == (road['tags'].get('oneway') == '-1')
        desired = max(8, road['width']*2)
        if kind == 'controlled_confluence' and not incoming:
            desired = max(desired, road['width']*4.5)
        if incoming:
            signals = [line.project(Point(c['position'])) for c in controls if road['id'] in c['roads']
                       and c['kind'] == 'traffic_signals'
                       and c['direction'] == ('backward' if road['tags'].get('oneway') == '-1' else 'forward')]
            if signals:
                desired = max(desired, min(signals)+6)
        cap = min(line.length-4.1, line.length*(.70 if incoming else .45))
        stations[road['id']] = min(desired, cap)
        if stations[road['id']] < 6:
            raise LayoutError('Approach too short for an independently owned junction')
    return stations


def discover_layouts(roads, xy, relations, controls, ground, islands, region):
    """Discover all in-crop vehicle junctions; accept audited layouts and retain structured rejection diagnostics."""
    incident = defaultdict(list)
    for road in roads:
        if road['tags']['highway'] != 'cycleway':
            for node in road['refs']:
                incident[node].append(road)
    nodes = {n for n, rs in incident.items()
             if sum(1 if n in (r['refs'][0], r['refs'][-1]) else 2 for r in rs) >= 3}
    candidates, diagnostics, layouts = [], [], []
    for node in sorted(nodes):
        if not region.covers(Point(xy[node])):
            continue
        record = {'node': node, 'source_ways': sorted(r['id'] for r in incident[node])}
        try:
            arms = physical_arms(roads, xy, node, nodes)
            oneway = all(a['road']['tags'].get('oneway') in ('yes', '1', 'true', '-1') for a in arms)
            relevant = [r for r in relations if any(m['role'] == 'via' and m['type'] == 'node'
                        and int(m['ref']) == node for m in r['members'])]
            kind = ('restricted' if relevant else 'controlled_confluence') if oneway else 'two_way'
            if oneway and any(node not in (a['road']['refs'][0], a['road']['refs'][-1]) for a in arms):
                raise LayoutError('Interior one-way branch needs a diverge/cluster policy')
            if any(a['road']['bridge'] or a['road']['tunnel'] for a in arms):
                raise LayoutError('Structure junction requires a separate deck layout')
            stations = mouth_stations(arms, controls, kind) if oneway else None
            ports = None if oneway else two_way_approaches(arms, xy, node)
            if kind == 'two_way':
                two_way_movements(ports, relations, node)
            record.update(kind=kind, arms=arms, stations=stations, ports=ports)
            candidates.append(record)
        except LayoutError as error:
            record.update(status='unresolved', reason=str(error))
            diagnostics.append(record)
    # Reserve a four-meter handoff collar when neighboring candidates share a short source way.
    demands = defaultdict(list)
    for candidate in candidates:
        if candidate['stations'] is not None:
            for arm in candidate['arms']:
                demands[arm['road']['id']].append((candidate, arm))
    for way, ends in demands.items():
        if len(ends) == 2:
            budget = ends[0][1]['road']['line'].length-4
            total = sum(c['stations'][way] for c, _ in ends)
            if total > budget:
                for candidate, _ in ends:
                    candidate['stations'][way] *= budget/total
    # Restriction-rich junctions take ownership first. This stable topology order does not name specific OSM IDs.
    candidates.sort(key=lambda r: (r['kind'] != 'restricted', -len(r['arms']), r['node']))
    for record in candidates:
        node, kind, stations = record['node'], record['kind'], record['stations']
        name = f'junction-{node}'
        try:
            resolved = build_layout(roads, xy, relations, ground, islands, node, stations, kind, controls,
                                    name, record['ports'])
            if not region.buffer(.001).covers(resolved['domain']):
                raise LayoutError('Junction ownership reaches beyond the render crop')
            if any(resolved['domain'].intersection(other['domain']).area > .001 for other in layouts):
                raise LayoutError('Independent junction domains overlap; cluster resolution required')
            audit_layout(resolved, resolved['pavement'])
            if kind == 'two_way':
                # Keep center paint on approach stems, outside the shared turning area; no guessed turn arrows.
                lines = [a['outward'].intersection(resolved['domain']).difference(Point(xy[node]).buffer(8))
                         for a in record['arms']]
                resolved['yellow_paint'] = unary_union(lines).buffer(.075)
            layouts.append(resolved)
            ground = resolved['pavement']
            diagnostics.append({'node': node, 'source_ways': record['source_ways'], 'kind': kind,
                                'status': 'resolved', 'name': name})
        except LayoutError as error:
            diagnostics.append({'node': node, 'source_ways': record['source_ways'], 'kind': kind,
                                'status': 'unresolved', 'reason': str(error)})
    return layouts, ground, sorted(diagnostics, key=lambda r: r['node'])
