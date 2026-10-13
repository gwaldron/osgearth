"""Resolve isolated pairs of close T-junctions with one lane per direction and explicit graph reachability."""

from collections import defaultdict

import numpy as np
from shapely.geometry import LineString, Point
from shapely.ops import unary_union

from intersections import LayoutError, audit_layout, build_layout, junction_domain
from layouts import physical_arms


def topology_ports(arms):
    """Create lane-free directional graph ports for local restrictions; reject multi-lane or arrow-qualified clusters."""
    from urban import lane_schema
    from lane_roles import lane_model
    ports = []
    for arm in arms:
        road = arm['road']
        forward, backward = lane_schema(road)
        if any(r['designated_modes'] or not r['car_eligible']
               for roles in lane_model(road)['roles'].values() for r in roles):
            raise LayoutError('Compound reserved lanes require a vehicle-scoped cluster policy')
        if max(forward, backward) != 1 or any(k.startswith('turn:lanes') for k in road['tags']):
            raise LayoutError('Compound junction currently needs one unindicated lane per direction')
        if road['bridge'] or road['tunnel'] or road['tags'].get('layer', '0') != '0':
            raise LayoutError('Compound junction must occupy one at-grade level')
        for travel, count in [('forward', forward), ('backward', backward)]:
            if count:
                ports.append({'way': arm['key']+':'+travel, 'arm': arm['key'], 'road': road, 'travel': travel,
                              'incoming': (arm['side'] == 'before') == (travel == 'forward')})
    return ports


def build_cluster(nodes, arms_by_node, roads, xy, relations, ground, islands, region, driving_side, center_line,
                  compact_pavement=False):
    """Trace permitted paths through a two-node cluster and fit one four-mouth surface; reject unsupported geometry."""
    from urban import approach_paint, legal_pairs, movement_turn, urban_approaches
    local, internal, external, restrictions, notes = {}, {}, [], [], []
    for node in nodes:
        arms = [dict(a, key=f'{node}/{a["key"]}', junction_node=node) for a in arms_by_node[node]]
        inside = [a for a in arms if a['refs'][-1] in nodes]
        if len(arms) != 3 or len(inside) != 1:
            raise LayoutError('Compound pass owns two T-junctions joined by one source corridor')
        internal[node] = inside[0]
        external.extend(a for a in arms if a is not inside[0])
        local[node] = topology_ports(arms)
        relevant = [r for r in relations if any(m['role'] == 'via' and m['type'] == 'node'
                                                and int(m['ref']) == node for m in r['members'])]
        if any(r['tags'].get('type') == 'connectivity' for r in relevant):
            raise LayoutError('Mapped compound lane connectivity requires multi-node relation handling')
        pairs, evidence, _ = legal_pairs(local[node], relations, node)
        local[node] = {'ports': local[node], 'pairs': pairs}
        restrictions.extend(dict(r, via_node=node) for r in evidence)
        notes.extend(r for p in local[node]['ports'] for r in p.get('relation_notes', []))
    first, second = nodes
    if internal[first]['outward'].hausdorff_distance(internal[second]['outward']) > .01:
        raise LayoutError('Compound link does not have one shared source alignment')
    center = np.mean([xy[n] for n in nodes], axis=0)
    ports = urban_approaches(external, {first: center}, first, driving_side, center_line)
    roots = {a['key']: a['junction_node'] for a in external}
    for port in ports:
        port['junction_node'] = roots[port['arm']]
        port['allocation_inferences'] = []
    ports[0]['relation_notes'] = notes
    if not region.buffer(.001).covers(junction_domain(ports)):
        raise LayoutError('Compound junction reaches beyond the render crop')
    graph = defaultdict(list)
    for node in nodes:
        for incoming, outgoing in sorted(local[node]['pairs']):
            graph[incoming].append(outgoing)
    for node, other in [(first, second), (second, first)]:
        inside = [p for p in local[node]['ports'] if p['arm'] == internal[node]['key'] and not p['incoming']]
        opposite = [p for p in local[other]['ports'] if p['arm'] == internal[other]['key'] and p['incoming']]
        if len(inside) != len(opposite):
            raise LayoutError('Compound source corridor has contradictory travel directions')
        if inside:
            graph[inside[0]['way']].append(opposite[0]['way'])
    exits = {p['way']: p for p in ports if not p['incoming']}
    assignments, routes = [], []
    for source in (p for p in ports if p['incoming']):
        pending = [(source['way'], [source['way']])]
        while pending:
            current, path = pending.pop()
            if current in exits:
                target = exits[current]
                if source['arm'] != target['arm']:
                    movement_turn(source, target)
                    assignments.append((source, 0, target, 0))
                    routes.append({'from_port': source['way'], 'to_port': target['way'], 'graph_ports': path,
                                   'basis': 'source-node reachability with local restrictions; geometry inferred'})
                continue
            pending.extend((following, path+[following]) for following in graph[current] if following not in path)
    if len({(a['way'], b['way']) for a, _, b, _ in assignments}) != len(assignments):
        raise LayoutError('Compound movement has multiple topological paths')
    all_pairs = {(a['way'], b['way']) for a in ports if a['incoming']
                 for b in ports if not b['incoming'] and a['arm'] != b['arm']}
    actual = {(a['way'], b['way']) for a, _, b, _ in assignments}
    name = 'junction-cluster-'+'-'.join(str(n) for n in nodes)
    layout = build_layout(roads, {first: center}, [], ground, islands, first, kind='urban', name=name,
                          approaches=ports, movement_solution=(assignments, restrictions, sorted(all_pairs-actual)),
                          compact_pavement=compact_pavement)
    by_port = {p['way']: p for p in ports}
    for connection in layout['connectors']:
        a, b = by_port[connection['from']], by_port[connection['to']]
        corridor = [a['road']['surface'], b['road']['surface']]
        if a['junction_node'] != b['junction_node']:
            corridor.extend(internal[n]['road']['surface'] for n in nodes)
        corridor = unary_union(corridor).buffer(4).buffer(-4)
        # The support check only reads this corridor. Keep clipping edges beyond the one-millimeter buffer reach.
        local_ground = ground.intersection(corridor.envelope.buffer(.01, join_style=2))
        allowed = corridor.intersection(local_ground.buffer(.001))
        if connection['line'].buffer(.9, cap_style=2).difference(allowed).area > .001:
            raise LayoutError('Compound path leaves its topologically permitted source corridor')
    layout.update(cluster_nodes=list(nodes), topological_routes=routes, arm_count=4,
                  junction_core=internal[first]['outward'].buffer(max(a['road']['width'] for a in external)/2+3))
    layout['assumptions'].extend([
        'One shared surface owns both close T-junctions; external ports keep their original source-node roots.',
        'Every external movement follows local directed reachability through the source corridor and its restrictions.',
        'No multi-lane compound allocation, lane-change route, priority, or signal coordination is inferred.'
    ])
    approach_paint(layout)
    audit_layout(layout, layout['pavement'])
    return layout


def discover_clusters(roads, xy, relations, ground, islands, region, nodes, driving_side, center_line,
                      compact_pavement=False):
    """Find isolated short-link components; accept bounded pairs before individual junctions and retain rejection reasons."""
    from urban import extend_arms
    arms, links = {}, defaultdict(set)
    for node in sorted(nodes):
        if not region.covers(Point(xy[node])):
            continue
        try:
            arms[node] = extend_arms(physical_arms(roads, xy, node, nodes), roads, xy, nodes)
        except LayoutError:
            continue
        minimum = max(a['road']['width'] for a in arms[node])/2+4
        for arm in arms[node]:
            other = arm['refs'][-1]
            if other in nodes and arm['outward'].length < min(30, minimum/.42):
                links[node].add(other)
                links[other].add(node)
    seen, owned, attempts, layouts = set(), {}, {}, []
    for seed in sorted(links):
        if seed in seen:
            continue
        component, pending = set(), [seed]
        while pending:
            node = pending.pop()
            if node in component:
                continue
            component.add(node)
            pending.extend(links[node]-component)
        seen |= component
        if len(component) != 2 or not component <= arms.keys():
            continue
        pair = tuple(sorted(component))
        try:
            layout = build_cluster(pair, arms, roads, xy, relations, ground, islands, region, driving_side, center_line,
                                   compact_pavement)
            if any(layout['domain'].intersection(j['domain']).area > .001 for j in layouts):
                raise LayoutError('Compound ownership overlaps another accepted cluster')
            if compact_pavement and not layouts:
                # Match the full-pavement path's global island removal at its first accepted cluster.
                ground = ground.difference(islands)
            layouts.append(layout)
            ground = (ground.difference(layout['domain']).union(layout['pavement'].intersection(layout['domain']))
                      if compact_pavement else layout['pavement'])
            owned.update({n: layout for n in pair})
        except LayoutError as error:
            attempts.update({n: {'nodes': list(pair), 'reason': str(error)} for n in pair})
    return layouts, ground, owned, attempts
