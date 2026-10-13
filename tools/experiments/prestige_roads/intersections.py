"""Resolve bounded source-backed one-way junctions before the road experiment is meshed.

This is an explicit supported case, not a general traffic-engineering or legal-routing solver.
All coordinates are local meters. Source records are read-only; inferred lane assignments are reported.
"""

import itertools
import math
from functools import lru_cache

import numpy as np
import shapely
from scipy.ndimage import gaussian_filter1d
from shapely.geometry import LineString, Point, Polygon, mapping
from shapely.ops import substring, unary_union


class LayoutError(ValueError):
    """A supported-layout precondition failed; programming errors must propagate instead of becoming diagnostics."""


TARGET_NODE = 2918219392


@lru_cache(maxsize=8)
def buffered_geometry(geometry, distance):
    """Cache a bounded set of immutable Shapely buffers shared by repeated paint and lane-support audits."""
    return geometry.buffer(distance)


def unit(vector):
    """Normalize a planar vector, rejecting coincident endpoints rather than inventing a direction."""
    length = np.linalg.norm(vector)
    if length < 1e-8:
        raise LayoutError('Degenerate intersection tangent')
    return np.asarray(vector)/length


def frame(line, station):
    """Sample a center and outward tangent on a source alignment using a short symmetric chord."""
    center = np.array(line.interpolate(station).coords[0])
    before = np.array(line.interpolate(max(0, station-.15)).coords[0])
    after = np.array(line.interpolate(min(line.length, station+.15)).coords[0])
    tangent = unit(after-before)
    return center, tangent, np.array([-tangent[1], tangent[0]])


def curve(start, end, start_direction, end_direction, handle):
    """Sample a bounded cubic with shared parameters so adjacent lane edges coincide exactly."""
    t = np.linspace(0, 1, 257)[:, None]
    return ((1-t)**3*start+3*(1-t)**2*t*(start+handle*start_direction)
            +3*(1-t)*t*t*(end-handle*end_direction)+t**3*end)


def collect_approaches(roads, xy, node, station):
    """Collect directed mouths at one shared endpoint; fail on unsupported modes, levels, or short approaches."""
    approaches = []
    for road in sorted(roads, key=lambda r: r['id']):
        if node not in road['refs']:
            continue
        tags = road['tags']
        if road['bridge'] or road['tunnel'] or tags.get('layer', '0') != '0':
            raise LayoutError('The selected junction must be one at-grade level')
        if tags['highway'] == 'cycleway' or tags.get('oneway') not in ('yes', '1', 'true', '-1'):
            raise LayoutError('This bounded junction layout supports explicit one-way vehicle approaches only')
        if road['refs'].count(node) != 1 or node not in (road['refs'][0], road['refs'][-1]):
            raise LayoutError('Interior/repeated junction nodes require an approach-splitting stage')
        mouth_station = station[road['id']] if isinstance(station, dict) else station
        if road['line'].length < mouth_station+4:
            raise LayoutError('Selected approach overlaps its next source-way endpoint')
        if any('conditional' in k or k.startswith('access') or k.startswith('vehicle') for k in tags):
            raise LayoutError('Conditional or access-qualified approaches require a vehicle/time policy')
        at_start = road['refs'][0] == node
        outward = road['line'] if at_start else LineString(list(road['line'].coords)[::-1])
        incoming = at_start == (tags.get('oneway') == '-1')
        center, tangent, normal = frame(outward, mouth_station)
        direction = -tangent if incoming else tangent
        left = np.array([-direction[1], direction[0]])
        pitch = road['width']/road['lanes']
        lanes = [center+left*pitch*(road['lanes']/2-i-.5) for i in range(road['lanes'])]
        approaches.append({'way': road['id'], 'road': road, 'outward': outward, 'incoming': incoming,
                           'center': center, 'tangent': tangent, 'normal': normal, 'direction': direction,
                           'left': left, 'lane_centers': lanes, 'pitch': pitch, 'width': road['width'],
                           'count': road['lanes'], 'station': mouth_station,
                           'angle': math.atan2(center[1]-xy[node][1], center[0]-xy[node][0])})
    if len(approaches) < 3:
        raise LayoutError('Selected node is not a multi-approach junction')
    return approaches


def resolve_movements(approaches, relations, node):
    """Resolve simple mandatory via-node restrictions; reject ambiguity instead of guessing legal movements."""
    incoming = {a['way']: a for a in approaches if a['incoming']}
    outgoing = {a['way']: a for a in approaches if not a['incoming']}
    candidates = set(itertools.product(incoming, outgoing))
    restrictions = []
    for relation in relations:
        members, tags = relation['members'], relation['tags']
        if not any(m['type'] == 'node' and m['role'] == 'via' and int(m['ref']) == node for m in members):
            if any(m['type'] == 'way' and int(m['ref']) in incoming for m in members):
                if any(m['role'] == 'via' and m['type'] == 'way' for m in members):
                    raise LayoutError('A relevant via-way restriction needs multi-junction resolution')
            continue
        if tags.get('type') != 'restriction' or tags.get('restriction') != 'only_straight_on':
            raise LayoutError('Selected junction requires an unsupported restriction form')
        if any('conditional' in k or k == 'except' or k.startswith('restriction:') for k in tags):
            raise LayoutError('Qualified restrictions require a vehicle/time policy')
        if len(members) != 3:
            raise LayoutError('Expected one from way, one via node, and one to way')
        from_ways = [int(m['ref']) for m in members if m['role'] == 'from' and m['type'] == 'way']
        to_ways = [int(m['ref']) for m in members if m['role'] == 'to' and m['type'] == 'way']
        if len(from_ways) != 1 or len(to_ways) != 1:
            raise LayoutError('Restriction lacks a unique from/to pair')
        source, target = from_ways[0], to_ways[0]
        if source not in incoming or target not in outgoing:
            raise LayoutError('Restriction contradicts selected source-way direction or has a missing member')
        candidates = {pair for pair in candidates if pair[0] != source or pair[1] == target}
        restrictions.append({'relation': relation['id'], 'from': source, 'to': target,
                             'restriction': tags['restriction']})
    for source in incoming:
        targets = [target for start, target in candidates if start == source]
        if len(targets) != 1 or not any(r['from'] == source for r in restrictions):
            raise LayoutError('This case requires exactly one source-backed mandatory exit per incoming approach')
        tags = incoming[source]['road']['tags']
        turns = tags.get('turn:lanes', '').split('|')
        if len(turns) != incoming[source]['count'] or any(turn != 'through' for turn in turns):
            raise LayoutError('Selected case requires consistent explicit through indications on incoming lanes')
    rejected = sorted(set(itertools.product(incoming, outgoing))-candidates)
    return sorted(candidates), restrictions, rejected


def assign_lanes(approaches, movements):
    """Infer order-preserving lane additions where incoming and outgoing counts balance; fail on lane drops."""
    by_id = {a['way']: a for a in approaches}
    assignments, additions = [], []
    for target in sorted({target for _, target in movements}):
        out = by_id[target]
        sources = [by_id[source] for source, end in movements if end == target]
        # Project upstream mouths onto the exit's left axis: leftmost incoming lane group stays leftmost.
        sources.sort(key=lambda a: (-float((a['center']-out['center']) @ out['left']), a['way']))
        if sum(a['count'] for a in sources) != out['count']:
            raise LayoutError('Lane drops, ambiguous expansions, and many-to-one merges need a separate taper solver')
        offset = 0
        for source in sources:
            for lane in range(source['count']):
                assignments.append((source, lane, out, offset+lane))
            offset += source['count']
        if len(sources) > 1:
            additions.append({'to': target, 'incoming_lane_counts': [a['count'] for a in sources],
                              'outgoing_lanes': out['count'], 'ordered_sources': [a['way'] for a in sources],
                              'policy': 'spatial order, preserve all lanes; inferred lane addition, not a lane drop'})
    return assignments, additions


def dashed(line, length=1.5, period=4.5, half_width=.075):
    """Construct metric paint dashes, dropping zero-length tail pieces."""
    if line.geom_type == 'MultiLineString':
        return unary_union([dashed(part, length, period, half_width) for part in line.geoms])
    if line.is_empty or line.geom_type != 'LineString':
        return Polygon()
    return unary_union([substring(line, s, min(s+length, line.length)).buffer(half_width, cap_style=2)
                        for s in np.arange(0, max(0, line.length-.05), period)])


def controlled_confluence(approaches, controls, relations, node):
    """Resolve two signal-controlled one-way approaches to a common exit; infer lateral allocation explicitly."""
    incoming = [a for a in approaches if a['incoming']]
    outgoing = [a for a in approaches if not a['incoming']]
    if len(incoming) != 2 or len(outgoing) != 1:
        raise LayoutError('Controlled confluence needs two incoming ways and one outgoing way')
    for relation in relations:
        if any(m['role'] == 'via' and m['type'] == 'node' and int(m['ref']) == node
               for m in relation['members']):
            raise LayoutError('Confluence restriction needs explicit resolution before lane assignment')
    target = outgoing[0]
    assignments, evidence = [], []
    for source in incoming:
        matching = [c for c in controls if c['kind'] == 'traffic_signals' and source['way'] in c['roads']
                    and c['direction'] == ('backward' if source['road']['tags'].get('oneway') == '-1' else 'forward')
                    and 0 < source['outward'].project(Point(c['position'])) < source['station']+8]
        if not matching:
            raise LayoutError('Confluence input lacks a mapped directional signal within its approach')
        if source['count'] > target['count']:
            raise LayoutError('A real lane drop needs a longitudinal taper, not coincident connector endpoints')
        # The full-width stream continues all lanes. A narrower stream takes lanes on its side of the exit.
        side = float((source['center']-target['center']) @ target['left'])
        offset = 0 if side >= 0 else target['count']-source['count']
        for lane in range(source['count']):
            assignments.append((source, lane, target, offset+lane))
        evidence.append({'from': source['way'], 'to': target['way'], 'signal_nodes': [c['node'] for c in matching],
                         'basis': 'shared OSM node and one-way directions; lane allocation inferred',
                         'source_lanes': source['count'], 'exit_lanes': list(range(offset+1, offset+source['count']+1))})
    return assignments, evidence


def straight_arrow(center, direction):
    """Build an illustrative through arrow in the lane frame; dimensions are an explicit experiment assumption."""
    left = np.array([-direction[1], direction[0]])
    outline = [(-.10, -1.8), (.10, -1.8), (.10, .5), (.48, .15), (0, 1.6),
               (-.48, .15), (-.10, .5)]
    return Polygon([center+left*x+direction*y for x, y in outline])


def source_corridor(source, target, lane, exit_lane, boundary=0):
    """Follow the source route around protected islands, easing lateral lane placement between its two mouths."""
    first = substring(source['outward'], 0, source['station'])
    second = substring(target['outward'], 0, target['station'])
    route = LineString([*list(first.coords)[::-1], *list(second.coords)[1:]])
    stations = np.linspace(0, route.length, 257)
    raw = np.array([route.interpolate(s).coords[0] for s in stations])
    smooth = gaussian_filter1d(raw, 1.5/(stations[1]-stations[0]), axis=0, mode='nearest')
    weight = np.clip(np.minimum(stations, route.length-stations)/5, 0, 1)
    weight = weight*weight*(3-2*weight)
    center = raw*(1-weight[:, None])+smooth*weight[:, None]
    tangent = np.gradient(center, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    normal[0], normal[-1] = source['left'], target['left']
    t = stations/route.length
    blend = t*t*t*(10-15*t+6*t*t)
    begin = source['pitch']*(source['count']/2-lane-.5+boundary)
    end = target['pitch']*(target['count']/2-exit_lane-.5+boundary)
    return center+normal*((1-blend)*begin+blend*end)[:, None]


def junction_domain(approaches):
    """Join angularly ordered physical mouths into one ownership polygon; reject overlapping cross sections."""
    physical = {a.get('arm', a['way']): a.get('mouth', a) for a in approaches}
    mouths = sorted(physical.values(), key=lambda a: a['angle'])
    ring = [a['center']+sign*a['normal']*a['width']/2 for a in mouths for sign in (-1, 1)]
    domain = Polygon(ring)
    if not domain.is_valid:
        raise LayoutError('Junction mouths overlap; cluster resolution is required')
    return domain


def build_layout(roads, xy, relations, ground, islands, node=TARGET_NODE, station=24.0,
                 kind='restricted', controls=None, name='central', approaches=None, movement_solution=None,
                 compact_pavement=False):
    """Solve a junction; optional compact pavement retains its domain, connectors, and complete approach paint context."""
    approaches = approaches or collect_approaches(roads, xy, node, station)
    evidence = []
    if kind == 'restricted':
        movements, restrictions, rejected = resolve_movements(approaches, relations, node)
        assignments, additions = assign_lanes(approaches, movements)
    elif kind == 'controlled_confluence':
        assignments, evidence = controlled_confluence(approaches, controls or [], relations, node)
        movements = sorted({(source['way'], target['way']) for source, _, target, _ in assignments})
        restrictions, rejected, additions = [], [], []
    elif kind == 'urban':
        from urban import urban_movements
        assignments, restrictions, rejected = (movement_solution if movement_solution is not None
                                               else urban_movements(approaches, relations, node))
        movements = sorted({(source['way'], target['way']) for source, _, target, _ in assignments})
        additions = []
    elif kind in ('two_way', 'urban'):
        from layouts import two_way_movements
        assignments, restrictions, rejected = two_way_movements(approaches, relations, node)
        movements = sorted({(source['way'], target['way']) for source, _, target, _ in assignments})
        additions = []
    else:
        raise LayoutError('Unknown intersection layout policy')
    domain = junction_domain(approaches)
    connectors, corridors = [], []
    follow_source = set()
    for source, lane, target, exit_lane in assignments:
        start, end = source['lane_centers'][lane], target['lane_centers'][exit_lane]
        handle = min(35.0 if kind == 'controlled_confluence' else 18.0,
                     np.linalg.norm(target['center']-source['center'])*.35)
        candidate = LineString(curve(start, end, source['direction'], target['direction'], handle))
        if kind != 'urban' and candidate.buffer(.9, cap_style=2).intersection(islands).area > .001:
            follow_source.add((source['way'], target['way']))
    for source, lane, target, exit_lane in assignments:
        start, end = source['lane_centers'][lane], target['lane_centers'][exit_lane]
        # All lanes of a movement share one handle and sampling, preserving exact internal ribbon boundaries.
        handle = min(35.0 if kind == 'controlled_confluence' else 18.0,
                     np.linalg.norm(target['center']-source['center'])*.35)
        points = curve(start, end, source['direction'], target['direction'], handle)
        left = curve(start+source['left']*source['pitch']/2, end+target['left']*target['pitch']/2,
                     source['direction'], target['direction'], handle)
        right = curve(start-source['left']*source['pitch']/2, end-target['left']*target['pitch']/2,
                      source['direction'], target['direction'], handle)
        if (source['way'], target['way']) in follow_source:
            points = source_corridor(source, target, lane, exit_lane)
            left = source_corridor(source, target, lane, exit_lane, .5)
            right = source_corridor(source, target, lane, exit_lane, -.5)
        if kind == 'urban':
            from urban import lane_curve
            points, left, right = lane_curve(source, lane, target, exit_lane, ground)
        ribbon = Polygon(np.vstack([left, right[::-1]]))
        if not ribbon.is_valid or not LineString(points).is_simple:
            raise LayoutError('Lane connector folds over itself')
        connectors.append({'from': source['way'], 'from_lane': lane+1, 'to': target['way'],
                           'to_lane': exit_lane+1, 'points': points, 'line': LineString(points),
                           'shape': ('cubic through' if len(points) == 257 else 'straight-arc-straight turn')
                           if kind == 'urban' else ('source corridor' if (source['way'], target['way']) in follow_source
                                                   else 'cubic'),
                           'left_edge': LineString(left), 'right_edge': LineString(right),
                           'corridor': ribbon, 'start_direction': source['direction'],
                           'end_direction': target['direction']})
        corridors.append(ribbon)
    # A short unchanged collar makes the new interior share its full perimeter with the source approach surfaces.
    interior = domain.buffer(-1.0)
    sweep = unary_union(corridors)
    replacement = sweep.union(ground.intersection(domain.difference(interior)))
    if kind in ('two_way', 'urban'):
        # Preserve approach shoulder/parking pavement when driving lanes use less than the mapped road width.
        replacement = replacement.union(ground.intersection(domain))
    replacement = replacement.buffer(1.5).buffer(-1.5).difference(islands).intersection(domain)
    pavement_base, pavement_islands = ground, islands
    if compact_pavement:
        # Curve selection still sees the original ground. Only the stored pavement and paint support are bounded.
        context = unary_union([domain, sweep]+[a['outward'].buffer(a['road']['width']) for a in approaches])
        context = context.envelope.buffer(2, join_style=2)
        pavement_base = ground.intersection(context)
        pavement_islands = islands.intersection(context)
    pavement = pavement_base.difference(interior).union(replacement).difference(pavement_islands)
    movement_ribbons = {pair: unary_union([c['corridor'] for c in connectors
                                          if (c['from'], c['to']) == pair]) for pair in movements}
    conflict_regions, conflicts = [], []
    for first, second in itertools.combinations(movements, 2):
        if kind == 'urban' and first[0] == second[0]:
            # Alternative turns from the same approach are branches; audit physical lane crossings separately.
            continue
        if first[1] == second[1] and kind not in ('controlled_confluence', 'two_way', 'urban'):
            continue
        overlap = movement_ribbons[first].intersection(movement_ribbons[second])
        if overlap.area > .1:
            conflict_regions.append(overlap.buffer(1.0))
            conflicts.append({'first': list(first), 'second': list(second), 'overlap_m2': overlap.area,
                              'meaning': 'converging movements; priority unresolved' if first[1] == second[1]
                              else 'at-grade crossing; no lane graph transfer or signal phase inferred'})
    conflict = unary_union(conflict_regions)
    paint, arrows = [], []
    for target in sorted({c['to'] for c in connectors}):
        if kind in ('two_way', 'urban'):
            continue
        ordered = sorted([c for c in connectors if c['to'] == target], key=lambda c: c['to_lane'])
        if kind == 'controlled_confluence':
            # Follow the widest continuing stream; joining-stream dashes stop before the overlapping corridors.
            primary = max((a for a in approaches if a['incoming']), key=lambda a: (a['count'], a['way']))
            ordered = sorted([c for c in ordered if c['from'] == primary['way']], key=lambda c: c['to_lane'])
        for left, right in zip(ordered, ordered[1:]):
            # Adjacent independently sampled cubics need not have equal vertex counts; one shared boundary wins.
            guide = left['right_edge']
            separator = dashed(guide)
            if kind != 'controlled_confluence':
                separator = separator.difference(conflict)
            if left['from'] != right['from']:
                # The added lane gains an internal separator only where its corridor meets the continuing lanes.
                separator = separator.intersection(right['corridor'].buffer(.75))
            paint.append(separator)
    for approach in approaches:
        if kind in ('two_way', 'urban'):
            continue
        if not approach['incoming']:
            continue
        center, tangent, _ = frame(approach['outward'], approach['station']+4)
        direction = -tangent
        left = np.array([-direction[1], direction[0]])
        for lane in range(approach['count']):
            position = center+left*approach['pitch']*(approach['count']/2-lane-.5)
            arrows.append(straight_arrow(position, direction))
    white = unary_union(paint+arrows).intersection(buffered_geometry(pavement, -.10))
    assumptions = [
        'Bounded one-way, single-level, mandatory-through case; not a general legal routing solver.',
        'Lane numbers run from left to right in the direction of travel; source lane counts are retained.',
        'Lane centers evenly divide accepted carriageway widths; shoulder and parking allocation is unresolved.',
        'Source restriction relations determine road-to-road movements; lateral lane assignment is inferred.',
        'Balanced incoming groups occupy exit lanes in spatial order; no lanes are dropped or duplicated.',
        'Mouth distances are recorded per approach; 1.5 m corner smoothing is inferred.',
        'Cubic connectors follow approach tangents; source-corridor fallback avoids protected islands.',
        'Existing footprint islands are protected; no new surveyed curb geometry or signal phases are claimed.',
        'Short dashed guidance and arrow dimensions are illustrative; stop bars and signal timing are not invented.',
        'At-grade path crossings become conflict records, never additional graph edges.'
    ]
    if kind == 'controlled_confluence':
        assumptions = [
            'Two directional signal-controlled input streams share one exit; signal phases are unknown.',
            'Each input preserves its lane order. The narrower stream uses exit lanes on its entry side.',
            'Overlapping streams are controlled conflicts, not a simultaneous collapse of all incoming lanes.',
            'Lane assignment, cubic shape, pavement width, and marking dimensions are inferred.',
            'No legal priority, signal schedule, swept vehicle path, or surveyed geometry is asserted.'
        ]
    if kind in ('two_way', 'urban'):
        assumptions = [
            'Right-hand traffic; one lane per direction on each physical arm; directional halves are separate ports.',
            'Unmapped directional splits of two-lane ways are inferred as one lane in each direction.',
            'Local geometric turn opportunities exclude U-turns and honor supported unconditional restrictions.',
            'Turn indications select rendered guidance, not a complete statement of legal access.',
            'Conditional, vehicle-qualified, and via-way restrictions require explicit policy and are rejected.',
            'Parking, shoulders, curb radii, priority, signal phases, and vehicle swept paths are not surveyed.'
        ]
    if kind == 'urban':
        assumptions = [
            'Traffic side comes from the explicit scene locale or a way override; lane numbering follows travel direction.',
            'Physical geometry supports traffic authorized under source access tags; public routability is not asserted.',
            'Missing even two-way counts split evenly; lane widths, mouths, curb returns and arrow shapes are inferred.',
            'Empty/none turn entries are unindicated; infer edge-lane turns and ordered through connections.',
            'Supported unconditional via-node restrictions use a private passenger-car profile; no signal phases inferred.',
            'Multiple turning opportunities may share an input or exit lane; conflicts do not create graph transfers.',
            'A bounded unindicated 2-to-1 turn peel-off is an exported hypothesis, never an invented legal restriction.',
            'Unexplained lane drops, U-turns, access-qualified lanes, and multi-node clusters remain unresolved.',
            'Passenger-car-width envelopes and curvature are geometric guards, not swept-vehicle certification.'
        ]
        if any(a.get('shared_single_lane') for a in approaches):
            assumptions.append('Two directional ports can share one narrow physical lane; passing priority is unknown.')
    return {'node': node, 'kind': kind, 'name': name, 'evidence': evidence,
            'position': np.asarray(xy[node]), 'approaches': approaches, 'movements': movements,
            'restrictions': restrictions, 'rejected_movements': rejected, 'lane_additions': additions,
            'connectors': connectors, 'domain': domain, 'pavement': pavement, 'paint': white,
            'conflict': conflict, 'conflicts': conflicts, 'islands': islands, 'assumptions': assumptions,
            'station': station, 'arrow_count': len(arrows)}


def audit_layout(layout, ground):
    """Verify source-backed edges, lane coverage, endpoint continuity, same-exit ordering, and pavement support."""
    connectors, approaches = layout['connectors'], layout['approaches']
    allowed = set(layout['movements'])
    errors, angles, unsupported, radii = [], [], [], []
    for approach in approaches:
        field = 'from' if approach['incoming'] else 'to'
        lane_field = field+'_lane'
        actual = [c[lane_field] for c in connectors if c[field] == approach['way']]
        required = [i+1 for i in approach.get('eligible_lanes', range(approach['count']))]
        multiple = layout['kind'] in ('controlled_confluence', 'two_way', 'urban')
        valid = sorted(set(actual)) == required if multiple else sorted(actual) == required
        if layout['kind'] == 'urban' and not approach['incoming']:
            valid = set(actual) <= set(required)
        if approach['incoming'] and layout['kind'] not in ('two_way', 'urban'):
            valid = sorted(actual) == required
        if not valid:
            raise LayoutError('Missing or multiply assigned approach lane')
    by_id = {a['way']: a for a in approaches}
    # Every support probe lies within this envelope; clip edges remain beyond the .003m buffer reach.
    support_bounds = unary_union([c['line'].buffer(.9, cap_style=2) for c in connectors]).envelope.buffer(.01)
    support = buffered_geometry(ground.intersection(support_bounds), .003)
    for connector in connectors:
        if (connector['from'], connector['to']) not in allowed:
            raise LayoutError('Lane graph violates a source movement restriction')
        points = connector['points']
        vectors = np.diff(points, axis=0)
        lengths = np.linalg.norm(vectors, axis=1)
        directions = vectors/np.maximum(lengths[:, None], 1e-9)
        turns = np.arccos(np.clip(np.sum(directions[1:]*directions[:-1], axis=1), -1, 1))
        radii.extend(((lengths[1:]+lengths[:-1])/(2*np.maximum(turns, 1e-9))).tolist())
        errors.extend([np.linalg.norm(points[0]-by_id[connector['from']]['lane_centers'][connector['from_lane']-1]),
                       np.linalg.norm(points[-1]-by_id[connector['to']]['lane_centers'][connector['to_lane']-1])])
        for tangent, reference in [(unit(points[1]-points[0]), connector['start_direction']),
                                   (unit(points[-1]-points[-2]), connector['end_direction'])]:
            angles.append(math.degrees(math.acos(float(np.clip(tangent @ reference, -1, 1)))))
        # A passenger-car-width corridor is a geometric envelope, not a steering or swept-vehicle simulation.
        outside = connector['line'].buffer(.9, cap_style=2).difference(support)
        unsupported.append(outside.area)
    if max(errors) > 1e-6 or max(angles) > 2 or max(unsupported) > .01:
        raise LayoutError(f'Invalid connector continuity/support: {max(errors)}, {max(angles)}, {max(unsupported)}')
    if layout['kind'] in ('two_way', 'urban') and min(radii) < 3:
        raise LayoutError('Lane path has a sampled radius under the 3 m prototype guard')
    for first, second in itertools.combinations(connectors, 2):
        if (layout['kind'] == 'urban' and first['from'] == second['from']
                and first['from_lane'] != second['from_lane'] and first['line'].intersects(second['line'])):
            raise LayoutError('Different incoming lanes cross before reaching their assigned exits')
        controlled = layout['kind'] in ('controlled_confluence', 'two_way', 'urban') and first['from'] != second['from']
        if first['to'] == second['to'] and not controlled and first['line'].intersects(second['line']):
            raise LayoutError('Lane order inversion in a shared exit')
    # Ignore only the perimeter tolerance introduced by the compiler's explicit 1 mm precision grid.
    lost_islands = ground.intersection(buffered_geometry(layout['islands'], -.0015)).area
    if lost_islands > 1e-8:
        raise LayoutError('Intersection construction filled a protected island')
    return {'approaches': len(approaches), 'source_restrictions_applied': len(layout['restrictions']),
            'permitted_road_movements': len(allowed), 'rejected_road_movements': len(layout['rejected_movements']),
            'lane_connectors': len(connectors),
            'inferred_bus_labels': sum(r['rendered'] for r in layout.get('lane_label_evidence', [])),
            'each_incoming_lane_has_one_connection': layout['kind'] not in ('two_way', 'urban'),
            'all_incoming_lanes_connected': True,
            'lane_coverage_scope': 'eligible private-car lanes; physical source lane numbers retained',
            'withheld_physical_lanes': [{'port': a['way'], **r} for a in approaches
                                        for r in a.get('lane_roles', []) if not r['car_eligible']],
            'unfed_outgoing_lanes': [{'port': a['way'], 'lane': i+1} for a in approaches if not a['incoming']
                                    for i in a.get('eligible_lanes', range(a['count']))
                                    if not any(c['to'] == a['way'] and c['to_lane'] == i+1
                                                                       for c in connectors)],
            'unexpected_same_exit_lane_crossings': 0, 'max_endpoint_error_m': float(max(errors)),
            'max_sampled_endpoint_tangent_error_degrees': max(angles),
            'minimum_sampled_path_radius_m': min(radii),
            'max_car_width_envelope_outside_pavement_m2': max(unsupported),
            'protected_island_interior_overlap_m2': lost_islands, 'island_boundary_tolerance_m': .0015,
            'at_grade_conflict_pairs': len(layout['conflicts']),
            'through_arrows' if layout['kind'] != 'urban' else 'source_indicated_arrows': layout['arrow_count']}


def export_layout(layout, output, to_geo, write_json, ground):
    """Write inspectable lane connectivity, assumptions, and geospatial geometry from the same layout used to render."""
    audit = audit_layout(layout, ground)
    records = []
    features = []
    for connector in layout['connectors']:
        record = {k: connector[k] for k in ('from', 'from_lane', 'to', 'to_lane')}
        record['shape'] = connector['shape']
        record['assignment'] = ('inferred spatial order; road movement constrained by source restriction'
                                if layout['kind'] == 'restricted' else 'inferred entry-side lanes at controlled confluence')
        if layout['kind'] in ('two_way', 'urban'):
            record['assignment'] = ('inferred ordered lanes; mapped turn indications and supported legal restrictions '
                                    'retained separately')
            ports = {a['way']: a for a in layout['approaches']}
            record['source_from_way'] = ports[connector['from']]['road']['id']
            record['source_to_way'] = ports[connector['to']]['road']['id']
            if layout['kind'] == 'urban':
                from urban import movement_turn
                record['turn'] = movement_turn(ports[connector['from']], ports[connector['to']])
                record['allocation_inferences'] = ports[connector['from']].get('allocation_inferences', [])
                record['source_access'] = {'from': ports[connector['from']]['access'],
                                           'to': ports[connector['to']]['access']}
                record['physical_lane_access'] = {
                    side: ports[connector[side]]['lane_roles'][connector[side+'_lane']-1]
                    for side in ('from', 'to')}
                mapped = [r for a in layout['approaches'] for r in a.get('lane_connectivity', [])
                          if r['from_port'] == connector['from'] and r['to_port'] == connector['to']]
                if mapped:
                    record['assignment'] = 'SOURCE lane connectivity relation; geometric path inferred'
                    record['connectivity_relations'] = [r['relation'] for r in mapped]
        record['xy_m'] = connector['points'].tolist()
        records.append(record)
        geometry = shapely.transform(connector['line'], to_geo.transform, interleaved=False)
        features.append({'type': 'Feature', 'properties': {k: v for k, v in record.items() if k != 'xy_m'},
                         'geometry': mapping(geometry)})
    report = {key: layout[key] for key in ('node', 'kind', 'name', 'movements', 'restrictions', 'rejected_movements',
                                         'lane_additions', 'conflicts', 'assumptions', 'evidence')}
    report['lane_connectors'], report['audit'] = records, audit
    if layout['kind'] == 'urban':
        report['allocation_inferences'] = [record for a in layout['approaches']
                                            for record in a.get('allocation_inferences', [])]
        report['inferred_lane_labels'] = layout.get('lane_label_evidence', [])
        report['vehicle_profile'] = 'private passenger car, authorized under source access conditions'
        report['relation_notes'] = [record for a in layout['approaches'] for record in a.get('relation_notes', [])]
        report['lane_connectivity'] = [record for a in layout['approaches'] for record in a.get('lane_connectivity', [])]
        if 'cluster_nodes' in layout:
            report['cluster_nodes'] = layout['cluster_nodes']
            report['topological_routes'] = layout['topological_routes']
    report['physical_arms'] = len({a.get('arm', a['way']) for a in layout['approaches']})
    report['approaches'] = [{'way': a['way'], 'incoming': a['incoming'], 'lanes': a['count'],
                             'width_m': a['width'], 'mouth_station_m': a['station'], 'mouth_xy_m': a['center'].tolist(),
                             'source_way': a['road']['id'], 'travel': a.get('travel'),
                             'driving_side': a.get('driving_side'), 'center_line_style': a.get('center_line'),
                             'source_access': a.get('access'),
                             'eligible_private_car_lanes': [i+1 for i in a.get('eligible_lanes', range(a['count']))],
                             'physical_lane_roles': a.get('lane_roles', []),
                             'shared_single_lane': a.get('shared_single_lane', False),
                             'arm_ends_at_junction': a.get('ends_at_junction'),
                             'junction_node': a.get('junction_node', layout['node']),
                             'lane_indications': [sorted(v) for v in a.get('indications', [])],
                             'source_chain': a.get('chain', [a['road']['id']]),
                             'downstream_indications': [sorted(v) for v in a.get('downstream_indications', [])],
                             'source_tags': a['road']['tags']} for a in layout['approaches']]
    prefix = 'intersection' if layout['name'] == 'central' else layout['name']
    write_json(output/f'{prefix}-layout.json', report)
    write_json(output/f'{prefix}-lanes.geojson', {'type': 'FeatureCollection', 'features': features})
    plot_layout(layout, output, ground)
    return audit


def plot_layout(layout, output, ground):
    """Render a static engineering plan of pavement, directed lane connectors, paint, and permitted movements."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 9))
    extent = layout['domain'].buffer(12).envelope
    geometry = ground.intersection(extent)
    polygons = [p for p in shapely.get_parts(geometry) if p.geom_type == 'Polygon']
    for polygon in polygons:
        ax.fill(*polygon.exterior.xy, color='#3c454b')
        for hole in polygon.interiors:
            ax.fill(*hole.xy, color='#e6eadb')
    colors = ['#7adcf2', '#fbd17a', '#d6adff', '#9bf2ac']
    for i, movement in enumerate(layout['movements']):
        group = [c for c in layout['connectors'] if (c['from'], c['to']) == movement]
        for j, connector in enumerate(group):
            line = connector['line']
            label = f'{movement[0]} to {movement[1]}' if j == 0 else None
            ax.plot(*line.xy, color=colors[i % len(colors)], lw=1.8, label=label)
            a, b = line.interpolate(.48, normalized=True), line.interpolate(.55, normalized=True)
            ax.annotate('', xy=(b.x, b.y), xytext=(a.x, a.y),
                        arrowprops={'arrowstyle': '->', 'color': colors[i % len(colors)], 'lw': 1.8})
    paint = layout['paint']
    for polygon in (p for p in shapely.get_parts(paint) if p.geom_type == 'Polygon'):
        ax.fill(*polygon.exterior.xy, color='white', alpha=.8)
    shown = set()
    for approach in layout['approaches']:
        if layout['kind'] in ('two_way', 'urban'):
            if approach['arm'] in shown:
                continue
            shown.add(approach['arm'])
            center = approach['mouth']['center']+approach['tangent']*7
            label = f'{approach["road"]["tags"].get("name", approach["road"]["id"])}\n{approach["road"]["lanes"]} total lanes'
            ax.text(*center, label, ha='center', va='center', fontsize=8,
                    bbox={'facecolor': 'white', 'alpha': .8, 'edgecolor': 'none'})
            continue
        center = approach['center']+approach['tangent']*9
        ax.text(*center, f'{approach["way"]}\n{approach["count"]} lanes '+('IN' if approach['incoming'] else 'OUT'),
                ha='center', va='center', fontsize=8, bbox={'facecolor': 'white', 'alpha': .8, 'edgecolor': 'none'})
    ax.set_xlim(extent.bounds[0], extent.bounds[2]); ax.set_ylim(extent.bounds[1], extent.bounds[3])
    ax.set_facecolor('#e6eadb'); ax.set_aspect('equal')
    ax.set_xlabel('Local east (m)'); ax.set_ylabel('Local north (m)')
    ax.set_title(f'Prestige Roads {layout["name"]}: source topology, inferred lane placement\n'
                 'Path crossings are conflicts, not graph connections; heights remain synthetic')
    ax.legend(loc='lower left', fontsize=8, framealpha=.95)
    prefix = 'intersection' if layout['name'] == 'central' else layout['name']
    fig.tight_layout(); fig.savefig(output/f'{prefix}-plan.png', dpi=160); plt.close(fig)
