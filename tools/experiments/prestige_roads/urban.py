"""Resolve bounded urban intersections for an explicit traffic side; retain source evidence and inference separately."""

import itertools
import math
from collections import defaultdict
from functools import lru_cache

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import substring, unary_union

from intersections import LayoutError, audit_layout, build_layout, curve, dashed, frame, junction_domain, straight_arrow
from intersections import buffered_geometry
from layouts import physical_arms
from semantics import motor_access, oneway_value, restriction_for_car
from connectivity import mapped_connections
from lane_roles import lane_model, role_signature


def lane_schema(road):
    """Return validated physical counts; reserved lanes retain their width and original physical lane numbers."""
    return lane_model(road)['counts']


def turn_indications(road, travel, count):
    """Parse left-to-right lane indications; empty/none means unindicated, never a prohibited movement."""
    tags = road['tags']
    oneway = oneway_value(tags) in ('yes', '1', 'true', '-1')
    oneway = oneway and not any(tags.get('oneway:'+mode) in ('no', '0', 'false') for mode in ('bus', 'psv'))
    key = 'turn:lanes' if oneway else 'turn:lanes:'+travel
    value = tags.get(key)
    if value is None:
        return [set() for _ in range(count)], None
    fields = value.split('|')
    if len(fields) != count:
        raise LayoutError(f'{key} has {len(fields)} entries for {count} directional lanes')
    result = []
    for field in fields:
        values = set(field.split(';'))-{'', 'none'}
        if not values <= {'left', 'right', 'through', 'slight_left', 'slight_right', 'sharp_left', 'sharp_right'}:
            raise LayoutError('Unsupported turn indication (merge/reverse needs another policy)')
        result.append(values)
    return result, key


def lane_pitch(road, forward, backward):
    """Allocate driving width before optional edge margins; a mapped narrow lane must not lose two fictitious shoulders."""
    physical = 1 if road['lanes'] == 1 and forward and backward else forward+backward
    margin = min(.6, max(0, road['width']-physical*2.5))
    pitch = min((road['width']-margin)/physical, 3.7)
    if pitch < (2.2 if physical == 1 else 2.5):
        raise LayoutError('Accepted road width cannot support the directional lanes')
    return pitch


def extend_arms(arms, roads, xy, junction_nodes):
    """Join compatible degree-two tag splits, retaining the near-junction road as the lane-schema authority."""
    for arm in arms:
        refs, chain = list(arm['refs']), [arm['road']['id']]
        while refs[-1] not in junction_nodes:
            candidates = [r for r in roads if r['id'] not in chain and refs[-1] in (r['refs'][0], r['refs'][-1])
                          and r['tags']['highway'] != 'cycleway']
            if len(candidates) != 1:
                break
            road = candidates[0]
            original_counts = lane_schema(arm['road'])
            next_counts = lane_schema(road)
            if arm['side'] == 'before':
                original_counts = original_counts[::-1]
            if road['refs'][0] != refs[-1]:
                next_counts = next_counts[::-1]
            original_roles = role_signature(lane_model(arm['road']))
            following_roles = role_signature(lane_model(road))
            if arm['side'] == 'before':
                original_roles = original_roles[::-1]
            if road['refs'][0] != refs[-1]:
                following_roles = following_roles[::-1]
            if (road['bridge'] or road['tunnel'] or road['tags'].get('name') != arm['road']['tags'].get('name')
                    or next_counts != original_counts or following_roles != original_roles
                    or road['lanes'] != arm['road']['lanes']
                    or abs(road['width']-arm['road']['width']) > .01):
                break
            more = road['refs'] if road['refs'][0] == refs[-1] else road['refs'][::-1]
            # Direction must agree in the common outward frame, even when the source way was digitized backwards.
            original_out = (arm['side'] == 'after') != (oneway_value(arm['road']['tags']) == '-1')
            following_out = (road['refs'][0] == refs[-1]) != (oneway_value(road['tags']) == '-1')
            if 0 in lane_schema(road) and original_out != following_out:
                break
            stop = next((i for i, n in enumerate(more[1:], 1) if n in junction_nodes), len(more)-1)
            refs.extend(more[1:stop+1])
            chain.append(road['id'])
        arm.update(refs=refs, outward=LineString([xy[n] for n in refs]), chain=chain,
                   ends_at_junction=refs[-1] in junction_nodes)
    return arms


def urban_approaches(arms, xy, node, driving_side='right', center_line='double_yellow'):
    """Construct incoming/outgoing lane ports on mixed one-way and two-way physical arms."""
    if len(arms) not in (3, 4, 5):
        raise LayoutError('Urban pass owns three to five arms at one node; larger clusters remain explicit')
    approaches = []
    largest = max(a['road']['width'] for a in arms)
    mouth_distance = max(14, largest/2+10)
    directions = sorted(math.atan2(*(np.asarray(a['outward'].interpolate(mouth_distance).coords[0])-xy[node])[::-1])
                        for a in arms)
    for first, second in zip(directions, directions[1:]+[directions[0]+2*math.pi]):
        angle = second-first
        if angle < math.radians(1):
            raise LayoutError('Nearly coincident arms need a shared mouth or cluster layout')
        if angle < math.pi/3:
            # Nearby fork/confluence mouths must separate before they become independent cross sections.
            mouth_distance = max(mouth_distance, largest/2/math.tan(angle/2)+2)
    for arm in arms:
        road, line = arm['road'], arm['outward']
        if road['bridge'] or road['tunnel'] or road['tags'].get('layer', '0') != '0':
            raise LayoutError('Urban junction requires one at-grade level')
        forward, backward = lane_schema(road)
        shared = road['lanes'] == 1 and forward == backward == 1
        traffic_side = road['tags'].get('driving_side', driving_side)
        model = lane_model(road, traffic_side)
        if traffic_side not in ('right', 'left'):
            raise LayoutError('Unsupported driving-side value')
        hand = 1 if traffic_side == 'right' else -1
        ends_at_junction = arm.get('ends_at_junction', True)
        available = line.length*.42 if ends_at_junction else max(0, line.length-2)
        station = min(mouth_distance, available)
        if station < largest/2+4:
            raise LayoutError('Short adjacent junction needs combined ownership')
        center, tangent, normal = frame(line, station)
        angle = math.atan2(center[1]-xy[node][1], center[0]-xy[node][0])
        mouth = {'center': center, 'normal': normal, 'width': road['width'], 'angle': angle}
        pitch = lane_pitch(road, forward, backward)
        for travel, count, opposing in [('forward', forward, backward), ('backward', backward, forward)]:
            if not count:
                continue
            if shared:
                opposing = 0
            incoming = (arm['side'] == 'before') == (travel == 'forward')
            direction = -tangent if incoming else tangent
            left = np.array([-direction[1], direction[0]])
            lane_center = center-left*opposing*pitch/2*hand
            indications, key = turn_indications(road, travel, count) if incoming else ([set()]*count, None)
            downstream = [set() for _ in range(count)]
            if not incoming and node in (road['refs'][0], road['refs'][-1]):
                downstream, _ = turn_indications(road, travel, count)
            if key and node not in (road['refs'][0], road['refs'][-1]):
                raise LayoutError('Turn-tagged way crosses an interior junction; indication extent needs review')
            approaches.append({'way': arm['key']+':'+travel, 'arm': arm['key'], 'road': road, 'mouth': mouth,
                               'outward': line, 'incoming': incoming, 'center': lane_center, 'tangent': tangent,
                               'normal': normal, 'direction': direction, 'left': left, 'station': station,
                               'lane_centers': [lane_center+left*pitch*(count/2-i-.5) for i in range(count)],
                               'pitch': pitch, 'width': count*pitch, 'count': count, 'angle': angle,
                               'travel': travel, 'opposing': opposing, 'indications': indications,
                               'driving_side': traffic_side, 'hand': hand, 'center_line': center_line,
                               'access': motor_access(road['tags']), 'shared_single_lane': shared,
                               'ends_at_junction': ends_at_junction,
                               'lane_roles': model['roles'][travel],
                               'eligible_lanes': [i for i, r in enumerate(model['roles'][travel]) if r['car_eligible']],
                               'indication_key': key, 'chain': arm['chain'], 'downstream_indications': downstream})
    return approaches


def movement_turn(source, target):
    """Classify an ordinary geometric turn from signed approach tangents, rejecting near-U-turns."""
    a, b = source['direction'], target['direction']
    angle = math.degrees(math.atan2(float(a[0]*b[1]-a[1]*b[0]), float(a @ b)))
    if abs(angle) > 155:
        raise LayoutError('Near-U-turn requires explicit turning geometry')
    return 'through' if abs(angle) < 35 else ('left' if angle > 0 else 'right')


def indication_family(values):
    """Reduce illustrative slight/sharp indications to ordinary left/right movement families."""
    return {v.split('_')[-1] if '_' in v else v for v in values}


def legal_pairs(approaches, relations, node):
    """Apply unconditional via-node restrictions for passenger cars; retain exceptions and ignored vehicle scopes."""
    incoming = [a for a in approaches if a['incoming']]
    outgoing = [a for a in approaches if not a['incoming']]
    pairs = {(a['way'], b['way']) for a in incoming for b in outgoing if a['arm'] != b['arm']}
    original, evidence = set(pairs), []
    ids = {a['road']['id'] for a in approaches}
    notes = []
    approaches[0]['relation_notes'] = notes
    for relation in relations:
        members, tags = relation['members'], relation['tags']
        relevant = any(m['role'] == 'via' and m['type'] == 'node' and int(m['ref']) == node for m in members)
        via_way = (any(m['type'] == 'way' and int(m['ref']) in ids for m in members)
                   and any(m['role'] == 'via' and m['type'] == 'way' for m in members))
        if not relevant and not via_way:
            continue
        if tags.get('type') == 'connectivity':
            if via_way:
                raise LayoutError('Via-way lane connectivity needs multi-junction resolution')
            continue
        try:
            rule, scope = restriction_for_car(tags)
        except ValueError as error:
            raise LayoutError(str(error)) from error
        if rule is None:
            notes.append({'relation': relation['id'], 'disposition': 'not applicable to private passenger car',
                          'reason': scope, 'source_tags': tags})
            continue
        if not relevant:
            raise LayoutError('Via-way restriction needs multi-junction resolution')
        if rule not in ('no_left_turn', 'no_right_turn', 'no_straight_on', 'no_u_turn',
                        'only_left_turn', 'only_right_turn', 'only_straight_on') or len(members) != 3:
            raise LayoutError('Unsupported urban restriction form')
        starts = [int(m['ref']) for m in members if m['type'] == 'way' and m['role'] == 'from']
        ends = [int(m['ref']) for m in members if m['type'] == 'way' and m['role'] == 'to']
        a = [p for p in incoming if len(starts) == 1 and p['road']['id'] == starts[0]]
        b = [p for p in outgoing if len(ends) == 1 and p['road']['id'] == ends[0]]
        if rule == 'no_u_turn' and len(starts) == 1 and starts == ends and a and b:
            evidence.append({'relation': relation['id'], 'from': starts[0], 'to': ends[0], 'restriction': rule,
                             'scope': scope, 'except': tags.get('except'),
                             'meaning': 'same-arm reverse paths already excluded; through paths remain permitted'})
            continue
        if len(a) != 1 or len(b) != 1:
            raise LayoutError('Restriction has missing/ambiguous directional from/to ports')
        pair = (a[0]['way'], b[0]['way'])
        if rule.startswith('only_'):
            pairs = {p for p in pairs if p[0] != pair[0] or p == pair}
        else:
            pairs.discard(pair)
        evidence.append({'relation': relation['id'], 'from': starts[0], 'to': ends[0], 'restriction': rule,
                         'scope': scope, 'except': tags.get('except')})
    return pairs, evidence, sorted(original-pairs)


def urban_movements(approaches, relations, node):
    """Assign ordered turns and bounded fork/peel-off hypotheses; retain every unmapped allocation explicitly."""
    pairs, restrictions, rejected = legal_pairs(approaches, relations, node)
    mapped = mapped_connections(approaches, relations, node, pairs)
    assignments = []
    outgoing = [a for a in approaches if not a['incoming'] and a.get('eligible_lanes', range(a['count']))]
    for source in (a for a in approaches if a['incoming']):
        eligible = source.get('eligible_lanes', list(range(source['count'])))
        if not eligible:
            continue
        source['allocation_inferences'] = []
        groups = defaultdict(list)
        for target in outgoing:
            if (source['way'], target['way']) in pairs:
                groups[movement_turn(source, target)].append(target)
        duplicates = any(len(targets) > 1 for targets in groups.values())
        fork = (duplicates and len([a for a in approaches if a['incoming']]) == 1
                and source['count'] == 1 and len(eligible) == 1 and len(outgoing) == 2 and set(groups) == {'through'}
                and all(a['count'] == 1 for a in outgoing) and not any(source['indications']))
        unmapped_duplicates = any(len(targets) > 1 and any((source['way'], t['way']) not in mapped for t in targets)
                                 for targets in groups.values())
        if unmapped_duplicates and not fork:
            raise LayoutError('Multiple exits in one turn family exceed the single-lane fork policy')
        if fork:
            source['allocation_inferences'].append({
                'kind': 'single_lane_fork', 'source_way': source['road']['id'],
                'target_ways': sorted(a['road']['id'] for a in outgoing),
                'basis': 'one mapped incoming lane and two mapped single-lane outgoing arms at one shared node',
                'meaning': 'alternative branches of one lane; priority and detailed turn indications unknown'})
        families = [indication_family(v) for v in source['indications']]
        restricted = any(len(a.get('eligible_lanes', range(a['count']))) != a['count'] for a in approaches)
        peel_off = (None if restricted or any((source['way'], t['way']) in mapped for t in groups.get('through', []))
                    else inferred_peel_off(source, groups, families))
        for turn, target in [(turn, target) for turn, targets in sorted(groups.items()) for target in targets]:
            exits_available = target.get('eligible_lanes', list(range(target['count'])))
            explicit = mapped.get((source['way'], target['way']))
            if explicit is not None:
                assignments.extend((source, i, target, j) for i, j in explicit['direct'])
                # A relation supersedes arrows only for this from/to pair. Keep other movement evidence intact.
                connected = {i for i, _ in explicit['direct']}
                source.setdefault('connectivity_arrow_overrides', set()).update(
                    i for i, values in enumerate(families) if turn in values and i not in connected)
                continue
            lanes = [i for i, values in enumerate(families) if turn in values and i in eligible]
            if turn == 'through':
                lanes = [i for i, values in enumerate(families) if i in eligible and (not values or turn in values)]
                if peel_off is not None:
                    lanes.remove(peel_off)
            elif not lanes:
                edge = eligible[0] if turn == 'left' else eligible[-1]
                lanes = [edge] if not families[edge] else []
            if not lanes:
                continue
            if len(lanes) > len(exits_available):
                raise LayoutError('Unexplained lane drop at urban junction; no coincident-endpoint merge inferred')
            if turn == 'left':
                exits = exits_available[:len(lanes)]
            elif turn == 'right':
                exits = exits_available[-len(lanes):]
            else:
                # Prefer continuing lanes when the exit introduces a mapped turn-only lane for its next junction.
                # This is an allocation preference, not a legal restriction on entering that lane.
                choices = itertools.combinations(exits_available, len(lanes))
                future = [indication_family(v) for v in target['downstream_indications']]
                exits = min(choices, key=lambda js: (sum(bool(future[j]) and 'through' not in future[j] for j in js),
                            sum(float((source['lane_centers'][i]-target['lane_centers'][j]) @ target['left'])**2
                                for i, j in zip(lanes, js))))
            assignments.extend((source, i, target, j) for i, j in zip(lanes, exits))
        actual = {i for a, i, _, _ in assignments if a['way'] == source['way']}
        if actual != set(eligible):
            raise LayoutError('Incoming lane has no supported exit; source arrows/restrictions need review')
        for i, indications in enumerate(families):
            if i not in eligible:
                continue
            supported = {movement_turn(a, b) for a, lane, b, _ in assignments if a['way'] == source['way'] and lane == i}
            if indications-supported and i not in source.get('connectivity_arrow_overrides', set()):
                raise LayoutError('Mapped turn indication has no compatible exit at this node')
    if not assignments:
        raise LayoutError('No eligible private-car movements; physical reserved lanes retained without a car layout')
    return assignments, restrictions, rejected


def inferred_peel_off(source, groups, families):
    """Propose a 2-to-1 one-way continuation with one unindicated edge lane leaving; never assert legal exclusivity."""
    targets = groups.get('through', [])
    sides = [turn for turn in ('left', 'right') if groups.get(turn)]
    if (source['count'] != 2 or source['opposing'] or len(targets) != 1 or targets[0]['count'] != 1
            or targets[0]['opposing'] or len(sides) != 1):
        return None
    through = [i for i, values in enumerate(families) if not values or 'through' in values]
    side = sides[0]
    lane = 0 if side == 'left' else 1
    if len(through) != 2 or families[lane] or len(groups[side]) != 1:
        return None
    source['allocation_inferences'].append({
        'kind': 'unmapped_turn_peel_off', 'source_way': source['road']['id'], 'lane': lane+1,
        'turn': side, 'target_way': groups[side][0]['road']['id'],
        'continuing_way': targets[0]['road']['id'], 'mapped_counts': [2, 1],
        'basis': '2-to-1 one-way continuation and one side exit after known restrictions; unindicated edge lane',
        'meaning': 'representative allocation hypothesis; missing turn tags or a real lane drop remain possible',
        'legal_exclusivity': 'unknown', 'painted_arrow': False})
    return lane


def arrow_glyph(center, direction, indications):
    """Build illustrative source-indicated through/left/right arrow combinations in a travel-oriented lane frame."""
    shapes = []
    left = np.array([-direction[1], direction[0]])
    families = indication_family(indications)
    if 'through' in families:
        shapes.append(straight_arrow(center, direction))
    for turn, sign in [('left', 1), ('right', -1)]:
        if turn not in families:
            continue
        outline = [(-.10, -1.8), (.10, -1.8), (.10, .35), (.9, .35), (.9, -.05),
                   (1.6, .55), (.9, 1.15), (.9, .75), (-.10, .75)]
        shapes.append(Polygon([center+left*(x*sign)+direction*y for x, y in outline]))
    return unary_union(shapes)


def lane_curve(source, lane, target, exit_lane, ground):
    """Fit a tangent-continuous straight/arc/straight turn inside the curb envelope; reject inadequate radius."""
    start, end = source['lane_centers'][lane], target['lane_centers'][exit_lane]
    u, v = source['direction'], target['direction']
    angle = math.atan2(float(u[0]*v[1]-u[1]*v[0]), float(u @ v))
    candidates = []
    if abs(angle) < math.radians(35):
        for fraction in (.25, .35, .45):
            candidates.append(curve(start, end, u, v, np.linalg.norm(end-start)*fraction))
    else:
        first, last = np.linalg.solve(np.column_stack([u, v]), end-start)
        tangent_ratio = math.tan(abs(angle)/2)
        maximum = min(first, last)/tangent_ratio*.999
        if maximum < 3.5:
            raise LayoutError('Turn has insufficient room for the 3.5 m centerline-radius floor')
        # The largest supported radius up to 12 m wins; no corner is cut through a protected block.
        for radius in np.linspace(3.5, min(12, maximum), 36):
            leg1, leg2 = first-radius*tangent_ratio, last-radius*tangent_ratio
            arc_start = start+u*leg1
            sign = 1 if angle > 0 else -1
            circle = arc_start+np.array([-u[1], u[0]])*radius*sign
            vector = arc_start-circle
            arc_length = abs(angle)*radius
            length = leg1+arc_length+leg2
            samples = np.linspace(0, length, 513)
            points = np.empty((len(samples), 2))
            before = samples < leg1
            during = ~before & (samples <= leg1+arc_length)
            after = ~(before | during)
            points[before] = start+samples[before, None]*u
            points[after] = end-(length-samples[after, None])*v
            angles = sign*(samples[during]-leg1)/radius
            # Keep the scalar libm trig functions for identical curve coordinates while batching vector arithmetic.
            cosines = np.fromiter((math.cos(theta) for theta in angles), dtype=float)
            sines = np.fromiter((math.sin(theta) for theta in angles), dtype=float)
            points[during] = circle+np.column_stack([vector[0]*cosines-vector[1]*sines,
                                                     vector[0]*sines+vector[1]*cosines])
            points[0], points[-1] = start, end
            candidates.append(points)
    # The two-meter guard exceeds both the 0.95m vehicle envelope and 0.001m curb tolerance.
    # Artificial clip edges therefore cannot affect any trial, even at the source footprint boundary.
    locations = np.concatenate(candidates)
    low, high = locations.min(axis=0)-2, locations.max(axis=0)+2
    envelope = ground.intersection(box(*low, *high)).buffer(.001)
    scores = [LineString(points).buffer(.95, cap_style=2).difference(envelope).area
              for points in candidates]
    supported = [i for i, score in enumerate(scores) if score < .001]
    index = supported[-1] if supported else int(np.argmin(scores))
    points = candidates[index]
    if not supported:
        raise LayoutError(f'Urban turn envelope cannot fit existing curb returns ({scores[index]:.3f} m2 outside)')
    tangent = np.gradient(points, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    normal[0], normal[-1] = source['left'], target['left']
    width = np.linspace(source['pitch'], target['pitch'], len(points))
    return points, points+normal*width[:, None]/2, points-normal*width[:, None]/2


@lru_cache(maxsize=1)
def bus_outline():
    """Cache a centered unit BUS outline with glyph holes intact; final paint is conventional vector geometry."""
    from matplotlib.font_manager import FontProperties
    from matplotlib.textpath import TextPath
    path = TextPath((0, 0), 'BUS', size=1, prop=FontProperties(family='DejaVu Sans', weight='bold'))
    outline = Polygon()
    for ring in path.to_polygons():
        outline = outline.symmetric_difference(Polygon(ring))
    west, south, east, north = outline.bounds
    return shapely.transform(outline, lambda p: (p-[(west+east)/2, (south+north)/2])/[east-west, north-south])


def bus_label_paint(layout, exclusions, arrows):
    """Place inferred BUS words in source-designated incoming lanes; retain omissions and access uncertainty."""
    patches, records = [], []
    occupied = unary_union(arrows)
    if exclusions is not None:
        occupied = occupied.union(exclusions)
    for port in layout['approaches']:
        if not port['incoming'] or port['road']['tags'].get('lane_markings') == 'no':
            continue
        limit = (port['outward'].length*.49 if port.get('ends_at_junction', True)
                 else max(0, port['outward'].length-2))
        for role in port.get('lane_roles', []):
            if not set(role['designated_modes']) & {'bus', 'psv'} or role['bus_access'] == 'no':
                continue
            record = {'port': port['way'], 'source_way': port['road']['id'], 'lane': role['lane'], 'text': 'BUS',
                      'rendered': False, 'basis': 'INFERRED word and placement from SOURCE designated lane role',
                      'car_access': role['car_access'], 'exclusive_bus_use': 'not asserted',
                      'width_m': min(2.4, port['pitch']-.5), 'length_m': 4.5}
            station = port['station']+7
            while station+2.5 < limit:
                origin, tangent, _ = frame(port['outward'], station)
                direction = -tangent
                left = np.array([-direction[1], direction[0]])
                lane = role['lane']-1
                center = origin+left*port['pitch']*((port['count']-port.get('hand', 1)*port['opposing'])/2-lane-.5)
                glyph = shapely.transform(bus_outline(), lambda p: center
                                          -left*p[:, :1]*record['width_m']+direction*p[:, 1:]*record['length_m'])
                if buffered_geometry(layout['pavement'], -.15).covers(glyph) and glyph.intersection(occupied).area < 1e-8:
                    patches.append(glyph)
                    occupied = occupied.union(glyph)
                    record.update(rendered=True, position_xy=center.tolist(), station_from_junction_m=station)
                    break
                station += 1
            if not record['rendered']:
                record['reason'] = 'No label placement clear of controls, crossings, arrows, and the approach handoff'
            records.append(record)
    layout['lane_label_evidence'] = records
    return patches


def approach_paint(layout, exclusions=None):
    """Paint lane stems and only mapped turn arrows; leave the shared intersection box free of guessed stripes."""
    white, yellow, arrows, evidence = [], [], [], []
    center = Point(layout['position'])
    core = layout.get('junction_core', center.buffer(max(a['road']['width'] for a in layout['approaches'])/2+3))
    for port in layout['approaches']:
        direction_sign = -1 if port['incoming'] else 1
        hand = port.get('hand', 1)
        start = max(0, port['station']-7)
        paint_limit = (port['outward'].length*.49 if port.get('ends_at_junction', True)
                       else max(0, port['outward'].length-2))
        stop = min(paint_limit, port['station']+12)
        line = substring(port['outward'], start, stop)
        separators = port['road']['tags'].get('lane_markings') != 'no'
        for lane in range(1, port['count']) if separators else ():
            offset = direction_sign*port['pitch']*((port['count']-hand*port['opposing'])/2-lane)
            separator = line.offset_curve(offset)
            white.append(dashed(separator, 3, 9).difference(core))
        if separators and port['opposing'] and port['incoming']:
            offset = direction_sign*hand*port['pitch']*(port['count']-port['opposing'])/2
            if port.get('center_line', 'double_yellow') == 'double_yellow':
                for delta in (-.12, .12):
                    yellow.append(line.offset_curve(offset+delta).buffer(.05, cap_style=2).difference(core))
            else:
                white.append(dashed(line.offset_curve(offset), 3, 6).difference(core))
        if not port['incoming']:
            continue
        for lane, indications in enumerate(port['indications']):
            if not indications:
                continue
            station = port['station']+7
            record = {'source_way': port['road']['id'], 'port': port['way'], 'lane': lane+1,
                      'tag': port['indication_key'], 'indications': sorted(indications), 'rendered': False,
                      'placement': 'inferred upstream of crossing/stop exclusions; source-indicated arrow'}
            if lane not in port.get('eligible_lanes', range(port['count'])):
                record['reason'] = 'private-car profile does not resolve this physical lane; source indication retained'
                evidence.append(record)
                continue
            if lane in port.get('connectivity_arrow_overrides', set()):
                record['reason'] = 'mapped connectivity overrides conflicting turn indication; glyph withheld'
                evidence.append(record)
                continue
            while station < paint_limit:
                origin, tangent, _ = frame(port['outward'], station)
                direction = -tangent
                left = np.array([-direction[1], direction[0]])
                position = origin+left*port['pitch']*((port['count']-hand*port['opposing'])/2-lane-.5)
                glyph = arrow_glyph(position, direction, indications)
                if (exclusions is None or glyph.intersection(exclusions).area < 1e-8):
                    if buffered_geometry(layout['pavement'], -.1).covers(glyph):
                        arrows.append(glyph)
                        record.update(rendered=True, position_xy=position.tolist(), station_from_junction_m=station,
                                      paint_area_m2=glyph.area)
                        break
                station += 1
            evidence.append(record)
    labels = bus_label_paint(layout, exclusions, arrows)
    layout['paint'] = unary_union([unary_union(white).intersection(layout['domain']), *arrows, *labels])
    inset = buffered_geometry(layout['pavement'], -.1)
    layout['paint'] = layout['paint'].intersection(inset)
    layout['yellow_paint'] = unary_union(yellow).intersection(layout['domain']).intersection(inset)
    layout['arrow_count'] = len(arrows)
    layout['evidence'] = evidence


def discover_urban(roads, xy, relations, controls, ground, islands, region,
                   driving_side='right', center_line='double_yellow', compact_pavement=False):
    """Resolve urban junctions; compact mode stores paint context and applies accepted domains to the shared pavement."""
    incident = defaultdict(list)
    for road in roads:
        if road['tags']['highway'] != 'cycleway':
            for node in road['refs']:
                incident[node].append(road)
    nodes = {n for n, rs in incident.items()
             if sum(1 if n in (r['refs'][0], r['refs'][-1]) else 2 for r in rs) >= 3}
    from clusters import discover_clusters
    layouts, ground, owned, attempts = discover_clusters(roads, xy, relations, ground, islands, region, nodes,
                                                        driving_side, center_line, compact_pavement)
    islands_removed = bool(layouts)
    diagnostics = []
    for node in sorted(nodes):
        if not region.covers(Point(xy[node])):
            continue
        record = {'node': node, 'kind': 'urban', 'source_ways': sorted(r['id'] for r in incident[node])}
        if node in owned:
            layout = owned[node]
            record.update(status='resolved', name=layout['name'], physical_arms=layout['arm_count'],
                          kind='urban_cluster', cluster_nodes=layout['cluster_nodes'])
            diagnostics.append(record)
            continue
        if node in attempts:
            record['cluster_attempt'] = attempts[node]
        try:
            arms = extend_arms(physical_arms(roads, xy, node, nodes), roads, xy, nodes)
            ports = urban_approaches(arms, xy, node, driving_side, center_line)
            if not region.buffer(.001).covers(junction_domain(ports)):
                raise LayoutError('Urban junction reaches beyond the render crop')
            layout = build_layout(roads, xy, relations, ground, islands, node, kind='urban',
                                  name=f'junction-{node}', approaches=ports, compact_pavement=compact_pavement)
            if any(layout['domain'].intersection(other['domain']).area > .001 for other in layouts):
                raise LayoutError('Urban ownership domains overlap; combined cluster required')
            approach_paint(layout)
            audit_layout(layout, layout['pavement'])
            layout['arm_count'] = len(arms)
            layouts.append(layout)
            if compact_pavement and not islands_removed:
                # Match the full-pavement path's global island removal at its first accepted junction.
                ground = ground.difference(islands)
                islands_removed = True
            ground = (ground.difference(layout['domain']).union(layout['pavement'].intersection(layout['domain']))
                      if compact_pavement else layout['pavement'])
            record.update(status='resolved', name=layout['name'], physical_arms=len(arms))
            inferences = [r for p in ports for r in p.get('allocation_inferences', [])]
            if inferences:
                record['allocation_inferences'] = inferences
        except LayoutError as error:
            record.update(status='unresolved', reason=str(error))
        diagnostics.append(record)
    return layouts, ground, diagnostics


def bounded_mask(geometry, bounds):
    """Cull overlay masks outside a guarded query box; invalid fast clips retain the original exact mask."""
    xmin, ymin, xmax, ymax = bounds
    clipped = shapely.clip_by_rect(geometry, xmin-.01, ymin-.01, xmax+.01, ymax+.01)
    return clipped if clipped.is_valid else geometry


def street_paint(roads, exclusions, ground, driving_side='right', center_line='double_yellow'):
    """Extend directional separators across urban blocks, leaving unsupported lane schemas unpainted."""
    white, yellow = [], []
    for road in roads:
        if road['bridge'] or road['tunnel'] or road['tags'].get('lane_markings') == 'no':
            continue
        try:
            forward, backward = lane_schema(road)
            pitch = lane_pitch(road, forward, backward)
        except LayoutError:
            continue
        count = forward+backward
        if road['lanes'] == 1 and forward and backward:
            continue
        side = road['tags'].get('driving_side', driving_side)
        if side not in ('left', 'right'):
            continue
        division = backward if side == 'right' else forward
        usable = road['surface'].buffer(-.2)
        if usable.is_empty:
            continue
        local_ground = bounded_mask(ground, usable.bounds)
        local_exclusions = bounded_mask(exclusions, usable.bounds)
        usable = usable.intersection(local_ground).difference(local_exclusions)
        for index in range(1, count):
            offset = pitch*(count/2-index)
            if forward and backward and index == division and center_line == 'double_yellow':
                for delta in (-.12, .12):
                    yellow.append(road['line'].offset_curve(offset+delta).buffer(.05, cap_style=2).intersection(usable))
            elif forward and backward and index == division:
                white.append(dashed(road['line'].offset_curve(offset), 3, 6).intersection(usable))
            else:
                white.append(dashed(road['line'].offset_curve(offset), 3, 9).intersection(usable))
    return unary_union(white), unary_union(yellow)


def urban_views(layouts, height, to_geo):
    """Create aerial and pedestrian views selected by layout evidence and arm count, without source-ID fixtures."""
    views = {}
    cases = {'turn-lanes': max(layouts, key=lambda j: j['arrow_count'])}
    for field, name in [('shared_single_lane', 'shared-lane'), ('lane_connectivity', 'mapped-lanes')]:
        candidates = [j for j in layouts if any(a.get(field) for a in j['approaches'])]
        if candidates:
            cases[name] = max(candidates, key=lambda j: len(j['connectors']))
    clusters = [j for j in layouts if 'cluster_nodes' in j]
    if clusters:
        cases['compound'] = max(clusters, key=lambda j: len(j['connectors']))
    role_cases = [j for j in layouts if any(r['designated_modes'] or not r['car_eligible']
                  for a in j['approaches'] for r in a.get('lane_roles', []))]
    if role_cases:
        cases['reserved-lanes'] = max(role_cases, key=lambda j: (
            sum(r['rendered'] for r in j.get('lane_label_evidence', [])), len(j['connectors'])))
    for count, name in [(3, 'three-way'), (4, 'four-way'), (5, 'five-way')]:
        candidates = [j for j in layouts if j['arm_count'] == count]
        if candidates:
            cases[name] = max(candidates, key=lambda j: len(j['connectors']))
    for kind, name in [('single_lane_fork', 'fork'), ('unmapped_turn_peel_off', 'inferred-turn')]:
        candidates = [j for j in layouts if any(r['kind'] == kind for a in j['approaches']
                                               for r in a.get('allocation_inferences', []))]
        if candidates:
            cases[name] = max(candidates, key=lambda j: len(j['connectors']))
    for name, layout in cases.items():
        center = layout['position']
        lon, lat = to_geo.transform(*center)
        z = float(height(*center)[0])
        views[name] = (lon, lat, -15, -70, 95, z)
        incoming = max((a for a in layout['approaches'] if a['incoming']),
                       key=lambda a: (sum(bool(v) for v in a['indications']), a['outward'].length))
        station = min(incoming['outward'].length*.7, incoming['station']+19)
        eye, tangent, _ = frame(incoming['outward'], station)
        direction = -tangent
        left = np.array([-direction[1], direction[0]])
        eye += left*incoming['pitch']*((incoming['count']-incoming.get('hand', 1)*incoming['opposing'])/2-.5)
        delta = center-eye
        dz = z+1.1-float(height(*eye)[0])-1.7
        views[name+'-street'] = (lon, lat, np.degrees(np.arctan2(delta[0], delta[1])),
                                np.degrees(np.arctan2(dz, np.linalg.norm(delta))),
                                np.linalg.norm([*delta, dz]), z+1.1)
    return views


def urban_stop_bars(records, roads, layouts, ground, crossing_passages, driving_side='right'):
    """Infer approach bars from mapped junction signals or directional stops, retaining per-approach assumptions."""
    from controls import build_stop_bars
    patches, exclusions, consumed = [], [], set()
    for record in records:
        record['bars'] = []
    for layout in layouts:
        for port in (a for a in layout['approaches'] if a['incoming']):
            matches = []
            for record in records:
                if record['element'] != 'node' or record['kind'] not in ('traffic_signals', 'stop'):
                    continue
                if any('conditional' in key for key in record['tags']):
                    continue
                root_node = port.get('junction_node', layout['node'])
                at_junction = record['node'] == root_node and record['kind'] == 'traffic_signals'
                on_arm = (record['node'] != root_node and port['road']['id'] in record['roads']
                          and record['direction'] in (port['travel'], 'both')
                          and port['outward'].distance(Point(record['position'])) < .1
                          and port['outward'].project(Point(record['position'])) < port['station']+8)
                if at_junction or on_arm:
                    matches.append((not on_arm, record, at_junction))
            if not matches:
                continue
            _, record, at_junction = sorted(matches, key=lambda item: (item[0], item[1]['node']))[0]
            station = max(a['road']['width'] for a in layout['approaches'])/2+4.5
            if not at_junction:
                station = max(station, port['outward'].project(Point(record['position'])))
            # Move upstream of every crossing passage encountered inside this approach's owned mouth.
            passage = crossing_passages.intersection(substring(port['outward'], 0, port['station']+3).buffer(.2))
            if not passage.is_empty:
                import shapely
                station = max(station, max(port['outward'].project(Point(p))
                                           for p in shapely.get_coordinates(passage))+1.0)
            available = (port['outward'].length*.48 if port.get('ends_at_junction', True)
                         else max(0, port['outward'].length-1))
            limit = min(available, port['station']+7)
            while station <= limit:
                origin, tangent, _ = frame(port['outward'], station)
                left = np.array([tangent[1], -tangent[0]])
                center = origin-left*port['opposing']*port['pitch']/2*port.get('hand', 1)
                half = port['width']/2-.1
                bar = LineString([center-left*half, center+left*half])
                paint = bar.buffer(.20, cap_style=2)
                if paint.intersection(crossing_passages).area < .001 and buffered_geometry(ground, .003).covers(paint):
                    break
                station += .5
            else:
                record.setdefault('unresolved_approaches', []).append(port['way'])
                continue
            detail = {'layout': layout['name'], 'port': port['way'], 'source_way': port['road']['id'],
                      'bar_xy': list(bar.coords), 'paint_area_m2': paint.area, 'stroke_width_m': .40,
                      'station_from_junction_m': station,
                      'placement_displacement_m': Point(record['position']).distance(bar.centroid),
                      'geometry_basis': 'INFERRED before crossing; no separately mapped stop-line alignment',
                      'control_scope': ('INFERRED all incoming approaches from shared-node junction signal; '
                                        'direction tag is ambiguous across incident ways'
                                        if at_junction else 'mapped directional approach control'),
                      'signal_phase': 'unknown'}
            record['bars'].append(detail)
            record['rendered'] = True
            consumed.add(record['node'])
            patches.append(paint)
            exclusions.append(bar.buffer(1.0, cap_style=2))
    # Preserve directional controls outside resolved junctions using the existing conservative policy.
    remainder = [r for r in records if r.get('node') not in consumed]
    paint, mask = build_stop_bars(remainder, roads, [], ground, crossing_passages, urban=True, driving_side=driving_side)
    patches.append(paint)
    exclusions.append(mask)
    for record in remainder:
        if record['rendered']:
            record['bars'] = [{key: record[key] for key in
                               ('bar_xy', 'paint_area_m2', 'stroke_width_m', 'geometry_basis')}]
    return unary_union(patches), unary_union(exclusions)
