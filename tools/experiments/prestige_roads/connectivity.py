"""Parse bounded OSM lane connectivity without turning lane-change possibilities into default lane paths."""

import re

from intersections import LayoutError


def parse_connections(value, source_count, target_count):
    """Return zero-based direct and lane-change pairs; reject malformed, repeated, shared, or out-of-range lanes."""
    direct, changes, seen = [], [], set()
    for field in value.split('|'):
        match = re.fullmatch(r'([1-9][0-9]*):(.+)', field)
        if match is None:
            raise LayoutError('Unsupported lane connectivity syntax or shared both-ways lane')
        source = int(match[1])-1
        if source >= source_count or source in seen:
            raise LayoutError('Mapped connectivity repeats or exceeds an incoming lane')
        seen.add(source)
        targets = set()
        for token in match[2].split(','):
            destination = re.fullmatch(r'([1-9][0-9]*)|\(([1-9][0-9]*)\)', token)
            if destination is None:
                raise LayoutError('Unsupported lane connectivity destination')
            target = int(destination[1] or destination[2])-1
            if target >= target_count or target in targets:
                raise LayoutError('Mapped connectivity repeats or exceeds an outgoing lane')
            targets.add(target)
            (changes if destination[2] else direct).append((source, target))
    # An actual lane drop needs longitudinal geometry; two default paths may not collapse at one endpoint.
    if len({target for _, target in direct}) != len(direct):
        raise LayoutError('Mapped many-to-one connectivity needs explicit lane-drop geometry')
    return sorted(direct), sorted(changes)


def mapped_connections(approaches, relations, node, legal_pairs):
    """Resolve unique via-node relation ports and retain source evidence; reject conflicts with access restrictions."""
    result, records = {}, []
    approaches[0]['lane_connectivity'] = records
    for port in approaches:
        port['connectivity_arrow_overrides'] = set()
    for relation in relations:
        tags, members = relation['tags'], relation['members']
        if tags.get('type') != 'connectivity':
            continue
        if not any(m['role'] == 'via' and m['type'] == 'node' and int(m['ref']) == node for m in members):
            continue
        if len(members) != 3 or any(k.startswith('connectivity:') for k in tags):
            raise LayoutError('Qualified or multi-member lane connectivity needs an explicit policy')
        starts = [int(m['ref']) for m in members if m['type'] == 'way' and m['role'] == 'from']
        ends = [int(m['ref']) for m in members if m['type'] == 'way' and m['role'] == 'to']
        source = [a for a in approaches if a['incoming'] and len(starts) == 1 and a['road']['id'] == starts[0]]
        target = [a for a in approaches if not a['incoming'] and len(ends) == 1 and a['road']['id'] == ends[0]]
        if len(source) != 1 or len(target) != 1:
            raise LayoutError('Lane connectivity has missing or ambiguous directional ports')
        a, b = source[0], target[0]
        if any(len(p.get('eligible_lanes', range(p['count']))) != p['count'] for p in (a, b)):
            raise LayoutError('Mapped connectivity across reserved lanes needs a vehicle-scoped relation policy')
        pair = (a['way'], b['way'])
        if pair not in legal_pairs:
            raise LayoutError('Mapped lane connectivity conflicts with a movement restriction or U-turn policy')
        direct, changes = parse_connections(tags.get('connectivity', ''), a['count'], b['count'])
        value = {'direct': direct, 'lane_changes': changes}
        if pair in result and result[pair] != value:
            raise LayoutError('Conflicting connectivity relations for the same directional port pair')
        result[pair] = value
        records.append({'relation': relation['id'], 'from_port': pair[0], 'to_port': pair[1],
                        'source_tags': tags, 'default_connections': [[i+1, j+1] for i, j in direct],
                        'lane_change_connections': [[i+1, j+1] for i, j in changes],
                        'basis': 'SOURCE lane assignment; geometry inferred; parenthesized lane changes not rendered'})
    return result
