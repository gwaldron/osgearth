"""Associate bridge families with retained OSM objects, independently of the solved road receiver graph."""

from collections import defaultdict
import gzip
import xml.etree.ElementTree as ET

import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import polygonize


def read_evidence(path, to_local):
    """Read complete outline/support geometries and obstacle masks; retain incomplete relations as diagnostics."""
    root = ET.fromstring(gzip.decompress(path.read_bytes()))
    nodes = {int(e.get('id')): to_local.transform(float(e.get('lon')), float(e.get('lat')))
        for e in root.findall('node')}
    elements = {(e.tag, int(e.get('id'))): e for e in root if e.tag in ('node', 'way', 'relation')}
    geometries, diagnostics, visiting = {}, [], set()

    def geometry(key):
        """Resolve ways and complete multipolygon rings without closing a missing source boundary by inference."""
        if key in geometries:
            return geometries[key]
        if key in visiting:
            return None
        visiting.add(key)
        element = elements.get(key)
        result = None
        if element is not None and key[0] == 'node':
            result = Point(nodes[key[1]])
        elif element is not None and key[0] == 'way':
            refs = [int(n.get('ref')) for n in element.findall('nd')]
            if len(refs) >= 2 and all(n in nodes for n in refs):
                coords = [nodes[n] for n in refs]
                result = Polygon(coords) if len(refs) >= 4 and refs[0] == refs[-1] else LineString(coords)
        elif element is not None and key[0] == 'relation':
            rings = defaultdict(list)
            complete = True
            for member in element.findall('member'):
                if member.get('role', '') not in ('', 'outer', 'inner'):
                    continue
                part = geometry((member.get('type'), int(member.get('ref'))))
                if part is None or part.geom_type not in ('Polygon', 'LineString'):
                    complete = False
                    break
                rings[member.get('role') or 'outer'].append(part.boundary if part.geom_type == 'Polygon' else part)
            if complete and rings['outer']:
                outer = shapely.union_all(list(polygonize(rings['outer'])))
                inner = shapely.union_all(list(polygonize(rings['inner'])))
                # A closed relation needs every outer segment accounted for.
                if not outer.is_empty and shapely.union_all(rings['outer']).difference(outer.boundary.buffer(1e-7)).is_empty:
                    result = outer.difference(inner)
        if result is not None:
            result = shapely.make_valid(result)
        geometries[key] = result
        visiting.remove(key)
        return result

    outlines, supports, obstacles = [], [], []
    for key, element in elements.items():
        tags = {t.get('k'): t.get('v') for t in element.findall('tag')}
        water = tags.get('natural') == 'water' or tags.get('waterway') in ('river', 'stream', 'canal', 'riverbank')
        railway = tags.get('railway') in ('rail', 'light_rail', 'tram', 'narrow_gauge')
        relevant = tags.get('man_made') == 'bridge' or 'bridge:support' in tags or water or railway
        if not relevant:
            continue
        part = geometry(key)
        identifier = key[0]+'/'+str(key[1])
        if part is None or part.is_empty:
            diagnostics.append({'source': identifier, 'reason': 'Incomplete source geometry', 'tags': tags})
            continue
        record = {'source': identifier, 'tags': tags, 'geometry': part}
        if tags.get('man_made') == 'bridge' and part.geom_type in ('Polygon', 'MultiPolygon'):
            outlines.append(record)
        if 'bridge:support' in tags:
            supports.append(record)
        if water or railway:
            # Waterlines without bank polygons get a conservative minimum corridor, explicitly inferred.
            mask = part if part.geom_type in ('Polygon', 'MultiPolygon') else part.buffer(8 if water else 3)
            obstacles.append(dict(record, geometry=mask, kind='water' if water else 'railway'))
    return {'outlines': outlines, 'supports': supports, 'obstacles': obstacles, 'diagnostics': diagnostics}


def select_assets(network, decks, evidence, region):
    """Group mapped bridge spans by a unique compatible outline; ambiguous or unsupported families stay explicit."""
    owners = {i: deck for deck in decks for i in deck.indices}
    groups = defaultdict(list)
    selections = {}
    for i, road in enumerate(network.roads):
        if i not in owners or not road['bridge']:
            continue
        candidates = []
        for outline in evidence['outlines']:
            level = outline['tags'].get('layer')
            if level is not None and road['tags'].get('layer', '0') != level:
                continue
            if road['line'].intersection(outline['geometry'].buffer(.3)).length >= road['line'].length*.9:
                candidates.append(outline)
        direct = road['tags'].get('bridge:structure')
        types = {o['tags']['bridge:structure'] for o in candidates if o['tags'].get('bridge:structure')}
        if direct:
            types.add(direct)
        selected = candidates[0] if len(candidates) == 1 else None
        conflict = len(types) > 1 or len(candidates) > 1
        family = next(iter(types)) if len(types) == 1 else 'beam' if not types else None
        movable = road['tags'].get('bridge') == 'movable' or 'bridge:movable' in road['tags']
        movable |= any(o['tags'].get('bridge') == 'movable' or 'bridge:movable' in o['tags'] for o in candidates)
        key = selected['source'] if selected is not None else 'way/'+str(road['id'])
        # Distinct mapped span types on a common outline must not silently become one family.
        key += '/'+str(family)
        groups[key].append(i)
        selections[i] = {'source_tags': road['tags'], 'outlines': [
            {'source': o['source'], 'tags': o['tags']} for o in candidates],
            'family': family, 'family_status': 'mapped' if types else 'inferred generic beam',
            'deferred': 'Conflicting bridge attribution' if conflict else 'Movable mechanism deferred' if movable else None}
    result = []
    for key, indices in sorted(groups.items()):
        roads = [network.roads[i] for i in indices]
        selection = [selections[i] for i in indices]
        family = selection[0]['family']
        reason = next((s['deferred'] for s in selection if s['deferred']), None)
        if family not in ('beam', 'truss', 'arch', 'suspension', 'cable-stayed'):
            reason = reason or 'Unsupported mapped bridge family'
        pavement = shapely.union_all([r['surface'].intersection(owners[i].pavement) for i, r in zip(indices, roads)])
        if any(not region.buffer(-.1).covers(r['line']) for r in roads):
            reason = reason or 'Bridge alignment clipped by region boundary'
        mapped_supports = [s for s in evidence['supports'] if s['geometry'].distance(pavement) < 3]
        result.append({'id': key, 'indices': indices, 'family': family,
            'family_status': selection[0]['family_status'], 'source_evidence': selection,
            'source_ways': [r['id'] for r in roads], 'pavement': pavement, 'deferred': reason,
            'mapped_supports': mapped_supports})
    return result
