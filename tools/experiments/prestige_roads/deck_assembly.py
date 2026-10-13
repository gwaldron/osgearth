"""Assemble source-connected deck sheets without collapsing stacked road crossings into one XY polygon."""

from collections import defaultdict
from itertools import combinations
from types import SimpleNamespace

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon


def structure_joins(network, structures, region, ordered, joins):
    """Complete local structure joins from shared OSM nodes, retaining all explicitly ordered crossing patches."""
    patches = defaultdict(list)
    for a, b, patch in joins:
        patches[tuple(sorted((a, b)))].append(patch)
    protected = defaultdict(list)
    for a, b, patch in ordered:
        protected[tuple(sorted((a, b)))].append(patch)
    evidence = []
    for node, index in network.node_index.items():
        incident = sorted(network.incident[index])
        for a, b in combinations(incident, 2):
            if not (structures[a] and structures[b]):
                continue
            roads = [network.roads[i] for i in (a, b)]
            overlap = roads[0]['surface'].intersection(roads[1]['surface']).intersection(region)
            overlap = overlap.difference(shapely.union_all(protected[(a, b)]))
            seed = Point(network.xy[index]).buffer(max(r['width'] for r in roads))
            # Follow the connected overlap to its end: an acute merge can extend well beyond a radius search.
            for part in shapely.get_parts(overlap):
                if part.intersects(seed):
                    patches[(a, b)].append(part)
            # Degree-two source splits often share only an end cap, with no positive-area overlap.
            contact = roads[0]['surface'].boundary.intersection(roads[1]['surface'].boundary)
            contact = contact.intersection(seed).intersection(region)
            if not contact.is_empty:
                patches[(a, b)].append(contact)
            if not overlap.is_empty or not contact.is_empty:
                evidence.append({'ways': [r['id'] for r in roads], 'node': node,
                    'basis': 'Mapped shared node; connected overlap component; ordered crossings protected'})
    return [(a, b, shapely.union_all(parts)) for (a, b), parts in sorted(patches.items())], evidence


def separate_inferred_edges(network, structures, owned, ordered, joins):
    """Remove unconnected, same-layer edge collisions caused by guessed widths; never trim an ordered crossing."""
    protected = {tuple(sorted((a, b))) for a, b, patch in ordered}
    connected = {tuple(sorted((a, b))) for a, b, patch in joins}
    active = [i for i, flag in enumerate(structures) if flag and not owned[i].is_empty]
    tree = shapely.STRtree([owned[i] for i in active])
    evidence = []
    for k, a in enumerate(active):
        for index in tree.query(owned[a], predicate='intersects'):
            b = active[int(index)]
            if b <= a or (a, b) in protected or (a, b) in connected:
                continue
            ra, rb = network.roads[a], network.roads[b]
            if ra['tags'].get('layer', '0') != rb['tags'].get('layer', '0'):
                continue
            if any(r.get('width_reason') != 'lane-based prototype default' for r in (ra, rb)):
                continue
            overlap = owned[a].intersection(owned[b])
            if overlap.area < 1e-7:
                continue
            # Preserve each road's center corridor. A genuine unresolved crossing needs attribution, not trimming.
            if any(overlap.distance(r['line']) < min(1.5, r['width']/4) for r in (ra, rb)):
                continue
            owned[a] = owned[a].difference(overlap)
            owned[b] = owned[b].difference(overlap)
            evidence.append({'ways': [ra['id'], rb['id']], 'removed_union_area_m2': overlap.area,
                'basis': 'Unconnected same-layer edges overlap with inferred widths; retain separate receivers',
                'source_tags_unchanged': True, 'inference': 'Omit ambiguous edge strip from both decks'})
    return evidence


def assemble(jobs, network, joins, structures, region, ordered=()):
    """Return continuous, paint-partitioned deck sheets and ownership evidence; reject XY unions across levels."""
    by_index = {job.road_index: job for job in jobs}
    parents = {i: i for i in by_index}
    members = {i: {i} for i in by_index}
    allowed = {tuple(sorted((a, b))): patch for a, b, patch in joins}

    def root(i):
        """Resolve a compiler-owned sheet identifier without changing the source road graph."""
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i

    deferred = []
    for a, b, patch in joins:
        if a not in by_index or b not in by_index:
            continue
        ra, rb = root(a), root(b)
        if ra == rb:
            continue
        if by_index[a].pavement.distance(by_index[b].pavement) > 1e-6:
            continue
        safe = True
        for x in members[ra]:
            for y in members[rb]:
                overlap = by_index[x].pavement.intersection(by_index[y].pavement)
                permitted = allowed.get(tuple(sorted((x, y))), Polygon())
                if overlap.difference(permitted.buffer(1e-7)).area > 1e-7:
                    safe = False
        if not safe:
            deferred.append({'ways': [network.roads[i]['id'] for i in (a, b)],
                'reason': 'A connected network crosses itself in XY; keep distinct deck sheets'})
            continue
        parents[rb] = ra
        members[ra].update(members.pop(rb))
    result = []
    for indices in sorted(members.values(), key=lambda group: min(network.roads[i]['id'] for i in group)):
        pieces = [by_index[i] for i in sorted(indices)]
        pavement = shapely.union_all([p.pavement for p in pieces])
        paint = {m: shapely.union_all([p.materials[m] for p in pieces]) for m in ('white', 'yellow')}
        neighbors = {outside for a, b, patch in joins for inside, outside in ((a, b), (b, a))
            if inside in indices and outside in by_index and outside not in indices}
        domain = DeckField(pieces, network, [by_index[i] for i in sorted(neighbors)], allowed)
        from deck_transitions import taper_joins
        pavement, transitions = taper_joins(pavement, indices, network, region, domain, ordered)
        openings, fascia_openings = [], []
        for a, b, patch in joins:
            inside = a if a in indices else b if b in indices else None
            outside = b if inside == a else a
            if inside is not None and outside not in indices:
                # Open the full join mouth; a neighboring sheet or terrain road must remain driveable.
                openings.append(network.roads[outside]['surface'].intersection(patch.buffer(1)).buffer(.4))
                if outside in by_index:
                    fascia_openings.append(by_index[outside].pavement.buffer(1e-7).intersection(patch.buffer(1e-7)))
        for piece in pieces:
            road = network.roads[piece.road_index]
            coords = np.asarray(road['line'].coords)
            for k, adjacent in ((0, 1), (-1, -2)):
                node = network.node_index[road['refs'][k]]
                if (network.incident[node] & indices)-{piece.road_index}:
                    continue
                tangent = coords[k]-coords[adjacent]
                tangent /= np.linalg.norm(tangent)
                side = np.array([-tangent[1], tangent[0]])
                cap = LineString([coords[k]-side*(road['width']/2+1), coords[k]+side*(road['width']/2+1)])
                openings.append(cap.buffer(.5, cap_style=2))
            if road['tags'].get('bridge:railing', road['tags'].get('railing')) == 'no':
                openings.append(piece.pavement.buffer(1))
            if not road['bridge'] and piece.road_index not in getattr(network, 'raised_receivers', set()):
                openings.append(piece.pavement.buffer(1))
        openings.append(region.boundary.buffer(.5))
        paths = pavement.buffer(-.20, join_style=2).boundary.difference(shapely.union_all(openings))
        paths = shapely.line_merge(paths)
        identifier = 'deck/' + str(min(network.roads[i]['id'] for i in indices))
        result.append(SimpleNamespace(pavement=pavement, materials=paint, height=domain.height,
            thickness=domain.thickness, indices=sorted(indices), surface_id=identifier, railing_paths=paths,
            fascia_openings=shapely.union_all(fascia_openings),
            field=domain, transitions=transitions, source_ways=[network.roads[i]['id'] for i in sorted(indices)]))
    return result, deferred


class DeckField:
    """Evaluate a continuous local join field while retaining original profiles away from shared deck edges."""

    def __init__(self, pieces, network, neighbors, joins):
        """Keep immutable owned footprints and profile callbacks; blend only within four meters of an interface."""
        self.owned_count = len(pieces)
        self.pieces = pieces+neighbors
        self.network = network
        indices = {p.road_index for p in self.pieces}
        self.join_masks = {key: patch.buffer(4) for key, patch in joins.items() if set(key) <= indices}
        self.max_adjustment = 0.

    def values(self, xy, thickness=False):
        """Blend connected footprint profiles, never profiles from another deck sheet; return finite elevations."""
        if len(self.pieces) == 1:
            piece = self.pieces[0]
            return piece.thickness(xy) if thickness else piece.height(*xy.T)[0]
        points = shapely.points(xy)
        distance = np.array([shapely.distance(p.pavement, points) for p in self.pieces])
        u = np.clip(distance/4, 0, 1)
        weight = 1-u*u*(3-2*u)
        owner = np.argmin(distance[:self.owned_count], axis=0)
        # A road that returns nearby at a different height is not a local blending neighbor. Gate every
        # contribution by the source-connected patch, including joins between separate crossing sheets.
        for j, piece in enumerate(self.pieces):
            enabled = owner == j
            for i, own in enumerate(self.pieces[:self.owned_count]):
                if i == j:
                    continue
                mask = self.join_masks.get(tuple(sorted((own.road_index, piece.road_index))))
                if mask is None:
                    continue
                candidate = np.flatnonzero((owner == i) & (weight[j] > 0))
                enabled[candidate] |= shapely.intersects_xy(mask, xy[candidate, 0], xy[candidate, 1])
            weight[j] *= enabled
        missing = weight.sum(axis=0) == 0
        weight[owner[missing], np.flatnonzero(missing)] = 1
        values = np.array([p.thickness(xy) if thickness else self.extended_height(p, xy) for p in self.pieces])
        result = (values*weight).sum(axis=0)/weight.sum(axis=0)
        if not thickness and len(xy):
            own = np.zeros(len(xy))
            for i, piece in enumerate(self.pieces[:self.owned_count]):
                selected = owner == i
                if selected.any():
                    own[selected] = piece.height(*xy[selected].T)[0]
            self.max_adjustment = max(self.max_adjustment, float(np.max(abs(result-own))))
        return result

    def extended_height(self, piece, xy):
        """Extend an endpoint's solved tangent through the small blend zone instead of creating a flat end pad."""
        network = self.network
        indices = network.indices[piece.road_index]
        z = piece.height(*xy.T)[0]
        coords = network.xy[indices]
        station = shapely.line_locate_point(network.roads[piece.road_index]['line'], shapely.points(xy))
        for k, adjacent in ((0, 1), (-1, -2)):
            delta = coords[k]-coords[adjacent]
            length = np.linalg.norm(delta)
            beyond = np.maximum(0, (xy-coords[k]) @ (delta/length))
            # Only stations clamped at that actual endpoint may extrapolate. A hairpin can return near it.
            end = 0 if k == 0 else network.stations[piece.road_index][-1]
            beyond = np.where(abs(station-end) < 1e-7, beyond, 0)
            z = z+beyond*(network.heights[indices[k]]-network.heights[indices[adjacent]])/length
        return z

    def height(self, x, y):
        """Provide the compiler triangulation callback for this one connected sheet."""
        x, y = np.broadcast_arrays(x, y)
        z = self.values(np.column_stack([x.ravel(), y.ravel()])).reshape(x.shape)
        return z, np.zeros(x.shape)

    def thickness(self, xy):
        """Keep the underside continuous at former source splits and taper only at real terrain landings."""
        return self.values(np.asarray(xy).reshape(-1, 2), thickness=True)
