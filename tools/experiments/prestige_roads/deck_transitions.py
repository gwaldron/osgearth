"""Inferred local deck-width transitions at mapped joins, independent of terrain grading."""

from itertools import combinations

import numpy as np
import shapely
from shapely.geometry import LineString, MultiPoint


def round_join_corners(pavement, zone, radius):
    """Round local concavities across short boundary segments; add pavement only inside the join's guard zone."""
    # A tangent arc at one vertex cannot bridge the short end-cap ledge next to a curved ramp.
    # Rolling an offset circle across the boundary handles several segments as one corner.
    closed = pavement.buffer(radius, quad_segs=16).buffer(-radius, quad_segs=16)
    delta = shapely.difference(closed, pavement, grid_size=1e-9)
    additions = [part for part in shapely.get_parts(delta) if part.area > 1e-6 and zone.covers(part)]
    # Keep whole additions, not clipped pieces: clipping an arc at the guard zone would make a new step.
    # Never shrink a source footprint at convex corners. Existing holes and foreign receivers are checked
    # by the caller before accepting this candidate. The precision grid only reconciles Boolean roundoff.
    return shapely.union_all([pavement, *additions], grid_size=1e-9) if additions else pavement


def taper_joins(pavement, indices, network, region, domain, ordered):
    """Fill square width steps within one sheet; reject unrelated receivers, holes, and unsafe new crossings."""
    roads = network.roads
    indices = set(indices)
    nodes = sorted({node for i in indices for node in (roads[i]['refs'][0], roads[i]['refs'][-1])})
    ordered_pairs = {(hi, lo) for hi, lo, patch in ordered}
    tree = shapely.STRtree([r['surface'] for r in roads])
    holes = shapely.union_all([shapely.Polygon(ring) for p in shapely.get_parts(pavement)
        if p.geom_type == 'Polygon' for ring in p.interiors])
    evidence = []
    for node in nodes:
        incident = network.incident[network.node_index[node]]
        if len(incident) < 2 or not incident <= indices:
            continue
        arms = sorted(incident)
        if any(node not in (roads[i]['refs'][0], roads[i]['refs'][-1]) for i in arms):
            continue
        widths = [roads[i]['width'] for i in arms]
        # Do not convex-hull a hairpin or a sharp junction. This rule is for through-road width changes
        # and shallow forks, with every arm within 45 degrees of a common continuation axis.
        tangents, lines = [], []
        for i in arms:
            coords = np.asarray(roads[i]['line'].coords)
            if roads[i]['refs'][0] != node:
                coords = coords[::-1]
            line = LineString(coords)
            tangent = coords[1]-coords[0]
            tangents.append(tangent/np.linalg.norm(tangent))
            lines.append(line)
        if any(abs(np.dot(a, b)) < np.cos(np.pi/4) for a, b in combinations(tangents, 2)):
            continue
        if not any(np.dot(a, b) < 0 for a, b in combinations(tangents, 2)):
            continue
        points, caps, reaches = [], [], []
        for i, line in zip(arms, lines):
            # Four times the full width change is an illustrative taper length, capped before the
            # next source node. It is rendering inference, not a road-design standard.
            reach = min(max(8., 4*(max(widths)-min(widths))), line.length*.45)
            reaches.append(reach)
            for station in (0., reach):
                at = np.asarray(line.interpolate(station).coords[0])
                tangent = np.asarray(line.interpolate(min(line.length, station+.05)).coords[0])
                tangent -= np.asarray(line.interpolate(max(0., station-.05)).coords[0])
                tangent /= np.linalg.norm(tangent)
                side = np.array([-tangent[1], tangent[0]])*roads[i]['width']/2
                points.extend([at-side, at+side])
                if station == 0:
                    caps.append(LineString([at-side, at+side]))
        if max(pavement.boundary.intersection(cap.buffer(1e-6)).length for cap in caps) < .25:
            continue
        envelope = MultiPoint(points).convex_hull
        # Equal-width arms also need a common join: differently oriented flat end caps leave a
        # thin wedge right to the node, which an inward railing offset expands into a deep notch.
        candidate = pavement.union(envelope).simplify(1e-9, preserve_topology=True)
        corner_radius = min(2., min(widths)/2)
        candidate = round_join_corners(candidate, envelope.buffer(corner_radius*2), corner_radius)
        added = candidate.difference(pavement)
        if added.area < 1e-6:
            continue
        reason = None
        if added.difference(region).area > 1e-7 or added.intersection(holes).area > 1e-7:
            reason = 'Transition reaches the crop boundary or an existing pavement island'
        for j in tree.query(added, predicate='intersects'):
            j = int(j)
            if j in indices:
                continue
            overlap = added.intersection(roads[j]['surface'])
            if overlap.area < 1e-7:
                continue
            upper = any((i, j) in ordered_pairs for i in arms)
            lower = any((j, i) in ordered_pairs for i in arms)
            if upper == lower:
                reason = 'Transition would reach an unrelated receiver without a unique crossing order'
                break
            xy = shapely.get_coordinates(shapely.segmentize(overlap.boundary, .5))
            xy = np.vstack([xy, overlap.centroid.coords])
            height = domain.height(*xy.T)[0]
            other = network.height(j, *xy.T)[0]
            clearance = height-domain.thickness(xy)-other if upper else other-.8-height
            if min(clearance) < 5.25:
                reason = 'Transition would violate the prototype crossing clearance'
                break
        record = {'node': node, 'source_ways': [roads[i]['id'] for i in arms],
            'source_widths_m': widths, 'reach_m': reaches, 'geometry_status': 'inferred',
            'basis': 'Shared endpoint cross-section envelope and local tangent corner arcs',
            'transition_kind': 'width_taper' if max(widths)-min(widths) >= .25 else 'equal_width_join',
            'corner_radius_max_m': corner_radius,
            'source_tags_unchanged': True, 'status': 'deferred' if reason else 'applied'}
        if reason:
            record['reason'] = reason
        else:
            # Overlay can retain duplicate/collinear vertices separated by floating-point roundoff.
            # Remove only nanometer-scale redundancy before constrained triangulation and refinement.
            pavement = candidate.simplify(1e-9, preserve_topology=True)
            record.update(added_area_m2=added.area, geometry=added)
        evidence.append(record)
    return pavement, evidence
