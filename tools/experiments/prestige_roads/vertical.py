"""Inferred connected vertical alignments with explicit grade and bridge-clearance constraints."""

import json
import math

import numpy as np
from scipy import sparse
from scipy.optimize import linprog
from shapely.geometry import Point, LineString
from shapely.ops import nearest_points


def solve_vertical(roads, xy, terrain, region, output):
    """Solve initialized road profiles/frames with bounded grades and clearance; reject infeasible constraints."""
    keys, positions, incidence, routes = {}, [], [], {}
    for road in roads:
        if road['tunnel']:
            continue
        stations, indices, cumulative = [], [], 0.0
        for segment, (a, b) in enumerate(zip(road['refs'], road['refs'][1:])):
            start, end = np.array(xy[a]), np.array(xy[b])
            length = np.linalg.norm(end-start)
            if length < 1e-6:
                continue
            count = max(1, math.ceil(length/15))
            for step in range(count):
                key = ('node', a) if step == 0 else ('station', road['id'], segment, step)
                if key not in keys:
                    keys[key] = len(positions)
                    positions.append(start+(end-start)*step/count)
                    incidence.append([])
                index = keys[key]
                incidence[index].append(road['bridge'])
                indices.append(index)
                stations.append(cumulative+length*step/count)
            cumulative += length
        key = ('node', road['refs'][-1])
        if key not in keys:
            keys[key] = len(positions)
            positions.append(np.array(xy[road['refs'][-1]]))
            incidence.append([])
        incidence[keys[key]].append(road['bridge'])
        indices.append(keys[key])
        stations.append(cumulative)
        routes[road['id']] = (np.array(stations), np.array(indices))
    positions = np.array(positions)
    observed = terrain(positions[:, 0], positions[:, 1])
    count = len(positions)
    rows, columns, values, limits = [], [], [], []
    costs = [0.0]*count+[1.0 if not all(v) else .15 for v in incidence]
    bounds = [(float(z)-25, float(z)+35) for z in observed]+[(0, None)]*count

    def inequality(coefficients, limit):
        """Append one sparse upper-bound inequality, merging repeated source stations in its caller."""
        row = len(limits)
        for index, coefficient in coefficients.items():
            rows.append(row)
            columns.append(index)
            values.append(coefficient)
        limits.append(limit)

    def absolute(coefficients, target, weight):
        """Introduce a nonnegative weighted absolute-error variable for a soft profile objective."""
        index = len(costs)
        costs.append(weight)
        bounds.append((0, None))
        inequality(coefficients | {index: -1}, target)
        inequality({k: -v for k, v in coefficients.items()} | {index: -1}, -target)

    for i, z in enumerate(observed):
        inequality({i: 1, count+i: -1}, z)
        inequality({i: -1, count+i: -1}, -z)
    for road in roads:
        if road['tunnel']:
            continue
        stations, indices = routes[road['id']]
        lengths = np.diff(stations)
        grade = .08 if road['bridge'] else .09
        for a, b, length in zip(indices, indices[1:], lengths):
            if a == b:
                continue
            inequality({a: 1, b: -1}, grade*length)
            inequality({a: -1, b: 1}, grade*length)
        for j in range(1, len(indices)-1):
            before, after = lengths[j-1], lengths[j]
            if min(before, after) < .01:
                continue
            coefficients = {}
            for index, coefficient in [(indices[j-1], 1/before),
                                       (indices[j], -1/before-1/after), (indices[j+1], 1/after)]:
                coefficients[index] = coefficients.get(index, 0)+coefficient
            absolute(coefficients, 0, 150 if not road['bridge'] else 600)

    def interpolation(road, point):
        """Return the two station weights at a projected source-alignment position."""
        stations, indices = routes[road['id']]
        station = road['line'].project(point)
        j = min(max(np.searchsorted(stations, station, side='right')-1, 0), len(stations)-2)
        t = np.clip((station-stations[j])/max(stations[j+1]-stations[j], 1e-9), 0, 1)
        weights = {int(indices[j]): float(1-t)}
        weights[int(indices[j+1])] = weights.get(int(indices[j+1]), 0)+float(t)
        return weights

    # Nearby genuine deck approaches may overlap because widths are inferred. Their shared rendered
    # ground cannot support incompatible hard end heights; this is a geometric tie, not a lane connection.
    caps = []
    ground_roads = [r for r in roads if not r['bridge'] and not r['tunnel']]
    for road in roads:
        if not road['bridge']:
            continue
        for endpoint in (0, -1):
            approaches = [r for r in ground_roads if road['refs'][endpoint] in r['refs']]
            if approaches:
                cap = LineString([road['left_edge'].coords[endpoint], road['right_edge'].coords[endpoint]])
                caps.append((road, endpoint, cap, approaches))
    cap_ties = []
    for i, (first, endpoint, cap, approaches) in enumerate(caps):
        for second, other_endpoint, other_cap, other_approaches in caps[i+1:]:
            if first['id'] == second['id'] or cap.distance(other_cap) >= 2:
                continue
            contacts = [a['surface'].intersection(b['surface']) for a in approaches for b in other_approaches]
            if not any(g.distance(cap) < 2 and g.distance(other_cap) < 2 for g in contacts):
                continue
            a, b = nearest_points(cap, other_cap)
            coefficients = interpolation(first, a)
            for index, value in interpolation(second, b).items():
                coefficients[index] = coefficients.get(index, 0)-value
            allowance = .10*max(a.distance(b), .02)
            inequality(coefficients, allowance)
            inequality({k: -v for k, v in coefficients.items()}, allowance)
            cap_ties.append({'first': first['id'], 'first_end': endpoint, 'second': second['id'],
                             'second_end': other_endpoint, 'maximum_height_difference_m': allowance,
                             'basis': 'INFERRED compatibility of overlapping rendered ground approaches'})
    crossings = []
    clearance_start = len(limits)
    for upper in roads:
        if not upper['bridge']:
            continue
        for lower in roads:
            if lower['bridge'] or lower['tunnel'] or set(upper['refs']) & set(lower['refs']):
                continue
            overlap = upper['surface'].intersection(lower['surface'])
            if overlap.area < .01:
                continue
            crossing = upper['line'].intersection(lower['line'])
            if crossing.is_empty or not region.covers(crossing):
                continue
            points = [overlap.representative_point()]
            points += [Point(p) for p in np.array(overlap.convex_hull.exterior.coords)[:, :2]]
            for point in points:
                coefficients = interpolation(lower, point)
                for index, value in interpolation(upper, point).items():
                    coefficients[index] = coefficients.get(index, 0)-value
                inequality(coefficients, -6.5)
            crossings.append({'upper': upper['id'], 'lower': lower['id'], 'overlap_constraints': len(points),
                              'inferred_surface_separation_m': 6.5, 'surveyed': False,
                              'upper_name': upper['tags'].get('name'), 'lower_name': lower['tags'].get('name')})
    matrix = sparse.coo_matrix((values, (rows, columns)), shape=(len(limits), len(costs))).tocsr()
    solution = linprog(costs, A_ub=matrix, b_ub=limits, bounds=bounds, method='highs',
                       options={'time_limit': 120})
    if not solution.success:
        (output/'vertical-failure.json').write_text(json.dumps({'reason': solution.message,
            'stations': count, 'grade_constraints': clearance_start, 'crossings': crossings}, indent=2)+'\n')
        raise ValueError('Connected vertical alignment is infeasible or incomplete: '+solution.message)
    z = solution.x[:count]
    violation = float(np.max(matrix @ solution.x-np.array(limits)))
    if violation > 1e-5:
        raise ValueError('Connected vertical solution violates its declared constraints')
    for road in roads:
        if road['tunnel']:
            continue
        stations, indices = routes[road['id']]
        dense = np.r_[0, np.cumsum(np.linalg.norm(np.diff(road['samples'], axis=0), axis=1))]
        road['heights'] = np.interp(dense, stations, z[indices])
        road['height_inference'] = 'INFERRED connected grade/clearance solution; DEM is a soft target, not a survey'
    record = {'stations': count, 'constraints': len(limits), 'region_bounds_m': list(region.bounds),
              'grade_ground_max': .09, 'grade_bridge_max': .08,
              'bridge_surface_separation_m': 6.5, 'source_delta_bounds_m': [-25, 35],
              'max_constraint_violation': violation, 'source_delta_quantiles_m': np.quantile(z-observed, [0, .5, 1]).tolist(),
              'absolute_delta_p95_m': float(np.quantile(abs(z-observed), .95)), 'crossings': crossings,
              'rendered_approach_ties': cap_ties,
              'policy': 'Inferred vertical design; grade and clearance limits are prototype assumptions'}
    (output/'vertical-design.json').write_text(json.dumps(record, indent=2)+'\n', encoding='utf-8')
    (output/'vertical-failure.json').unlink(missing_ok=True)
    return {key[1]: float(z[index]-observed[index]) for key, index in keys.items() if key[0] == 'node'}
