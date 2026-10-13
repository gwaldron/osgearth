"""Fair mapped cable spans independently of interior DEM noise while retaining network landing anchors."""

from types import SimpleNamespace

import numpy as np
import shapely

from bridge_evidence import read_evidence, select_assets


def fair_cable_spans(network, ordered, source, to_local, region, grade_caps=None, margin=.08):
    """Replace isolated cable-span interiors by a common broad crown with smooth landing transitions.

    Shared endpoint variables remain exact. Internal junctions, unsupported attribution, excessive grades,
    and mutually stacked candidate spans defer to the accepted network solution. This is a derived visual
    profile, not surveyed bridge geometry. The source network solve is never overwritten on disk.
    """
    decks = [SimpleNamespace(indices=[i], pavement=r['surface']) for i, r in enumerate(network.roads)
        if r['bridge'] and r['surface'].intersects(region)]
    evidence = read_evidence(source, to_local)
    assets = select_assets(network, decks, evidence, region)
    selected = {i for asset in assets if not asset['deferred'] and asset['family'] in ('cable-stayed', 'suspension')
        for i in asset['indices']}
    records = []
    for asset in assets:
        if asset['deferred'] or asset['family'] not in ('cable-stayed', 'suspension'):
            continue
        pending = []
        for i in asset['indices']:
            road = network.roads[i]
            indices, stations = network.indices[i], network.stations[i]
            length = float(stations[-1])
            original = network.heights[indices].copy()
            record = {'way': road['id'], 'asset_id': asset['id'], 'family': asset['family'],
                'status': 'deferred', 'length_m': length, 'source_evidence': asset['source_evidence']}
            records.append(record)
            if length < 60 or any(network.incident[int(k)]-{i} for k in indices[1:-1]):
                record['reason'] = 'Short span or internal graph connection needs a joint profile solve'
                continue
            crossings = [(a, b, patch) for a, b, patch in ordered if i in (a, b)]
            if any((b if a == i else a) in selected for a, b, patch in crossings):
                record['reason'] = 'Stacked candidate spans need a coupled profile solve'
                continue
            u = stations/length
            chord = original[0]*(1-u)+original[-1]*u
            transition = min(40., length*.12)
            # The central span has constant added elevation. Quadratic end transitions meet it with zero
            # extra slope, but allow a nonzero landing tangent where a nearby road crossing needs clearance.
            crown = (1-(1-np.minimum(stations/transition, 1))**2)*(
                1-(1-np.minimum((length-stations)/transition, 1))**2)
            minimum, maximum = 0., np.inf
            for upper, lower, patch in crossings:
                boundary = shapely.get_coordinates(shapely.segmentize(patch.boundary, .5))
                xmin, ymin, xmax, ymax = patch.bounds
                x, y = np.meshgrid(np.arange(xmin, xmax, 1), np.arange(ymin, ymax, 1))
                grid = np.column_stack([x.ravel(), y.ravel()])
                grid = grid[shapely.intersects_xy(patch, *grid.T)]
                xy = np.vstack([boundary, grid])
                station = shapely.line_locate_point(road['line'], shapely.points(xy))
                baseline = np.interp(station, stations, chord)
                weight = np.interp(station, stations, crown)
                other = network.height(lower if upper == i else upper, *xy.T)[0]
                # The accepted QP anchors can have tens of micrometers of residual. Reserve 0.1 mm of the
                # interpolation allowance for that residual; the 5.25 m clearance target is unchanged.
                required = other+(5.25+.8+margin-.0001)*(1 if upper == i else -1)
                positive = weight > 1e-9
                if np.any((baseline[~positive] < required[~positive]-1e-6) if upper == i else
                          (baseline[~positive] > required[~positive]+1e-6)):
                    minimum, maximum = 1., 0.
                    break
                bound = (required[positive]-baseline[positive])/weight[positive]
                if len(bound):
                    if upper == i:
                        minimum = max(minimum, float(np.max(bound)))
                    else:
                        maximum = min(maximum, float(np.min(bound)))
            candidate = chord+crown*minimum
            grade = float(np.max(abs(np.diff(candidate)/np.diff(stations))))
            limit = (grade_caps or {}).get(road['tags'].get('highway'), .08)
            if minimum > maximum+1e-8 or grade > limit or minimum > length*.04:
                record['reason'] = 'Single smooth crown cannot satisfy clearance and grade constraints'
                continue
            candidate[[0, -1]] = original[[0, -1]]
            slopes = np.diff(original)/np.diff(stations)
            record.update(crown_rise_m=minimum, max_grade_percent=grade*100, transition_length_m=transition,
                old_slope_reversals=int(np.sum(slopes[1:]*slopes[:-1] < 0)),
                old_max_grade_percent=float(np.max(abs(slopes))*100),
                max_height_change_m=float(np.max(abs(candidate-original))),
                endpoint_change_m=0., stations_m=stations.tolist(), before_m=original.tolist(), after_m=candidate.tolist())
            pending.append((indices, stations, chord, crown, minimum, maximum, limit, original, record))
        # Carriageways on one physical outline share the same crown rather than bowing in opposite directions.
        common = max((item[4] for item in pending), default=0.)
        permitted = len(pending) == len(asset['indices']) and all(common <= item[5]+1e-8 and
            np.max(abs(np.diff(item[2]+common*item[3])/np.diff(item[1]))) <= item[6] for item in pending)
        for indices, stations, chord, crown, minimum, maximum, limit, original, record in pending:
            if not permitted:
                record['reason'] = 'Physical bridge carriageways need an unresolved coupled profile'
                continue
            candidate = chord+common*crown
            candidate[[0, -1]] = original[[0, -1]]
            network.heights[indices] = candidate
            record.update(status='applied', crown_rise_m=common, after_m=candidate.tolist(),
                max_grade_percent=float(np.max(abs(np.diff(candidate)/np.diff(stations)))*100),
                max_height_change_m=float(np.max(abs(candidate-original))))
    return {'policy': 'Mapped cable/suspension spans: fixed landings, common broad crown and tangent end transitions',
        'height_status': 'inferred', 'interpolation_guard_m': margin, 'roads': records}
