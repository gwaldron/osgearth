"""Choose render receivers without changing mapped bridge attribution or solved road profiles."""

import numpy as np
import shapely


def raised_approaches(network, ordered, mapped_structures, region, separation=6., height_step=2.):
    """Find ground-tagged approaches whose near-vertical edge cannot share a grid with a lower road.

    The conservative inference promotes the complete source way to geometry, not to an OSM bridge.
    Only neighborhoods of independently ordered road crossings participate. Dimensions and support
    type remain unknown. Six meters covers the five-meter grading shoulder plus a one-meter filtering allowance;
    thresholds describe this experiment's grid receiver policy, not survey data.
    """
    roads = network.roads
    tree = shapely.STRtree([r['surface'] for r in roads])
    result = {}
    for upper, lower, overlap in ordered:
        if mapped_structures[lower]:
            continue
        patch = overlap.intersection(region)
        if patch.is_empty:
            continue
        coordinates = shapely.get_coordinates(shapely.segmentize(patch.boundary, 2.))
        coordinates = np.vstack([coordinates, np.array(patch.centroid.coords)])
        for j in tree.query(patch.buffer(separation), predicate='intersects'):
            j = int(j)
            if j == lower or mapped_structures[j]:
                continue
            distance = shapely.distance(shapely.points(coordinates), roads[j]['surface'])
            close = coordinates[distance < separation]
            if not len(close):
                continue
            delta = network.height(j, *close.T)[0] - network.height(lower, *close.T)[0]
            if np.max(delta) > height_step:
                evidence = {'lower_way': roads[lower]['id'], 'crossing_upper_way': roads[upper]['id'],
                    'height_step_m': float(np.max(delta)), 'nearest_distance_m': float(np.min(distance))}
                result.setdefault(j, []).append(evidence)
    return result


def protect_lower_receivers(network, road_index, mask, structures, clearance=2., margin=6.):
    """Keep an elevated landing's terrain support away from nearby lower ground receivers and shoulders."""
    if mask.is_empty:
        return mask
    for j, other in enumerate(network.roads):
        if structures[j] or j == road_index or not other['surface'].intersects(mask.buffer(margin)):
            continue
        nearby = other['surface'].intersection(mask.buffer(margin))
        coordinates = shapely.get_coordinates(shapely.segmentize(nearby.boundary, 2.))
        if not len(coordinates):
            continue
        delta = network.height(road_index, *coordinates.T)[0] - network.height(j, *coordinates.T)[0]
        if np.max(delta) > clearance:
            mask = mask.difference(other['surface'].buffer(margin))
    return mask
