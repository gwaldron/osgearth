"""Illustrative bridge edge railings, attached to the actual exported deck triangles."""

import numpy as np
import shapely

from terrain import ExactSurface


def quad(vertices, center):
    """Triangulate a planar box face with outward winding relative to its containing segment."""
    a, b, c, d = np.asarray(vertices)
    if np.dot(np.cross(b-a, c-a), (a+b+c+d)/4-center) < 0:
        a, b, c, d = d, c, b, a
    return [[a, b, c], [a, c, d]]


def sweep(points, width, height):
    """Sweep a rectangular rail with shared miter sections and caps only at its two open ends."""
    points = np.asarray(points)
    direction = np.diff(points[:, :2], axis=0)
    direction /= np.linalg.norm(direction, axis=1)[:, None]
    left = np.column_stack([-direction[:, 1], direction[:, 0]])
    normal = np.vstack([left[0], left[:-1]+left[1:], left[-1]])
    normal /= np.linalg.norm(normal, axis=1)[:, None]
    # Clamp sharp source corners rather than allowing an arbitrarily long miter to intrude into a lane.
    scale = np.ones(len(points))
    scale[1:-1] = np.minimum(2., 1/np.maximum(.01, np.sum(normal[1:-1]*left[1:], axis=1)))
    offsets = np.column_stack([normal*scale[:, None]*width/2, np.zeros(len(points))])
    vertical = np.array([0, 0, height/2])
    rings = np.stack([points-offsets-vertical, points+offsets-vertical,
                      points+offsets+vertical, points-offsets+vertical], axis=1)
    triangles = []
    for a, b in zip(rings, rings[1:]):
        center = (a.mean(axis=0)+b.mean(axis=0))/2
        for k in range(4):
            j = (k+1) % 4
            triangles.extend(quad([a[k], a[j], b[j], b[k]], center))
    triangles.extend(quad(rings[0], (points[0]+points[1])/2))
    triangles.extend(quad(rings[-1], (points[-1]+points[-2])/2))
    return triangles


def export_railings(bridge, deck, writer):
    """Append inferred steel railings on exposed boundaries; source negatives, joins, and landings are pre-masked."""
    bridge.railings = []
    triangles = []
    surface = ExactSurface(deck)
    for path in shapely.get_parts(bridge.railing_paths):
        if path.geom_type != 'LineString' or path.length < .4:
            continue
        xy = shapely.get_coordinates(shapely.segmentize(path, 1.0))
        keep = np.r_[True, np.linalg.norm(np.diff(xy, axis=0), axis=1) > 1e-6]
        xy = xy[keep]
        if len(xy) < 2:
            continue
        z, distance = surface(*xy.T)
        if np.max(distance) > .001:
            raise ValueError('Railing left its assembled deck footprint')
        base = np.column_stack([xy, z])
        bridge.railings.append(base.tolist())
        for height in (.60, 1.10):
            triangles.extend(sweep(base+[0, 0, height], .075, .09))
        count = max(1, int(np.ceil(path.length/2)))
        stations = np.linspace(0, path.length, count+1)
        if path.is_ring:
            stations = stations[:-1]
        posts = shapely.get_coordinates(shapely.line_interpolate_point(path, stations))
        heights, distance = surface(*posts.T)
        if np.max(distance) > .001:
            raise ValueError('Railing post left its deck footprint')
        for xy, z in zip(posts, heights):
            # A short horizontal sweep supplies a square vertical post with independent hard face normals.
            axis = np.array([[xy[0]-.04, xy[1], z+.575], [xy[0]+.04, xy[1], z+.575]])
            triangles.extend(sweep(axis, .08, 1.15))
    if not triangles:
        return 0
    name = 'galvanized steel'
    matches = [i for i, m in enumerate(writer.doc['materials']) if m['name'] == name]
    if matches:
        material = matches[0]
    else:
        material = len(writer.doc['materials'])
        writer.doc['materials'].append({'name': name, 'doubleSided': False, 'pbrMetallicRoughness': {
            'baseColorFactor': [.52, .56, .59, 1], 'metallicFactor': .8, 'roughnessFactor': .38}})
    return writer.add('Inferred bridge railings', np.asarray(triangles), material)
