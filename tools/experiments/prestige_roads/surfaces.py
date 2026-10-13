"""Local surface construction helpers for the bounded Prestige Roads experiment."""

import numpy as np
import shapely
from shapely.geometry import Polygon
from scipy import sparse
from scipy.sparse.linalg import spsolve, factorized


def refine_triangles(triangles, height, max_edge):
    """Split long shared edges consistently and sample one height field; retain original short boundary edges."""
    for _ in range(12):
        xy = triangles[:, :, :2]
        lengths = np.linalg.norm(np.roll(xy, -1, axis=1)-xy, axis=2)
        split = lengths > max_edge
        if not split.any():
            return triangles
        result, sampled = [], []
        for tri, flags in zip(triangles, split):
            if not flags.any():
                result.append(tri)
                continue
            ring = []
            for i in range(3):
                ring.append(tri[i, :2])
                if flags[i]:
                    ring.append((tri[i, :2]+tri[(i+1) % 3, :2])*.5)
            for child in shapely.constrained_delaunay_triangles(Polygon(ring)).geoms:
                points = np.array(shapely.orient_polygons(child).exterior.coords[:3])
                sampled.append(len(result))
                result.append(np.column_stack([points, np.zeros(3)]))
        triangles = np.array(result)
        # Preserve untouched parent heights; sample new faces in bounded batches instead of per triangle.
        # Batching also bounds the nearest-segment sampler's temporary coordinate/neighbor arrays.
        for start in range(0, len(sampled), 8192):
            indices = np.asarray(sampled[start:start+8192])
            points = triangles[indices, :, :2]
            z, _ = height(points[:, :, 0], points[:, :, 1])
            triangles[indices, :, 2] = z
    raise ValueError('Surface refinement exceeded its bounded iteration count')


def mesh_normals(positions, smooth):
    """Average shared-position normals, falling back to each face when opposing faces cancel at a seam."""
    tri = positions.astype(float).reshape(-1, 3, 3)
    normals = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
    normals = np.repeat(normals, 3, axis=0)
    if smooth:
        face_normals = normals
        unique, inverse = np.unique(positions, axis=0, return_inverse=True)
        sums = np.zeros((len(unique), 3))
        np.add.at(sums, inverse, normals)
        normals = sums[inverse]
        cancelled = np.linalg.norm(normals, axis=1) < 1e-12
        normals[cancelled] = face_normals[cancelled]
    return normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)


def vertical_walls(geometry, height, bottom_offset, top_offset, parts):
    """Extrude oriented polygon rings with outward-facing walls; offsets are relative to the shared height field."""
    walls = []
    for polygon in parts(shapely.orient_polygons(shapely.segmentize(geometry, 2))):
        for ring in [polygon.exterior, *polygon.interiors]:
            coords = np.array(ring.coords)[:, :2]
            z, _ = height(coords[:, 0], coords[:, 1])
            for i in range(len(coords)-1):
                a, b = np.r_[coords[i], z[i]+top_offset], np.r_[coords[i+1], z[i+1]+top_offset]
                c, d = b-[0, 0, top_offset-bottom_offset], a-[0, 0, top_offset-bottom_offset]
                walls.extend([[a, c, b], [a, d, c]])
    return np.array(walls).reshape(-1, 3, 3)


def minimum_clearance(upper, lower, region, thickness, return_position=False):
    """Measure every triangle overlap; optionally return the worst planar position for diagnostic camera placement."""
    ground_faces = shapely.polygons(lower[:, :, :2])
    tree = shapely.STRtree(ground_faces)
    minimum, intersections, position = float('inf'), 0, None
    for top in upper:
        top_poly = Polygon(top[:, :2]).intersection(region)
        if top_poly.is_empty or top_poly.area < 1e-8:
            continue
        top_plane = np.linalg.solve(np.column_stack([top[:, :2], np.ones(3)]), top[:, 2])
        for index in tree.query(top_poly, predicate='intersects'):
            overlap = top_poly.intersection(ground_faces[index])
            if overlap.area < 1e-8:
                continue
            bottom = lower[index]
            bottom_plane = np.linalg.solve(np.column_stack([bottom[:, :2], np.ones(3)]), bottom[:, 2])
            points = shapely.get_coordinates(overlap)
            clearance = np.column_stack([points, np.ones(len(points))]) @ (top_plane-bottom_plane)-thickness
            index = int(clearance.argmin())
            if clearance[index] < minimum:
                minimum = float(clearance[index])
                position = points[index].tolist()
            intersections += 1
    if not intersections:
        raise ValueError('Clearance constraint has no overlapping rendered pavement triangles')
    return (minimum, intersections, position) if return_position else (minimum, intersections)


def fit_pavement(triangles, base_height, constraints, fidelity=.001, curvature_m2=4, limit_grade=False):
    """Fit a smooth height residual on connected pavement; preserve hard ties and reject invalid weights or steep fits."""
    if not np.isfinite([fidelity, curvature_m2]).all() or fidelity <= 0 or curvature_m2 < 0:
        raise ValueError('Surface fidelity must be positive and curvature weight nonnegative')
    points, inverse = np.unique(np.round(triangles[:, :, :2].reshape(-1, 2), 9), axis=0, return_inverse=True)
    ids = inverse.reshape(-1, 3)
    initial = np.zeros(len(points))
    np.add.at(initial, inverse, triangles[:, :, 2].ravel())
    initial /= np.bincount(inverse)
    base = base_height(points[:, 0], points[:, 1])
    initial -= base
    xy = points[ids]
    x, y = xy[:, :, 0], xy[:, :, 1]
    determinant = (x[:, 1]-x[:, 0])*(y[:, 2]-y[:, 0])-(x[:, 2]-x[:, 0])*(y[:, 1]-y[:, 0])
    area = abs(determinant)*.5
    safe = np.where(abs(determinant) > 1e-12, determinant, 1e-12)
    gx = np.stack([y[:, 1]-y[:, 2], y[:, 2]-y[:, 0], y[:, 0]-y[:, 1]], axis=1)/safe[:, None]
    gy = np.stack([x[:, 2]-x[:, 1], x[:, 0]-x[:, 2], x[:, 1]-x[:, 0]], axis=1)/safe[:, None]
    # Finite-element Dirichlet energy respects triangle geometry, including thin boundary triangles.
    local = area[:, None, None]*(gx[:, :, None]*gx[:, None, :]+gy[:, :, None]*gy[:, None, :])
    rows = np.broadcast_to(ids[:, :, None], local.shape).ravel()
    columns = np.broadcast_to(ids[:, None, :], local.shape).ravel()
    laplace = sparse.coo_matrix((local.ravel(), (rows, columns)), shape=(len(points), len(points))).tocsc()
    mass = np.zeros(len(points))
    np.add.at(mass, ids.ravel(), np.repeat(area/3, 3))
    mass = np.maximum(mass, 1e-8)
    fixed = np.zeros(len(points), dtype=bool)
    target = initial.copy()
    locations = shapely.points(points)
    for geometry, height, tolerance in constraints:
        mask = shapely.distance(locations, geometry) <= tolerance
        values = height(points[:, 0], points[:, 1])[0]-base
        if np.any(abs(values[mask & fixed]-target[mask & fixed]) > .025):
            raise ValueError('Bridge and underpass surface constraints conflict')
        fixed |= mask
        target[mask] = values[mask]
    free = ~fixed
    matrix = laplace+sparse.diags(mass*fidelity)
    # The mass-lumped squared Laplacian penalizes curvature within connected pavement, not across islands.
    # Its weight has units of square meters; hard deck and underpass heights remain exact constraints.
    if curvature_m2:
        matrix += curvature_m2*(laplace.T @ sparse.diags(1/mass) @ laplace)
    rhs = fidelity*mass*initial-matrix[:, fixed] @ target[fixed]
    target[free] = spsolve(matrix[free][:, free], rhs[free])
    iterations = 0
    if limit_grade:
        # Split triangle gradients from vertex heights. Projecting each gradient onto a disk enforces
        # a slope bound over whole faces, while shared vertices and fixed deck ties retain continuity.
        triangle_rows = np.repeat(np.arange(len(ids)), 3)
        gradient_x = sparse.coo_matrix((gx.ravel(), (triangle_rows, ids.ravel())),
                                      shape=(len(ids), len(points))).tocsr()
        gradient_y = sparse.coo_matrix((gy.ravel(), (triangle_rows, ids.ravel())),
                                      shape=(len(ids), len(points))).tocsr()
        rho = 50.0
        system = matrix+rho*laplace
        factor = factorized(system[free][:, free].tocsc())
        constant_rhs = fidelity*mass*initial-system[:, fixed] @ target[fixed]-rho*(laplace @ base)
        gradient = np.column_stack([gradient_x @ (base+target), gradient_y @ (base+target)])
        projected = gradient*np.minimum(1, .14/np.maximum(np.linalg.norm(gradient, axis=1), 1e-12))[:, None]
        dual = np.zeros_like(projected)
        for iterations in range(1, 601):
            weighted = area[:, None]*(projected-dual)
            rhs = constant_rhs+rho*(gradient_x.T @ weighted[:, 0]+gradient_y.T @ weighted[:, 1])
            target[free] = factor(rhs[free])
            gradient = np.column_stack([gradient_x @ (base+target), gradient_y @ (base+target)])
            candidate = gradient+dual
            projected = candidate*np.minimum(1, .14/np.maximum(np.linalg.norm(candidate, axis=1), 1e-12))[:, None]
            residual = gradient-projected
            dual += residual
            significant = area > .01
            if (np.linalg.norm(residual[significant], axis=1).max() < .002
                    and np.linalg.norm(gradient[significant], axis=1).max() < .145):
                break
    if not np.isfinite(target).all():
        raise ValueError('Pavement surface solve produced nonfinite heights')
    heights = base+target
    result = np.concatenate([points[ids], heights[ids][..., None]], axis=2)
    normals = np.cross(result[:, 1]-result[:, 0], result[:, 2]-result[:, 0])
    grade = np.linalg.norm(normals[:, :2], axis=1)/np.maximum(abs(normals[:, 2]), 1e-12)
    report = {'vertices': len(points), 'fixed_vertices': int(fixed.sum()), 'fidelity': fidelity, 'curvature_m2': curvature_m2,
              'grade_p95_percent': float(np.quantile(grade[area > .01], .95)*100),
              'grade_max_percent_faces_above_001m2': float(grade[area > .01].max()*100),
              'area_above_15_percent_grade_m2': float(area[grade > .15].sum())}
    if limit_grade:
        report.update(gradient_projection_iterations=iterations, projected_grade_target_percent=14.0)
    if report['grade_max_percent_faces_above_001m2'] > 15:
        raise ValueError(f'Pavement surface exceeds the experimental grade limit: {report}')
    return result, report
