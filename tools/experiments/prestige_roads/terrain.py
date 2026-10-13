"""Retained ReadyMap elevation sampling in the same EGM96/WGS84 conventions as osgEarth."""

import hashlib
import json
from pathlib import Path
import warnings

import numpy as np
import rasterio


URL = 'https://readymap.org/readymap/tiles/1.0.0/116/'
GRID = Path(__file__).resolve().parents[3]/'src/osgEarthDrivers/vdatum_egm96/EGM96Grid.h'


def bilinear(values, column, row):
    """Sample a finite node grid including its closed outer edge; reject requests outside its coverage."""
    column, row = np.broadcast_arrays(np.asarray(column, float), np.asarray(row, float))
    if (np.any(column < -1e-7) or np.any(row < -1e-7)
            or np.any(column > values.shape[1]-1+1e-7) or np.any(row > values.shape[0]-1+1e-7)):
        raise ValueError('Elevation query is outside the retained grid')
    x = np.clip(column, 0, values.shape[1]-1)
    y = np.clip(row, 0, values.shape[0]-1)
    ix = np.minimum(x.astype(int), values.shape[1]-2)
    iy = np.minimum(y.astype(int), values.shape[0]-2)
    fx, fy = x-ix, y-iy
    return ((1-fy)*((1-fx)*values[iy, ix]+fx*values[iy, ix+1])
            +fy*((1-fx)*values[iy+1, ix]+fx*values[iy+1, ix+1]))


class ReadyMap:
    """Read immutable retained tiles; interpolate node heights and add osgEarth's EGM96 geoid separation."""
    def __init__(self, directory, to_geo):
        """Validate provenance, dimensions and finite coverage before exposing a read-only sampler."""
        self.directory = Path(directory)
        self.metadata = json.loads((self.directory/'manifest.json').read_text(encoding='utf-8'))
        metadata_hash = hashlib.sha256((self.directory/'tilemapresource.xml').read_bytes()).hexdigest()
        if metadata_hash != self.metadata['metadata_sha256']:
            raise ValueError('Retained elevation metadata hash differs from its manifest')
        self.to_geo = to_geo
        self.level = self.metadata['level']
        self.step = 180/2**self.level
        self.tiles = {}
        for item in self.metadata['tiles']:
            path = self.directory/item['file']
            if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
                raise ValueError('Retained elevation tile hash differs from its manifest')
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', rasterio.errors.NotGeoreferencedWarning)
                with rasterio.open(path) as dataset:
                    data = dataset.read(1).astype(float)
            if data.shape != (257, 257) or not np.isfinite(data).all() or np.any(data <= -32000):
                raise ValueError('ReadyMap tile has unsupported dimensions or missing elevation samples')
            self.tiles[(item['x'], item['y'])] = data
        payload = GRID.read_bytes()
        text = payload.decode('utf-8').split('s_egm96grid[] = {', 1)[1].split('};', 1)[0]
        self.geoid = np.fromstring(text, sep=',', dtype=np.float32).reshape(721, 1441)/100
        self.metadata['geoid_grid_sha256'] = hashlib.sha256(payload).hexdigest()
        self.metadata['output_datum'] = 'WGS84 ellipsoid; source EGM96 orthometric + geoid separation'
        self.metadata['sample_interpretation'] = '257 node samples span each TMS tile edge to edge; bilinear interpolation'

    def geographic(self, longitude, latitude, ellipsoid=True):
        """Sample supported geographic positions; missing tiles fail instead of silently returning sea level."""
        longitude, latitude = np.broadcast_arrays(longitude, latitude)
        shape = longitude.shape
        lon, lat = longitude.ravel(), latitude.ravel()
        tx = np.floor((lon+180)/self.step).astype(int)
        ty = np.floor((lat+90)/self.step).astype(int)
        result = np.empty(len(lon))
        for x, y in np.unique(np.column_stack([tx, ty]), axis=0):
            if (x, y) not in self.tiles:
                raise ValueError(f'Missing retained ReadyMap tile {self.level}/{x}/{y}')
            mask = (tx == x) & (ty == y)
            column = (lon[mask]-(-180+x*self.step))/self.step*256
            row = (-90+(y+1)*self.step-lat[mask])/self.step*256
            result[mask] = bilinear(self.tiles[(x, y)], column, row)
        if ellipsoid:
            result += bilinear(self.geoid, np.mod(lon, 360)*4, (90-lat)*4)
        return result.reshape(shape)

    def __call__(self, x, y):
        """Return ellipsoid heights at local projected XY coordinates, preserving broadcast array shape."""
        x, y = np.broadcast_arrays(x, y)
        lon, lat = self.to_geo.transform(x, y)
        return self.geographic(np.asarray(lon), np.asarray(lat))


class ExactSurface:
    """Sample the actual constrained road triangles; extend to the closest boundary only for shoulder construction."""
    def __init__(self, triangles):
        """Index nondegenerate triangle footprints and local planar gradients without re-triangulating islands."""
        import shapely
        a = triangles[:, 0]
        e, f = triangles[:, 1]-a, triangles[:, 2]-a
        determinant = e[:, 0]*f[:, 1]-e[:, 1]*f[:, 0]
        keep = abs(determinant) > 1e-12
        a, e, f, determinant = a[keep], e[keep], f[keep], determinant[keep]
        self.origin = a
        self.gradient = np.column_stack([(e[:, 2]*f[:, 1]-f[:, 2]*e[:, 1])/determinant,
                                         (e[:, 0]*f[:, 2]-f[:, 0]*e[:, 2])/determinant])
        self.faces = shapely.polygons(triangles[keep, :, :2])
        self.tree = shapely.STRtree(self.faces)

    def __call__(self, x, y):
        """Return triangle-exact elevations and distance to support; bounded chunks limit raster-query memory."""
        import shapely
        x, y = np.broadcast_arrays(x, y)
        points = np.column_stack([x.ravel(), y.ravel()])
        z, distance = np.empty(len(points)), np.empty(len(points))
        for start in range(0, len(points), 50000):
            xy = points[start:start+50000]
            geometries = shapely.points(xy)
            pairs, d = self.tree.query_nearest(geometries, all_matches=False, return_distance=True)
            order = np.argsort(pairs[0])
            indices, delta = pairs[1, order], d[order]
            positions = xy.copy()
            outside = delta > 1e-8
            if outside.any():
                links = shapely.shortest_line(geometries[outside], self.faces[indices[outside]])
                positions[outside] = shapely.get_coordinates(shapely.get_point(links, -1))
            z[start:start+len(xy)] = self.origin[indices, 2]+np.sum(
                (positions-self.origin[indices, :2])*self.gradient[indices], axis=1)
            distance[start:start+len(xy)] = delta
        return z.reshape(x.shape), distance.reshape(x.shape)


def grade_report(triangles):
    """Measure surface slopes without certifying a draped DEM as a drivable road."""
    cross = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    area = abs(cross[:, 2])/2
    grade = np.linalg.norm(cross[:, :2], axis=1)/np.maximum(abs(cross[:, 2]), 1e-12)
    return {'grade_p95_percent': float(np.quantile(grade[area > .01], .95)*100),
            'grade_max_percent_faces_above_001m2': float(grade[area > .01].max()*100),
            'area_above_15_percent_grade_m2': float(area[grade > .15].sum()),
            'policy': 'Diagnostic terrain drape; no grade acceptance or engineering clearance asserted'}


def bridge_audit(roads, pavement):
    """Expose geometric clearance failures of DEM-derived decks; the five-meter screen is an inference, not survey data."""
    from surfaces import minimum_clearance
    records = []
    for upper in roads:
        if not upper['bridge']:
            continue
        for lower in roads:
            if lower['bridge'] or lower['tunnel'] or set(upper['refs']) & set(lower['refs']):
                continue
            if upper['surface'].intersection(lower['surface']).area < .001:
                continue
            clearance, count, position = minimum_clearance(upper['mesh'], pavement, lower['surface'], .7, True)
            records.append({'upper': upper['id'], 'lower': lower['id'], 'minimum_m': clearance,
                            'triangle_overlaps_checked': count, 'worst_position_local_m': position,
                            'deck_thickness_inferred_m': .7,
                            'screening_clearance_inferred_m': 5.0,
                            'status': 'needs_vertical_design' if clearance < 5 else 'passes_inferred_screen',
                            'source_upper_tags': upper['tags'], 'source_lower_tags': lower['tags']})
    return {'checks': records, 'failed_screen': sum(r['minimum_m'] < 5 for r in records),
            'surveyed_clearance': False, 'bridge_over_bridge_checked': False,
            'policy': 'Final mesh clearance screen; profile clearance alone does not certify the fitted pavement'}


def comparison_views(output, triangles, terrain, region, to_geo):
    """Measure centroid road/DEM separation and save identical cut/fill camera targets for both terrain variants."""
    import shapely
    points = triangles.mean(axis=1)
    area = abs(np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])[:, 2])/2
    separation = points[:, 2]+.12-terrain(points[:, 0], points[:, 1])
    valid = (area > .01) & (shapely.distance(shapely.points(points[:, :2]), region.boundary) > 25)
    candidates = np.flatnonzero(valid)
    if not len(candidates):
        raise ValueError('No interior pavement samples for terrain comparison')
    record = {'centroid_sampling': True, 'road_minus_natural_min_m': float(separation.min()),
              'road_minus_natural_max_m': float(separation.max()),
              'estimated_buried_area_fraction': float(area[separation < 0].sum()/area.sum()),
              'estimated_area_over_025m_above_terrain_fraction': float(area[separation > .25].sum()/area.sum()),
              'road_render_offset_m': .12, 'views': {}}
    views = {}
    for label, choice in [('terrain-cut', np.argmin), ('terrain-fill', np.argmax)]:
        index = candidates[choice(separation[candidates])]
        x, y, z = points[index]
        lon, lat = to_geo.transform(x, y)
        views[label] = (lon, lat, -30, -30, 70, z+.12)
        record['views'][label] = {'longitude': lon, 'latitude': lat, 'road_height_hae_m': z+.12,
                                  'road_minus_natural_m': float(separation[index])}
    (output/'natural-terrain-audit.json').write_text(json.dumps(record, indent=2)+'\n', encoding='utf-8')
    return views


def write_overlay(output, region, to_geo, terrain, road_surface, ground, pixel_size=1.0, shoulder=18.0):
    """Write a sparse ellipsoid-height overlay from the road surface, feathered into unchanged natural terrain."""
    import shapely
    from shapely.geometry import box
    from rasterio.transform import from_bounds
    extent = region.buffer(100).bounds
    geographic = shapely.transform(box(*extent), to_geo.transform, interleaved=False).bounds
    width = int(np.ceil((extent[2]-extent[0])/pixel_size))
    height = int(np.ceil((extent[3]-extent[1])/pixel_size))
    west, south, east, north = geographic
    lon = np.linspace(west, east, width, endpoint=False)+(east-west)/(width*2)
    lat = np.linspace(north, south, height, endpoint=False)-(north-south)/(height*2)
    from pyproj import Transformer
    to_local = Transformer.from_crs('EPSG:4326', to_geo.source_crs, always_xy=True)
    xx, yy = to_local.transform(*np.meshgrid(lon, lat))
    natural = terrain(xx, yy)
    road, distance = road_surface(xx, yy)
    # The one-meter collar supplies room for terrain triangles near curbs. The remaining shoulder fades to zero.
    t = np.clip((distance-1.0)/shoulder, 0, 1)
    weight = 1-(6*t**5-15*t**4+10*t**3)
    modified = natural+(road-.08-natural)*weight
    active = distance < shoulder+1
    nodata = -32768.0
    data = np.where(active, modified, nodata).astype('float32')
    profile = {'driver': 'GTiff', 'width': width, 'height': height, 'count': 1, 'dtype': 'float32',
               'crs': 'EPSG:4326', 'transform': from_bounds(*geographic, width, height),
               'compress': 'deflate', 'nodata': nodata}
    with rasterio.open(output/'grading-overlay.tif', 'w', **profile) as ds:
        ds.write(data, 1)
        ds.update_tags(description='Road-owned grading; ellipsoid meters; no-data leaves underlying ReadyMap unchanged')
    delta = (modified-natural)[active]
    # Prepared distance predicates index the footprint boundary. Computing every full polygon distance
    # made this independent guard dominate city exports; one ULP retains the original >= radius rule.
    shapely.prepare(ground)
    outside = ~shapely.dwithin(ground, shapely.points(xx.ravel(), yy.ravel()),
                              np.nextafter(float(shoulder+1), -np.inf)).reshape(xx.shape)
    if np.any(data[outside] != nodata):
        raise ValueError('Road grading modifies pixels beyond the declared influence footprint')
    record = {'pixel_size_target_m': pixel_size, 'width': width, 'height': height, 'shoulder_m': shoulder,
              'collar_m': 1.0, 'roadbed_offset_m': -.08, 'nodata_outside_influence': True,
              'active_pixels': int(active.sum()), 'delta_min_m': float(delta.min()), 'delta_max_m': float(delta.max()),
              'abs_delta_p95_m': float(np.quantile(abs(delta), .95)), 'source': terrain.metadata,
              'height_source': 'Exact constrained ground-road triangles; bridge decks excluded'}
    (output/'terrain-audit.json').write_text(json.dumps(record, indent=2)+'\n', encoding='utf-8')
    return record
