"""Approximate Boston road grading with an ordinary elevation grid and terrain image layers."""

from collections import defaultdict
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np
import rasterio
from rasterio.features import rasterize
from rasterio.transform import from_bounds
from scipy.ndimage import distance_transform_edt, gaussian_filter
import shapely
from shapely.geometry import box, mapping, shape

import compile as compiler
import raster_roads as planar
from terrain import ReadyMap, bilinear


ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT/'output/boston-gridded-roads'
BBOX = (-71.078, 42.349, -71.061, 42.3588)
PIXEL = 1.0
COLLAR = 2.0
SHOULDER = 12.0
NODATA = -32768.0


def write_json(path, value):
    """Write finite, inspectable metadata and GeoJSON; propagate failed writes."""
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def export_materials(region):
    """Crop the retained planar compiler output, including complete supported paint and curb bands."""
    source = planar.OUTPUT/'render-polygons.pkl'
    # This checkpoint was generated locally by raster_roads.py; never load arbitrary downloaded pickles.
    with source.open('rb') as stream:
        material = pickle.load(stream)
    result, counts = {}, {}
    for name, polygons in material.items():
        values = np.asarray(polygons, dtype=object)
        selected = values[shapely.intersects(values, region)]
        # Geographic reprojection can leave microscopic invalid rings; normalize well below paint width.
        selected = shapely.set_precision(shapely.make_valid(selected), 1e-9)
        selected = np.array([p for geometry in selected for p in compiler.parts(geometry)], dtype=object)
        clipped = shapely.intersection(selected, region, grid_size=1e-9)
        result[name] = shapely.union_all(clipped, grid_size=1e-9)
        features = [{'type': 'Feature', 'properties': {'material': name}, 'geometry': mapping(p)}
                    for p in compiler.parts(result[name])]
        write_json(OUTPUT/f'{name}.geojson', {'type': 'FeatureCollection', 'features': features})
        counts[name] = len(features)
    return result, counts, hashlib.sha256(source.read_bytes()).hexdigest()


def road_profiles(region, natural):
    """Sample source-connected centerlines; junction blending gives a common approximate road elevation."""
    compiler.REGION = region
    compiler.TERRAIN = natural
    roads, xy, flags, relations = compiler.read_source(planar.SOURCE)
    excluded = [{'id': r['id'], 'tags': r['tags']} for r in roads if r['bridge'] or r['tunnel']]
    roads = [r for r in roads if not r['bridge'] and not r['tunnel']]
    neighbors = defaultdict(set)
    for road in roads:
        road['samples'] = shapely.get_coordinates(shapely.segmentize(road['line'], 2.0))
        road['heights'] = natural(*road['samples'].T)
        for a, b in zip(road['refs'], road['refs'][1:]):
            neighbors[a].add(b)
            neighbors[b].add(a)
    nodes = [node for node, adjacent in neighbors.items() if len(adjacent) >= 3]
    junctions = np.array([(*xy[node], 0.0) for node in nodes]).reshape(-1, 3)
    sampler = compiler.Heights(roads, junctions, nodes)
    features = []
    for road in roads:
        lon, lat = compiler.TO_GEO.transform(*road['samples'].T)
        features.append({'type': 'Feature',
                         'properties': {'osm_way': road['id'], 'source_tags': road['tags'],
                                        'width_m': road['width'], 'width_reason': road['width_reason'],
                                        'height_inference': 'ReadyMap centerline; shared-node junction blend over 18m'},
                         'geometry': {'type': 'LineString',
                                      'coordinates': np.column_stack([lon, lat, road['heights']]).tolist()}})
    write_json(OUTPUT/'centerlines.geojson', {'type': 'FeatureCollection', 'features': features})
    write_json(OUTPUT/'source-evidence.json', {
        'source_sha256': hashlib.sha256(planar.SOURCE.read_bytes()).hexdigest(),
        'excluded_structures': excluded, 'width_diagnostics': flags, 'relations': relations,
        'junction_nodes': nodes, 'height_datum': 'WGS84 ellipsoid',
        'profile_Z': 'Natural centerline samples before junction blending; inferred, not measured road heights'})
    return roads, sampler, junctions


def write_grid(footprint, sampler, natural, geographic):
    """Rasterize road ownership and feather approximate centerline heights into the base DEM on a regular grid."""
    local = shapely.transform(geographic, compiler.TO_LOCAL.transform, interleaved=False)
    west, south, east, north = geographic.bounds
    width = int(np.ceil((local.bounds[2]-local.bounds[0])/PIXEL))
    height = int(np.ceil((local.bounds[3]-local.bounds[1])/PIXEL))
    transform = from_bounds(west, south, east, north, width, height)
    lon = west+(np.arange(width)+.5)*transform.a
    lat = north+(np.arange(height)+.5)*transform.e
    xx, yy = compiler.TO_LOCAL.transform(*np.meshgrid(lon, lat))
    base = natural(xx, yy)
    road_mask = rasterize([(mapping(footprint), 1)], out_shape=(height, width), transform=transform).astype(bool)
    if not road_mask.any():
        raise ValueError('No pavement cells in the grading region')
    targets = np.zeros_like(base)
    rows, columns = np.where(road_mask)
    for first in range(0, len(rows), 40000):
        r, c = rows[first:first+40000], columns[first:first+40000]
        targets[r, c] = sampler(xx[r, c], yy[r, c])[0]
    distance, indices = distance_transform_edt(~road_mask, sampling=PIXEL, return_indices=True)
    targets = targets[tuple(indices)]
    t = np.clip((distance-COLLAR)/SHOULDER, 0, 1)
    weight = 1-(6*t**5-15*t**4+10*t**3)
    active = distance < COLLAR+SHOULDER
    # Sub-cell smoothing rounds ownership handoffs. This intentionally accepts a small grid approximation.
    softened = gaussian_filter(base+(targets-base)*weight, .65)
    final = np.where(active, softened, base)
    data = np.where(active, final, NODATA).astype('float32')
    profile = dict(driver='GTiff', width=width, height=height, count=1, dtype='float32', crs='EPSG:4326',
                   transform=transform, nodata=NODATA, compress='deflate', tiled=True)
    with rasterio.open(OUTPUT/'road-grading.tif', 'w', **profile) as dataset:
        dataset.write(data, 1)
        dataset.update_tags(description='Approximate OSM road flattening; regular grid; WGS84 ellipsoid meters')
    np.savez_compressed(OUTPUT/'grid-audit-inputs.npz', base=base.astype('float32'), road_mask=road_mask,
                        active=active, target=targets.astype('float32'), final=final.astype('float32'),
                        transform=np.array(tuple(transform)))
    delta = final[active]-base[active]
    record = {'width': width, 'height': height, 'target_cell_size_m': PIXEL,
              'active_cells': int(active.sum()), 'pavement_cells': int(road_mask.sum()),
              'collar_m': COLLAR, 'feather_m': SHOULDER, 'smoothing_sigma_cells': .65,
              'cut_min_m': float(delta.min()), 'fill_max_m': float(delta.max()),
              'absolute_change_p95_m': float(np.quantile(abs(delta), .95)),
              'road_target_error_p95_m': float(np.quantile(abs(final[road_mask]-targets[road_mask]), .95)),
              'road_target_error_max_m': float(abs(final[road_mask]-targets[road_mask]).max()),
              'outside_influence_is_nodata': bool((data[~active] == NODATA).all()),
              'all_active_values_finite': bool(np.isfinite(data[active]).all())}
    return final, base, transform, road_mask, record


def sample_grid(values, transform, x, y):
    """Bilinearly sample the exported cell-centered grid in local coordinates for diagnostics and cameras."""
    lon, lat = compiler.TO_GEO.transform(x, y)
    return bilinear(values, (np.asarray(lon)-transform.c)/transform.a-.5,
                    (np.asarray(lat)-transform.f)/transform.e-.5)


def audit_cross_sections(roads, footprint, final, base, transform, region):
    """Compare transverse grades before and after grading on supported straight road interiors."""
    pairs, spans = [], []
    for road in roads:
        line = road['line']
        half = road['width']*.3
        for distance in np.arange(25.0, line.length-25, 20.0):
            center = np.array(line.interpolate(distance).coords[0])
            a = np.array(line.interpolate(distance-2).coords[0])
            b = np.array(line.interpolate(distance+2).coords[0])
            tangent = b-a
            normal = np.array([-tangent[1], tangent[0]])/max(np.linalg.norm(tangent), 1e-8)
            cross = np.array([center-normal*half, center+normal*half])
            if not shapely.covers(region, shapely.points(cross)).all():
                continue
            geo = shapely.transform(shapely.points(cross), compiler.TO_GEO.transform, interleaved=False)
            if shapely.covers(footprint, geo).all():
                pairs.append(cross)
                spans.append(2*half)
    if not pairs:
        raise ValueError('No supported road cross-sections inside the study region')
    points = np.asarray(pairs)
    before = sample_grid(base, transform, points[:, :, 0], points[:, :, 1])
    after = sample_grid(final, transform, points[:, :, 0], points[:, :, 1])
    old = abs(np.diff(before, axis=1)[:, 0])/np.asarray(spans)*100
    new = abs(np.diff(after, axis=1)[:, 0])/np.asarray(spans)*100
    return {'cross_sections': len(points), 'raw_transverse_grade_median_percent': float(np.median(old)),
            'graded_transverse_grade_median_percent': float(np.median(new)),
            'raw_transverse_grade_p95_percent': float(np.quantile(old, .95)),
            'graded_transverse_grade_p95_percent': float(np.quantile(new, .95)),
            'policy': 'Diagnostic spans; nearby junctions and parallel alignments can still affect the approximation'}


def write_scenes(final, base, transform, region):
    """Use stock gridded terrain, ordinary elevation/image layers, and native map-clamped context."""
    from city_context import context_xml
    context_source = ROOT/'output/boston-region-readymap-design'
    counts = {}
    for name in ['buildings', 'vegetation']:
        data = json.loads((context_source/f'{name}.geojson').read_text())
        data['features'] = [f for f in data['features'] if shapely.intersects(shape(f['geometry']), region)]
        counts[name] = len(data['features'])
        write_json(OUTPUT/f'{name}.geojson', data)
    assets = (ROOT.parents[2]/'data/procedural2/pbr').as_posix()
    context = context_xml(assets).split('<city>', 1)[1].rsplit('</city>', 1)[0]
    images = ''
    for name in ['asphalt', 'curb', 'gutter', 'yellow', 'white']:
        color = '#'+''.join(f'{c:02x}' for c in planar.PALETTE[name])
        images += f'''
    <FeatureImage name="Road {name}" min_level="14" max_data_level="19" tile_size="512">
        <features><OGRFeatures><url>{name}.geojson</url></OGRFeatures></features>
        <styles><style type="text/css">default {{ fill: {color}; }}</style></styles>
        <profile>global-geodetic</profile><cache_policy usage="no_cache"/>
    </FeatureImage>'''
    views = {'overview': (-71.0695, 42.354, -25, -52, 1600),
             'junction': (-71.0720629, 42.3546549, -30, -55, 110),
             'street': (-71.0669402142964, 42.352538202694134, 90.89815084081376, -5, 32)}
    camera_record = {}
    for mode, values in [('graded', final), ('raw', base)]:
        for detail, pixels, grid in [('fine', 64, 33), ('coarse', 256, 17)]:
            for view, (lon, lat, heading, pitch, distance) in views.items():
                x, y = compiler.TO_LOCAL.transform(lon, lat)
                elevation = float(sample_grid(values, transform, x, y))
                if view == 'street':
                    back = -np.array([np.sin(np.radians(heading)), np.cos(np.radians(heading))])
                    for _ in range(12):
                        eye = np.array([x, y])+back*distance*np.cos(np.radians(pitch))
                        eye_ground = sample_grid(values, transform, *eye)
                        pitch = -float(np.degrees(np.arcsin(np.clip((eye_ground+2.0-elevation)/distance, -.8, .8))))
                overlay = ('<GDALElevation name="Approximate road grading"><url>road-grading.tif</url>'
                           '<interpolation>bilinear</interpolation></GDALElevation>') if mode == 'graded' else ''
                text = f'''<map name="Boston regular-grid road grading / {mode} / {detail} / {view}">
    <libraries>osgEarthPrestige</libraries>
    <options><terrain tile_size="{grid}" lod_method="screen_space" tile_pixel_size="{pixels}"
        screen_space_error="0" max_lod="21" gpu_tessellation="false" skirt_ratio="0.005"/></options>
    <sky driver="simple" hours="17" ambient="0.45"/>
    <TMSImage name="ReadyMap imagery"><url>https://readymap.org/readymap/tiles/1.0.0/7/</url></TMSImage>
    <TMSElevation name="ReadyMap elevation"><url>https://readymap.org/readymap/tiles/1.0.0/116/</url>
        <vdatum>egm96</vdatum></TMSElevation>
    {overlay}
    {images}
    {context}
    <viewpoints home="0"><viewpoint name="{view}" long="{lon}" lat="{lat}" height="{elevation}"
        heading="{heading}" pitch="{pitch}" range="{distance}"/></viewpoints>
</map>
'''
                (OUTPUT/f'{mode}-{detail}-{view}.earth').write_text(text, encoding='utf-8')
                camera_record[f'{mode}-{view}'] = [lon, lat, elevation, heading, pitch, distance]
    write_json(OUTPUT/'cameras.json', camera_record)
    return counts


def main():
    """Build a bounded Boston grid from retained planar OSM output without changing the rendering engine."""
    OUTPUT.mkdir(parents=True, exist_ok=True)
    pending = OUTPUT/'build-in-progress.json'
    write_json(pending, {'status': 'Building; an exception leaves this marker to prevent viewing partial output'})
    geographic = box(*BBOX)
    region = shapely.transform(geographic, planar.TO_LOCAL.transform, interleaved=False)
    context = region.buffer(60, join_style=2)
    planar.configure_compiler(context)
    natural = ReadyMap(ROOT/'source/boston-region-readymap', compiler.TO_GEO)
    extent = shapely.transform(box(*context.bounds), compiler.TO_GEO.transform, interleaved=False).envelope
    print('Cropping the retained Boston pavement, paint, and curb polygons', flush=True)
    materials, counts, material_hash = export_materials(extent)
    print('Sampling connected road centerlines from ReadyMap', flush=True)
    roads, sampler, junctions = road_profiles(context, natural)
    footprint = materials['asphalt'].union(materials['curb'])
    print('Writing the regular elevation overlay', flush=True)
    final, base, transform, mask, audit = write_grid(footprint, sampler, natural, extent)
    audit.update(audit_cross_sections(roads, materials['asphalt'], final, base, transform, region))
    audit['source_material_sha256'] = material_hash
    audit['source_terrain'] = natural.metadata
    audit['centerline_ways'] = len(roads)
    audit['source_junction_nodes'] = len(junctions)
    audit['material_features'] = counts
    audit['context_features'] = write_scenes(final, base, transform, extent)
    audit['region_bbox'] = BBOX
    audit['renderer'] = 'Stock regular terrain grid; GDALElevation and FeatureImage; no road or terrain mesh'
    audit['water'] = 'No water grading implemented; future bed/bank targets can feed the same height grid'
    audit['curbs'] = 'Retained inferred physical-width curb/gutter material bands; no vertical curb geometry'
    write_json(OUTPUT/'audit.json', audit)
    pending.unlink()
    print(json.dumps({k: v for k, v in audit.items() if k != 'source_terrain'}, indent=2), flush=True)


if __name__ == '__main__':
    main()
