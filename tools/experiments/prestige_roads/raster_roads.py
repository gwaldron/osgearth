"""Bake a bounded Boston road network to transparent, BC3-compressed geodetic XYZ tiles."""

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime, timezone
import gzip
import hashlib
import io
import json
from pathlib import Path
import pickle
import struct
import tempfile
import time
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from pyproj import CRS, Transformer
import shapely
from shapely.geometry import Point, Polygon, box, mapping
from shapely.ops import unary_union

ROOT = Path(__file__).parent
ORIGIN = (-71.080, 42.350)
LOCAL = CRS.from_proj4(f'+proj=aeqd +lat_0={ORIGIN[1]} +lon_0={ORIGIN[0]} +datum=WGS84 +units=m')
TO_GEO = Transformer.from_crs(LOCAL, 'EPSG:4326', always_xy=True)
TO_LOCAL = Transformer.from_crs('EPSG:4326', LOCAL, always_xy=True)
REGION = box(-5000, -5000, 5000, 5000)
GEO_REGION = shapely.transform(shapely.segmentize(REGION, 100), TO_GEO.transform, interleaved=False)
# Keep the compiled square as source context; clip every rendered material to the requested circular footprint.
RASTER_LEVEL = 16
RASTER_RADIUS_M = 3000
RASTER_REGION = Point(0, 0).buffer(RASTER_RADIUS_M, quad_segs=512)
GEO_RASTER_REGION = shapely.transform(RASTER_REGION, TO_GEO.transform, interleaved=False)
SOURCE = ROOT/'source'/'boston-metro.osm.gz'
OUTPUT = ROOT/'output'/'boston-metro-raster'
TILE_SIZE = 512
ASPHALT = (65, 68, 71)
WHITE = (229, 227, 216)
YELLOW = (231, 188, 46)
CURB = (181, 178, 166)
GUTTER = (105, 105, 101)
PALETTE = {'asphalt': ASPHALT, 'curb': CURB, 'gutter': GUTTER, 'yellow': YELLOW, 'white': WHITE}
RASTER_LAYERS = None
PAINT_NETWORK = None
MATERIAL_CONTEXT = None
SCENE_TEMPLATE = 'boston-region'
SCENE_NAME = 'boston-metro'


def configure_compiler(region):
    """Configure the existing geometry compiler in this process without changing other scene definitions."""
    import compile as compiler
    compiler.SCENES[SCENE_NAME] = dict(compiler.SCENES[SCENE_TEMPLATE], origin=ORIGIN, bbox=GEO_REGION.bounds)
    compiler.configure_scene(SCENE_NAME)
    compiler.REGION = region
    return compiler


def core_region(index):
    """Return a deterministic two-kilometer work cell; geometry ownership is assigned by source position."""
    column, row = index % 5, index // 5
    return box(-5000+column*2000, -5000+row*2000, -3000+column*2000, -3000+row*2000)


def owner(position):
    """Assign boundary points to exactly one work cell, using half-open intervals and clamped outer edges."""
    column, row = np.clip(np.floor((np.asarray(position)+5000)/2000).astype(int), 0, 4)
    return int(row*5+column)


def clip_rectangle(geometry, rectangle):
    """Clip polygonal pavement to a rectangle, discarding zero-area fragments and falling back for invalid fast clips."""
    from compile import parts
    clipped = shapely.clip_by_rect(geometry, *rectangle.bounds)
    if not clipped.is_valid:
        clipped = geometry.intersection(rectangle)
    if clipped.geom_type not in ('Polygon', 'MultiPolygon'):
        polygons = list(parts(clipped))
        clipped = unary_union(polygons) if polygons else Polygon()
    return clipped


def prepare_network(separate_levels=False):
    """Build source-backed planar road footprints once, excluding structures before connectivity discovery."""
    compiler = configure_compiler(REGION.buffer(250, join_style=2))
    roads, xy, flags, relations = compiler.read_source(SOURCE)
    from structure_vectors import separate_structure
    excluded = [{'id': r['id'], 'version': r['version'], 'tags': r['tags'],
                 'reason': 'bridge' if r['bridge'] else 'tunnel' if r['tunnel'] else 'separate layer/level'}
                for r in roads if r['bridge'] or r['tunnel'] or (separate_levels and separate_structure(r))]
    excluded_ids = {r['id'] for r in excluded}
    roads = [r for r in roads if r['id'] not in excluded_ids]
    print(f'Preparing {len(roads):,} surface ways; excluding {len(excluded):,} structure ways', flush=True)
    graph, _ = compiler.solve_profiles(roads, xy)
    from transitions import discover_transitions
    transitions, transition_diagnostics = discover_transitions(roads, compiler.REGION)
    raw = unary_union([r['surface'] for r in roads])
    # Closing is local to eight meters. A sixteen-meter halo keeps work-cell cuts outside the affected core.
    # Components disconnected within the halo retain the conservative foreign-component guard.
    rounded = []
    for index in range(25):
        core = core_region(index)
        if index % 5 == 0:
            print(f'Rounding pavement neighborhoods {index+1}/25', flush=True)
        local = raw.intersection(core.buffer(16, join_style=2))
        rounded.append(compiler.rounded_ground_components(local).intersection(core))
    # Keep source context outside the crop for complete approach profiles; only the final render is cropped.
    ground = unary_union(rounded).union(raw.difference(REGION))
    islands = unary_union([Polygon(ring) for p in compiler.parts(ground) for ring in p.interiors])
    junctions = [n for n, adjacent in graph.items() if len({v for v, _ in adjacent}) >= 3]
    package = (roads, xy, relations, ground, islands, junctions, transitions)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    with (OUTPUT/'network.pkl').open('wb') as stream:
        pickle.dump(package, stream, protocol=5)
    write_json(OUTPUT/'source-road-evidence.json', {
        'source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(), 'excluded_structures': excluded,
        'roads': [{k: r[k] for k in ('id', 'version', 'tags', 'refs', 'lanes', 'width', 'width_reason',
                                    'lane_count_reason')} for r in roads],
        'width_flags': flags, 'restriction_relations': relations, 'transition_diagnostics': transition_diagnostics})
    return package


def load_network():
    """Load this experiment's generated planar checkpoint; callers only consume locally produced pickle files."""
    with (OUTPUT/'network.pkl').open('rb') as stream:
        return pickle.load(stream)


def compile_layout_cell(index):
    """Resolve a haloed neighborhood, committing whole junction domains only in their unique owning cell."""
    from urban import discover_urban
    roads, xy, relations, ground, islands, _, transitions = load_network()
    core = core_region(index)
    halo = core.buffer(220, join_style=2)
    compiler = configure_compiler(halo)
    roads = [r for r in roads if r['line'].intersects(halo)]
    ground, islands = ground.intersection(halo), islands.intersection(halo)
    locale = compiler.SCENES[compiler.SCENE]
    layouts, _, diagnostics = discover_urban(roads, xy, relations, [], ground, islands, halo,
                                            locale['driving_side'], locale['center_line'], compact_pavement=True)
    own = []
    for layout in layouts:
        if not REGION.covers(Point(layout['position'])) or owner(layout['position']) != index:
            continue
        if any(t['domain'].intersection(layout['domain']).area > .001 for t in transitions):
            diagnostics.append({'node': layout['node'], 'status': 'unresolved',
                                'reason': 'Junction overlaps a longitudinal taper; raster pass withholds junction'})
            continue
        # Retain the full accepted domain across cell edges, but shed unrelated neighborhood pavement.
        layout['pavement'] = layout['pavement'].intersection(layout['domain'].buffer(25))
        own.append(layout)
    diagnostics = [d for d in diagnostics if REGION.covers(Point(xy[d['node']])) and owner(xy[d['node']]) == index]
    with (OUTPUT/f'layout-cell-{index:02d}.pkl').open('wb') as stream:
        pickle.dump((own, diagnostics), stream, protocol=5)
    return index, len(roads), len(own)


def accepted_domains(layouts):
    """Preserve sequential ownership decisions while testing only spatially overlapping earlier accepted domains."""
    tree = shapely.STRtree([layout['domain'] for layout in layouts])
    accepted = set()
    for index, layout in enumerate(layouts):
        neighbors = tree.query(layout['domain'])
        if not any(other in accepted and layout['domain'].intersection(layouts[other]['domain']).area > .001
                   for other in neighbors):
            accepted.add(index)
    return accepted


def assemble_layouts():
    """Merge whole owned domains with the common base, rejecting cross-cell overlaps instead of double painting."""
    roads, xy, relations, ground, islands, junctions, transitions = load_network()
    candidates, record_groups = [], []
    for index in range(25):
        with (OUTPUT/f'layout-cell-{index:02d}.pkl').open('rb') as stream:
            own, records = pickle.load(stream)
        record_groups.append(records)
        candidates.extend((layout, records) for layout in own)
    accepted = accepted_domains([layout for layout, _ in candidates])
    layouts = []
    for index, (layout, records) in enumerate(candidates):
        if index in accepted:
            layouts.append(layout)
        else:
            records.append({'node': layout['node'], 'status': 'unresolved',
                            'reason': 'Cross-cell junction ownership overlap; layout withheld'})
    diagnostics = [record for group in record_groups for record in group]
    domains = unary_union([j['domain'] for j in layouts])
    patches = unary_union([j['pavement'].intersection(j['domain']) for j in layouts])
    ground = ground.difference(domains).union(patches).intersection(REGION)
    with (OUTPUT/'resolved.pkl').open('wb') as stream:
        pickle.dump((roads, xy, ground, junctions, transitions, layouts), stream, protocol=5)
    write_json(OUTPUT/'layout-discovery.json', {'junctions': diagnostics, 'unique_layouts': len(layouts),
                                              'lane_connectors': sum(len(j['connectors']) for j in layouts)})
    features = []
    for layout in layouts:
        for connection in layout['connectors']:
            properties = {k: connection[k] for k in ('from', 'from_lane', 'to', 'to_lane')}
            properties.update(layout=layout['name'], inference='Lane path geometry and allocation are inferred',
                              source_restrictions=layout['restrictions'])
            features.append({'type': 'Feature', 'properties': properties,
                             'geometry': mapping(shapely.transform(connection['line'], TO_GEO.transform,
                                                                   interleaved=False))})
    write_json(OUTPUT/'lane-connectivity.geojson', {'type': 'FeatureCollection', 'features': features})
    print(f'Assembled {len(layouts):,} whole junction layouts', flush=True)


def load_paint_network():
    """Cache the immutable resolved checkpoint once per paint worker; callers copy layouts before adding paint fields."""
    global PAINT_NETWORK
    if PAINT_NETWORK is None:
        with (OUTPUT/'resolved.pkl').open('rb') as stream:
            PAINT_NETWORK = pickle.load(stream)
    return PAINT_NETWORK


def compile_paint_cell(index):
    """Reuse existing crossing, stop-bar, and lane-paint policies; clip final paint only after shared ownership."""
    from controls import read_controls
    from urban import approach_paint, street_paint, urban_stop_bars
    roads, xy, ground, junction_nodes, transitions, layouts = load_paint_network()
    core = core_region(index)
    halo = core.buffer(100, join_style=2)
    compiler = configure_compiler(halo)
    roads = [r for r in roads if r['surface'].intersects(halo)]
    ground = clip_rectangle(ground, halo)
    layouts = [dict(j) for j in layouts if j['domain'].intersects(halo)]
    transitions = [t for t in transitions if t['domain'].intersects(halo)]
    crossings = compiler.read_crossings(SOURCE, roads, xy)
    crossing_paint, crossing_exclusions, passages = compiler.crossing_markings(crossings, roads, None, None, True)
    controls = read_controls(SOURCE, roads, xy, halo)
    locale = compiler.SCENES[compiler.SCENE]
    stop_paint, stop_exclusions = urban_stop_bars(controls, roads, layouts, ground, passages, locale['driving_side'])
    suppression = crossing_exclusions.union(stop_exclusions)
    for layout in layouts:
        # Paint uses the final shared surface, not the compact discovery checkpoint's stored neighborhood.
        layout['pavement'] = ground
        approach_paint(layout, suppression)
    domains = unary_union([j['domain'] for j in layouts+transitions])
    junction_disks = unary_union([Point(xy[n]).buffer(11) for n in junction_nodes if halo.covers(Point(xy[n]))])
    exclusions = unary_union([junction_disks, domains, suppression])
    white, yellow = street_paint(roads, exclusions, ground, locale['driving_side'], locale['center_line'])
    edge_domains = unary_union([j['domain'] for j in layouts if j['kind'] not in ('two_way', 'urban')]
                               +[t['domain'] for t in transitions])
    edge_guidance = ground.buffer(-.30).boundary.intersection(edge_domains).buffer(.07)
    left_edges = unary_union([c['left_edge'].buffer(.8) for j in layouts for c in j['connectors']
                             if c['from_lane'] == 1 and j['kind'] not in ('two_way', 'urban')]
                            +[t['left_edge'].buffer(.8) for t in transitions])
    guidance_white = unary_union([j['paint'] for j in layouts+transitions]
                                 +[edge_guidance.difference(left_edges)]).difference(suppression)
    guidance_yellow = unary_union([j.get('yellow_paint', Polygon()) for j in layouts]
                                  +[edge_guidance.intersection(left_edges)]).difference(suppression)
    guidance = guidance_white.union(guidance_yellow)
    intrusion = guidance.intersection(passages.union(stop_exclusions)).area
    if intrusion > .001:
        raise ValueError(f'Marking exclusion failure in cell {index}: {intrusion}')
    paint = {'white': unary_union([white, guidance_white, crossing_paint['white'], stop_paint]),
             'yellow': unary_union([yellow, guidance_yellow, crossing_paint['yellow']])}
    paint = {k: v.intersection(ground).intersection(core) for k, v in paint.items()}
    crossings = [r for r in crossings if REGION.covers(Point(r['position'])) and owner(r['position']) == index]
    controls = [r for r in controls if 'position' in r and REGION.covers(Point(r['position']))
                and owner(r['position']) == index]
    with (OUTPUT/f'paint-cell-{index:02d}.pkl').open('wb') as stream:
        pickle.dump(paint, stream, protocol=5)
    layout_evidence = [{'name': j['name'], 'arrows': j['evidence'], 'lane_labels': j.get('lane_label_evidence', [])}
                       for j in layouts if owner(j['position']) == index]
    write_json(OUTPUT/f'marking-evidence-{index:02d}.json', {'crossings': crossings, 'controls': controls,
                                                          'layout_markings': layout_evidence,
                                                          'guidance_intrusion_m2': intrusion})
    return index, sum(r['rendered'] for r in crossings), sum(len(r.get('bars', [])) for r in controls)


def compile_geometry(workers, prepared=False, layouts_ready=False):
    """Run bounded planar work cells in worker processes, then retain one seamless geographic render database."""
    if not prepared:
        prepare_network()
    operations = (compile_paint_cell,) if layouts_ready else (compile_layout_cell, compile_paint_cell)
    for operation in operations:
        prefix = 'layout' if operation == compile_layout_cell else 'paint'
        pending = [index for index in range(25)
                   if not prepared or not (OUTPUT/f'{prefix}-cell-{index:02d}.pkl').exists()]
        concurrency = min(workers, 4) if operation == compile_paint_cell else workers
        with ProcessPoolExecutor(max_workers=concurrency) as pool:
            jobs = [pool.submit(operation, index) for index in pending]
            for done in as_completed(jobs):
                print(operation.__name__, done.result(), flush=True)
        if operation == compile_layout_cell:
            assemble_layouts()
    assemble_render_layers(workers)


def load_material_context(ground, urban, curb_domains):
    """Initialize read-only surface indexes in a material worker without loading the large lane-curve checkpoint."""
    global MATERIAL_CONTEXT
    MATERIAL_CONTEXT = (ground, np.array(urban, dtype=object), shapely.STRtree(urban),
                        np.array(curb_domains, dtype=object), shapely.STRtree(curb_domains))


def material_cell(index):
    """Return pavement and physical-width curb/gutter polygons for one uniquely owned one-kilometer window."""
    from compile import parts
    ground, urban, urban_tree, curb_domains, domain_tree = MATERIAL_CONTEXT
    cell = box(-5000+index%10*1000, -5000+index//10*1000, -4000+index%10*1000, -4000+index//10*1000)
    halo = cell.buffer(2, join_style=2)
    local = clip_rectangle(ground, halo)
    support = unary_union(urban[urban_tree.query(halo, predicate='intersects')]).intersection(halo).buffer(.5)
    mouths = unary_union(curb_domains[domain_tree.query(halo, predicate='intersects')]).intersection(halo)
    support = support.union(mouths.buffer(.5))
    # Remove the synthetic crop boundary before making bands. Cell boundaries lie outside the two-meter halo.
    edge = local.boundary.difference(REGION.boundary.buffer(.01)).intersection(support)
    curb = edge.buffer(.22).difference(local).intersection(cell).intersection(REGION)
    gutter = edge.buffer(.25).intersection(local).intersection(cell)
    return {'asphalt': list(parts(local.intersection(cell))), 'curb': list(parts(curb)), 'gutter': list(parts(gutter))}


def assemble_render_layers(workers=4):
    """Combine retained paint with inferred physical-width curb/gutter bands along the final pavement boundary."""
    with (OUTPUT/'resolved.pkl').open('rb') as stream:
        roads, _, ground, _, _, layouts = pickle.load(stream)
    compiler = configure_compiler(REGION)
    layers = {key: [] for key in PALETTE}
    for index in range(25):
        with (OUTPUT/f'paint-cell-{index:02d}.pkl').open('rb') as stream:
            paint = pickle.load(stream)
        for colour, geometry in paint.items():
            layers[colour].extend(compiler.parts(geometry))
    urban = [r['surface'] for r in roads if r['tags']['highway'] in
             ('primary', 'secondary', 'tertiary', 'residential', 'unclassified', 'living_street', 'service')
             and r['tags'].get('kerb') != 'no']
    curb_domains = [j['domain'] for j in layouts if any(
        a['road']['tags']['highway'] in ('primary', 'secondary', 'tertiary', 'residential', 'unclassified',
                                        'living_street', 'service') and a['road']['tags'].get('kerb') != 'no'
        for a in j['approaches'])]
    with ProcessPoolExecutor(max_workers=workers, initializer=load_material_context,
                             initargs=(ground, urban, curb_domains)) as pool:
        for count, materials in enumerate(pool.map(material_cell, range(100)), 1):
            for name, polygons in materials.items():
                layers[name].extend(polygons)
            if count % 10 == 0:
                print(f'Assembled pavement, curbs, and gutters: {count}/100 windows', flush=True)
    geographic = {key: [shapely.transform(shapely.segmentize(p, 10), TO_GEO.transform, interleaved=False)
                       for p in polygons] for key, polygons in layers.items()}
    with (OUTPUT/'render-polygons.pkl').open('wb') as stream:
        pickle.dump(geographic, stream, protocol=5)
    write_json(OUTPUT/'geometry-summary.json', {
        'origin': ORIGIN, 'size_m': [10000, 10000], 'bbox': GEO_REGION.bounds,
        'surface_ways_in_context': len(roads), 'surface_ways_in_region': sum(r['line'].intersects(REGION) for r in roads),
        'unique_layouts': len(layouts), 'lane_connections': sum(len(j['connectors']) for j in layouts),
        'pavement_area_m2': ground.area, 'source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        'layer_area_m2': {key: sum(p.area for p in polygons) for key, polygons in layers.items()},
        'view_candidates': [{'name': j['name'], 'position': j['position'].tolist(), 'arrow_count': j['arrow_count'],
                             'connections': len(j['connectors'])} for j in layouts
                            if np.linalg.norm(j['position']) < 2200],
        'curb_policy': 'INFERRED 0.22m concrete curb and 0.25m shaded gutter at urban pavement union edges; no height',
        'policy': 'Existing planar road/layout/marking compiler; no terrain solve or mesh triangulation.'})


def tile_bounds(level, x, y):
    """Return north-origin XYZ bounds for osgEarth's two-by-one geodetic root grid, in longitude/latitude."""
    span = 180.0/(1 << level)
    return (-180+x*span, 90-(y+1)*span, -180+(x+1)*span, 90-y*span)


def tile_keys(level):
    """Enumerate only tiles intersecting the circular output footprint, including transparent water tiles."""
    west, south, east, north = GEO_RASTER_REGION.bounds
    span = 180.0/(1 << level)
    for y in range(int(np.floor((90-north)/span)), int(np.ceil((90-south)/span))):
        for x in range(int(np.floor((west+180)/span)), int(np.ceil((east+180)/span))):
            if GEO_RASTER_REGION.intersects(box(*tile_bounds(level, x, y))):
                yield level, x, y


def load_raster_layers():
    """Initialize per-process polygon indexes once; the database contains WGS84 planar render shapes."""
    global RASTER_LAYERS
    with (OUTPUT/'render-polygons.pkl').open('rb') as stream:
        polygons = pickle.load(stream)
    RASTER_LAYERS = {name: (np.array(shapes, dtype=object), shapely.STRtree(shapes))
                     for name, shapes in polygons.items()}


def transparent_image(size):
    """Keep asphalt RGB beneath zero alpha so compression/filtering does not introduce black edge fringes."""
    return Image.new('RGBA', size, (*ASPHALT, 0))


def rasterize_bounds(bounds, size=512, supersample=4):
    """Resolve material priority per subsample, then area-filter premultiplied colors and coverage together."""
    from rasterio.features import rasterize
    from rasterio.transform import from_bounds
    width = size*supersample
    transform = from_bounds(*bounds, width, width)
    window = box(*bounds)
    shapes = []
    colors = np.array([(0, 0, 0)]+list(PALETTE.values()), dtype=np.float32)
    for value, name in enumerate(PALETTE, 1):
        geometries, tree = RASTER_LAYERS.get(name, ([], None))
        if tree is None:
            continue
        selected = geometries[tree.query(window, predicate='intersects')]
        shapes.extend((shape, value) for shape in selected)
    if not shapes:
        return transparent_image((size, size))
    pixels = rasterize(shapes, out_shape=(width, width), transform=transform, fill=0, dtype='uint8')
    if not GEO_RASTER_REGION.covers(window):
        # Crop before area filtering so boundary pixels retain correct fractional coverage for every material.
        footprint = rasterize([(GEO_RASTER_REGION, 1)], out_shape=(width, width), transform=transform,
                              fill=0, dtype='uint8')
        pixels[footprint == 0] = 0
    alpha = (pixels != 0).reshape(size, supersample, size, supersample).mean(axis=(1, 3), dtype=np.float32)
    premultiplied = np.stack([colors[pixels, channel].reshape(size, supersample, size, supersample)
                             .mean(axis=(1, 3), dtype=np.float32) for channel in range(3)], axis=-1)
    rgba = np.empty((size, size, 4), dtype=np.uint8)
    rgba[:, :, :3] = ASPHALT
    np.divide(premultiplied, alpha[:, :, None], out=premultiplied, where=alpha[:, :, None] > 0)
    rgba[:, :, :3] = np.where(alpha[:, :, None] > 0, np.rint(premultiplied), rgba[:, :, :3])
    rgba[:, :, 3] = np.rint(alpha*255)
    return Image.fromarray(rgba)


def reduce_rgba(image, size):
    """Area-filter premultiplied RGBA, then restore straight-alpha color for DDS and osgEarth compositing."""
    result = image.resize(size, Image.Resampling.BOX)
    pixels = np.asarray(result).copy()
    pixels[pixels[:, :, 3] == 0, :3] = ASPHALT
    return Image.fromarray(pixels)


def bc3_alpha_blocks(image, encoded):
    """Replace encoder alpha with explicit nearest-palette BC3 blocks, preserving coverage and zero-alpha pixels."""
    alpha = np.asarray(image)[:, :, 3]
    alpha = np.pad(alpha, ((0, (-alpha.shape[0]) % 4), (0, (-alpha.shape[1]) % 4)), mode='edge')
    blocks = alpha.reshape(alpha.shape[0]//4, 4, alpha.shape[1]//4, 4).transpose(0, 2, 1, 3).reshape(-1, 16)
    high, low = blocks.max(axis=1).astype(np.uint16), blocks.min(axis=1).astype(np.uint16)
    palette = np.column_stack([high, low]+[((7-i)*high+i*low)//7 for i in range(1, 7)])
    error = np.abs(blocks[:, :, None].astype(np.int16)-palette[:, None, :].astype(np.int16))
    indices = error.argmin(axis=2).astype(np.uint64)
    packed = np.sum(indices << (np.arange(16, dtype=np.uint64)*3), axis=1)
    bits = high.astype(np.uint64) | (low.astype(np.uint64) << 8) | (packed << 16)
    payload = np.frombuffer(encoded[128:], dtype=np.uint8).copy().reshape(-1, 16)
    payload[:, :8] = bits.astype('<u8').view(np.uint8).reshape(-1, 8)
    return encoded[:128]+payload.tobytes()


def write_dds(image, path):
    """Encode BC3/DXT5 blocks plus all ten mip levels into a native DDS texture; no runtime encoder is needed."""
    levels, current = [], image
    while True:
        stream = io.BytesIO()
        current.save(stream, format='DDS', pixel_format='DXT5')
        levels.append(bc3_alpha_blocks(current, stream.getvalue()))
        if current.width == 1:
            break
        current = reduce_rgba(current, (max(1, current.width//2), max(1, current.height//2)))
    header = bytearray(levels[0][:128])
    flags = (struct.unpack_from('<I', header, 8)[0] | 0x20000 | 0x80000) & ~0x8
    struct.pack_into('<I', header, 8, flags)
    # Pillow's encoder supplies blocks correctly but its linear-size field is not the compressed level size.
    struct.pack_into('<I', header, 20, len(levels[0])-128)
    struct.pack_into('<I', header, 28, len(levels))
    struct.pack_into('<I', header, 108, 0x1000 | 0x8 | 0x400000)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(header+b''.join(level[128:] for level in levels))


def tile_path(key, compressed=True):
    """Locate an encoded tile or its lossless build intermediate using exactly the same geodetic tile address."""
    level, x, y = key
    return OUTPUT/('tiles' if compressed else 'build-png')/str(level)/str(x)/(str(y)+('.dds' if compressed else '.png'))


def bake_tile(key):
    """Bake the requested LOD using 32-by-32 subpixel coverage in bounded-memory subwindows."""
    level, x, y = key
    if level != RASTER_LEVEL:
        raise ValueError(f'This experiment is restricted to LOD {RASTER_LEVEL}')
    west, south, east, north = tile_bounds(*key)
    step = (east-west)/4
    image = transparent_image((512, 512))
    for row in range(4):
        for column in range(4):
            bounds = (west+column*step, north-(row+1)*step, west+(column+1)*step, north-row*step)
            image.paste(rasterize_bounds(bounds, size=128, supersample=32), (column*128, row*128))
    intermediate = tile_path(key, False)
    intermediate.parent.mkdir(parents=True, exist_ok=True)
    image.save(intermediate, compress_level=3)
    destination = tile_path(key)
    write_dds(image, destination)
    return {'z': level, 'x': x, 'y': y, 'bytes': destination.stat().st_size,
            'nontransparent_pixels': int(np.count_nonzero(np.asarray(image)[:, :, 3])),
            'sha256': hashlib.sha256(destination.read_bytes()).hexdigest()}


def bake_pyramid(workers):
    """Build the requested LOD slice and record the circular footprint and every compressed tile for validation."""
    start, records = time.perf_counter(), []
    write_json(OUTPUT/'tile-build-in-progress.json', {'started_utc': datetime.now(timezone.utc).isoformat()})
    for level in (RASTER_LEVEL,):
        keys = list(tile_keys(level))
        with ProcessPoolExecutor(max_workers=workers, initializer=load_raster_layers) as pool:
            for count, record in enumerate(pool.map(bake_tile, keys, chunksize=4), 1):
                records.append(record)
                if count % 10 == 0 or count == len(keys):
                    print(f'LOD {level}: {count}/{len(keys)} compressed tiles', flush=True)
    write_json(OUTPUT/'tile-manifest.json', {
        'profile': 'global-geodetic', 'root_tiles': [2, 1], 'xyz_y_origin': 'north', 'tile_size': 512,
        'levels': [RASTER_LEVEL], 'coverage_samples_per_pixel': [32, 32],
        'format': 'DDS BC3/DXT5, straight alpha, complete 10-level mip chain',
        'render_bbox': GEO_RASTER_REGION.bounds,
        'render_footprint': {'shape': 'geodesic circle', 'center_lon_lat': ORIGIN, 'radius_m': RASTER_RADIUS_M,
                             'polygon_radial_error_bound_m': RASTER_RADIUS_M*(1-np.cos(np.pi/2048))},
        'source_context': json.loads((OUTPUT/'geometry-summary.json').read_text()),
        'tile_count': len(records), 'bytes': sum(r['bytes'] for r in records),
        'seconds': time.perf_counter()-start, 'tiles': records})
    write_scenes()
    (OUTPUT/'tile-build-in-progress.json').unlink()


def write_scenes():
    """Emit plain XYZImageLayer scenes over ReadyMap; source and terrain use the same geodetic tile grid."""
    viewpoints = {'overview': (*ORIGIN, -90, 13500), 'back-bay': (-71.080, 42.350, -65, 1800),
                  'intersection': (-71.075, 42.350, -70, 300), 'cambridge': (-71.096, 42.366, -65, 1500)}
    if (OUTPUT/'geometry-summary.json').exists():
        layouts = json.loads((OUTPUT/'geometry-summary.json').read_text(encoding='utf-8')).get('view_candidates', [])
        painted_arrows = {}
        for path in OUTPUT.glob('marking-evidence-*.json'):
            for item in json.loads(path.read_text(encoding='utf-8')).get('layout_markings', []):
                painted_arrows[item['name']] = sum(a['rendered'] for a in item['arrows'])
        candidates = [j for j in layouts if np.linalg.norm(j['position']) < 2200
                      and painted_arrows.get(j['name'], j['arrow_count']) > 0]
        if candidates:
            selected = max(candidates, key=lambda j: (painted_arrows.get(j['name'], j['arrow_count']),
                                                       j['connections']))
            lon, lat = TO_GEO.transform(*selected['position'])
            viewpoints['intersection'] = (lon, lat, -70, 400)
            write_json(OUTPUT/'detail-view-evidence.json', {'layout': selected['name'], 'position': [lon, lat],
                'source_indicated_arrows': painted_arrows.get(selected['name'], selected['arrow_count'])})
    for name, (lon, lat, pitch, distance) in viewpoints.items():
        pixel_size = 16 if name == 'overview' else 128
        terrain_max_lod = RASTER_LEVEL if name == 'overview' else 19
        content = f'''<map name="Prestige Roads / Boston 3km radius / LOD {RASTER_LEVEL} BC3 XYZ / {name}">
    <options><profile>global-geodetic</profile><screen_space_error>0</screen_space_error>
        <terrain tile_size="33" lod_method="screen_space" tile_pixel_size="{pixel_size}"
            screen_space_error="0" max_lod="{terrain_max_lod}"/></options>
    <TMSImage name="ReadyMap imagery"><url>https://readymap.org/readymap/tiles/1.0.0/7/</url></TMSImage>
    <TMSElevation name="ReadyMap elevation"><url>https://readymap.org/readymap/tiles/1.0.0/116/</url>
        <vdatum>egm96</vdatum></TMSElevation>
    <XYZImage name="Prestige surface roads / precompressed BC3">
        <url>tiles/{{z}}/{{x}}/{{y}}.dds</url><profile>global-geodetic</profile>
        <tile_size>512</tile_size><min_level>{RASTER_LEVEL}</min_level>
        <max_data_level>{RASTER_LEVEL}</max_data_level><invert_y>false</invert_y><cache_policy usage="no_cache"/>
        <read_options>dds_flip</read_options>
        <texture_compression>none</texture_compression>
        <attribution>OpenStreetMap contributors / ODbL; geometry and paint dimensions inferred</attribution>
    </XYZImage>
    <viewpoints home="0"><viewpoint name="{name}" lat="{lat}" long="{lon}" height="20"
        heading="0" pitch="{pitch}" range="{distance}"/></viewpoints>
</map>
'''
        (OUTPUT/f'boston-xyz-{name}.earth').write_text(content, encoding='utf-8', newline='\r\n')


def write_json(path, value):
    """Write UTF-8 diagnostics; filesystem errors propagate to the caller."""
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')


def fetch_source(pbf=None):
    """Extract complete highway ways and referenced controls from a retained Geofabrik PBF; retain source hashes."""
    if SOURCE.exists():
        print('Reusing retained Boston metro source', flush=True)
        return
    import osmium
    pbf = Path(pbf or Path(tempfile.gettempdir())/'prestige-massachusetts-261009.osm.pbf')
    url = 'https://download.geofabrik.de/north-america/us/massachusetts-261009.osm.pbf'
    if not pbf.exists():
        print('Downloading retained Massachusetts PBF', flush=True)
        urllib.request.urlretrieve(url, pbf)
    context = shapely.transform(REGION.buffer(250), TO_GEO.transform, interleaved=False)
    west, south, east, north = context.bounds
    root = ET.Element('osm', version='0.6', generator='Prestige Roads regional PBF extraction')
    way_ids, node_ids = set(), set()
    print('Extracting source topology from Massachusetts', flush=True)
    for obj in osmium.FileProcessor(pbf).with_locations():
        if obj.is_way() and ('highway' in obj.tags or 'road_marking' in obj.tags):
            coords = [(n.lon, n.lat) for n in obj.nodes]
            if len(coords) < 2:
                continue
            xs, ys = zip(*coords)
            if max(xs) < west or min(xs) > east or max(ys) < south or min(ys) > north:
                continue
            element = ET.SubElement(root, 'way', id=str(obj.id), version=str(obj.version))
            way_ids.add(obj.id)
            for n in obj.nodes:
                ET.SubElement(element, 'nd', ref=str(n.ref))
                node_ids.add(n.ref)
            for tag in obj.tags:
                ET.SubElement(element, 'tag', k=tag.k, v=tag.v)
        elif obj.is_relation() and obj.tags.get('type') in ('restriction', 'connectivity'):
            if not any(m.type == 'w' and m.ref in way_ids for m in obj.members):
                continue
            element = ET.SubElement(root, 'relation', id=str(obj.id), version=str(obj.version))
            for member in obj.members:
                ET.SubElement(element, 'member', type={'n': 'node', 'w': 'way', 'r': 'relation'}[member.type],
                              ref=str(member.ref), role=member.role)
                if member.type == 'n':
                    node_ids.add(member.ref)
            for tag in obj.tags:
                ET.SubElement(element, 'tag', k=tag.k, v=tag.v)
    selector = osmium.filter.IdFilter(node_ids).enable_for(osmium.osm.NODE)
    for obj in osmium.FileProcessor(pbf, entities=osmium.osm.NODE).with_filter(selector):
        element = ET.SubElement(root, 'node', id=str(obj.id), version=str(obj.version),
                                lon=str(obj.lon), lat=str(obj.lat))
        for tag in obj.tags:
            ET.SubElement(element, 'tag', k=tag.k, v=tag.v)
    if len(root.findall('node')) != len(node_ids):
        raise ValueError('Regional source has missing referenced nodes')
    payload = gzip.compress(ET.tostring(root, encoding='utf-8', xml_declaration=True), mtime=0)
    SOURCE.parent.mkdir(parents=True, exist_ok=True)
    SOURCE.write_bytes(payload)
    write_json(SOURCE.with_suffix('.json'), {
        'url': url, 'extracted_utc': datetime.now(timezone.utc).isoformat(),
        'pbf_sha256': hashlib.sha256(pbf.read_bytes()).hexdigest(), 'sha256': hashlib.sha256(payload).hexdigest(),
        'nodes': len(root.findall('node')), 'ways': len(root.findall('way')),
        'relations': len(root.findall('relation')), 'render_origin': ORIGIN, 'render_size_m': [10000, 10000],
        'context_m': 250, 'attribution': 'OpenStreetMap contributors',
        'license': 'https://www.openstreetmap.org/copyright',
        'policy': ('All referenced way nodes retained; parent restrictions retained even when a member is outside '
                   'context. Original versions/tags preserved; contributor identifiers absent.')})
    print(f'Retained {len(payload):,} source bytes and {len(root.findall("way")):,} ways', flush=True)


def main():
    """Execute an explicit raster experiment stage, preserving downloaded source evidence on reruns."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=['fetch', 'geometry', 'layouts', 'paint', 'render', 'tiles', 'scenes'])
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args()
    if args.stage == 'fetch':
        fetch_source()
    elif args.stage == 'geometry':
        compile_geometry(args.workers)
    elif args.stage == 'layouts':
        compile_geometry(args.workers, prepared=True)
    elif args.stage == 'paint':
        compile_geometry(args.workers, prepared=True, layouts_ready=True)
    elif args.stage == 'render':
        assemble_render_layers(args.workers)
    elif args.stage == 'tiles':
        bake_pyramid(args.workers)
    elif args.stage == 'scenes':
        write_scenes()


if __name__ == '__main__':
    main()
