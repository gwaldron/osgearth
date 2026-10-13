"""Recompile a bounded Boston patch from OSM with the current planar compiler and show its vectors directly."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import pickle
import time

import numpy as np
import shapely
from shapely.geometry import box, mapping
from shapely.ops import unary_union

import compile as compiler
import raster_roads as planar


ROOT = Path(__file__).resolve().parent


def fingerprint():
    """Record all current compiler Python sources so a generated preview can be tied to its implementation."""
    return {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(ROOT.glob('*.py'))}


def fresh_planar_compile(region, output):
    """Run existing source, layout, paint, and curb policies; every intermediate is replaced in this invocation."""
    planar.REGION, planar.OUTPUT = region, output
    planar.PAINT_NETWORK = None
    roads, _, _, _, _, _, _ = planar.prepare_network(separate_levels=True)
    cells = [i for i in range(25) if planar.core_region(i).intersection(region).area > 0]
    for index in range(25):
        if index in cells:
            print('Fresh junction layouts:', planar.compile_layout_cell(index), flush=True)
        else:
            with (output/f'layout-cell-{index:02d}.pkl').open('wb') as stream:
                pickle.dump(([], []), stream, protocol=5)
    planar.assemble_layouts()
    for index in cells:
        print('Fresh crossings, controls, and lane paint:', planar.compile_paint_cell(index), flush=True)
    roads, _, ground, _, _, layouts = planar.load_paint_network()
    layers = {name: [] for name in planar.PALETTE}
    for index in cells:
        with (output/f'paint-cell-{index:02d}.pkl').open('rb') as stream:
            for name, geometry in pickle.load(stream).items():
                layers[name].extend(compiler.parts(geometry))
    urban_classes = {'primary', 'secondary', 'tertiary', 'residential', 'unclassified', 'living_street', 'service'}
    urban = [r['surface'] for r in roads if r['tags']['highway'] in urban_classes and r['tags'].get('kerb') != 'no']
    curb_domains = [j['domain'] for j in layouts if any(
        a['road']['tags']['highway'] in urban_classes and a['road']['tags'].get('kerb') != 'no'
        for a in j['approaches'])]
    planar.load_material_context(ground, urban, curb_domains)
    for index in range(100):
        cell = box(-5000+index % 10*1000, -5000+index//10*1000,
                   -4000+index % 10*1000, -4000+index//10*1000)
        if cell.intersection(region).area > 0:
            for name, polygons in planar.material_cell(index).items():
                layers[name].extend(polygons)
    return layers, roads, layouts, cells


def ring_path(polygon, center):
    """Preserve polygon holes in north-up SVG coordinates expressed in local meters."""
    rings = []
    for ring in [polygon.exterior, *polygon.interiors]:
        xy = np.asarray(ring.coords)-center
        rings.append('M'+' L'.join(f'{x:.3f},{-y:.3f}' for x, y in xy)+' Z')
    return ' '.join(rings)


def write_views(layers, center, size, output, title='Boston', views=None):
    """Export direct vector FeatureModel scenes and SVG plans; no terrain, grading, or imagery is compiled."""
    models, paths, exports = [], [], {}
    for index, name in enumerate(['asphalt', 'curb', 'gutter', 'yellow', 'white']):
        color = '#'+''.join(f'{v:02x}' for v in planar.PALETTE[name])
        geographic = shapely.transform(shapely.segmentize(layers[name], 10), planar.TO_GEO.transform, interleaved=False)
        if not shapely.is_valid(geographic).all():
            # Preserve the local polygons; normalize only sub-millimeter geographic reprojection debris.
            geographic = shapely.set_precision(shapely.make_valid(geographic), 1e-9)
        polygons = [p for geometry in geographic for p in compiler.parts(geometry)]
        if not shapely.is_valid(polygons).all():
            raise ValueError(f'Invalid geographic {name} geometry')
        features = [{'type': 'Feature', 'properties': {'material': name}, 'geometry': mapping(p)} for p in polygons]
        planar.write_json(output/f'{name}.geojson', {'type': 'FeatureCollection', 'features': features})
        exports[name] = {'polygons': len(polygons), 'area_m2': sum(p.area for p in layers[name])}
        paths.append(f'<g id="{name}" fill="{color}" fill-rule="evenodd">')
        paths.extend(f'<path d="{ring_path(p, center)}"/>' for p in layers[name])
        paths.append('</g>')
        models.append(f'''
    <OGRFeatures name="{name}-source"><url>{name}.geojson</url>
        <attribution>OpenStreetMap contributors / ODbL; inferred dimensions and markings</attribution>
    </OGRFeatures>
    <FeatureModel name="Road {name}" features="{name}-source">
        <styles><style type="text/css">default {{
            fill: {color}; altitude-offset: {index*.02+1}; render-lighting: false;
            render-depth-test: false; render-order: {index+1};
        }}</style></styles>
    </FeatureModel>''')
    lon, lat = planar.TO_GEO.transform(*center)
    views = views or {'overview': (lon, lat, size*1.8), 'intersection': (-71.0536274, 42.3536759, 160)}
    for name, (lon, lat, distance) in views.items():
        earth = f'''<map name="Freshly compiled {title} road vectors / {name}">
    <options><terrain color="#c8ccc6"/></options>
    {''.join(models)}
    <viewpoints home="0"><viewpoint name="{name}" long="{lon}" lat="{lat}" height="1"
        heading="0" pitch="-90" range="{distance}"/></viewpoints>
</map>
'''
        (output/f'{name}.earth').write_text(earth, encoding='utf-8', newline='\r\n')
        position = np.array(planar.TO_LOCAL.transform(lon, lat))-center
        span = size if name == 'overview' else distance*.65 if name == 'bridge' else 105
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="1200" '
               f'viewBox="{position[0]-span/2} {-position[1]-span/2} {span} {span}">'
               f'<title>Freshly compiled {title} road vectors</title>'
               '<desc>OpenStreetMap contributors / ODbL. Dimensions and some markings inferred.</desc>'
               f'<rect x="{-size}" y="{-size}" width="{size*2}" height="{size*2}" fill="#c8ccc6"/>'
               +''.join(paths)+'</svg>')
        (output/f'{name}.svg').write_text(svg, encoding='utf-8')
    return exports


def main():
    """Compile the same 900 m downtown patch from OSM, reject invalid output, and record source/code provenance."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'output/boston-vectors-current')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    pending = output/'build-in-progress.json'
    started = datetime.now(timezone.utc).isoformat()
    start = time.perf_counter()
    source_hashes = fingerprint()
    planar.write_json(pending, {'started_utc': started, 'policy': 'Do not view until this marker is removed'})
    center = np.array(planar.TO_LOCAL.transform(-71.0565, 42.3535))
    size = 900.0
    region = box(*(center-size/2), *(center+size/2))
    layers, roads, layouts, cells = fresh_planar_compile(region, output)
    for name, geometries in layers.items():
        if not geometries or not shapely.is_valid(geometries).all():
            raise ValueError(f'Missing or invalid freshly compiled {name} polygons')
    pavement = unary_union(layers['asphalt'])
    outside = {name: unary_union(layers[name]).difference(pavement).area for name in ['white', 'yellow', 'gutter']}
    if max(outside.values()) > .001:
        raise ValueError(f'Paint or gutter outside pavement: {outside}')
    exports = write_views(layers, center, size, output)
    from structure_vectors import export_structures
    planar.configure_compiler(region.buffer(250, join_style=2))
    structures = export_structures(region, output, planar.SOURCE)
    records = [json.loads((output/f'marking-evidence-{i:02d}.json').read_text()) for i in cells]
    manifest = {'started_utc': started, 'finished_utc': datetime.now(timezone.utc).isoformat(),
                'compile_seconds': time.perf_counter()-start, 'source_osm': str(planar.SOURCE),
                'source_sha256': hashlib.sha256(planar.SOURCE.read_bytes()).hexdigest(),
                'compiler_sha256': source_hashes, 'reused_geometry_checkpoints': False,
                'center': [-71.0565, 42.3535], 'size_m': size,
                'surface_ways_in_context': len(roads), 'junction_layouts': len(layouts),
                'lane_connections': sum(len(j['connectors']) for j in layouts), 'materials': exports,
                'crosswalks': sum(c['rendered'] for r in records for c in r['crossings']),
                'stop_bars': sum(len(c.get('bars', [])) for r in records for c in r['controls']),
                'guidance_intrusion_m2': max(r['guidance_intrusion_m2'] for r in records),
                'outside_pavement_m2': outside,
                'structures': structures,
                'policy': ('Fresh ground and separate structure vectors; topology retained; '
                           'heights unresolved; no tunnel surfaces')}
    if fingerprint() != source_hashes:
        raise ValueError('Compiler sources changed during compilation; rerun before viewing')
    planar.write_json(output/'manifest.json', manifest)
    pending.unlink()
    print(json.dumps({k: v for k, v in manifest.items() if k != 'compiler_sha256'}, indent=2), flush=True)


if __name__ == '__main__':
    main()
