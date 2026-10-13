"""Retain OSM building and vegetation evidence and compose native osgEarth clamping consumers."""

import argparse
import gzip
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import rasterio
import shapely
from shapely.geometry import LineString, Point, Polygon, mapping, shape
from shapely.ops import polygonize, unary_union

import compile as compiler
from terrain import ReadyMap


def dimensions(tags):
    """Resolve visual building height and base offset from source dimensions or explicit, bounded assumptions."""
    def meters(value):
        """Parse a single metric or feet value; ambiguous lists and unsupported units remain unresolved."""
        match = re.fullmatch(r'\s*(\d+(?:\.\d+)?)\s*(m|meters|metres|ft|feet|\')?\s*', value or '')
        if not match:
            return None
        result = float(match[1])*(.3048 if match[2] in ('ft', 'feet', "'") else 1)
        return result if 0 <= result <= 500 else None

    height = meters(tags.get('height', tags.get('building:height')))
    reason = 'source height'
    if height is None:
        try:
            levels = float(tags.get('building:levels', ''))
        except ValueError:
            levels = 0
        height = levels*3.2 if 0 < levels <= 130 else 10.0
        reason = 'INFERRED 3.2m per mapped level' if 0 < levels <= 130 else 'INFERRED 10m fallback'
    base = meters(tags.get('min_height'))
    if base is None:
        try:
            base = max(0, float(tags.get('building:min_level', 0)))*3.2
        except ValueError:
            base = 0
    if height <= base:
        return None
    return {'render_height': height-base, 'base_offset': base, 'height_basis': reason}


def context_features(source, region, to_local):
    """Read complete polygon rings and mapped trees, retaining omissions and avoiding duplicate multipolygon members."""
    with gzip.open(source, 'rb') as stream:
        root = ET.parse(stream).getroot()
    nodes = {int(e.get('id')): (float(e.get('lon')), float(e.get('lat'))) for e in root.findall('node')}
    ways = {int(e.get('id')): e for e in root.findall('way')}
    consumed, records, omissions = set(), [], []

    def tags(element):
        """Copy OSM tags without changing the retained source snapshot."""
        return {e.get('k'): e.get('v') for e in element.findall('tag')}

    def relevant(properties):
        """Select building, vegetation and water evidence needed by this bounded city composition."""
        return (properties.get('building', 'no') != 'no' or properties.get('building:part', 'no') != 'no'
                or properties.get('natural') in ('wood', 'water', 'tree', 'scrub')
                or properties.get('landuse') in ('forest', 'grass', 'meadow', 'recreation_ground')
                or properties.get('leisure') in ('park', 'garden'))

    def coordinates(element):
        """Return complete source coordinates; missing references are diagnosed instead of closing partial rings."""
        refs = [int(e.get('ref')) for e in element.findall('nd')]
        if any(n not in nodes for n in refs):
            raise ValueError('missing node references')
        return [nodes[n] for n in refs]

    def retain(element, geometry):
        """Clip valid projected geometry to the demo boundary and preserve its source identity and tags."""
        projected = shapely.transform(shapely.make_valid(geometry), to_local.transform, interleaved=False)
        projected = projected.intersection(region)
        if not projected.is_empty:
            records.append({'source_type': element.tag, 'source_id': int(element.get('id')),
                            'source_version': element.get('version'), 'tags': tags(element), 'geometry': projected})

    for element in root.findall('relation'):
        properties = tags(element)
        if properties.get('type') != 'multipolygon' or not relevant(properties):
            continue
        try:
            outer, inner, members = [], [], []
            for member in element.findall('member'):
                if member.get('type') != 'way' or member.get('role', '') not in ('', 'outer', 'inner'):
                    continue
                ref = int(member.get('ref'))
                if ref not in ways:
                    raise ValueError('missing multipolygon member')
                members.append(ref)
                (inner if member.get('role') == 'inner' else outer).append(LineString(coordinates(ways[ref])))
            rings = list(polygonize(outer))
            if not rings or sum(r.length for r in rings) < sum(l.length for l in outer)-.000001:
                raise ValueError('open multipolygon outer ring')
            holes = list(polygonize(inner))
            if sum(r.length for r in holes) < sum(line.length for line in inner)-.000001:
                raise ValueError('open multipolygon inner ring')
            geometry = unary_union(rings).difference(unary_union(holes))
            retain(element, geometry)
            consumed.update(members)
        except ValueError as error:
            omissions.append({'type': 'relation', 'id': element.get('id'), 'reason': str(error)})
    for ref, element in ways.items():
        if ref in consumed or not relevant(tags(element)):
            continue
        try:
            points = coordinates(element)
            if len(points) < 4 or points[0] != points[-1]:
                continue
            retain(element, Polygon(points))
        except ValueError as error:
            omissions.append({'type': 'way', 'id': ref, 'reason': str(error)})
    for element in root.findall('node'):
        if tags(element).get('natural') == 'tree':
            retain(element, Point(nodes[int(element.get('id'))]))
    return records, omissions


def export_context(output, source, region, to_local, to_geo):
    """Export 2D native-clamping inputs; distinguish mapped tree positions from inferred woodland populations."""
    records, omissions = context_features(source, region, to_local)
    building_records = [r for r in records if r['tags'].get('building', 'no') != 'no'
                        or r['tags'].get('building:part', 'no') != 'no']
    parts = unary_union([r['geometry'] for r in building_records if r['tags'].get('building:part', 'no') != 'no'
                         and r['geometry'].area >= .5 and dimensions(r['tags']) is not None])
    buildings, vegetation, excluded_trees, excluded_buildings, conflicts = [], [], [], [], []
    footprints = unary_union([r['geometry'] for r in building_records])
    boundary = json.loads((output/'boundary.geojson').read_text())
    ground = unary_union([shapely.transform(shape(f['geometry']), to_local.transform, interleaved=False)
                          for f in boundary['features']])
    exclusions = ground.buffer(.3).union(footprints)

    def feature(record, geometry, properties):
        """Create one geographic feature with source provenance and a separate set of rendering attributes."""
        return {'type': 'Feature', 'id': f'{record["source_type"]}/{record["source_id"]}',
                'properties': {'source_id': record['source_id'], 'source_type': record['source_type'],
                               'source_version': record['source_version'], 'source_tags': record['tags'], **properties},
                'geometry': mapping(shapely.transform(geometry, to_geo.transform, interleaved=False))}

    for record in building_records:
        geometry = record['geometry']
        if record['tags'].get('building:part', 'no') == 'no':
            geometry = geometry.difference(parts)
        geometry = unary_union(list(compiler.parts(geometry)))
        values = dimensions(record['tags'])
        if geometry.area < .5 or values is None:
            excluded_buildings.append({'type': record['source_type'], 'id': record['source_id'],
                'reason': 'parts replace parent/tiny clipped area' if geometry.area < .5 else 'invalid height/base'})
            continue
        # Short wall segments let map clamping follow the graded surface around the complete footprint.
        geometry = shapely.segmentize(geometry, 3)
        overlap = geometry.intersection(ground).area
        if overlap > 1 and values['base_offset'] == 0:
            conflicts.append({'type': record['source_type'], 'id': record['source_id'], 'area_m2': overlap})
        buildings.append(feature(record, geometry, {**record['tags'], **values}))
    for record in records:
        if record['tags'].get('natural') == 'tree':
            if exclusions.covers(record['geometry']):
                excluded_trees.append(record['source_id'])
            else:
                vegetation.append(feature(record, record['geometry'], {**record['tags'], 'natural': 'tree',
                    'placement_basis': 'mapped point; rendered species and size inferred'}))
        elif record['tags'].get('natural') == 'wood' or record['tags'].get('landuse') == 'forest':
            geometry = record['geometry'].difference(exclusions.buffer(1))
            geometry = unary_union(list(compiler.parts(geometry)))
            if not geometry.is_empty:
                vegetation.append(feature(record, geometry, {**record['tags'], 'natural': 'wood',
                    'placement_basis': 'INFERRED deterministic scatter within mapped woodland'}))
    for name, features in [('buildings', buildings), ('vegetation', vegetation)]:
        compiler.write_json(output/(name+'.geojson'), {'type': 'FeatureCollection', 'features': features})
    audit = {'building_features': len(buildings), 'mapped_trees_exported': sum(
                 f['properties'].get('natural') == 'tree' for f in vegetation),
             'woodland_polygons': sum(f['properties'].get('natural') == 'wood' for f in vegetation),
             'excluded_tree_ids': excluded_trees, 'incomplete_source_polygons': omissions,
             'excluded_buildings': excluded_buildings,
             'building_road_planar_overlaps': conflicts,
             'overlap_policy': 'Planar candidates only; source footprints retained, no inferred setback or pad imposed',
             'building_height_basis': {basis: sum(f['properties']['height_basis'] == basis for f in buildings)
                 for basis in sorted({f['properties']['height_basis'] for f in buildings})},
             'clamping': 'Native map elevation queries, one-meter requested resolution, combined ReadyMap plus grading',
             'footprint_policy': 'Mapped parts replace overlapping parent footprints; no building pad flattening inferred',
             'tree_policy': 'Mapped points outside road/building footprints; inferred species/size assets and woodland scatter'}
    compiler.write_json(output/'city-context-audit.json', audit)
    return audit


def context_xml(asset_root):
    """Compose real building and Prestige vegetation consumers of the final map elevation stack."""
    return f'''<city>
    <OGRFeatures name="city-buildings"><url>buildings.geojson</url>
        <attribution>OpenStreetMap contributors / ODbL; heights may be inferred</attribution></OGRFeatures>
    <OGRFeatures name="city-vegetation"><url>vegetation.geojson</url>
        <attribution>OpenStreetMap contributors / ODbL; tree species and sizes inferred</attribution></OGRFeatures>
    <FeatureModel name="OSM buildings clamped to final ground" features="city-buildings">
        <lighting>true</lighting><max_range>12000</max_range>
        <styles><style type="text/css"><![CDATA[
            default {{
                fill: #b8afa1;
                extrusion-height: feature.properties.render_height;
                extrusion-flatten: true;
                extrusion-roof-style: roof;
                extrusion-wall-gradient: 0.25;
                altitude-clamping: terrain;
                altitude-technique: map;
                altitude-binding: vertex;
                altitude-resolution: 1m;
                altitude-offset: feature.properties.base_offset;
            }}
            roof {{ fill: #747c85; }}
        ]]></style></styles>
    </FeatureModel>
    <prestige:vegetation xmlns:prestige="osgEarthPrestige" name="OSM trees on final graded ground">
        <sources><source name="mapped"><features>city-vegetation</features></source></sources>
        <coverage>
            <rule source="mapped" group="trees" key="natural" value="tree" action="points"/>
            <rule source="mapped" group="trees" key="natural" value="wood" action="include"/>
        </coverage>
        <seed>29</seed><asset_budget_mb>64</asset_budget_mb><profile>global-geodetic</profile>
        <elevation_resolution>1m</elevation_resolution>
        <assets><asset name="broadleaf" near="{asset_root}/broadleaf-near.osg"
                       coarse="{asset_root}/broadleaf-coarse.osg"/></assets>
        <groups><group name="trees" asset="trees" density="2500" cell_level="16" render_cell_level="14"
             max_range="10000" lod_pixels="70" min_scale="0.75" max_scale="1.15">
            <models><model>broadleaf</model></models>
        </group></groups>
    </prestige:vegetation>
    </city>'''


def compose(scene, surface):
    """Add native clamped context to successful road maps; keep base maps available for diagnostic comparison."""
    compiler.configure_scene(scene)
    root = Path(__file__).parent
    output = root/'output'/(scene+'-readymap-'+surface)
    if any((output/name).exists() for name in ('surface-failure.json', 'vertical-failure.json')):
        raise ValueError('The road surface was rejected; resolve it before composing the city demo')
    audit = export_context(output, root/'source'/(scene+'.osm.gz'), compiler.REGION, compiler.TO_LOCAL, compiler.TO_GEO)
    asset_root = (root.parents[2]/'data/procedural2/pbr').resolve().as_posix()
    if not Path(asset_root, 'broadleaf-near.osg').exists():
        raise ValueError('Required shared vegetation assets are missing')
    additions = context_xml(asset_root)
    additions = additions.split('<city>', 1)[1].rsplit('</city>', 1)[0]
    views = json.loads((output/'viewpoints.json').read_text())
    cameras = city_views(scene, root, output)
    for mode in views['modes']:
        for sse in views['terrain_sse']:
            for view in views['views']:
                name = f'{mode}-sse{sse}-{view}.earth'
                text = (output/name).read_text()
                text = text.replace('<options>', '<libraries>osgEarthPrestige</libraries>\n    <options>', 1)
                text = text.replace('    <viewpoints', additions+'\n    <viewpoints', 1)
                (output/('city-'+name)).write_text(text, encoding='utf-8')
            base = (output/f'city-{mode}-sse{sse}-overview.earth').read_text()
            for view, camera in cameras.items():
                lon, lat, heading, pitch, distance, altitude = camera
                selected = (f'<viewpoint name="{view}" long="{lon}" lat="{lat}" height="{altitude}" '
                            f'heading="{heading}" pitch="{pitch}" range="{distance}"/>')
                text = re.sub(r'<viewpoint name="overview"[^>]*/>', selected, base, count=1)
                (output/f'city-{mode}-sse{sse}-{view}.earth').write_text(text, encoding='utf-8')
    compiler.write_json(output/'city-viewpoints.json', {**views, 'views': [*views['views'], *cameras]})
    audit['city_maps'] = (len(views['views'])+len(cameras))*len(views['modes'])*len(views['terrain_sse'])
    compiler.write_json(output/'city-context-audit.json', audit)
    print(json.dumps(audit, indent=2), flush=True)


def city_views(scene, root, output):
    """Choose matching city cameras, sampling the combined elevation for useful street and park targets."""
    terrain = ReadyMap(root/'source'/(scene+'-readymap'), compiler.TO_GEO)
    choices = {'boston-region': {'city-overview': (*compiler.ORIGIN, -25, -52, 2300),
                                'city-park': (-71.0705, 42.3542, 65, -32, 520)},
               'pittsburgh-region': {'city-overview': (*compiler.ORIGIN, -25, -52, 2600),
                                    'city-park': (-80.0067, 40.4484, 150, -32, 600)}}
    checks = json.loads((output/'bridge-terrain-audit.json').read_text())['checks']
    located = [check for check in checks if check.get('worst_position_local_m')]
    if located:
        worst = min(located, key=lambda check: check['minimum_m'])
        lon, lat = compiler.TO_GEO.transform(*worst['worst_position_local_m'])
        choices[scene]['city-bridge-check'] = (lon, lat, -30, -35, 160)
    exported_audit = output/'terrain-export-audit.json'
    if exported_audit.exists():
        grading = json.loads(exported_audit.read_text())
        location = grading.get('worst_location')
        if location and grading.get('vertices_below_overlay', 0):
            choices[scene]['city-grading-check'] = (location['lon'], location['lat'], -40, -55, 60)
    result = {}
    with rasterio.open(output/'grading-overlay.tif') as dataset:
        for name, camera in choices[scene].items():
            lon, lat = camera[:2]
            sample = float(next(dataset.sample([(lon, lat)]))[0])
            if sample == dataset.nodata or not np.isfinite(sample):
                sample = float(terrain(*compiler.TO_LOCAL.transform(lon, lat)))
            result[name] = (*camera, sample+1)
    result['city-street'] = street_camera(output, terrain)
    compiler.write_json(output/'city-cameras.json', result)
    return result


def street_camera(output, terrain):
    """Place a pedestrian camera on a resolved lane approach, outside mapped buildings; fail if none is suitable."""
    buildings = json.loads((output/'buildings.geojson').read_text())['features']
    obstacles = shapely.STRtree([shapely.transform(shape(f['geometry']), compiler.TO_LOCAL.transform,
                                                 interleaved=False) for f in buildings])
    boundary = json.loads((output/'boundary.geojson').read_text())['features']
    ground = unary_union([shapely.transform(shape(f['geometry']), compiler.TO_LOCAL.transform,
                                            interleaved=False) for f in boundary])
    shapely.prepare(ground)
    discovery = json.loads((output/'layout-discovery.json').read_text())['junctions']
    candidates = []
    for item in discovery:
        if item['status'] != 'resolved':
            continue
        layout = json.loads((output/(item['name']+'-layout.json')).read_text())
        for connector in layout['lane_connectors']:
            points = np.asarray(connector['xy_m'])
            tangent = points[1]-points[0]
            tangent /= max(np.linalg.norm(tangent), 1e-9)
            eye = points[0]-tangent*12
            target = points[len(points)//2]
            if not ground.covers(Point(eye)) or len(obstacles.query(Point(eye), predicate='dwithin', distance=2)):
                continue
            if len(obstacles.query(LineString([eye, target]), predicate='intersects')):
                continue
            candidates.append((float(np.linalg.norm(eye)), item['name'], eye, target))
    if not candidates:
        raise ValueError('No resolved approach supplies a building-free street camera')
    _, layout, eye, target = min(candidates, key=lambda item: (item[0], item[1]))
    geographic = np.column_stack(compiler.TO_GEO.transform(*np.array([eye, target]).T))
    with rasterio.open(output/'grading-overlay.tif') as dataset:
        heights = np.array([float(value[0]) for value in dataset.sample(geographic)])
        missing = (heights == dataset.nodata) | ~np.isfinite(heights)
        if missing.any():
            heights[missing] = terrain(*np.array([eye, target])[missing].T)
    delta = target-eye
    dz = heights[1]+1.5-heights[0]-1.7
    camera = (*geographic[1], float(np.degrees(np.arctan2(delta[0], delta[1]))),
              float(np.degrees(np.arctan2(dz, np.linalg.norm(delta)))),
              float(np.linalg.norm([*delta, dz])), float(heights[1]+1.5))
    compiler.write_json(output/'city-street-camera-audit.json', {
        'layout': layout, 'eye_local_m': eye.tolist(), 'target_local_m': target.tolist(),
        'eye_on_pavement': True, 'building_clearance_min_m': 2, 'building_free_sightline': True,
        'eye_height_above_sampled_ground_m': 1.7})
    return camera


def main():
    """Compose selected retained city scenes after their road compilation has completed successfully."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', choices=('boston-region', 'pittsburgh-region'))
    parser.add_argument('--surface', choices=('design', 'fit', 'drape'), default='design')
    args = parser.parse_args()
    compose(args.scene, args.surface)


if __name__ == '__main__':
    main()
