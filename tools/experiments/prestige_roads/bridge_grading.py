"""Connect the Congress Street deck and regular-grid road grading through one explicit elevation profile."""

import argparse
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import shapely
from shapely.geometry import GeometryCollection, LineString, mapping, shape
from scipy.optimize import Bounds, LinearConstraint, minimize, linprog

import bridge_slice as deck
from bridge_slice import Bridge, SOURCE, WAYS
from city_fixtures import ROOT

OUTPUT = ROOT/'output/bridge-grading-boston'
BASE = ROOT/'output/road-grading-boston/after-close.earth'


def digest(path):
    """Identify retained source and profile inputs without mutating them."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save_json(path, value):
    """Publish human-readable evidence after successful preparation or validation."""
    path.write_text(json.dumps(value, indent=2)+'\n', encoding='utf-8')


class SharedBridge(Bridge):
    """Retain source XY geometry while sharing a single station/elevation solution with GradingLayer."""

    def __init__(self):
        """Assemble the two source-connected ground approaches and bridge chain without spatial snapping."""
        super().__init__()
        edges = json.loads((SOURCE/'road-topology-edges.geojson').read_text(encoding='utf-8'))['features']
        sequence = [next(e for e in edges if e['id'] == self.connections[0]['ground_edge'])]
        sequence += [next(e for e in edges if e['properties']['road_id'] == way) for way in WAYS]
        sequence += [next(e for e in edges if e['id'] == self.connections[1]['ground_edge'])]
        for a, b in zip(sequence, sequence[1:]):
            if a['properties']['to_node'] != b['properties']['from_node']:
                raise ValueError('The profile chain must share source nodes and orientation')
        coords = []
        lengths = []
        for edge in sequence:
            line = self.local(edge['geometry'])
            points = list(line.coords)
            if coords and np.linalg.norm(np.array(coords[-1])-points[0]) > 1e-5:
                raise ValueError('Connected source edges have unequal endpoint XY')
            coords.extend(points if not coords else points[1:])
            lengths.append(line.length)
        self.alignment = LineString(coords)
        self.bridge_start = lengths[0]
        self.bridge_end = self.alignment.length-lengths[-1]
        self.ground_way_ids = [sequence[0]['properties']['road_id'], sequence[-1]['properties']['road_id']]
        # Preserve every original bend as well as regularly spaced profile samples.
        nodes = np.r_[0, np.cumsum(np.linalg.norm(np.diff(np.array(coords), axis=0), axis=1))]
        self.solved_stations = np.unique(np.r_[nodes, np.arange(0, self.alignment.length, 2), self.alignment.length])
        self.solved_xy = shapely.get_coordinates(shapely.line_interpolate_point(self.alignment, self.solved_stations))
        self.solved_heights = None
        self.base_samples = None
        self.sequence = sequence

    def station(self, xy):
        """Locate nearest stations on the complete source alignment, including its actual curved approaches."""
        values = np.asarray(xy)
        return shapely.line_locate_point(self.alignment, shapely.points(values.reshape(-1, 2))).reshape(values.shape[:-1])

    def height(self, x, y):
        """Interpolate the same unsmoothed centerline elevations exported to the grading layer."""
        s = self.station(np.stack(np.broadcast_arrays(x, y), axis=-1))
        return np.interp(s, self.solved_stations, self.solved_heights), np.zeros(np.shape(x))

    def thickness(self, xy):
        """Taper the separate illustrative slab within four meters of each ground interface."""
        s = self.station(xy)
        u = np.clip(np.minimum(s-self.bridge_start, self.bridge_end-s)/4.0, 0, 1)
        return .75*u*u*(3-2*u)

    def footprints(self):
        """Use compiled ground-road masks and bounded landing support pads, leaving the central span ungraded."""
        profiles = json.loads((ROOT/'output/road-grading-boston/road-profiles.geojson').read_text(encoding='utf-8'))['features']
        polygons = []
        for feature in profiles:
            p = feature['properties']
            if p['source_way_ids'] not in self.ground_way_ids or p['grading_junction']:
                continue
            geometry = self.local(feature['geometry'])
            polygons += [part for part in geometry.geoms if part.geom_type in ('Polygon', 'MultiPolygon')]
        if not polygons:
            raise ValueError('No authoritative approach grading masks')
        for lo, hi in ((self.bridge_start-6, self.bridge_start+8), (self.bridge_end-8, self.bridge_end+6)):
            from shapely.ops import substring
            # Support the terrain triangles surrounding the cap, not just samples directly under pavement.
            # This is an inferred grading footprint; authoritative pavement/paint geometry stays unchanged.
            apron = substring(self.alignment, lo, hi).buffer(10, cap_style='flat')
            polygons.append(apron)
        return shapely.make_valid(shapely.union_all(polygons))


def base_scene():
    """Rebase the existing Boston grading stack without changing its layer policies or source data."""
    tree = ET.fromstring(BASE.read_text(encoding='utf-8'))
    for element in tree.iter():
        if element.tag == 'url' and element.text and '://' not in element.text:
            element.text = (BASE.parent/element.text).resolve().as_posix()
        for attr in ('color', 'packed', 'normal'):
            if attr in element.attrib:
                element.set(attr, (BASE.parent/element.get(attr)).resolve().as_posix())
    ET.register_namespace('prestige', 'osgEarthPrestige')
    return tree


def prepare(bridge):
    """Publish complete sampling inputs from the existing graded stack, retaining source hashes for the solve."""
    OUTPUT.mkdir(parents=True, exist_ok=True)
    lon, lat = bridge.to_geo.transform(*bridge.solved_xy.T)
    np.savetxt(OUTPUT/'profile-query.csv', np.column_stack([np.arange(len(lon)), lon, lat]), delimiter=',',
        header='id,longitude,latitude', comments='', fmt='%.16g')
    (OUTPUT/'base.earth').write_text(ET.tostring(base_scene(), encoding='unicode'), encoding='utf-8')
    save_json(OUTPUT/'inputs.json', {'source_sha256': {str(p.resolve()): digest(p) for p in
        [SOURCE/'structures.geojson', SOURCE/'road-topology-edges.geojson',
         ROOT/'output/road-grading-boston/road-profiles.geojson', BASE]},
        'connection_evidence': bridge.connections, 'profile_query_sha256': digest(OUTPUT/'profile-query.csv'),
        'bridge_start_station_m': bridge.bridge_start, 'bridge_end_station_m': bridge.bridge_end})
    print('Prepared', len(lon), 'profile points; bridge stations', bridge.bridge_start, bridge.bridge_end, flush=True)


def solve(bridge):
    """Fit an explicitly inferred grade-limited profile; sparse grid and deck subsequently consume these same samples."""
    inputs = json.loads((OUTPUT/'inputs.json').read_text(encoding='utf-8'))
    for path, expected in inputs['source_sha256'].items():
        if digest(Path(path)) != expected:
            raise ValueError('Source changed after preparing profile samples: '+path)
    if digest(OUTPUT/'profile-query.csv') != inputs['profile_query_sha256']:
        raise ValueError('Profile query changed after preparing the sample manifest')
    sampled = np.loadtxt(OUTPUT/'profile-samples.csv', delimiter=',', skiprows=1)
    query = np.loadtxt(OUTPUT/'profile-query.csv', delimiter=',', skiprows=1)
    if sampled.shape != (len(query), 5) or np.max(abs(sampled[:, :3]-query)) > 1e-10:
        raise ValueError('Profile terrain samples are incomplete or do not match the prepared query')
    if not np.isfinite(sampled).all():
        raise ValueError('Nonfinite profile input')
    s = bridge.solved_stations
    terrain = sampled[:, 4]
    bridge.base_samples = sampled
    # The bridge-containing DEM is only a rendering obstacle, never a mapped water surface or design clearance.
    lower, upper = np.full(len(s), -np.inf), np.full(len(s), np.inf)
    inside = (s >= bridge.bridge_start+8) & (s <= bridge.bridge_end-8)
    lower[inside] = terrain[inside]+1.25
    lower[[0, -1]], upper[[0, -1]] = terrain[[0, -1]], terrain[[0, -1]]
    d1 = np.diff(np.eye(len(s)), axis=0)/np.diff(s)[:, None]
    d2 = np.diff(d1, axis=0)
    target = terrain.copy()
    target[inside] += 1.25
    matrix = np.eye(len(s))+5000*(d2.T@d2)

    def objective(z):
        """Prefer the observed road profile while penalizing abrupt grade changes."""
        return float((z-target)@(z-target)+5000*np.sum((d2@z)**2))

    def gradient(z):
        """Provide the exact derivative for the bounded quadratic profile fit."""
        return 2*(matrix@z-target)

    grade_limit = .085
    feasible = linprog(np.zeros(len(s)), A_ub=np.r_[d1, -d1], b_ub=np.full(2*len(d1), grade_limit),
        bounds=list(zip(lower, upper)), method='highs')
    if not feasible.success:
        raise ValueError('Infeasible declared profile grade/obstacle constraints: '+feasible.message)
    result = minimize(objective, feasible.x, jac=gradient, method='SLSQP',
        bounds=Bounds(lower, upper), constraints=LinearConstraint(d1, -grade_limit, grade_limit),
        options={'maxiter': 1000, 'ftol': 1e-9})
    if not result.success or np.max(abs(d1@result.x)) > grade_limit+1e-6 or np.min(result.x-lower) < -1e-6:
        raise ValueError('No accepted grade-limited profile: '+result.message)
    bridge.solved_heights = result.x
    return {'max_grade_percent': float(np.max(abs(d1@result.x))*100),
        'obstacle_margin_m': float(np.min((result.x-terrain)[inside])),
        'sample_source': 'Existing ReadyMap plus terrain-sampled road GradingLayer, ellipsoid meters',
        'inference': '8.5-percent illustrative grade cap and 1.25m interior DEM obstacle margin; not surveyed clearance'}


def scene(bridge, view, max_lod=22, model=True):
    """Place ordinary terrain grading beneath a separate deck with shared materials and explicit receiver ownership."""
    tree = base_scene()
    tree.set('name', 'Boston bridge / shared absolute profile / '+view)
    for feature in tree.findall('OGRFeatures'):
        name = feature.get('name')
        material = 'asphalt' if name == 'compiled-asphalt' else name.removeprefix('paint-')
        if material in ('asphalt', 'white', 'yellow'):
            feature.find('url').text = (deck.OUTPUT/f'ground-{material}.geojson').resolve().as_posix()
    source = ET.SubElement(tree, 'OGRFeatures', name='Shared bridge approach profile')
    ET.SubElement(source, 'url').text = 'grading-profile.geojson'
    grading = ET.SubElement(tree, '{osgEarthPrestige}grading', name='Shared profile approach grading',
        features='Shared bridge approach profile')
    for key, value in {'road_footprints':'true', 'profile_elevation':'absolute', 'profile_transition':10,
        'min_level':16, 'max_data_level':16, 'tile_size':257, 'line_width':80, 'max_line_width':80,
        'buffer_width':5, 'max_buffer_width':5, 'profile_offset':'[profile_offset_m]'}.items():
        ET.SubElement(grading, key).text = str(value)
    if model:
        layer = ET.SubElement(tree, 'Model', name='Bridge / same solved profile', lighting='true')
        ET.SubElement(layer, 'url').text = 'roads.gltf'
        ET.SubElement(layer, 'location', long=str(bridge.origin[0]), lat=str(bridge.origin[1]), alt='0', mode='absolute')
    terrain = tree.find('options/terrain')
    terrain.set('max_lod', str(max_lod))
    terrain.set('morph_terrain', 'true')
    terrain.set('min_tile_range_factor', '7')
    terrain.set('screen_space_error', '8')
    choices = {'overview':(.5,330,-45,20), 'west-landing':(.0,35,-15,117),
        'east-landing':(1.,35,-15,-63), 'deck':(.5,25,-55,27)}
    fraction, distance, pitch, heading = choices[view]
    station = bridge.bridge_start+fraction*(bridge.bridge_end-bridge.bridge_start)
    xy = np.asarray(bridge.alignment.interpolate(station).coords)[0]
    lon, lat = bridge.to_geo.transform(*xy)
    viewpoints = tree.find('viewpoints'); viewpoints.clear(); viewpoints.set('home','0')
    ET.SubElement(viewpoints,'viewpoint',name=view,long=str(lon),lat=str(lat),
        height=str(float(bridge.height(*xy)[0])),heading=str(heading),pitch=str(pitch),range=str(distance))
    return ET.tostring(tree, encoding='unicode')


def build(bridge):
    """Export the shared profile, bounded regular-grid inputs, deck mesh, scenes, and provenance together."""
    inference = solve(bridge)
    lon, lat = bridge.to_geo.transform(*bridge.solved_xy.T)
    line = LineString(np.column_stack([lon, lat, bridge.solved_heights]))
    mask = bridge.footprints()
    props = {'profile_id':'boston/congress/shared-v1','source_way_ids':[e['properties']['road_id'] for e in bridge.sequence],
        'elevation_datum':'WGS84 ellipsoid meters','height_evidence':'inferred','ground_receiver':'terrain/boston',
        'structure_receiver':'bridge/congress','profile_transition_m':10, 'attachment_apron_m':8,
        'landing_grading_half_width_m':10,
        'interpolation':'Nearest XY centerline station, piecewise linear Z', **inference}
    save_json(OUTPUT/'solved-profile.geojson', {'type':'FeatureCollection','features':[
        {'type':'Feature','id':1,'properties':props,'geometry':mapping(line)}]})
    # The deck and terrain must not occupy the same driving-surface plane beneath the short overlap.
    # Keep profile Z identical and record the terrain receiver's small separation as its own policy.
    inputs = []
    for identifier, receiver_mask, offset in ((1, mask.difference(bridge.pavement), 0.0),
        (2, mask.intersection(bridge.pavement), -.03)):
        receiver_geo = shapely.transform(receiver_mask, bridge.to_geo.transform, interleaved=False)
        inputs.append({'type':'Feature','id':identifier,'properties':{**props,'profile_offset_m':offset,
            'offset_basis':'Numerical terrain/deck separation beneath attachment overlap' if offset else 'Ground approach'},
            'geometry':mapping(GeometryCollection([line,receiver_geo]))})
    save_json(OUTPUT/'grading-profile.geojson', {'type':'FeatureCollection','features':inputs})
    shutil.copytree(deck.OUTPUT/'materials', OUTPUT/'materials', dirs_exist_ok=True)
    counts, areas, error = deck.export_deck(bridge, OUTPUT)
    for view in ('overview','west-landing','east-landing','deck'):
        (OUTPUT/f'{view}.earth').write_text(scene(bridge,view),encoding='utf-8')
    for lod in (18,19,20,22):
        for side in ('west-landing','east-landing'):
            (OUTPUT/f'{side}-lod{lod}.earth').write_text(scene(bridge,side,lod),encoding='utf-8')
    (OUTPUT/'terrain-only.earth').write_text(scene(bridge,'overview',model=False),encoding='utf-8')
    np.savetxt(OUTPUT/'solved-profile.csv',np.column_stack([bridge.solved_stations,bridge.solved_heights,
        bridge.base_samples[:,4]]),delimiter=',',header='station_m,ellipsoid_m,preceding_elevation_m',comments='')
    save_json(OUTPUT/'manifest.json', {'source':json.loads((OUTPUT/'inputs.json').read_text(encoding='utf-8')),
        'profile_sha256':digest(OUTPUT/'solved-profile.geojson'), 'grading_sha256':digest(OUTPUT/'grading-profile.geojson'),
        'terrain_samples_sha256':digest(OUTPUT/'profile-samples.csv'),'generator_sha256':digest(Path(__file__)),
        'height_policy':props,'native_lod':16,'tile_size':257,'triangles':counts,'material_areas_m2':areas,
        'terrain_receiver_offset_m':-.03, 'offset_scope':'Landing support overlapping the bridge pavement only',
        'footprint_error_m2':error,'grading_mask_area_m2':mask.area,
        'limitations':['Temporary inferred heights, not surveyed engineering or water clearance',
            'Single unbranched profile; elevated junctions and bridge families remain separate work',
            'Regular-grid sampling approximates exact deck interface; runtime attachment audit required']})
    print(json.dumps(inference),counts,'footprint error',error,flush=True)


def main():
    """Prepare sampling or build the paired outputs; never overwrite the original bridge prototype."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('prepare','build'))
    action=parser.parse_args().action
    bridge=SharedBridge()
    marker = OUTPUT/'build-in-progress.json'
    if action == 'build':
        save_json(marker, {'status':'incomplete'})
    (prepare if action=='prepare' else build)(bridge)
    if action == 'build':
        marker.unlink()


if __name__=='__main__':
    main()
