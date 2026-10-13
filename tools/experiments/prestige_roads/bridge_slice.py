"""Build one source-connected Congress Street roadway deck on the unchanged Boston terrain."""

import argparse
import hashlib
import importlib
import json
import xml.etree.ElementTree as ET

import numpy as np
from pyproj import Transformer
from scipy.interpolate import RegularGridInterpolator
from scipy.optimize import Bounds, LinearConstraint, minimize
import shapely
from shapely.geometry import Polygon, mapping, shape

from bridge_materials import create_materials
from city_fixtures import ROOT
from surface_splatting import camera_terrain


WAYS = ('85890998', '822604874', '822604875')
SOURCE = ROOT/'output/render-fixture-boston'
OUTPUT = ROOT/'output/bridge-boston-congress'
compiler = importlib.import_module('compile')


def write_json(path, value):
    """Publish readable UTF-8 evidence or asset metadata, propagating filesystem errors."""
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')


class Bridge:
    """Own immutable source polygons and a local station frame for one unbranched bridge chain."""

    def __init__(self):
        """Resolve source-node continuity and reject gaps, branching, or absent material inputs."""
        self.manifest = json.loads((SOURCE/'manifest.json').read_text(encoding='utf-8'))
        self.to_local = Transformer.from_crs(4326, self.manifest['metric_crs'], always_xy=True)
        self.to_geo = Transformer.from_crs(self.manifest['metric_crs'], 4326, always_xy=True)
        self.origin = self.manifest['origin']
        self.to_ecef = Transformer.from_crs(4979, 4978, always_xy=True)
        lon, lat = np.deg2rad(self.origin)
        self.enu = np.array([[-np.sin(lon), np.cos(lon), 0],
            [-np.sin(lat)*np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)],
            [np.cos(lat)*np.cos(lon), np.cos(lat)*np.sin(lon), np.sin(lat)]])
        self.anchor = np.array(self.to_ecef.transform(*self.origin, 0))
        structures = json.loads((SOURCE/'structures.geojson').read_text(encoding='utf-8'))['features']
        self.features = [f for f in structures if f['properties']['road_id'] in WAYS]
        self.materials = {m: shapely.union_all([self.local(f['geometry']) for f in self.features
            if f['properties']['material'] == m]) for m in ('asphalt', 'white', 'yellow')}
        self.pavement = self.materials['asphalt']
        if self.pavement.geom_type != 'Polygon' or not self.pavement.is_valid:
            raise ValueError('The first slice requires a connected valid roadway polygon')
        edges = json.loads((SOURCE/'road-topology-edges.geojson').read_text(encoding='utf-8'))['features']
        chain = [next(e for e in edges if e['properties']['road_id'] == way) for way in WAYS]
        for a, b in zip(chain, chain[1:]):
            if a['properties']['to_node'] != b['properties']['from_node']:
                raise ValueError('The selected bridge chain is not source-connected')
        self.connections = []
        for entry, key in ((chain[0], 'from_node'), (chain[-1], 'to_node')):
            node = entry['properties'][key]
            adjoining = [e for e in edges if e['properties']['road_id'] not in WAYS and
                         node in (e['properties']['from_node'], e['properties']['to_node'])]
            if len(adjoining) != 1 or adjoining[0]['properties']['bridge'] or adjoining[0]['properties']['tunnel']:
                raise ValueError('Expected one unambiguous ground connection at each landing')
            self.connections.append({'node': node, 'bridge_edge': entry['id'], 'ground_edge': adjoining[0]['id']})
        coords = [chain[0]['geometry']['coordinates'][0], chain[-1]['geometry']['coordinates'][-1]]
        self.a, self.b = np.array(self.to_local.transform(*np.asarray(coords).T)).T
        self.length = float(np.linalg.norm(self.b-self.a))
        self.forward = (self.b-self.a)/self.length
        self.left = np.array([-self.forward[1], self.forward[0]])
        entry = np.asarray(self.local(chain[0]['geometry']).coords)
        exit_points = np.asarray(self.local(chain[-1]['geometry']).coords)
        self.entry_tangent = (entry[1]-entry[0])/np.linalg.norm(entry[1]-entry[0])
        self.exit_tangent = (exit_points[-1]-exit_points[-2])/np.linalg.norm(exit_points[-1]-exit_points[-2])
        self.terrain = camera_terrain('boston', self.manifest['cameras'])
        self.stations = np.linspace(-1, self.length+1, int(self.length/2)+3)
        self.offsets = np.linspace(-5, 5, 21)
        self.rendered = None
        self.landing_corrections = []
        self.profile_stations = None
        self.profile_heights = None

    def local(self, geometry):
        """Project a source geometry in doubles without quantizing its authoritative boundary."""
        return shapely.transform(shape(geometry), self.to_local.transform, interleaved=False)

    def st(self, xy):
        """Convert XY positions to station/lateral coordinates in the bridge's nearly straight frame."""
        delta = np.asarray(xy)-self.a
        return delta @ self.forward, delta @ self.left

    def xy(self, s, t):
        """Convert broadcastable station/offset arrays to local metric XY coordinates."""
        s, t = np.broadcast_arrays(s, t)
        return self.a+s[..., None]*self.forward+t[..., None]*self.left

    def ground(self, xy):
        """Read the retained runtime terrain snapshot; fail outside its bounded sample rectangle."""
        if self.rendered is None:
            lon, lat = self.to_geo.transform(*np.asarray(xy).T)
            return self.terrain.geographic(lon, lat)
        s, t = self.st(xy)
        z = self.rendered(np.column_stack([s, t]))
        for end, offsets, corrections in self.landing_corrections:
            weight = np.clip(1-abs(s-end)/4, 0, 1)
            weight = weight*weight*(3-2*weight)
            z += np.interp(t, offsets, corrections)*weight
        return z

    def attach_samples(self):
        """Validate probe coordinates and establish a read-only interpolator of actual terrain intersections."""
        query = np.loadtxt(OUTPUT/'terrain-query.csv', delimiter=',', skiprows=1)
        samples = np.loadtxt(OUTPUT/'terrain-sampled.csv', delimiter=',', skiprows=1)
        levels = np.loadtxt(OUTPUT/'terrain-sampled.csv.lod.csv', delimiter=',', skiprows=1)
        if len(levels) != len(query) or not np.all(levels[:, 1] == 19):
            raise ValueError('The reference terrain snapshot must resolve every query at LOD 19')
        if samples.shape != (len(query), 4) or not np.isfinite(samples).all():
            raise ValueError('Terrain snapshot is incomplete or contains missing heights')
        if np.max(abs(samples[:, :3]-query)) > 1e-10:
            raise ValueError('Terrain snapshot does not match the prepared geographic query')
        self.rendered = RegularGridInterpolator((self.stations, self.offsets),
            samples[:, 3].reshape(len(self.stations), len(self.offsets)), bounds_error=True)
        landing_path = OUTPUT/'landing-sampled.csv'
        if landing_path.is_file():
            landing = np.loadtxt(landing_path, delimiter=',', skiprows=1)
            landing_levels = np.loadtxt(OUTPUT/'landing-sampled.csv.lod.csv', delimiter=',', skiprows=1)
            if len(landing_levels) != len(landing) or not np.all(landing_levels[:, 1] == 19):
                raise ValueError('Landing refinement must use the reference terrain LOD')
            xy = np.column_stack(self.to_local.transform(landing[:, 1], landing[:, 2]))
            s0, t0 = self.st(xy)
            delta = landing[:, 3]-self.rendered(np.column_stack([s0, t0]))
            for end in (0, self.length):
                indices = np.flatnonzero(abs(s0-end) < .1)
                order = indices[np.argsort(t0[indices])]
                if len(order) < 3:
                    raise ValueError('Landing refinement lacks a complete cross-section')
                self.landing_corrections.append((end, t0[order], delta[order]))
        s = np.linspace(0, self.length, 161)
        ground = self.ground(self.xy(s, 0))
        u = np.clip(np.minimum(s, self.length-s)/40, 0, 1)
        weight = u*u*(3-2*u)
        across = np.max(np.stack([self.ground(self.xy(s, t)) for t in (-3.7, 0, 3.7)]), axis=0)
        lower = ground+(across-ground+.75)*weight+2*np.sin(np.pi*s/self.length)**2
        lower[[0, -1]] = ground[[0, -1]]
        upper = np.full(len(s), np.inf)
        upper[[0, -1]] = lower[[0, -1]]
        difference = np.diff(np.eye(len(s)), axis=0)
        curvature = np.diff(np.eye(len(s)), n=2, axis=0)
        matrix = curvature.T @ curvature*5000+np.eye(len(s))

        def objective(z):
            """Penalize profile bends and unnecessary lift above the explicit clearance lower bound."""
            delta = z-lower
            bend = curvature @ z
            return float(delta @ delta+5000*(bend @ bend))

        def gradient(z):
            """Supply the exact quadratic gradient to the bounded profile solve."""
            return 2*(matrix @ z-lower)

        grade = .15
        result = minimize(objective, lower, jac=gradient, method='SLSQP',
            bounds=Bounds(lower, upper), constraints=LinearConstraint(difference, -grade*np.diff(s), grade*np.diff(s)),
            options={'maxiter': 600, 'ftol': 1e-9})
        if not result.success or np.min(result.x-lower) < -1e-6:
            raise ValueError('No accepted grade-constrained profile: '+result.message)
        self.profile_stations, self.profile_heights = s, result.x

    def profile_station(self, xy):
        """Align stations to source end caps over four meters without altering the centerline profile."""
        xy = np.asarray(xy)
        s, _ = self.st(xy)
        entry = (xy-self.a) @ self.entry_tangent / (self.forward @ self.entry_tangent)
        exit_s = self.length-(self.b-xy) @ self.exit_tangent / (self.forward @ self.exit_tangent)
        for end, aligned in ((0, entry), (self.length, exit_s)):
            weight = np.clip(1-abs(s-end)/4, 0, 1)
            weight = weight*weight*(3-2*weight)
            s = s+(aligned-s)*weight
        return s

    def height(self, x, y):
        """Apply a constrained center profile and retain the terrain cross-section at both landing boundaries."""
        xy = np.column_stack([np.ravel(x), np.ravel(y)])
        s = self.profile_station(xy)
        span = np.interp(s, self.profile_stations, self.profile_heights)
        blend = np.clip(np.minimum(s, self.length-s)/40, 0, 1)
        blend = blend*blend*(3-2*blend)
        center = self.ground(self.xy(np.clip(s, -1, self.length+1), 0))
        z = span+(self.ground(xy)-center)*(1-blend)
        return z.reshape(np.shape(x)), np.zeros(np.shape(x))

    def thickness(self, xy):
        """Taper the illustrative 0.75-meter slab to zero at its terrain interfaces, without changing terrain."""
        s = self.profile_station(xy)
        u = np.clip(np.minimum(s, self.length-s)/40, 0, 1)
        return .75*u*u*(3-2*u)


class DeckGLTF(compiler.GLTF):
    """Write a bridge-only model; terrain remains osgEarth's ordinary grid and is never exported as a mesh."""

    def __init__(self, bridge, output=OUTPUT):
        """Share generated material maps with terrain; retain the caller-owned destination for model publication."""
        super().__init__(output)
        self.bridge = bridge
        self.doc['images'], self.doc['textures'], self.doc['materials'] = [], [], []
        for name in ('asphalt', 'concrete', 'white', 'yellow'):
            color = len(self.doc['images'])
            for suffix in ('color', 'orm'):
                index = len(self.doc['images'])
                self.doc['images'].append({'uri': f'materials/{name}-{suffix}.png'})
                self.doc['textures'].append({'sampler': 0, 'source': index})
            self.doc['materials'].append({'name': name, 'doubleSided': False, 'pbrMetallicRoughness': {
                'baseColorFactor': [1, 1, 1, 1], 'baseColorTexture': {'index': color},
                'metallicRoughnessTexture': {'index': color+1}, 'roughnessFactor': 1, 'metallicFactor': 1},
                'occlusionTexture': {'index': color+1}})

    def add(self, name, triangles, material, smooth=False):
        """Preserve exact ellipsoid elevations and use the terrain material's anchored ENU texture phase."""
        if not len(triangles):
            return 0
        xyz = np.asarray(triangles).reshape(-1, 3)
        lon, lat = self.bridge.to_geo.transform(xyz[:, 0], xyz[:, 1])
        world = np.column_stack(self.bridge.to_ecef.transform(lon, lat, xyz[:, 2]))
        enu = (world-self.bridge.anchor) @ self.bridge.enu.T
        positions = np.asarray(enu[:, [0, 2, 1]]*[1, 1, -1], dtype='<f4')
        # glTF image rows are top-first; the terrain PNG loader uses bottom-first storage.
        uv = enu[:, :2]*[1, -1]
        if material == 1:
            uv = np.column_stack([xyz[:, 0], xyz[:, 2]])
        triangles = positions.astype(float).reshape(-1, 3, 3)
        area2 = np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0]), axis=1)
        keep = np.repeat(area2 >= 1e-10, 3)
        positions, uv = positions[keep], uv[keep]
        if not len(positions):
            return 0
        normals = compiler.mesh_normals(positions, smooth)
        attributes = {'POSITION': self.accessor(positions, 'VEC3'), 'NORMAL': self.accessor(normals, 'VEC3'),
                      'TEXCOORD_0': self.accessor(uv, 'VEC2')}
        self.doc['meshes'].append({'name': name, 'primitives': [{'attributes': attributes, 'material': material}]})
        self.doc['nodes'].append({'name': name, 'mesh': len(self.doc['meshes'])-1})
        self.doc['scenes'][0]['nodes'].append(len(self.doc['nodes'])-1)
        return len(positions)//3


def scene(bridge, view, model=True, dynamic=False, cutoff=None):
    """Derive a demo from the tested ground renderer, using the same material files on ground and deck."""
    text = (SOURCE/'marking-comparison/mask-intersection-8.earth').read_text(encoding='utf-8')
    text = text.replace('../render-contract/', '../render-fixture-boston/render-contract/')
    tree = ET.fromstring(text)
    for layer in tree:
        if layer.tag == '{osgEarthPrestige}surface_splatting':
            name = 'asphalt' if layer.get('name') == 'Prestige asphalt' else layer.get('name').split()[-1]
            material = layer.find('material')
            material.attrib.clear()
            material.attrib.update(color=f'materials/{name}-color.png', packed=f'materials/{name}-orm.png', layout='orm')
    terrain = tree.find('options/terrain')
    terrain.set('max_lod', str(cutoff if cutoff is not None else 22 if dynamic else 19))
    terrain.set('screen_space_error', '8' if dynamic or cutoff is not None else '.5')
    terrain.set('min_tile_range_factor', '7' if dynamic or cutoff is not None else '40')
    terrain.set('morph_terrain', 'true' if dynamic else 'false')
    terrain.set('morph_imagery', 'true' if dynamic else 'false')
    tree.set('name', f'Congress Street / inferred roadway deck / {view}')
    if model:
        for feature_layer in tree.findall('OGRFeatures'):
            name = feature_layer.get('name')
            material = 'asphalt' if name == 'compiled-asphalt' else name.removeprefix('paint-')
            feature_layer.find('url').text = f'ground-{material}.geojson'
        layer = ET.SubElement(tree, 'Model', name='Congress Street: inferred closed bridge', lighting='true')
        ET.SubElement(layer, 'url').text = 'roads.gltf'
        ET.SubElement(layer, 'location', long=str(bridge.origin[0]), lat=str(bridge.origin[1]), alt='0', mode='absolute')
    views = {'overview': (.5, 330, -45, 20), 'west-landing': (.02, 28, -20, 117),
             'east-landing': (.98, 28, -20, -63), 'deck': (.5, 18, -65, 27),
             'underside': (.5, 16, 1, 27)}
    fraction, distance, pitch, heading = views[view]
    xy = bridge.xy(fraction*bridge.length, 0)
    lon, lat = bridge.to_geo.transform(*xy)
    height = float(bridge.ground(xy[None, :])[0])+.1
    if model:
        height = float(bridge.height(np.array([xy[0]]), np.array([xy[1]]))[0][0])
        if view == 'underside':
            height -= 1.0
    if cutoff is not None:
        camera = bridge.manifest['cameras']['intersection']
        lon, lat = camera['longitude'], camera['latitude']
        height = float(bridge.terrain.geographic(lon, lat))+.1
        distance, pitch, heading = 400, -90, 0
    viewpoints = tree.find('viewpoints')
    viewpoints.clear()
    viewpoints.set('home', '0')
    ET.SubElement(viewpoints, 'viewpoint', name=view, long=str(lon), lat=str(lat), height=str(height),
                  heading=str(heading), pitch=str(pitch), range=str(distance))
    ET.register_namespace('prestige', 'osgEarthPrestige')
    return ET.tostring(tree, encoding='unicode')


def prepare(bridge):
    """Write bounded terrain query points and a fixed-LOD scene; require a runtime snapshot before building geometry."""
    OUTPUT.mkdir(parents=True, exist_ok=True)
    create_materials(OUTPUT/'materials')
    s, t = np.meshgrid(bridge.stations, bridge.offsets, indexing='ij')
    xy = bridge.xy(s.ravel(), t.ravel())
    lon, lat = bridge.to_geo.transform(xy[:, 0], xy[:, 1])
    query = np.column_stack([np.arange(len(xy)), lon, lat])
    np.savetxt(OUTPUT/'terrain-query.csv', query, delimiter=',', header='id,longitude,latitude', comments='', fmt='%.14g')
    (OUTPUT/'terrain-only.earth').write_text(scene(bridge, 'overview', model=False), encoding='utf-8')
    for lod in (14, 15):
        (OUTPUT/f'cutoff-{lod}.earth').write_text(scene(bridge, 'overview', model=False, cutoff=lod), encoding='utf-8')
    write_json(OUTPUT/'source-selection.json', {'ways': list(WAYS), 'connections': bridge.connections,
        'span_m': bridge.length, 'pavement_area_m2': bridge.pavement.area,
        'source_features': [f['properties'] for f in bridge.features if f['properties']['material'] == 'asphalt'],
        'source_hashes': {name: hashlib.sha256((SOURCE/name).read_bytes()).hexdigest() for name in
                          ('structures.geojson', 'road-topology-edges.geojson', 'manifest.json')}})
    print('Prepared', len(query), 'terrain points; span', bridge.length, flush=True)


def prepare_deck_surface(bridge, max_edge=1):
    """Triangulate the authoritative slab once; structural planning and final materials share these exact triangles."""
    deck = compiler.triangulate(bridge.pavement, bridge.height, max_edge=max_edge)
    # Boolean joins can leave roundoff-size slivers with effectively coincident vertices. Discard these
    # before classifying slab edges; the unchanged area-conservation guard below still protects coverage.
    ab, ac = deck[:, 1, :2]-deck[:, 0, :2], deck[:, 2, :2]-deck[:, 0, :2]
    deck = deck[np.abs(ab[:, 0]*ac[:, 1]-ab[:, 1]*ac[:, 0]) > 1e-12]
    return deck


def export_deck(bridge, output_folder=OUTPUT, writer=None, max_edge=1, retain_surface=False, prepared_deck=None):
    """Export a slab, optionally reusing caller-owned triangles already used to plan its attached structure."""
    deck = prepare_deck_surface(bridge, max_edge) if prepared_deck is None else prepared_deck
    paint = {m: bridge.materials[m].intersection(bridge.pavement) for m in ('white', 'yellow')}
    if paint['white'].intersection(paint['yellow']).area > 1e-7:
        raise ValueError('Ambiguous white/yellow ownership on bridge deck')
    output = {m: [] for m in ('asphalt', 'white', 'yellow')}
    # Split the actual solved triangles. Every material inherits the same affine height, without coplanar overlays.
    for tri in deck:
        footprint = Polygon(tri[:, :2])
        origin = tri[0]
        gradient = np.linalg.solve(tri[1:, :2]-origin[:2], tri[1:, 2]-origin[2])

        def planar_height(x, y):
            """Interpolate this immutable deck triangle; paint partition vertices cannot introduce height disagreement."""
            return origin[2]+(x-origin[0])*gradient[0]+(y-origin[1])*gradient[1], np.zeros(np.shape(x))

        remainder = footprint
        for material in ('white', 'yellow'):
            if paint[material].intersects(footprint):
                piece = footprint.intersection(paint[material])
                output[material].extend(compiler.triangulate(piece, planar_height))
                remainder = remainder.difference(piece)
        output['asphalt'].extend(compiler.triangulate(remainder, planar_height))
    gltf = writer if writer is not None else DeckGLTF(bridge, output_folder)
    counts, areas = {}, {}
    for material, index in (('asphalt', 0), ('white', 2), ('yellow', 3)):
        tris = np.asarray(output[material]).reshape(-1, 3, 3)
        counts[material] = gltf.add('Bridge '+material, tris, index, smooth=True)
        areas[material] = float(shapely.area(shapely.polygons(tris[:, :, :2])).sum()) if len(tris) else 0
    underside = deck[:, [0, 2, 1]].copy()
    underside[:, :, 2] -= bridge.thickness(underside[:, :, :2].reshape(-1, 2)).reshape(-1, 3)
    counts['underside'] = gltf.add('Bridge underside', underside, 1)
    walls = []
    edges = {}
    for triangle in deck:
        for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
            key = tuple(sorted((tuple(np.round(a, 8)), tuple(np.round(b, 8)))))
            edges.setdefault(key, []).append((a, b))
    for pairs in edges.values():
        if len(pairs) > 2:
            raise ValueError('Nonmanifold slab edge')
        if len(pairs) != 1:
            continue
        a, b = pairs[0]
        segments = [(a, b)]
        if hasattr(bridge, 'fascia_openings') and not bridge.fascia_openings.is_empty:
            edge = LineString([a[:2], b[:2]])
            segments = []
            for part in shapely.get_parts(edge.difference(bridge.fascia_openings)):
                if part.geom_type != 'LineString' or part.length < 1e-8:
                    continue
                xy = np.array(part.coords)[[0, -1], :2]
                fraction = np.linalg.norm(xy-a[:2], axis=1)/np.linalg.norm(b[:2]-a[:2])
                z = a[2]+fraction*(b[2]-a[2])
                segments.append(np.column_stack([xy, z]))
        for a, b in segments:
            c, d = a.copy(), b.copy()
            c[2] -= bridge.thickness(a[None, :2])[0]
            d[2] -= bridge.thickness(b[None, :2])[0]
            walls.extend([[a, c, d], [a, d, b]])
    counts['fascia'] = gltf.add('Illustrative concrete slab edge', np.asarray(walls), 1)
    area_error = abs(sum(areas.values())-bridge.pavement.area)
    if area_error > 1e-5:
        raise ValueError(f'Deck partition changed the authoritative footprint by {area_error} square meters')
    if hasattr(bridge, 'railing_paths'):
        from bridge_railings import export_railings
        counts['railings'] = export_railings(bridge, deck, gltf)
    if retain_surface:
        from terrain import ExactSurface
        bridge.exported_surface = ExactSurface(deck)
    if writer is None:
        gltf.save()
        np.savez_compressed(output_folder/'deck-validation.npz', deck=deck,
            **{m: np.asarray(v).reshape(-1, 3, 3) for m, v in output.items()})
    return counts, areas, area_error


def build(bridge):
    """Partition one solved deck into nonoverlapping material triangles and export a complete illustrative slab."""
    selection = json.loads((OUTPUT/'source-selection.json').read_text(encoding='utf-8'))
    for name, digest in selection['source_hashes'].items():
        if hashlib.sha256((SOURCE/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Source changed since terrain-query preparation: '+name)
    bridge.attach_samples()
    receiver_transfer = {}
    apron = shapely.union_all([Polygon(bridge.xy([s-1, s+2, s+2, s-1], [-8, -8, 8, 8]))
        if s == 0 else Polygon(bridge.xy([s-2, s+1, s+1, s-2], [-8, -8, 8, 8]))
        for s in (0, bridge.length)]).intersection(bridge.pavement)
    for material in ('asphalt', 'white', 'yellow'):
        collection = json.loads((SOURCE/'render-contract'/(material+'.geojson')).read_text(encoding='utf-8'))
        removed = 0.0
        for feature in collection['features']:
            geometry = bridge.local(feature['geometry'])
            if not geometry.intersects(bridge.pavement):
                continue
            clipped = geometry.difference(bridge.pavement.difference(apron))
            removed += geometry.area-clipped.area
            feature['geometry'] = mapping(shapely.transform(clipped, bridge.to_geo.transform, interleaved=False))
        collection['features'] = [f for f in collection['features'] if not shape(f['geometry']).is_empty]
        continuation = bridge.materials[material].intersection(apron)
        if not continuation.is_empty:
            collection['features'].append({'type': 'Feature', 'id': 'congress/landing-continuation/'+material,
                'properties': {'material': material, 'receiver_id': 'terrain/boston',
                    'surface_id': 'congress/landing-attachment', 'source_way_ids': list(WAYS),
                    'basis': 'Two-meter material continuation beneath the explicit deck landing; inferred attachment policy'},
                'geometry': mapping(shapely.transform(continuation, bridge.to_geo.transform, interleaved=False))})
        write_json(OUTPUT/f'ground-{material}.geojson', collection)
        receiver_transfer[material] = removed
    counts, areas, area_error = export_deck(bridge)
    s = np.linspace(0, bridge.length, 1001)
    xy = bridge.xy(s, 0)
    z = bridge.height(xy[:, 0], xy[:, 1])[0]
    if np.max(abs(np.gradient(z, s))) > .150001:
        raise ValueError('Final centerline exceeds the declared demonstration grade limit')
    ground = bridge.ground(xy)
    interior = (s >= 40) & (s <= bridge.length-40)
    np.savetxt(OUTPUT/'profile.csv', np.column_stack([s, z, ground, np.gradient(z, s)*100]), delimiter=',',
               header='station_m,deck_ellipsoid_m,terrain_ellipsoid_m,grade_percent', comments='')
    for dynamic in (False, True):
        for view in ('overview', 'west-landing', 'east-landing', 'deck', 'underside'):
            (OUTPUT/f'{"dynamic" if dynamic else "fixed"}-{view}.earth').write_text(
                scene(bridge, view, dynamic=dynamic), encoding='utf-8')
    context = ET.fromstring(scene(bridge, 'overview', dynamic=True))
    imagery = ET.Element('TMSImage', name='Boston imagery / repository sample source')
    ET.SubElement(imagery, 'url').text = 'https://readymap.org/readymap/tiles/1.0.0/22/'
    context.insert(2, imagery)
    (OUTPUT/'context-overview.earth').write_text(ET.tostring(context, encoding='unicode'), encoding='utf-8')
    boundary = np.asarray(shapely.segmentize(bridge.pavement, .05).exterior.coords)[:, :2]
    station, _ = bridge.st(boundary)
    terminal = boundary[(station < .025) | (station > bridge.length-.025)]
    height = bridge.height(terminal[:, 0], terminal[:, 1])[0]
    lon, lat = bridge.to_geo.transform(terminal[:, 0], terminal[:, 1])
    np.savetxt(OUTPUT/'landing-query.csv', np.column_stack([np.arange(len(lon)), lon, lat]), delimiter=',',
               header='id,longitude,latitude', comments='', fmt='%.14g')
    np.save(OUTPUT/'landing-heights.npy', height)
    report = {'prototype': 'Congress Street closed roadway slab', 'source_ways': list(WAYS),
        'connections': bridge.connections, 'terrain': 'Unchanged ReadyMap; fixed LOD 19, 17-node tiles, morph disabled',
        'terrain_snapshot_sha256': hashlib.sha256((OUTPUT/'terrain-sampled.csv').read_bytes()).hexdigest(),
        'terrain_snapshot_scene_sha256': hashlib.sha256((OUTPUT/'terrain-only.earth').read_bytes()).hexdigest(),
        'source_hashes': selection['source_hashes'],
        'generator_hashes': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (ROOT/'bridge_slice.py', ROOT/'bridge_materials.py')},
        'heights': {'datum': 'WGS84 ellipsoid meters', 'status': 'Inferred demonstration profile, not surveyed',
            'basis': 'Grade-constrained smooth profile above rendered terrain, with 40m slab/cross-section transitions',
            'clearance_lower_bound': '0.75m tapered slab plus 2m*sin(pi*station/span)^2; illustrative, not water clearance',
            'grade_limit_percent': 15, 'endpoint_ellipsoid_m': [float(z[0]), float(z[-1])],
            'max_longitudinal_grade_percent': float(max(abs(np.gradient(z, s)))*100),
            'min_interior_underside_to_terrain_m': float(np.min(z[interior]-.75-ground[interior]))},
        'triangles': counts, 'material_area_m2': areas, 'footprint_area_error_m2': area_error,
        'ground_overlap_transferred_to_bridge_m2': receiver_transfer,
        'landing_material_continuation_m': 2,
        'paint_policy': 'Exact deck-triangle partition, no overlay, no depth bias, no terrain receiver',
        'material_policy': 'Same albedo/ORM files, one-meter ENU repeat and origin on terrain/deck; geometric normals',
        'limitations': ['Roadway only: compiler lane-based width 7.4m; source full-bridge width 22.9m retained separately',
            'Movable span rendered closed; no bascule mechanism, piers, sidewalks, or surveyed structural geometry',
            'DEM may include bridge/building elevations; clearance is against rendered terrain, not verified water level',
            'Fixed snapshot attachment; dynamic scenes expose LOD mismatch rather than conceal it']}
    write_json(OUTPUT/'manifest.json', report)
    print(json.dumps(report, indent=2), flush=True)


def main():
    """Prepare a snapshot query or build from its completed result; never rebuild or modify the frozen vector fixture."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'build'))
    args = parser.parse_args()
    bridge = Bridge()
    if args.action == 'build':
        write_json(OUTPUT/'build-in-progress.json', {'status': 'incomplete'})
    (prepare if args.action == 'prepare' else build)(bridge)
    if args.action == 'build':
        (OUTPUT/'build-in-progress.json').unlink()


if __name__ == '__main__':
    main()
