#!/usr/bin/env python3
"""Compile a bounded OSM road/terrain experiment; outputs are disposable, source IDs are retained.

Requires numpy, shapely >= 2.1, pyproj, scipy, rasterio, matplotlib, and Pillow.
Default heights are controlled; --terrain readymap uses retained EGM96 terrain converted to ellipsoid heights.
"""
import argparse
import collections
import gzip
import hashlib
import heapq
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from pyproj import CRS, Transformer
import rasterio
from rasterio.transform import from_bounds
from scipy.spatial import cKDTree
from scipy.interpolate import LinearNDInterpolator
import shapely
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union, substring
from surfaces import fit_pavement, mesh_normals, minimum_clearance, refine_triangles, vertical_walls
from intersections import audit_layout, export_layout
from scenes import SCENES
from semantics import oneway_value
from layouts import discover_layouts
from transitions import discover_transitions, export_transition
from controls import read_controls, build_stop_bars, contraflow_control_views


SCENE = 'alewife'
TERRAIN = None
SURFACE_POLICY = 'fit'
CLASSES = {'motorway', 'trunk', 'primary', 'secondary', 'tertiary', 'residential',
           'unclassified', 'motorway_link', 'trunk_link', 'primary_link', 'secondary_link', 'cycleway'}


def configure_scene(name):
    """Select a bounded experiment and rebuild geospatial transforms before any source or mesh work."""
    global SCENE, ORIGIN, BBOX, LOCAL, TO_LOCAL, TO_GEO, TO_ECEF, REGION, ANCHOR, LON, LAT, ENU
    SCENE = name
    if name not in SCENES:
        raise ValueError('Unknown experiment scene')
    ORIGIN, BBOX = SCENES[name]['origin'], SCENES[name]['bbox']
    LOCAL = CRS.from_proj4(f'+proj=aeqd +lat_0={ORIGIN[1]} +lon_0={ORIGIN[0]} +datum=WGS84 +units=m')
    TO_LOCAL = Transformer.from_crs('EPSG:4326', LOCAL, always_xy=True)
    TO_GEO = Transformer.from_crs(LOCAL, 'EPSG:4326', always_xy=True)
    TO_ECEF = Transformer.from_crs('EPSG:4979', 'EPSG:4978', always_xy=True)
    REGION = shapely.transform(box(*BBOX), TO_LOCAL.transform, interleaved=False)
    ANCHOR = np.array(TO_ECEF.transform(*ORIGIN, 0.0))
    LON, LAT = np.radians(ORIGIN)
    ENU = np.array([[-np.sin(LON), np.cos(LON), 0],
                    [-np.sin(LAT)*np.cos(LON), -np.sin(LAT)*np.sin(LON), np.cos(LAT)],
                    [np.cos(LAT)*np.cos(LON), np.cos(LAT)*np.sin(LON), np.sin(LAT)]])


configure_scene('alewife')


def base_height(x, y):
    """Return the known smooth roadbed height in meters for this controlled slope experiment."""
    return TERRAIN(x, y) if TERRAIN is not None else 20.0 + .012*np.asarray(x) + .009*np.asarray(y)


def parts(geometry):
    """Iterate nonempty polygon components; ignore line/point debris from clipping."""
    if geometry.is_empty:
        return
    if geometry.geom_type == 'Polygon':
        yield geometry
    elif hasattr(geometry, 'geoms'):
        for child in geometry.geoms:
            yield from parts(child)


def write_json(path, value):
    """Write deterministic UTF-8 diagnostic or interchange data, propagating I/O errors."""
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def read_source(path):
    """Read complete way references from the snapshot; fail on missing road nodes before geometry work."""
    with gzip.open(path, 'rb') as stream:
        root = ET.parse(stream).getroot()
    nodes = {int(n.get('id')): (float(n.get('lon')), float(n.get('lat'))) for n in root.findall('node')}
    xy = {key: TO_LOCAL.transform(*value) for key, value in nodes.items()}
    roads, flags = [], []
    for element in root.findall('way'):
        tags = {t.get('k'): t.get('v') for t in element.findall('tag')}
        if tags.get('highway') not in CLASSES.union(SCENES[SCENE].get('additional_classes', [])):
            continue
        refs = [int(n.get('ref')) for n in element.findall('nd')]
        missing = [n for n in refs if n not in xy]
        if missing:
            raise ValueError(f'Way {element.get("id")} has missing nodes: {missing}')
        line = LineString([xy[n] for n in refs])
        if not line.intersects(REGION):
            continue
        lane_text = tags.get('lanes', '1' if oneway_value(tags) in ('yes', '1', 'true', '-1') else '2')
        lanes = int(lane_text) if lane_text.isdigit() else 2
        lane_reason = 'source lanes tag' if 'lanes' in tags else 'prototype directional default'
        raw_width = tags.get('width')
        if ('lanes' not in tags and oneway_value(tags) in ('no', '0', 'false')
                and tags['highway'] in ('residential', 'unclassified', 'tertiary')):
            try:
                if 2.2 <= float(raw_width or 0) <= 3.5:
                    lanes, lane_reason = 1, 'INFERRED shared single lane from narrow mapped width; passing priority unknown'
            except ValueError:
                pass
        inferred = lanes*3.4 + (1.4 if tags['highway'] in ('motorway', 'trunk') else .6)
        if tags['highway'] == 'cycleway':
            lanes, inferred = 1, 3.2
        width = inferred
        reason = 'lane-based prototype default'
        if raw_width:
            try:
                candidate = float(raw_width)
                minimum, maximum = (0.75, 8.0) if tags['highway'] == 'cycleway' else (lanes*2.5, lanes*4.5+4)
                if lane_reason.startswith('INFERRED shared'):
                    minimum = 2.2
                if minimum <= candidate <= maximum:
                    width, reason = candidate, 'explicit OSM width'
                else:
                    flags.append({'way': element.get('id'), 'width': raw_width, 'lanes': lanes,
                                  'used_width': inferred, 'reason': 'width/lane conflict; prototype override'})
            except ValueError:
                flags.append({'way': element.get('id'), 'width': raw_width, 'reason': 'unsupported width units'})
        road = {'id': int(element.get('id')), 'version': element.get('version'), 'refs': refs, 'tags': tags,
                'line': line, 'lanes': lanes, 'driving_side': SCENES[SCENE]['driving_side'],
                'lane_count_reason': lane_reason, 'width': width, 'width_reason': reason,
                'bridge': tags.get('bridge', 'no') != 'no', 'tunnel': tags.get('tunnel', 'no') != 'no'}
        roads.append(road)
    relations = []
    for rel in root.findall('relation'):
        tags = {t.get('k'): t.get('v') for t in rel.findall('tag')}
        if tags.get('type') not in ('restriction', 'connectivity'):
            continue
        members = [dict(m.attrib) for m in rel.findall('member')]
        if any(m['type'] == 'way' and int(m['ref']) in {r['id'] for r in roads} for m in members):
            relations.append({'id': rel.get('id'), 'tags': tags, 'members': members})
    return roads, xy, flags, relations


def bridge_frames(roads):
    """Share cross-section tangents at compatible degree-two deck/approach ends; leave junctions to their layout solver."""
    incident = collections.defaultdict(list)
    result = {}
    for road in roads:
        if not road['tunnel']:
            for endpoint in (0, -1):
                incident[road['refs'][endpoint]].append((road, endpoint))
    for group in incident.values():
        if len(group) != 2 or not any(r['bridge'] for r, _ in group):
            continue
        if len({r['tags']['highway'] == 'cycleway' for r, _ in group}) != 1:
            continue
        outward = []
        for road, endpoint in group:
            vector = road['samples'][1 if endpoint == 0 else -2]-road['samples'][endpoint]
            outward.append(vector/np.linalg.norm(vector))
        if float(outward[0] @ outward[1]) > -.5:
            continue
        common = outward[0]-outward[1]
        common /= np.linalg.norm(common)
        for index, (road, endpoint) in enumerate(group):
            sign = (1 if index == 0 else -1)*(1 if endpoint == 0 else -1)
            result[(road['id'], endpoint)] = common*sign
    return result


def solve_profiles(roads, xy):
    """Solve synthetic deck-clearance constraints, then share smooth graph-distance profiles at source nodes."""
    graph = collections.defaultdict(list)
    bridge_nodes = collections.defaultdict(set)
    node_roads = collections.defaultdict(list)
    for road in roads:
        if road['tunnel']:
            continue
        refs = road['refs']
        if road['bridge'] and TERRAIN is None:
            # Explicit experimental structure heights: highway over bikeway versus short water crossings.
            role = 'highway' if road['tags']['highway'] in ('trunk', 'motorway') else 'water-crossing'
            bridge_nodes[role].update(refs)
        for node in (refs[0], refs[-1]):
            node_roads[node].append(road)
        for a, b in zip(refs, refs[1:]):
            length = math.dist(xy[a], xy[b])
            graph[a].append((b, length))
            graph[b].append((a, length))
    distances = {}
    for role, seeds in sorted(bridge_nodes.items()):
        distance = {node: 0.0 for node in seeds}
        queue = [(0.0, node) for node in sorted(seeds)]
        heapq.heapify(queue)
        while queue:
            d, node = heapq.heappop(queue)
            if d != distance[node]:
                continue
            for neighbor, length in graph[node]:
                nd = d+length
                if nd < 240 and nd < distance.get(neighbor, float('inf')):
                    distance[neighbor] = nd
                    heapq.heappush(queue, (nd, neighbor))
        distances[role] = distance
    targets = {'highway': 3.0, 'water-crossing': 1.0} if TERRAIN is None else {}
    constraints = []
    for upper in roads:
        if TERRAIN is None and upper['bridge'] and upper['tags']['highway'] in ('trunk', 'motorway'):
            footprint = upper['line'].buffer(upper['width']/2, cap_style=2)
            for lower in roads:
                if lower['tags']['highway'] == 'cycleway' and footprint.intersects(lower['line']):
                    if not set(upper['refs']) & set(lower['refs']):
                        constraints.append((upper, lower, footprint))
    for iteration in range(8):
        for road in roads:
            coords, samples = [], []
            refs = road['refs']
            for a, b in zip(refs, refs[1:]):
                length = math.dist(xy[a], xy[b])
                for s in np.linspace(0, length, max(2, math.ceil(length/2)+1))[:-1]:
                    point = np.array(xy[a])+(np.array(xy[b])-xy[a])*(s/max(length, 1e-9))
                    lift = 0.0
                    for role, distance in distances.items():
                        d = min(distance.get(a, 240)+s, distance.get(b, 240)+length-s)
                        own_role = 'highway' if road['tags']['highway'] in ('trunk', 'motorway') else 'water-crossing'
                        if road['bridge'] and role == own_role:
                            d = 0.0
                        t = np.clip(d/220, 0, 1)
                        lift = max(lift, targets[role]*(1-(6*t**5-15*t**4+10*t**3)))
                    coords.append(point)
                    samples.append(float(base_height(*point))+lift)
            point = xy[refs[-1]]
            lift = max((targets[role]*(1-(6*t**5-15*t**4+10*t**3))
                       for role, distance in distances.items()
                       for t in [np.clip(distance.get(refs[-1], 240)/220, 0, 1)]), default=0.0)
            coords.append(point)
            samples.append(float(base_height(*point))+lift)
            road['samples'], road['heights'] = np.array(coords), np.array(samples)
        deficit = 0.0
        for upper, lower, footprint in constraints:
            points = shapely.get_coordinates(shapely.segmentize(lower['line'].intersection(footprint), .5))
            lower_z, _ = Heights([lower])(points[:, 0], points[:, 1])
            upper_z = base_height(points[:, 0], points[:, 1])+targets['highway']
            deficit = max(deficit, float(np.max(5.25+.7+lower_z-upper_z)))
        if deficit < 1e-6:
            break
        targets['highway'] += deficit+.001
    else:
        raise ValueError('Synthetic bridge clearance constraints did not converge')
    if TERRAIN is not None:
        for road in roads:
            if road['bridge']:
                stations = np.r_[0, np.cumsum(np.linalg.norm(np.diff(road['samples'], axis=0), axis=1))]
                road['heights'] = np.interp(stations, [0, stations[-1]], road['heights'][[0, -1]])
                road['height_inference'] = 'INFERRED straight deck between terrain-sampled endpoints; clearance unverified'
    lifts = {}
    for node in graph:
        lifts[node] = max((targets[role]*(1-(6*t**5-15*t**4+10*t**3))
                          for role, distance in distances.items()
                          for t in [np.clip(distance.get(node, 240)/220, 0, 1)]), default=0.0)
    shared_frames = bridge_frames(roads)
    for road in roads:
        width = np.full(len(road['samples']), road['width'])
        cumulative = np.r_[0, np.cumsum(np.linalg.norm(np.diff(road['samples'], axis=0), axis=1))]
        for endpoint, d in ((road['refs'][0], cumulative), (road['refs'][-1], cumulative[-1]-cumulative)):
            incident = node_roads[endpoint]
            if len(incident) == 2 and len({r['tags']['highway'] == 'cycleway' for r in incident}) == 1:
                common_width = sum(r['width'] for r in incident)/2
                t = np.clip(d/20, 0, 1)
                width += (common_width-road['width'])*(1-t*t*(3-2*t))
        tangent = np.gradient(road['samples'], axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        for endpoint, d in ((0, cumulative), (-1, cumulative[-1]-cumulative)):
            common = shared_frames.get((road['id'], endpoint))
            if common is not None:
                t = np.clip(d/max(.01, min(6, cumulative[-1]/3)), 0, 1)
                weight = (1-t*t*(3-2*t))[:, None]
                tangent = tangent*(1-weight)+common*weight
                tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
        left = road['samples']+normal*width[:, None]/2
        right = road['samples']-normal*width[:, None]/2
        road['surface'] = shapely.make_valid(Polygon(np.vstack([left, right[::-1]]))).intersection(REGION)
        road['left_edge'], road['right_edge'] = LineString(left), LineString(right)
        road['profile_targets'] = dict(targets)
    return graph, lifts


def junction_profile_distances(roads, junction_nodes):
    """Measure junction influence along ground source paths; never transfer height across disconnected nearby roads."""
    graph = collections.defaultdict(list)
    for road in roads:
        coordinates = np.asarray(road['line'].coords)[:, :2]
        for a, b, length in zip(road['refs'], road['refs'][1:], np.linalg.norm(np.diff(coordinates, axis=0), axis=1)):
            graph[a].append((b, float(length)))
            graph[b].append((a, float(length)))
    nearest = {node: (0.0, index) for index, node in enumerate(junction_nodes) if node in graph}
    queue = [(distance, index, node) for node, (distance, index) in nearest.items()]
    heapq.heapify(queue)
    while queue:
        distance, index, node = heapq.heappop(queue)
        if nearest[node] != (distance, index):
            continue
        for other, length in graph[node]:
            candidate = distance+length, index
            if candidate[0] <= 18 and candidate < nearest.get(other, (float('inf'), 0)):
                nearest[other] = candidate
                heapq.heappush(queue, (*candidate, other))
    distances, seeds = [], []
    for road in roads:
        stations = np.r_[0, np.cumsum(np.linalg.norm(np.diff(np.asarray(road['line'].coords)[:, :2], axis=0), axis=1))]
        dense = np.r_[0, np.cumsum(np.linalg.norm(np.diff(road['samples'], axis=0), axis=1))]
        segment = np.clip(np.searchsorted(stations, dense, side='right')-1, 0, len(stations)-2)
        values = np.array([nearest.get(node, (float('inf'), 0)) for node in road['refs']])
        before = values[segment, 0]+dense-stations[segment]
        after = values[segment+1, 0]+stations[segment+1]-dense
        first = before <= after
        distances.append(np.minimum(before, after))
        seeds.append(np.where(first, values[segment, 1], values[segment+1, 1]).astype(int))
    return (np.concatenate([d[:-1] for d in distances]), np.concatenate([d[1:] for d in distances]),
            np.concatenate([s[:-1] for s in seeds]), np.concatenate([s[1:] for s in seeds]))


class Heights:
    """Read-only nearest-segment height sampler for a connected family of non-overlapping road surfaces."""
    def __init__(self, roads, junctions=None, junction_nodes=None):
        """Copy alignments; optional source junction IDs restrict blending to distances along the ground road graph."""
        self.a = np.concatenate([r['samples'][:-1] for r in roads])
        self.b = np.concatenate([r['samples'][1:] for r in roads])
        self.za = np.concatenate([r['heights'][:-1] for r in roads])
        self.zb = np.concatenate([r['heights'][1:] for r in roads])
        self.tree = cKDTree((self.a+self.b)*.5)
        self.junctions = junctions
        self.junction_distances = None
        if junctions is not None and len(junctions):
            self.junction_tree = cKDTree(junctions[:, :2])
            self.junction_heights = base_height(junctions[:, 0], junctions[:, 1])+junctions[:, 2]
            if junction_nodes is not None:
                self.junction_distances = junction_profile_distances(roads, junction_nodes)

    def junction_neighbors(self, points, segments, fractions):
        """Return the closest permitted junction and its distance; infinite distance has zero blending weight."""
        if self.junction_distances is None:
            return self.junction_tree.query(points)
        da, db, ja, jb = self.junction_distances
        length = np.linalg.norm(self.b[segments]-self.a[segments], axis=1)
        before, after = da[segments]+fractions*length, db[segments]+(1-fractions)*length
        return np.minimum(before, after), np.where(before <= after, ja[segments], jb[segments])

    def __call__(self, x, y):
        """Return road height plus distance to its alignment for array-like planar coordinates."""
        shape = np.broadcast(np.asarray(x), np.asarray(y)).shape
        points = np.column_stack([np.broadcast_to(x, shape).ravel(), np.broadcast_to(y, shape).ravel()])
        _, ids = self.tree.query(points, k=min(8, len(self.a)))
        if ids.ndim == 1:
            ids = ids[:, None]
        a, b = self.a[ids], self.b[ids]
        v = b-a
        t = np.clip(np.sum((points[:, None]-a)*v, axis=2)/np.maximum(np.sum(v*v, axis=2), 1e-12), 0, 1)
        d2 = np.sum((points[:, None]-(a+t[:, :, None]*v))**2, axis=2)
        choice = np.argmin(d2, axis=1)
        row = np.arange(len(points))
        if TERRAIN is not None:
            z = self.za[ids]+t*(self.zb[ids]-self.za[ids])
            z = z[row, choice]
            if self.junctions is not None and len(self.junctions):
                distance, node = self.junction_neighbors(points, ids[row, choice], t[row, choice])
                blend = np.clip(distance/18, 0, 1)
                weight = 1-(6*blend**5-15*blend**4+10*blend**3)
                z = z*(1-weight)+self.junction_heights[node]*weight
            return z.reshape(shape), np.sqrt(d2[row, choice]).reshape(shape)
        za = self.za-base_height(self.a[:, 0], self.a[:, 1])
        zb = self.zb-base_height(self.b[:, 0], self.b[:, 1])
        residual = za[ids]+t*(zb[ids]-za[ids])
        lift = residual[row, choice]
        if self.junctions is not None and len(self.junctions):
            distance, node = self.junction_neighbors(points, ids[row, choice], t[row, choice])
            t = np.clip(distance/18, 0, 1)
            weight = 1-(6*t**5-15*t**4+10*t**3)
            lift = lift*(1-weight)+self.junctions[node, 2]*weight
        z = base_height(points[:, 0], points[:, 1])+lift
        return z.reshape(shape), np.sqrt(d2[row, choice]).reshape(shape)


def bridge_constraints(roads, ground_roads, collars=True):
    """Tie decks to topologically connected six-meter approach collars; never pull nearby unconnected roads upward."""
    constraints = []
    for road in roads:
        if not road['bridge']:
            continue
        for endpoint in (0, -1):
            node = road['refs'][endpoint]
            approaches = [r for r in ground_roads if node in r['refs']]
            if not approaches:
                continue
            cap = LineString([road['left_edge'].coords[endpoint], road['right_edge'].coords[endpoint]])
            cap = cap.intersection(unary_union([r['surface'] for r in approaches]).buffer(.001))
            constraints.append((cap, Heights([road]), .025))
            if not collars:
                continue
            for approach in approaches:
                if node not in (approach['refs'][0], approach['refs'][-1]):
                    continue
                outward = (approach['line'] if node == approach['refs'][0]
                           else LineString(list(approach['line'].coords)[::-1]))
                collar = substring(outward, 0, min(6, outward.length)).buffer(approach['width']/2, cap_style=2)
                collar = collar.intersection(approach['surface'])
                constraints.append((collar, Heights([approach]), .001))
        if collars and road['tags']['highway'] in ('trunk', 'motorway'):
            for lower in ground_roads:
                if lower['tags']['highway'] == 'cycleway' and road['surface'].intersects(lower['surface']):
                    if not set(road['refs']) & set(lower['refs']):
                        footprint = road['surface'].buffer(4).intersection(lower['surface'].buffer(.2))
                        constraints.append((footprint, Heights([lower]), 0.0))
    return constraints


def rounded_ground_components(ground, radius=4):
    """Round existing pavement components without welding separate road patches through newly added fillets."""
    polygons = list(parts(ground))
    tree = shapely.STRtree(polygons)
    # Closing adds points at most radius meters from a component. Reserve that expansion around every
    # foreign component, plus a small guard that survives the later millimeter precision normalization.
    guards = [polygon.buffer(radius+.02) for polygon in polygons]
    rounded = []
    for index, polygon in enumerate(polygons):
        others = tree.query(polygon, predicate='dwithin', distance=2*radius+.04)
        forbidden = unary_union([guards[j] for j in others if j != index])
        additions = polygon.buffer(radius).buffer(-radius).difference(polygon).difference(forbidden)
        combined = polygon.union(additions)
        rounded.extend(piece for piece in parts(combined) if piece.intersection(polygon).area > 1e-8)
    return unary_union(rounded)


class MeshSurface:
    """Interpolate the solved mesh vertices for terrain and edge assets; paint uses the exact original triangles."""
    def __init__(self, triangles):
        """Extend solved vertex heights into nearby land using a triangulated field with flat outer guard points."""
        points, indices = np.unique(np.round(triangles[:, :, :2].reshape(-1, 2), 9), axis=0, return_index=True)
        heights = triangles[:, :, 2].ravel()[indices]-base_height(points[:, 0], points[:, 1])
        self.vertices = cKDTree(points)
        self.residuals = heights
        west, south, east, north = REGION.buffer(350).bounds
        guards = np.array([(x, y) for x in np.linspace(west, east, 20) for y in (south, north)] +
                          [(x, y) for x in (west, east) for y in np.linspace(south, north, 20)])
        self.field = LinearNDInterpolator(np.vstack([points, guards]), np.r_[heights, np.zeros(len(guards))],
                                          fill_value=0.0)

    def __call__(self, x, y):
        """Return continuous linear interpolation through solved vertices, with base terrain outside the guard region."""
        x, y = np.broadcast_arrays(x, y)
        residual = self.field(x, y)
        distance, index = self.vertices.query(np.column_stack([x.ravel(), y.ravel()]))
        # Qhull can omit almost-collinear samples; exact boundary exports must retain the solved vertex value.
        residual = np.where(distance.reshape(x.shape) < 1e-7, self.residuals[index].reshape(x.shape), residual)
        return base_height(x, y)+residual, np.zeros(x.shape)


def triangulate(geometry, height, offset=0, max_edge=0):
    """Constrained-triangulate polygon interiors, preserving holes; return counterclockwise XYZ triangles."""
    triangles = []
    for polygon in parts(geometry):
        # Cleanup has already quantized the boundary. Clear that precision model before interpolation:
        # densifying a fixed-precision polygon again can re-snap its existing intermediate vertices.
        polygon = shapely.segmentize(shapely.set_precision(polygon, 0), 3.0)
        for tri in shapely.constrained_delaunay_triangles(polygon).geoms:
            points = np.array(tri.exterior.coords[:3])[:, :2]
            a, b = points[1]-points[0], points[2]-points[0]
            cross = a[0]*b[1]-a[1]*b[0]
            if cross < 0:
                points = points[[0, 2, 1]]
            z, _ = height(points[:, 0], points[:, 1])
            triangles.append(np.column_stack([points, z+offset]))
    result = np.array(triangles).reshape(-1, 3, 3)
    return refine_triangles(result, height, max_edge) if len(result) and max_edge else result


def validate_pavement_footprint(ground, pavement, output):
    """Reject invalid footprints or lost triangle area before asset export, retaining a launcher-visible failure marker."""
    vectors = np.cross(pavement[:, 1]-pavement[:, 0], pavement[:, 2]-pavement[:, 0])
    area = float(np.abs(vectors[:, 2]).sum()/2)
    error = abs(area-ground.area)
    if not ground.is_valid or not np.isfinite(area) or error >= 1e-5:
        write_json(output/'surface-failure.json', {'stage': 'pavement footprint',
                   'reason': 'Invalid ground geometry or triangulated area mismatch', 'area_error_m2': error,
                   'policy': 'Reject before exporting road assets; preserve the original area acceptance guard'})
        raise ValueError(f'Pavement footprint failed area conservation: {error} m2')
    return area


class GLTF:
    """Own a small, conventional glTF mesh/material bundle, with no virtual textures or runtime compiler."""
    def __init__(self, output):
        """Start an empty scene and aligned binary buffer; output must be a writable directory."""
        self.output, self.blob = output, bytearray()
        self.doc = {'asset': {'version': '2.0', 'generator': 'Prestige Roads bounded experiment'},
                    'scene': 0, 'scenes': [{'nodes': []}], 'nodes': [], 'meshes': [], 'materials': [],
                    'accessors': [], 'bufferViews': [], 'buffers': [],
                    'samplers': [{'magFilter': 9729, 'minFilter': 9987, 'wrapS': 10497, 'wrapT': 10497}],
                    'images': [{'uri': 'asphalt.png'}], 'textures': [{'sampler': 0, 'source': 0}]}
        for name, color, rough in [('asphalt', [1, 1, 1, 1], .88), ('concrete', [.55, .53, .48, 1], .9),
                                  ('white paint', [.88, .87, .78, 1], .65), ('yellow paint', [.9, .64, .12, 1], .65)]:
            material = {'name': name, 'doubleSided': False, 'pbrMetallicRoughness': {
                'baseColorFactor': color, 'metallicFactor': 0, 'roughnessFactor': rough}}
            if name == 'asphalt':
                material['pbrMetallicRoughness']['baseColorTexture'] = {'index': 0}
            self.doc['materials'].append(material)

    def accessor(self, data, kind):
        """Append float vertex data with bounds to the owned buffer and return its glTF accessor index."""
        data = np.asarray(data, dtype='<f4')
        while len(self.blob) % 4:
            self.blob.append(0)
        index = len(self.doc['bufferViews'])
        self.doc['bufferViews'].append({'buffer': 0, 'byteOffset': len(self.blob), 'byteLength': data.nbytes,
                                       'target': 34962})
        self.blob.extend(data.tobytes())
        accessor = {'bufferView': index, 'componentType': 5126, 'count': len(data), 'type': kind,
                    'min': data.min(axis=0).tolist(), 'max': data.max(axis=0).tolist()}
        self.doc['accessors'].append(accessor)
        return len(self.doc['accessors'])-1

    def add(self, name, triangles, material, smooth=False):
        """Append an ENU-anchored drawable and return its triangle count, dropping float-collapsed clipping slivers."""
        if not len(triangles):
            return 0
        xyz = triangles.reshape(-1, 3)
        lon, lat = TO_GEO.transform(xyz[:, 0], xyz[:, 1])
        render_offset = .12 if TERRAIN is not None else 0.0
        ecef = np.column_stack(TO_ECEF.transform(lon, lat, xyz[:, 2]+render_offset))
        enu = (ecef-ANCHOR) @ ENU.T
        positions = np.asarray(enu[:, [0, 2, 1]] * [1, 1, -1], dtype='<f4')
        tri = positions.astype(float).reshape(-1, 3, 3)
        area2 = np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1)
        keep = np.repeat(area2 > 2e-8, 3)
        positions, xyz = positions[keep], xyz[keep]
        if not len(positions):
            return 0
        tri = positions.reshape(-1, 3, 3)
        normals = mesh_normals(positions, smooth)
        attributes = {'POSITION': self.accessor(positions, 'VEC3'),
                      'NORMAL': self.accessor(normals, 'VEC3'),
                      'TEXCOORD_0': self.accessor(xyz[:, :2]/4, 'VEC2')}
        self.doc['meshes'].append({'name': name, 'primitives': [{'attributes': attributes, 'material': material}]})
        self.doc['nodes'].append({'name': name, 'mesh': len(self.doc['meshes'])-1})
        self.doc['scenes'][0]['nodes'].append(len(self.doc['nodes'])-1)
        return len(tri)

    def save(self):
        """Write complete conventional glTF and binary files; propagate write failures."""
        self.doc['buffers'] = [{'uri': 'roads.bin', 'byteLength': len(self.blob)}]
        (self.output/'roads.bin').write_bytes(self.blob)
        write_json(self.output/'roads.gltf', self.doc)


def boundary_geojson(geometry, height):
    """Export identical road boundary XY samples and elevations for osgEarth terrain constraints."""
    features = []
    for polygon in parts(shapely.segmentize(shapely.set_precision(geometry, 0), 3)):
        rings = []
        for ring in [polygon.exterior, *polygon.interiors]:
            coords = np.array(ring.coords)[:, :2]
            lon, lat = TO_GEO.transform(coords[:, 0], coords[:, 1])
            z, _ = height(coords[:, 0], coords[:, 1])
            rings.append(np.column_stack([lon, lat, z]).tolist())
        features.append({'type': 'Feature', 'properties': {'experimental_elevation': True},
                         'geometry': {'type': 'Polygon', 'coordinates': rings}})
    return {'type': 'FeatureCollection', 'features': features}


def write_terrain(output, ground, height, roads):
    """Create a controlled DEM with smoothly graded shoulders and a neutral contextual image, entirely offline."""
    extent = REGION.buffer(300).bounds
    geographic = shapely.transform(box(*extent), TO_GEO.transform, interleaved=False).bounds
    size = 1024
    west, south, east, north = geographic
    lon = np.linspace(west, east, size, endpoint=False)+(east-west)/(size*2)
    lat = np.linspace(north, south, size, endpoint=False)-(north-south)/(size*2)
    xx, yy = TO_LOCAL.transform(*np.meshgrid(lon, lat))
    road_z, _ = height(xx, yy)
    points = shapely.points(xx.ravel(), yy.ravel())
    distance = shapely.distance(points, ground).reshape(xx.shape)
    terrain_z = base_height(xx, yy)+1.25*np.sin(xx/33)*np.sin(yy/47)
    blend = np.clip(distance/18, 0, 1)
    blend = blend*blend*(3-2*blend)
    z = road_z*(1-blend)+terrain_z*blend
    for upper in roads:
        if upper['bridge'] and upper['tags']['highway'] in ('trunk', 'motorway'):
            for lower in roads:
                if lower['tags']['highway'] == 'cycleway' and upper['surface'].intersects(lower['surface']):
                    if set(upper['refs']) & set(lower['refs']):
                        continue
                    opening = lower['line'].buffer(lower['width']/2+4).intersection(upper['surface'].buffer(2))
                    d = shapely.distance(points, opening).reshape(xx.shape)
                    t = np.clip(d/6, 0, 1)
                    weight = 1-t*t*(3-2*t)
                    lower_z, _ = Heights([lower])(xx, yy)
                    z = z*(1-weight)+lower_z*weight
    # Keep every ground-road boundary at the same height as the emitted pavement.
    z = np.where(distance < 1e-6, road_z, z)
    profile = {'driver': 'GTiff', 'height': size, 'width': size, 'crs': 'EPSG:4326',
               'transform': from_bounds(*geographic, size, size), 'compress': 'deflate'}
    with rasterio.open(output/'terrain.tif', 'w', count=1, dtype='float32', **profile) as dataset:
        dataset.write(z.astype('float32'), 1)
        dataset.update_tags(description='CONTROLLED EXPERIMENTAL HEIGHTS, WGS84 ellipsoid; not survey data')
    noise = 4*np.sin(xx/7)*np.cos(yy/8)+7*np.sin(xx/103)
    color = np.stack([115+noise, 125+noise, 94+noise]).clip(0, 255).astype('uint8')
    if SCENE != 'alewife':
        color = np.stack([148+noise, 148+noise, 142+noise]).clip(0, 255).astype('uint8')
    with rasterio.open(output/'ground.tif', 'w', count=3, dtype='uint8', **profile) as dataset:
        dataset.write(color)
    return geographic


def drape_paint(paint, support, index=None):
    """Split paint on support faces; an optional caller-owned index must reference the same unchanged support array."""
    if index is None:
        index = shapely.STRtree(shapely.polygons(support[:, :, :2]))
    faces = index.geometries
    result = []
    for triangle in paint:
        footprint = Polygon(triangle[:, :2])
        for face_id in index.query(footprint, predicate='intersects'):
            face = support[face_id]
            clipped = footprint.intersection(faces[face_id])
            for polygon in parts(clipped):
                for tri in shapely.constrained_delaunay_triangles(polygon).geoms:
                    xy = np.array(shapely.orient_polygons(tri).exterior.coords[:3])
                    weights = np.linalg.solve(np.vstack([face[:, :2].T, np.ones(3)]),
                                              np.vstack([xy.T, np.ones(3)]))
                    z = weights.T @ face[:, 2]+.018
                    result.append(np.column_stack([xy, z]))
    return np.array(result).reshape(-1, 3, 3)


def road_markings(roads, junctions, pavement, height, ground):
    """Generate illustrative lane strips with metric dash spacing; suppress uncertain junction paint."""
    if SCENE != 'alewife':
        from urban import street_paint
        white, yellow = street_paint(roads, junctions, ground, SCENES[SCENE]['driving_side'], SCENES[SCENE]['center_line'])
        return (drape_paint(triangulate(white, height), pavement),
                drape_paint(triangulate(yellow, height), pavement))
    white, yellow = [], []
    ground_edge = ground.buffer(-.30).boundary
    for road in roads:
        if road['tunnel']:
            continue
        own_height = Heights([road]) if road['bridge'] else height
        support = road['mesh'] if road['bridge'] else pavement
        line = road['line']
        usable = road['surface'].buffer(-.3)
        if not road['bridge']:
            usable = usable.difference(junctions)
        for lane in range(1, road['lanes']):
            lateral = road['width']*(lane/road['lanes']-.5)
            offset = line.offset_curve(lateral)
            if offset.geom_type != 'LineString':
                continue
            center = road['tags'].get('oneway') != 'yes' and abs(lateral) < .5
            if center:
                paint = offset.buffer(.075, cap_style=2).intersection(usable)
            else:
                pieces = [substring(offset, s, min(s+3, offset.length)).buffer(.075, cap_style=2)
                          for s in np.arange(0, max(0, offset.length-1), 9)]
                paint = unary_union(pieces).intersection(usable)
            (yellow if center else white).append(drape_paint(triangulate(paint, own_height), support))
        if road['tags']['highway'] in ('trunk', 'motorway', 'trunk_link', 'motorway_link'):
            for lateral in (-road['width']/2+.3, road['width']/2-.3):
                guide = line.offset_curve(lateral)
                if road['bridge']:
                    edge = guide.buffer(.07).intersection(usable)
                else:
                    # Use the same inset perimeter as resolved junctions, avoiding doubled or offset handoff stripes.
                    edge = ground_edge.intersection(guide.buffer(1.0)).buffer(.07).difference(junctions)
                direction = road['tags'].get('oneway')
                left_edge = (direction in ('yes', '1', 'true') and lateral > 0) or (direction == '-1' and lateral < 0)
                (yellow if left_edge else white).append(drape_paint(triangulate(edge, own_height), support))
    empty = np.empty((0, 3, 3))
    return np.concatenate(white) if white else empty, np.concatenate(yellow) if yellow else empty


def road_details(gltf, roads, ground, height):
    """Add bounded curb and bridge-edge solids; these are illustrative assets, not surveyed structure designs."""
    count = 0
    urban = unary_union([r['surface'] for r in roads if r['tags']['highway'] in
                         ('secondary', 'residential', 'unclassified') and not r['bridge'] and not r['tunnel']])
    # Follow the final union edge, so no curb is drawn through a connecting carriageway.
    curb = ground.buffer(.22).difference(ground).intersection(urban.buffer(.5)).intersection(REGION)
    if SCENE != 'alewife':
        sidewalk = ground.buffer(2.8).difference(ground.buffer(.22)).intersection(REGION)
        count += gltf.add('Inferred sidewalk bands', triangulate(sidewalk, height, .13), 1)
        if TERRAIN is not None:
            outside = ground.buffer(2.8).intersection(REGION)
            count += gltf.add('Inferred sidewalk foundation edges', vertical_walls(outside, height, -.3, .13, parts), 1)
    count += gltf.add('Urban curb tops', triangulate(curb, height, .13), 1)
    count += gltf.add('Urban curb faces', vertical_walls(curb, height, -.3 if TERRAIN is not None else 0, .13, parts), 1)
    for road in roads:
        if not road['bridge']:
            continue
        own_height = Heights([road])
        rail_height = .85 if road['tags']['highway'] != 'cycleway' else .18
        width = .16 if rail_height > .5 else .10
        rail = unary_union([road[side].buffer(width, cap_style=2)
                            for side in ('left_edge', 'right_edge')]).intersection(REGION)
        count += gltf.add(f'Bridge {road["id"]} edge tops', triangulate(rail, own_height, rail_height), 1)
        count += gltf.add(f'Bridge {road["id"]} edge faces',
                          vertical_walls(rail, own_height, 0, rail_height, parts), 1)
        if TERRAIN is None and road['tags']['highway'] in ('trunk', 'motorway'):
            for endpoint in (0, -1):
                left = np.array(road['left_edge'].coords[endpoint])
                right = np.array(road['right_edge'].coords[endpoint])
                support = LineString([left, right]).buffer(.4, cap_style=2).intersection(REGION)
                count += gltf.add(f'Bridge {road["id"]} abutment cap {endpoint}',
                                  triangulate(support, own_height, -.7), 1)
                count += gltf.add(f'Bridge {road["id"]} abutment faces {endpoint}',
                                  vertical_walls(support, own_height,
                                                 -road['profile_targets']['highway']-1, -.7, parts), 1)
    return count


def read_crossings(path, roads, xy):
    """Retain crossing tags and associated road IDs; distinguish explicit absence of paint from missing style data."""
    with gzip.open(path, 'rb') as stream:
        root = ET.parse(stream).getroot()
    records = []
    for element in root.findall('node'):
        tags = {t.get('k'): t.get('v') for t in element.findall('tag')}
        node = int(element.get('id'))
        if tags.get('highway') != 'crossing' or not REGION.covers(Point(xy[node])):
            continue
        associated = [r for r in roads if node in r['refs'] and r['tags']['highway'] != 'cycleway']
        legacy = {'unmarked': 'no', 'zebra': 'zebra', 'marked': 'yes'}
        style = tags.get('crossing:markings', legacy.get(tags.get('crossing'), 'unknown'))
        records.append({'node': node, 'tags': tags, 'roads': [r['id'] for r in associated], 'style': style,
                        'style_basis': ('crossing:markings' if 'crossing:markings' in tags else 'crossing tag'),
                        'rendered': False, 'position': xy[node]})
    return records


def crossing_section(record, by_id):
    """Join a crossing's compatible degree-two way split; reject branches, differing widths/levels, and sharp bends."""
    roads = [by_id[wid] for wid in record['roads']]
    if len(roads) == 1:
        return roads[0]
    if len(roads) != 2 or any(record['node'] not in (r['refs'][0], r['refs'][-1]) for r in roads):
        return None
    a, b = roads
    if (a['bridge'] != b['bridge'] or a['tunnel'] != b['tunnel']
            or a['tags'].get('layer', '0') != b['tags'].get('layer', '0')
            or abs(a['width']-b['width']) > .01 or a['lanes'] != b['lanes']):
        return None
    outward = [list(r['line'].coords) if r['refs'][0] == record['node'] else list(r['line'].coords)[::-1]
               for r in roads]
    vectors = [np.asarray(line[1])-line[0] for line in outward]
    vectors = [v/max(np.linalg.norm(v), 1e-9) for v in vectors]
    if float(vectors[0] @ vectors[1]) > -math.cos(math.radians(25)):
        return None
    record['alignment_basis'] = 'INFERRED section through compatible source-way split; both source IDs retained'
    return dict(a, line=LineString(outward[0][::-1]+outward[1][1:]), surface=a['surface'].union(b['surface']))


def crossing_markings(records, roads, pavement, height, geometry_only=False):
    """Return crossing meshes or planar paint polygons; geometry-only mode needs no pavement mesh or height sampler."""
    patches, exclusions, stop_exclusions = {'white': [], 'yellow': []}, [], []
    by_id = {r['id']: r for r in roads}
    support_index = None
    for record in records:
        if not record['roads']:
            continue
        road = (crossing_section(record, by_id) if SCENE != 'alewife' else by_id[record['roads'][0]])
        if road is None:
            record['unresolved'] = 'Ambiguous carriageway association; no guessed crossing alignment'
            continue
        marked = record['style'] in ('zebra', 'lines', 'yes', 'dots', 'dashes')
        if not marked and SCENE == 'alewife':
            continue
        if road['bridge'] or road['tunnel']:
            continue
        point = Point(record['position'])
        s = road['line'].project(point)
        before = road['line'].interpolate(max(0, s-1))
        after = road['line'].interpolate(min(road['line'].length, s+1))
        tangent = np.array([after.x-before.x, after.y-before.y])
        tangent /= np.linalg.norm(tangent)
        normal = np.array([-tangent[1], tangent[0]])
        center = np.array(record['position'])
        half = road['width']/2-.25
        shapes = []
        style = record['style']
        if style == 'yes':
            style = SCENES[SCENE].get('unspecified_crossing_style', 'lines')
            record['pattern_basis'] = 'INFERRED regional pattern; source confirms paint presence only'
        else:
            record['pattern_basis'] = 'SOURCE marking style; stripe/stud dimensions and alignment inferred'
        colour = record['tags'].get('crossing:markings:colour', SCENES[SCENE].get('crossing_colour', 'white'))
        record['colour_basis'] = ('SOURCE crossing:markings:colour' if 'crossing:markings:colour' in record['tags']
                                  else 'INFERRED scene locale default')
        if colour not in patches:
            record['unresolved'] = 'Unsupported crossing paint colour; passage retained without substituted paint'
            marked = False
        # Ambiguous marked crossings get simple paired lines; the recorded inference remains visible.
        stripes = [(v, min(v+.45, half), -1.6, 1.6) for v in np.arange(-half, half, .9)]
        if style != 'zebra':
            stripes = [(-half, half, -1.6, -1.4), (-half, half, 1.4, 1.6)]
        if style in ('dots', 'dashes'):
            length, period = (.15, 1.0) if style == 'dots' else (.5, 1.0)
            stripes = [(v, min(v+length, half), y-.075, y+.075)
                       for v in np.arange(-half, half, period) for y in (-1.5, 1.5)]
        for left, right, back, front in stripes:
            shapes.append(Polygon([center+normal*x+tangent*y
                                   for x, y in [(left, back), (right, back), (right, front), (left, front)]]))
        footprint = unary_union(shapes).intersection(road['surface'])
        if marked:
            if geometry_only:
                patches[colour].append(footprint)
            else:
                if support_index is None:
                    support_index = shapely.STRtree(shapely.polygons(pavement[:, :, :2]))
                patches[colour].append(drape_paint(triangulate(footprint, height), pavement, support_index))
        exclusions.append(point.buffer(road['width']/2+2))
        # Keep stop bars clear of the crossing's full passage, without the broader lane-paint suppression disk.
        passage = Polygon([center+normal*x+tangent*y
                           for x, y in [(-half, -1.6), (half, -1.6), (half, 1.6), (-half, 1.6)]])
        stop_exclusions.append(passage.buffer(.5))
        record['passage_reserved'] = True
        record['rendered'] = marked
        record['render_style'] = 'unpainted' if not marked else 'paired lines' if style == 'lines' else style
        record['render_colour'] = colour if marked else None
    if geometry_only:
        return {colour: unary_union(shapes) for colour, shapes in patches.items()}, \
            unary_union(exclusions), unary_union(stop_exclusions)
    meshes = {colour: np.concatenate(shapes) if shapes else np.empty((0, 3, 3)) for colour, shapes in patches.items()}
    return meshes, unary_union(exclusions), unary_union(stop_exclusions)


def surface_views(triangles):
    """View the steepest significant pavement face; ignore faces below one square decimeter."""
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    grades = np.linalg.norm(normals[:, :2], axis=1)/np.maximum(abs(normals[:, 2]), 1e-12)
    grades[abs(normals[:, 2]) <= .02] = 0
    index = int(np.argmax(grades))
    if grades[index] < .04:
        return {}
    point = triangles[index].mean(axis=0)
    lon, lat = TO_GEO.transform(point[0], point[1])
    normal = normals[index]
    heading = float(np.degrees(np.arctan2(-normal[0]/normal[2], -normal[1]/normal[2]))+90)
    return {'surface-detail': (lon, lat, heading, -55, 85, float(point[2])),
            'surface-street': (lon, lat, heading, -8, 45, float(point[2]))}


def crossing_views(records, height):
    """Select reproducible close oblique views of supported dotted and yellow crossings away from the render boundary."""
    views = {}
    for name, field, value in [('crossing-dots', 'render_style', 'dots'), ('crossing-yellow', 'render_colour', 'yellow')]:
        candidates = [r for r in records if r['rendered'] and r.get(field) == value
                      and REGION.boundary.distance(Point(r['position'])) > 25]
        if not candidates:
            continue
        target = max(candidates, key=lambda r: (sum(math.dist(r['position'], q['position']) < 30
                                                    for q in records if q['rendered']),
                                                 -math.hypot(*r['position']), -r['node']))
        point = target['position']
        lon, lat = TO_GEO.transform(*point)
        views[name] = (lon, lat, -15, -55, 60, float(height(*point)[0]))
    return views


def close_views(roads, height, crossings):
    """Create reproducible pedestrian-height viewpoints using the same fitted surface as the road mesh."""
    views = {}
    bike = next((r for r in roads if r['id'] == 1302609756), None)
    if bike is None:
        return views
    bridge = next(r for r in roads if r['id'] == 39452967)
    crossing = bike['line'].intersection(bridge['line'])
    targets = [('street', bike, crossing)]
    zebra = next((c for c in crossings if c['rendered'] and c['style'] == 'zebra'), None)
    if zebra:
        road = next(r for r in roads if r['id'] == zebra['roads'][0])
        targets.append(('crosswalk', road, Point(zebra['position'])))
    for name, road, target in targets:
        station = road['line'].project(target)
        eye = road['line'].interpolate(max(0, station-24))
        if eye.distance(target) < 12 or not REGION.covers(eye):
            eye = road['line'].interpolate(min(road['line'].length, station+24))
        eye_z = float(height(eye.x, eye.y)[0])+1.7
        target_z = float(height(target.x, target.y)[0])+1.1
        dx, dy, dz = target.x-eye.x, target.y-eye.y, target_z-eye_z
        heading = np.degrees(np.arctan2(dx, dy))
        pitch = np.degrees(np.arctan2(dz, np.hypot(dx, dy)))
        lon, lat = TO_GEO.transform(target.x, target.y)
        views[name] = (lon, lat, heading, pitch, np.linalg.norm([dx, dy, dz]), target_z)
    return views


def write_earth(output, mode, sse, viewpoint='overview', extra_views=None):
    """Write a runnable, network-independent osgEarth map variant with explicit terrain controls."""
    if TERRAIN is not None:
        from terrain_maps import write_real_earth
        return write_real_earth(output, mode, sse, viewpoint, extra_views, SCENE, ORIGIN, base_height(0, 0))
    replacement = mode == 'replacement'
    constraint = '''<TerrainConstraint name="Pavement boundary: replace terrain" features="road-boundary"
        min_level="14" has_elevation="true" remove_interior="true"/>''' if replacement else ''
    views = {'overview': (-71.1424, 42.3978, -30, -48, 950, 24),
             'bridge': (-71.14387, 42.39826, 70, -12, 120, 24),
             'junction': (-71.14155, 42.39745, -15, -18, 160, 24),
             'far': (-71.1424, 42.3978, -20, -70, 7000, 24)}
    if SCENE != 'alewife':
        views = {'overview': (*ORIGIN, -18, -65, 650, 20),
                 'junction': (*ORIGIN, 0, -65, 150, 20),
                 'far': (*ORIGIN, -20, -70, 7000, 20)}
    views.update(extra_views or {})
    order = [viewpoint]
    order += [name for name in views if name not in order]
    view_xml = []
    for name in order:
        if name not in views:
            continue
        lon, lat, heading, pitch, distance, altitude = views[name]
        view_xml.append(f'<viewpoint name="{name}" long="{lon}" lat="{lat}" height="{altitude}" '
                        f'heading="{heading}" pitch="{pitch}" range="{distance}"/>')
    viewpoints = '\n        '.join(view_xml)
    map_text = f'''<map name="Prestige Roads / {SCENE.title()} / {mode} / SSE {sse}">
    <options><terrain tile_size="33" screen_space_error="{sse}" morph_terrain="true"
        gpu_tessellation="false" skirt_ratio="0.005"/></options>
    <sky driver="simple" hours="17" ambient="0.35"/>
    <GDALImage name="Controlled ground colors"><url>ground.tif</url>
        <attribution>OpenStreetMap contributors / ODbL. Terrain, heights, and paint are experimental.</attribution>
    </GDALImage>
    <GDALElevation name="CONTROLLED heights, not survey data"><url>terrain.tif</url></GDALElevation>
    <OGRFeatures name="road-boundary"><url>boundary.geojson</url></OGRFeatures>
    {constraint}
    <Model name="Compiled OSM pavement and structures" lighting="true">
        <url>roads.gltf</url><location long="{ORIGIN[0]}" lat="{ORIGIN[1]}" alt="0" mode="absolute"/>
    </Model>
    <viewpoints home="0" time="3">
        {viewpoints}
    </viewpoints>
</map>
'''
    (output/f'{mode}-sse{sse}-{viewpoint}.earth').write_text(map_text, encoding='utf-8')


def main():
    """Compile the retained snapshot, run geometric checks, and write maps plus a machine-readable experiment report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=SCENES, default='alewife')
    parser.add_argument('--source', type=Path)
    parser.add_argument('--terrain', choices=('synthetic', 'readymap'), default='synthetic')
    parser.add_argument('--surface', choices=('fit', 'drape', 'design'), default='fit')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--survey', action='store_true', help='Write discovery diagnostics without meshes or maps')
    args = parser.parse_args()
    global TERRAIN, SURFACE_POLICY
    configure_scene(args.scene)
    TERRAIN, SURFACE_POLICY = None, args.surface
    if args.terrain == 'readymap':
        from terrain import ReadyMap
        TERRAIN = ReadyMap(Path(__file__).parent/'source'/(SCENE+'-readymap'), TO_GEO)
    args.source = args.source or Path(__file__).parent/'source'/(SCENES[SCENE].get('source', SCENE)+'.osm.gz')
    default_name = '' if SCENE == 'alewife' else SCENE
    if TERRAIN is not None:
        default_name = SCENE+'-readymap-'+SURFACE_POLICY
    output = (args.output or Path(__file__).parent/'output'/default_name).resolve()
    output.mkdir(parents=True, exist_ok=True)
    start = time.perf_counter()
    roads, xy, flags, relations = read_source(args.source)
    control_records = read_controls(args.source, roads, xy, REGION)
    graph, lifts = solve_profiles(roads, xy)
    if TERRAIN is not None and SURFACE_POLICY == 'design':
        from vertical import solve_vertical
        lifts.update(solve_vertical(roads, xy, TERRAIN, REGION, output))
    transitions, transition_diagnostics = discover_transitions(roads, REGION)
    ground_roads = [r for r in roads if not r['bridge'] and not r['tunnel']]
    degree = {n: len({v for v, _ in adjacent}) for n, adjacent in graph.items()}
    ground_nodes = {node for road in ground_roads for node in road['refs']}
    junction_nodes = [n for n, d in degree.items() if d >= 3
                      and (TERRAIN is None or SURFACE_POLICY != 'design' or n in ground_nodes)]
    junction_samples = np.array([(*xy[n], lifts[n]) for n in junction_nodes])
    height = Heights(ground_roads, junction_samples,
                     junction_nodes=junction_nodes if TERRAIN is not None and SURFACE_POLICY == 'design' else None)
    ground = unary_union([r['surface'] for r in ground_roads])
    # Fillet concave corners while restoring every source footprint island instead of filling narrow holes.
    islands = unary_union([Polygon(ring) for p in parts(ground) for ring in p.interiors])
    ground = ground.buffer(2).buffer(-2).difference(islands).intersection(REGION)
    if SCENE != 'alewife':
        from urban import discover_urban
        # Four-meter inferred curb returns round block corners; protect the remaining block interiors.
        raw_ground = unary_union([r['surface'] for r in ground_roads])
        ground = (rounded_ground_components(raw_ground) if TERRAIN is not None and SURFACE_POLICY == 'design'
                  else raw_ground.buffer(4).buffer(-4)).intersection(REGION)
        islands = unary_union([Polygon(ring) for p in parts(ground) for ring in p.interiors])
        layouts, ground, discovery = discover_urban(roads, xy, relations, control_records, ground, islands, REGION,
                                                   SCENES[SCENE]['driving_side'], SCENES[SCENE]['center_line'])
    else:
        layouts, ground, discovery = discover_layouts(roads, xy, relations, control_records, ground, islands, REGION)
    if args.survey:
        write_json(output/'discovery-survey.json', {
            'scene': SCENE, 'source_sha256': hashlib.sha256(args.source.read_bytes()).hexdigest(),
            'bbox': BBOX, 'road_ways': len(roads), 'width_flags': flags,
            'junctions': discovery, 'transitions': transition_diagnostics,
            'layouts': [{'node': item['node'], 'kind': item['kind'], 'arms': item.get('arm_count'),
                         'audit': audit_layout(item, ground)} for item in layouts]})
        print(f'{SCENE}: {len(roads)} roads; {len(layouts)}/{len(discovery)} junction layouts', flush=True)
        return
    if not layouts:
        raise ValueError('No supported junctions discovered; inspect source completeness and layout diagnostics')
    if any(t['domain'].intersection(j['domain']).area > .001 for t in transitions for j in layouts):
        raise ValueError('Junction and taper ownership overlap')
    write_json(output/'layout-discovery.json', {'junctions': discovery, 'transitions': transition_diagnostics})
    # Collapse sub-millimeter buffer/overlay debris before meshing; original OSM coordinates remain unchanged.
    ground = shapely.set_precision(ground, .001)
    ground = shapely.segmentize(ground, 3)
    # Urban paint is finalized after crossings and controls. Its export and audit occur once, below.
    intersection_audits = ({item['name']: export_layout(item, output, TO_GEO, write_json, ground) for item in layouts}
                           if SCENE == 'alewife' else {})
    transition_audits = {item['name']: export_transition(item, output, ground, TO_GEO, write_json) for item in transitions}
    if TERRAIN is not None and SURFACE_POLICY == 'drape':
        height = lambda x, y: (base_height(x, y), np.zeros(np.broadcast(x, y).shape))
    pavement = triangulate(ground, height, max_edge=4)
    constraints = bridge_constraints(roads, ground_roads, collars=SURFACE_POLICY != 'design' or TERRAIN is None)
    if TERRAIN is None:
        pavement, surface_audit = fit_pavement(pavement, base_height, constraints)
        height = MeshSurface(pavement)
    else:
        from terrain import ExactSurface, grade_report
        if SURFACE_POLICY in ('fit', 'design'):
            center_height = float(base_height(0, 0))
            reference = lambda x, y: np.full(np.broadcast(x, y).shape, center_height)
            try:
                pavement, surface_audit = fit_pavement(pavement, reference, constraints,
                                                     fidelity=.01 if SURFACE_POLICY == 'design' else .04,
                                                     curvature_m2=16 if SURFACE_POLICY == 'design' else 4,
                                                     limit_grade=SURFACE_POLICY == 'design')
            except ValueError as error:
                write_json(output/'surface-failure.json', {'stage': 'real-terrain pavement fit',
                           'reason': str(error), 'terrain_source': TERRAIN.metadata,
                           'policy': 'Do not relax the grade guard or export a rejected fit'})
                raise
        else:
            surface_audit = grade_report(pavement)
        height = ExactSurface(pavement)
    tri_area = validate_pavement_footprint(ground, pavement, output)
    gltf = GLTF(output)
    count = gltf.add('Ground pavement: shared junction surface', pavement, 0, smooth=True)
    triangles = {'pavement': count, 'bridge_top': 0, 'bridge_underside': 0, 'bridge_fascia': 0}
    for road in roads:
        if not road['bridge']:
            continue
        bridge_height = Heights([road])
        top = triangulate(road['surface'], bridge_height, max_edge=4)
        road['mesh'] = top
        triangles['bridge_top'] += gltf.add(f'Bridge {road["id"]} pavement', top, 0, smooth=True)
        triangles['bridge_underside'] += gltf.add(f'Bridge {road["id"]} underside', top[:, [0, 2, 1], :]-[0, 0, .7], 1)
        walls = []
        for polygon in parts(shapely.segmentize(road['surface'], 3)):
            coords = np.array(shapely.orient_polygons(polygon).exterior.coords)[:, :2]
            z, _ = bridge_height(coords[:, 0], coords[:, 1])
            for i in range(len(coords)-1):
                a, b = np.r_[coords[i], z[i]], np.r_[coords[i+1], z[i+1]]
                c, d = b-[0, 0, .7], a-[0, 0, .7]
                walls.extend([[a, c, b], [a, d, c]])
        triangles['bridge_fascia'] += gltf.add(f'Bridge {road["id"]} deck fascia', np.array(walls), 1)
    junctions = unary_union([Point(xy[n]).buffer(11) for n, d in degree.items() if d >= 3])
    crossing_records = read_crossings(args.source, roads, xy)
    crossing_paint, crossing_exclusions, crossing_passages = crossing_markings(crossing_records, roads, pavement, height)
    if SCENE != 'alewife':
        from urban import urban_stop_bars
        stop_paint, stop_exclusions = urban_stop_bars(control_records, roads, layouts, ground, crossing_passages,
                                                      SCENES[SCENE]['driving_side'])
    else:
        stop_paint, stop_exclusions = build_stop_bars(control_records, roads, layouts, ground, crossing_passages)
    stop_count = sum(len(c.get('bars', [None] if c['rendered'] else [])) for c in control_records)
    if SCENE != 'alewife':
        from urban import approach_paint
        for item in layouts:
            approach_paint(item, crossing_exclusions.union(stop_exclusions))
        intersection_audits = {item['name']: export_layout(item, output, TO_GEO, write_json, ground) for item in layouts}
    resolved_domains = unary_union([item['domain'] for item in layouts+transitions])
    exclusions = unary_union([junctions, crossing_exclusions, resolved_domains, stop_exclusions])
    white, yellow = road_markings(roads, exclusions, pavement, height, ground)
    triangles['paint'] = gltf.add('Illustrative white paint; omitted inside junctions', white, 2)
    triangles['paint'] += gltf.add('Illustrative yellow paint; omitted inside junctions', yellow, 3)
    triangles['crosswalks'] = gltf.add('Source-tagged white crosswalk markings', crossing_paint['white'], 2)
    triangles['crosswalks'] += gltf.add('Source-tagged yellow crosswalk markings', crossing_paint['yellow'], 3)
    resolved_paint = unary_union([item['paint'] for item in layouts+transitions])
    edge_domains = unary_union([item['domain'] for item in layouts if item['kind'] not in ('two_way', 'urban')]
                              +[item['domain'] for item in transitions])
    edge_guidance = ground.buffer(-.30).boundary.intersection(edge_domains).buffer(.07)
    left_edges = unary_union([item['left_edge'].buffer(.8) for resolved in layouts
                              for item in resolved['connectors'] if item['from_lane'] == 1
                              and resolved['kind'] not in ('two_way', 'urban')]
                             +[t['left_edge'].buffer(.8) for t in transitions])
    yellow_edges = edge_guidance.intersection(left_edges).difference(crossing_exclusions.union(stop_exclusions))
    yellow_edges = yellow_edges.union(unary_union([item.get('yellow_paint', Polygon()) for item in layouts]))
    yellow_edges = yellow_edges.difference(crossing_exclusions.union(stop_exclusions))
    resolved_paint = resolved_paint.union(edge_guidance.difference(left_edges))
    resolved_paint = resolved_paint.difference(crossing_exclusions.union(stop_exclusions))
    guidance = resolved_paint.union(yellow_edges)
    marking_audit = {'guidance_in_crosswalk_passages_m2': guidance.intersection(crossing_passages).area,
                     'guidance_in_stop_bar_exclusions_m2': guidance.intersection(stop_exclusions).area}
    if max(marking_audit.values()) > .001:
        raise ValueError('Resolved guidance intrudes into a crosswalk or stop-bar exclusion')
    write_json(output/'marking-audit.json', marking_audit)
    triangles['intersection_paint'] = gltf.add('Resolved intersection guidance and source through arrows',
                                              drape_paint(triangulate(resolved_paint, height), pavement), 2)
    triangles['intersection_paint'] += gltf.add('US-style left pavement edges at resolved junctions',
                                               drape_paint(triangulate(yellow_edges, height), pavement), 3)
    triangles['stop_bars'] = gltf.add('Inferred stop bars at mapped directional traffic controls',
                                     drape_paint(triangulate(stop_paint, height), pavement), 2)
    triangles['details'] = road_details(gltf, roads, ground, height)
    triangles['total'] = sum(triangles.values())
    clearances = []
    for road in roads:
        if road['bridge'] and road['tags']['highway'] in ('trunk', 'motorway'):
            for lower in ground_roads:
                if lower['tags']['highway'] == 'cycleway' and road['surface'].intersects(lower['surface']):
                    if not set(road['refs']) & set(lower['refs']):
                        clearance, count = minimum_clearance(road['mesh'], pavement, lower['surface'], .7)
                        clearances.append({'upper': road['id'], 'lower': lower['id'], 'minimum_m': clearance,
                                           'required_m': 5.0 if TERRAIN is None else None,
                                           'triangle_overlaps_checked': count})
                        if TERRAIN is None and clearance < 5:
                            raise ValueError(f'Compiled surface violates the 5 m experimental clearance: {clearance}')
    terrain_bridges = None
    if TERRAIN is not None:
        from terrain import bridge_audit
        terrain_bridges = bridge_audit(roads, pavement)
        write_json(output/'bridge-terrain-audit.json', terrain_bridges)
    gltf.save()
    # Numerical texture content has a known 4 m repeat; no baked road layout or light/shadow.
    rng = np.random.default_rng(23)
    grain = np.clip(rng.normal(77, 8, (512, 512)), 48, 112).astype('uint8')
    Image.fromarray(np.stack([grain, grain, grain], axis=-1)).save(output/'asphalt.png')
    boundary = boundary_geojson(ground, height)
    write_json(output/'boundary.geojson', boundary)
    if TERRAIN is None:
        write_terrain(output, ground, height, roads)
    else:
        from terrain import write_overlay
        write_overlay(output, REGION, TO_GEO, TERRAIN, height, ground)
    extra_views = close_views(roads, height, crossing_records)
    if TERRAIN is not None:
        from terrain import comparison_views
        extra_views.update(comparison_views(output, pavement, TERRAIN, REGION, TO_GEO))
    extra_views.update(surface_views(pavement))
    extra_views.update(contraflow_control_views(control_records, height, TO_GEO, REGION))
    # Familiar camera names are presentation aliases only; layout selection and mouth geometry have no ID table.
    aliases = {2918219392: 'intersection', 13505628685: 'west-merge',
               13433510068: 'south-merge', 61321054: 'east-crossing'}
    layout = max(layouts, key=lambda item: (item['arrow_count'], len(item['approaches'])))
    center = layout['position']
    lon, lat = TO_GEO.transform(*center)
    altitude = float(height(*center)[0])
    extra_views['intersection'] = (lon, lat, 0, -65, 112, altitude)
    incoming = next(a for a in layout['approaches'] if a['incoming'])
    eye = incoming['outward'].interpolate(35)
    dx, dy = center-np.array(eye.coords[0])
    eye_z = float(height(eye.x, eye.y)[0])+1.7
    distance = np.hypot(dx, dy)
    extra_views['intersection-street'] = (lon, lat, np.degrees(np.arctan2(dx, dy)),
                                         np.degrees(np.arctan2(altitude+1.1-eye_z, distance)),
                                         np.hypot(distance, altitude+1.1-eye_z), altitude+1.1)
    for item in layouts:
        focus = item['domain'].centroid
        longitude, latitude = TO_GEO.transform(focus.x, focus.y)
        span = max(item['domain'].bounds[2]-item['domain'].bounds[0],
                   item['domain'].bounds[3]-item['domain'].bounds[1])
        extra_views[item['name']] = (longitude, latitude, 0, -65, max(70, span*1.6),
                                     float(height(focus.x, focus.y)[0]))
        if item['node'] in aliases:
            extra_views[aliases[item['node']]] = extra_views[item['name']]
    two_way = [item for item in layouts if item['kind'] == 'two_way']
    if two_way:
        selected = max(two_way, key=lambda item: REGION.boundary.distance(item['domain'].centroid))
        extra_views['two-way'] = extra_views[selected['name']]
        incoming = max((a for a in selected['approaches'] if a['incoming']), key=lambda a: a['outward'].length)
        eye = incoming['outward'].interpolate(incoming['station']+12)
        eye = Point(np.array(eye.coords[0])-incoming['left']*incoming['pitch']/2)
        target = Point(selected['position'])
        z = float(height(target.x, target.y)[0])+1.1
        delta = np.array([target.x-eye.x, target.y-eye.y, z-float(height(eye.x, eye.y)[0])-1.7])
        lon, lat = TO_GEO.transform(target.x, target.y)
        extra_views['two-way-street'] = (lon, lat, np.degrees(np.arctan2(delta[0], delta[1])),
                                         np.degrees(np.arctan2(delta[2], np.hypot(*delta[:2]))),
                                         np.linalg.norm(delta), z)
    for item in transitions:
        focus = item['route'].interpolate(.5, normalized=True)
        lon, lat = TO_GEO.transform(focus.x, focus.y)
        extra_views['lane-drop'] = (lon, lat, -70, -65, item['route'].length*2.5, float(height(focus.x, focus.y)[0]))
        target = item['route'].interpolate(item['taper_length_m']*.65)
        eye = item['route'].interpolate(item['taper_length_m']*.35)
        dx, dy = target.x-eye.x, target.y-eye.y
        z = float(height(target.x, target.y)[0])+1.1
        dz = z-float(height(eye.x, eye.y)[0])-1.7
        lon, lat = TO_GEO.transform(target.x, target.y)
        extra_views['lane-drop-street'] = (lon, lat, np.degrees(np.arctan2(dx, dy)),
                                           np.degrees(np.arctan2(dz, np.hypot(dx, dy))),
                                           np.linalg.norm([dx, dy, dz]), z)
    if SCENE != 'alewife':
        from urban import urban_views
        extra_views.update(urban_views(layouts, height, TO_GEO))
        extra_views.update(crossing_views(crossing_records, height))
    base_views = ('overview', 'bridge', 'junction', 'far') if SCENE == 'alewife' else ('overview', 'junction', 'far')
    modes = ('replacement', 'overlay') if TERRAIN is None else ('raw', 'graded')
    for mode in modes:
        for sse in (2, 8, 32):
            for viewpoint in (*base_views, *extra_views):
                write_earth(output, mode, sse, viewpoint, extra_views)
    write_json(output/'viewpoints.json', {'views': [*base_views, *extra_views],
                                        'modes': list(modes), 'terrain_sse': [2, 8, 32]})
    generated = ('line', 'surface', 'samples', 'heights', 'mesh', 'left_edge', 'right_edge')
    records = [{k: v for k, v in r.items() if k not in generated} for r in roads]
    write_json(output/'source-records.json', {'roads': records, 'relations': relations, 'width_overrides': flags,
                                            'crossings': crossing_records, 'traffic_controls': control_records})
    write_json(output/'traffic-controls.json', {
        'records': control_records,
        'assumptions': ['Directional signal nodes establish control evidence, not a surveyed stop-line shape.',
                        'Rendered bars use inferred 0.40 m solid strokes; signal phases and priority are unknown.',
                        'Placement follows the resolved lane surface; source positions and displacements are retained.'],
        'mapped_controls': len(control_records),
        'explicit_stop_lines': sum(c['explicit_stop_line'] for c in control_records),
        'rendered_stop_bars': stop_count})
    road_ids = {r['id'] for r in roads}
    unresolved = sorted({int(m['ref']) for r in relations for m in r['members']
                         if m['type'] == 'way' and int(m['ref']) not in road_ids})
    holes = sum(len(p.interiors) for p in parts(ground))
    # Report projected crossings that a coordinate-based graph would incorrectly turn into intersections.
    crossings = []
    for i, a in enumerate(roads):
        for b in roads[i+1:]:
            intersection = a['line'].intersection(b['line'])
            if intersection.geom_type != 'Point' or not REGION.covers(intersection):
                continue
            if set(a['refs']) & set(b['refs']):
                continue
            ha = float(Heights([a])(intersection.x, intersection.y)[0])
            hb = float(Heights([b])(intersection.x, intersection.y)[0])
            crossings.append({'ways': [a['id'], b['id']],
                              'height_difference_m': None if a['tunnel'] or b['tunnel'] else abs(ha-hb),
                              'connected': False, 'bridge': a['bridge'] or b['bridge'],
                              'tunnel_geometry_omitted': a['tunnel'] or b['tunnel']})
    report = {'scene': SCENE, 'source_sha256': hashlib.sha256(args.source.read_bytes()).hexdigest(), 'bbox': BBOX,
              'road_ways': len(roads), 'bridges': sum(r['bridge'] for r in roads),
              'missing_road_nodes': 0, 'junction_nodes': sum(d >= 3 for d in degree.values()),
              'junction_nodes_inside_crop': sum(d >= 3 and REGION.covers(Point(xy[n])) for n, d in degree.items()),
              'restriction_relations_retained': len(relations), 'relation_way_references_outside_subset': unresolved,
              'terrain_source': 'synthetic' if TERRAIN is None else TERRAIN.metadata,
              'road_render_offset_m': 0.0 if TERRAIN is None else .12,
              'surface_policy': 'synthetic' if TERRAIN is None else SURFACE_POLICY,
              'width_overrides': len(flags), 'pavement_area_m2': ground.area, 'preserved_holes': holes,
              'profile_policy': ('Controlled graph profiles with a curvature-regularized pavement fit' if TERRAIN is None
                                 else 'Inferred connected grade/clearance profiles' if SURFACE_POLICY == 'design'
                                 else ('ReadyMap centerline fit' if SURFACE_POLICY == 'fit' else 'Direct ReadyMap DEM drape')
                                 +'; endpoint-derived bridge decks are unverified'),
              'profile_seed_lifts_m': roads[0]['profile_targets'],
              'surface_fit': surface_audit,
              'resolved_intersections': intersection_audits,
              'lane_transitions': transition_audits,
              'layout_discovery': discovery,
              'directional_controls_retained': len(control_records),
              'stop_bars_rendered': stop_count,
              'deck_clearance_checks': clearances,
              'real_terrain_bridge_diagnostics': terrain_bridges,
              'crossings_tagged': len(crossing_records),
              'crosswalks_rendered': sum(c['rendered'] for c in crossing_records),
              'triangulated_area_error_m2': abs(tri_area-ground.area), 'triangles': triangles,
              'nonconnecting_projected_crossings': crossings, 'compile_seconds': time.perf_counter()-start,
              'limitations': [('Controlled elevations, not a survey' if TERRAIN is None else
                               'ReadyMap 90m source; inferred profiles and bridge decks are not surveyed'),
                              'No production lane movement solver',
                              'Illustrative paint; 18 mm surface offset, not a finished material overlay',
                              'Resident road model; terrain pages, road geometry does not yet page; no distant tile bake',
                              'Restriction members outside subset are retained but not solved',
                              'Experimental surface fit; '
                              'production crossfall and lane movement design remain open']}
    write_json(output/'metrics.json', report)
    (output/'surface-failure.json').unlink(missing_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(12, 6))
    for polygon in parts(ground):
        ax.fill(*polygon.exterior.xy, color='#44494b')
        for hole in polygon.interiors:
            ax.fill(*hole.xy, color='#dce2cf')
    for road in roads:
        if road['bridge']:
            for polygon in parts(road['surface']):
                ax.fill(*polygon.exterior.xy, color='#ce8847')
    ax.set_facecolor('#dce2cf')
    ax.set_aspect('equal')
    ax.set_title(f'Prestige Roads / {SCENE.title()} OSM geometry / orange = separate bridge surfaces')
    ax.set_xlabel('Local east (m)'); ax.set_ylabel('Local north (m)')
    fig.tight_layout(); fig.savefig(output/'plan.png', dpi=150); plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
