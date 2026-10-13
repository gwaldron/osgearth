"""Basic attribute-selected bridge members; pavement, profiles, paint and terrain remain compiler-owned."""

from collections import Counter, defaultdict

import numpy as np
import shapely
from shapely.geometry import LineString, MultiPoint, Point

from bridge_evidence import read_evidence, select_assets
from bridge_railings import quad, sweep
from terrain import ReadyMap


def member_box(a, b, width, depth):
    """Return a capped rectangular member with outward winding, including vertical columns; reject zero length."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    axis = b-a
    length = np.linalg.norm(axis)
    if length < 1e-6 or min(width, depth) <= 0 or not np.isfinite([*a, *b, width, depth]).all():
        raise ValueError('Invalid procedural bridge member')
    axis /= length
    reference = np.array([0., 0., 1.]) if abs(axis[2]) < .9 else np.array([0., 1., 0.])
    side = np.cross(axis, reference)
    side *= width/2/np.linalg.norm(side)
    up = np.cross(side/np.linalg.norm(side), axis)*depth/2
    offsets = np.array([-side-up, side-up, side+up, -side+up])
    rings = np.array([a+offsets, b+offsets])
    center = (a+b)/2
    triangles = quad(rings[0], center)+quad(rings[1], center)
    for k in range(4):
        j = (k+1) % 4
        triangles.extend(quad([rings[0, k], rings[0, j], rings[1, j], rings[1, k]], center))
    return np.asarray(triangles)


class AssetFrame:
    """A straight principal span with cross-sections taken from actual deck coverage, including divided decks."""

    def __init__(self, asset, network, owners):
        """Reject broad branching/curved alignments; a procedural family must not span an interchange as one bridge."""
        self.asset, self.network, self.owners = asset, network, owners
        lines = [network.roads[i]['line'] for i in asset['indices']]
        xy = np.concatenate([shapely.get_coordinates(shapely.segmentize(line, 4)) for line in lines])
        self.origin = xy.mean(axis=0)
        _, _, axes = np.linalg.svd(xy-self.origin, full_matrices=False)
        self.forward = axes[0]
        if np.dot(self.forward, np.asarray(lines[0].coords)[-1]-np.asarray(lines[0].coords)[0]) < 0:
            self.forward *= -1
        self.side = np.array([-self.forward[1], self.forward[0]])
        station = (xy-self.origin) @ self.forward
        self.origin += self.forward*station.min()
        self.length = float(station.max()-station.min())
        lateral = (xy-self.origin) @ self.side
        self.radius = float(np.max(abs(lateral))+max(network.roads[i]['width'] for i in asset['indices'])+5)
        if self.length < 5 or np.ptp(lateral) > max(20, self.length*.22):
            raise ValueError('Span is too short, curved, or branched for the basic family templates')

    def sections(self, station, pavement=None):
        """Intersect the physical pavement with a transverse line; do not fill median gaps or roadway holes."""
        center = self.origin+self.forward*station
        line = LineString([center-self.side*self.radius, center+self.side*self.radius])
        sections = []
        for part in shapely.get_parts(line.intersection(self.asset['pavement'] if pavement is None else pavement)):
            if part.geom_type == 'LineString' and part.length > .5:
                coords = np.asarray(part.coords)
                values = (coords-center) @ self.side
                sections.append((float(values.min()), float(values.max())))
        return sorted(sections)

    def xyz(self, station, lateral, offset=0):
        """Attach to the nearest owned exported deck triangle; side members may extend beyond the slab edge."""
        xy = self.origin+self.forward*station+self.side*lateral
        owner = min((self.owners[i] for i in self.asset['indices']), key=lambda d: d.pavement.distance(Point(xy)))
        z = float(owner.exported_surface(np.array([xy[0]]), np.array([xy[1]]))[0][0])
        return np.r_[xy, z+offset]

    def edge(self, station, side, offset=0):
        """Find the outer edge on a given side; a missing cross-section is an explicit unsupported layout."""
        sections = self.sections(station)
        if not sections:
            raise ValueError('No pavement at a structural cross-section')
        lateral = sections[0][0]-.35 if side < 0 else sections[-1][1]+.35
        return self.xyz(station, lateral, offset)

    def underside(self, station, lateral):
        """Read the existing slab thickness, including tapered landings, instead of assuming a new constant slab."""
        point = self.xyz(station, lateral)
        owner = min((self.owners[i] for i in self.asset['indices']),
            key=lambda d: d.pavement.distance(Point(point[:2])))
        point[2] -= float(owner.thickness(point[None, :2])[0])
        return point


class Members:
    """Collect semantic members with conservative corridor checks before batching render geometry."""

    def __init__(self, asset, network, evidence, ground):
        """Reference the unchanged source/profile data and a retained base-elevation sampler."""
        self.asset, self.network, self.evidence, self.ground = asset, network, evidence, ground
        self.records, self.rejected, self.triangles = [], [], defaultdict(list)
        self.tree = shapely.STRtree([r['surface'] for r in network.roads])
        self.obstacles = shapely.STRtree([o['geometry'] for o in evidence['obstacles']])

    def add(self, kind, a, b, width, depth, material='steel', foundation=False, mapped=None, triangles=None):
        """Validate actual box or caller-supplied sweep triangles against road and foundation exclusion corridors."""
        swept = triangles is not None
        triangles = member_box(a, b, width, depth) if triangles is None else np.asarray(triangles)
        footprint = MultiPoint(triangles[:, :, :2].reshape(-1, 2)).convex_hull
        zmin, zmax = float(triangles[:, :, 2].min()), float(triangles[:, :, 2].max())
        reason = None
        for index in self.tree.query(footprint.buffer(.5), predicate='intersects'):
            index = int(index)
            if index in self.asset['indices']:
                continue
            road = self.network.roads[index]
            part = footprint.buffer(.5).intersection(road['surface'])
            xy = shapely.get_coordinates(part)
            if not len(xy):
                continue
            z = self.network.height(index, *xy.T)[0]
            if zmin < float(np.max(z))+5.25 and zmax > float(np.min(z))-.8:
                reason = 'Conflicts with road clearance corridor: way/'+str(road['id'])
                break
        if foundation:
            for index in self.obstacles.query(footprint.buffer(.5), predicate='intersects'):
                obstacle = self.evidence['obstacles'][int(index)]
                if obstacle['kind'] == 'water' and mapped is not None:
                    continue
                reason = 'Foundation conflicts with '+obstacle['kind']+' '+obstacle['source']
                break
        if reason:
            self.rejected.append({'kind': kind, 'center_xy': footprint.centroid.coords[0], 'reason': reason})
            return False
        record = {'kind': kind, 'a': list(a), 'b': list(b), 'width_m': width, 'depth_m': depth,
            'material': material, 'geometry_status': 'inferred', 'foundation': foundation,
            'mapped_support': mapped, 'footprint': footprint, 'z_min_m': zmin, 'z_max_m': zmax}
        if swept:
            record['section_basis'] = 'Vertical depth; shared transverse miter sections'
        self.records.append(record)
        self.triangles[material].extend(triangles)
        return True

    def bearing(self, frame, station, lateral, kind='pier', width=1., mapped=None):
        """Build a support up to the solved underside, with a half-meter illustrative embed into retained base DEM."""
        top = frame.underside(station, lateral)+[0, 0, .03]
        try:
            base = float(self.ground(np.array([top[0]]), np.array([top[1]]))[0])
        except ValueError:
            self.rejected.append({'kind': kind, 'reason': 'Outside retained terrain coverage'})
            return False
        if top[2]-base < .25:
            self.rejected.append({'kind': kind, 'reason': 'No resolved visible support height above coarse base DEM'})
            return False
        return self.add(kind, [top[0], top[1], base-.5], top, width, width, 'concrete', True, mapped)


def build_beam(frame, members):
    """Place simple piers and end bearings with bounded station searches, keeping rejected corridors open."""
    length = frame.length
    stations = np.linspace(2, length-2, max(2, int(np.ceil(length/28))+1))
    for n, station in enumerate(stations):
        endpoint = n in (0, len(stations)-1)
        for delta in (0, -2, 2, -4, 4, -6, 6):
            candidate = station+delta
            if candidate < 1 or candidate > length-1 or (endpoint and delta != 0):
                continue
            sections = frame.sections(candidate)
            accepted = False
            for lo, hi in sections:
                accepted |= members.bearing(frame, candidate, (lo+hi)/2,
                    'end bearing' if endpoint else 'pier', min(1.5, (hi-lo)*.24))
            if accepted:
                break


def build_edge_girders(frame, members, material):
    """Seat continuous side girders in every slab edge, including divided-deck medians; fail as one replacement."""
    footprints, intervals = [], []
    # Divided carriageways have staggered caps. Trim only bounded end transitions when a connected road conflicts.
    for pavement in shapely.get_parts(frame.asset['pavement']):
        if pavement.geom_type != 'Polygon' or pavement.interiors:
            raise ValueError('Branched or perforated deck needs a resolved structural edge layout')
        extent = (shapely.get_coordinates(pavement)-frame.origin) @ frame.forward
        low, high = float(extent.min()), float(extent.max())
        start, end = low+1, high-1
        for attempt in range(12):
            record_count = len(members.records)
            sizes = {key: len(value) for key, value in members.triangles.items()}
            stations = np.linspace(start, end, max(3, int(np.ceil((end-start)/2))+1))
            sections = [frame.sections(s, pavement) for s in stations]
            if end-start < 4 or any(len(parts) != 1 for parts in sections):
                raise ValueError('Changing deck cross-section requires a resolved structural edge layout')
            rejected_at = None
            for side, edge in ((-1, 0), (1, 1)):
                points = [frame.xyz(s, parts[0][edge]+side*.15, .42)
                    for s, parts in zip(stations, sections)]
                # Shared miter rings eliminate cracks and coplanar overlapping caps between adjacent segments.
                triangles = np.asarray(sweep(points, .8, 1.))
                for k, (a, b) in enumerate(zip(points, points[1:])):
                    faces = triangles[k*8:(k+1)*8]
                    if k == 0:
                        faces = np.concatenate([faces, triangles[-4:-2]])
                    if k == len(points)-2:
                        faces = np.concatenate([faces, triangles[-2:]])
                    if not members.add('deck edge girder', a, b, .8, 1., material, triangles=faces):
                        rejected_at = (stations[k]+stations[k+1])/2
                        break
                    footprint = members.records[-1]['footprint']
                    if footprint.intersection(pavement).area < np.linalg.norm(b[:2]-a[:2])*.10:
                        raise ValueError('Structural edge lacks sufficient attachment to its carriageway')
                if rejected_at is not None:
                    break
            if rejected_at is None:
                footprints.extend(r['footprint'] for r in members.records[record_count:])
                intervals.append((start, end))
                break
            members.records = members.records[:record_count]
            for key in members.triangles:
                members.triangles[key] = members.triangles[key][:sizes.get(key, 0)]
            if rejected_at < low+6 and start < low+6:
                start += 1
            elif rejected_at > high-6 and end > high-6:
                end -= 1
            else:
                raise ValueError('Structural edge conflicts with another road; retain ordinary edge treatment')
        else:
            raise ValueError('No bounded edge transition free of connected-road conflicts')
    members.edge_station_range = (max(i[0] for i in intervals), min(i[1] for i in intervals))
    if members.edge_station_range[1]-members.edge_station_range[0] < 4:
        raise ValueError('Separate slabs have no shared supported span for this bridge family')
    members.edge_coverage = shapely.union_all(footprints).buffer(.02)


def required_member(members, kind, a, b, width, depth, material='steel'):
    """Reject an incomplete primary structure atomically, retaining the existing deck and fallback railings."""
    if not members.add(kind, a, b, width, depth, material):
        raise ValueError('Required structural member rejected: '+kind)


def build_truss(frame, members, variant='parker'):
    """Build Pratt webs sloping down toward midspan; default to Parker's polygonal upper chord and inclined ends."""
    if variant not in ('parker', 'pratt'):
        raise ValueError('Unsupported inferred truss variant')
    build_edge_girders(frame, members, 'steel')
    panels = max(6, int(np.ceil(frame.length/12))*2)
    stations = np.linspace(*members.edge_station_range, panels+1)
    height = float(np.clip(frame.length/12, 2.5, 8))
    u = np.linspace(0, 1, panels+1)
    rises = height*(.6+.4*(1-abs(2*u-1))) if variant == 'parker' else np.full(panels+1, height)
    rises[[0, -1]] = 0
    for side in (-1, 1):
        lower = np.array([frame.edge(s, side, 1.) for s in stations])
        upper = lower.copy()
        upper[:, 2] += rises
        for i in range(panels):
            required_member(members, 'truss lower chord', lower[i], lower[i+1], .22, .25)
            required_member(members, 'truss end post' if i in (0, panels-1) else 'truss upper chord',
                upper[i], upper[i+1], .25, .30)
            if 0 < i < panels-1:
                required_member(members, 'truss diagonal', upper[i] if i < panels/2 else lower[i],
                    lower[i+1] if i < panels/2 else upper[i+1], .14, .18)
        for a, b in zip(lower[1:-1], upper[1:-1]):
            required_member(members, 'truss vertical', a, b, .15, .18)
        for point in lower:
            offset = np.r_[frame.forward*.22, 0]
            required_member(members, 'truss deck gusset', point-offset, point+offset, .12, .45)
    for station in (2, frame.length-2):
        for lo, hi in frame.sections(station):
            members.bearing(frame, station, (lo+hi)/2, 'end bearing', min(1.2, (hi-lo)*.3))


def build_arch(frame, members):
    """Use a clearly inferred tied through-arch template, so an added arch cannot steal under-deck clearance."""
    stations = np.linspace(1, frame.length-1, max(16, int(np.ceil(frame.length/3))))
    u = (stations-stations[0])/(stations[-1]-stations[0])
    rise = float(np.clip(frame.length*.16, 4, 30))
    for side in (-1, 1):
        deck = np.array([frame.edge(s, side, .3) for s in stations])
        arch = deck.copy()
        arch[:, 2] += 4*rise*u*(1-u)
        for a, b in zip(arch, arch[1:]):
            members.add('arch rib', a, b, .45, .65)
        for i in range(2, len(stations)-2, 2):
            members.add('arch hanger', deck[i], arch[i], .07, .07)
    for station in (2, frame.length-2):
        for lo, hi in frame.sections(station):
            members.bearing(frame, station, (lo+hi)/2, 'end bearing', min(1.5, (hi-lo)*.25))


def build_cables(frame, members, suspension=False):
    """Construct paired side towers and either suspended cables/hangers or direct stays; dimensions are inferred."""
    length = frame.length
    if length < 45:
        raise ValueError('Span is too short for the basic cable bridge template')
    build_edge_girders(frame, members, 'concrete')
    edge_start, edge_end = members.edge_station_range
    tower_height = float(np.clip(length*.12, 9, 65))
    tower_stations = []
    for fraction in (.22, .78):
        found = False
        for shift in (0, -5, 5, -10, 10, -15, 15):
            station = length*fraction+shift
            if not .1*length < station < .9*length:
                continue
            bases = [frame.edge(station, side, -.8) for side in (-1, 1)]
            # Towers must have both legs. Roll back a partial candidate instead of leaving a floating half portal.
            record_count = len(members.records)
            sizes = {key: len(value) for key, value in members.triangles.items()}
            ground = []
            try:
                ground = [float(members.ground(np.array([p[0]]), np.array([p[1]]))[0]) for p in bases]
            except ValueError:
                pass
            top_z = max(p[2] for p in bases)+.8+tower_height
            for base, z in zip(bases, ground):
                members.add('tower leg', [base[0], base[1], min(z-.5, base[2]-1)],
                    [base[0], base[1], top_z], .85, .85, 'concrete', True)
            if len(members.records) != record_count+2:
                members.records = members.records[:record_count]
                for key in members.triangles:
                    members.triangles[key] = members.triangles[key][:sizes.get(key, 0)]
                continue
            tops = [np.r_[p[:2], top_z] for p in bases]
            members.add('tower crossbeam', tops[0], tops[1], .85, .9, 'concrete')
            tower_stations.append((station, tops))
            found = True
            break
        if not found:
            raise ValueError('No unobstructed foundation corridor for both tower legs')
    (s0, tops0), (s1, tops1) = tower_stations
    if s1-s0 < length*.25:
        raise ValueError('Tower intervals do not define a supported central span')
    if not suspension:
        for station in np.linspace(1, length-1, max(12, int(length/9))):
            if not members.edge_station_range[0] <= station <= members.edge_station_range[1]:
                continue
            if min(abs(station-s0), abs(station-s1)) < 3:
                continue
            tops = tops0 if station < (s0+s1)/2 else tops1
            for k, side in enumerate((-1, 1)):
                anchor = frame.edge(station, side, 1.1)
                if members.add('stay cable', anchor, tops[k], .09, .09):
                    offset = np.r_[frame.forward*.24, 0]
                    center = anchor-[0, 0, .08]
                    required_member(members, 'stay anchorage', center-offset, center+offset, .3, .30)
    else:
        for k, side in enumerate((-1, 1)):
            for start, end, top_a, top_b, central in (
                (edge_start, s0, frame.edge(edge_start, side, 1), tops0[k], False),
                (s0, s1, tops0[k], tops1[k], True),
                (s1, edge_end, tops1[k], frame.edge(edge_end, side, 1), False)):
                stations = np.linspace(start, end, max(3, int(np.ceil((end-start)/4))+1))
                u = (stations-start)/(end-start)
                cable = np.array([frame.edge(s, side) for s in stations])
                cable[:, 2] = (1-u)*top_a[2]+u*top_b[2]
                if central:
                    cable[:, 2] -= 4*(tower_height-3)*u*(1-u)
                for a, b in zip(cable, cable[1:]):
                    members.add('suspension main cable', a, b, .22, .22)
                for s, top in zip(stations[1:-1], cable[1:-1]):
                    bottom = frame.edge(s, side, .3)
                    if top[2] > bottom[2]+.15:
                        members.add('suspension hanger', bottom, top, .06, .06)
    for station in (2, length-2):
        for lo, hi in frame.sections(station):
            members.bearing(frame, station, (lo+hi)/2, 'end bearing', min(1.5, (hi-lo)*.25))


def plan_families(source, network, decks, region, terrain_directory, to_local, to_geo):
    """Plan structures against prepared slab triangles; mask fallback rails only after a complete edge replacement."""
    evidence = read_evidence(source, to_local)
    assets = select_assets(network, decks, evidence, region)
    ground = ReadyMap(terrain_directory, to_geo)
    owners = {i: deck for deck in decks for i in deck.indices}
    plans = []
    for asset in assets:
        report = {k: v for k, v in asset.items() if k not in ('pavement', 'mapped_supports', 'indices')}
        report['mapped_support_evidence'] = [{'source': s['source'], 'tags': s['tags']} for s in asset['mapped_supports']]
        report['dimension_status'] = 'Inferred visual template; no surveyed dimensions'
        report['template_variant'] = {'arch': 'tied through arch', 'truss': 'parker'}.get(asset['family'], asset['family'])
        report['template_variant_status'] = 'inferred' if asset['family'] in ('arch', 'truss') else 'family template'
        report['edge_treatment'] = 'Existing generic railings retained'
        members = None
        if not asset['deferred']:
            candidate = Members(asset, network, evidence, ground)
            try:
                frame = AssetFrame(asset, network, owners)
                # Mapped piers/pylons need a shared multi-deck association before replacing a template.
                if asset['mapped_supports']:
                    raise ValueError('Mapped supports retained; exact multi-deck support association needs resolution')
                if asset['family'] in ('suspension', 'cable-stayed'):
                    build_cables(frame, candidate, asset['family'] == 'suspension')
                else:
                    {'beam': build_beam, 'truss': build_truss, 'arch': build_arch}[asset['family']](frame, candidate)
                members = candidate
                if hasattr(members, 'edge_coverage'):
                    removed = 0.
                    for deck in decks:
                        if not set(deck.indices).intersection(asset['indices']):
                            continue
                        before = deck.railing_paths
                        after = before.difference(members.edge_coverage)
                        removed += before.length-after.length
                        deck.railing_paths = after
                    report['edge_treatment'] = 'Slab-connected edge girders; generic rails replaced within girder coverage'
                    report['generic_railing_replaced_m'] = removed
                    report['shared_edge_station_range_m'] = members.edge_station_range
                    report['edge_attachment'] = {'girder_width_m': .8, 'girder_height_m': 1.,
                        'girder_axis_outboard_m': .15, 'nominal_slab_overlap_m': .25,
                        'girder_bottom_below_deck_m': .08, 'endpoint_transition_max_m': 6.}
            except ValueError as error:
                report['deferred'] = str(error)
                report['rejected_candidates'] = candidate.rejected
                members = None
        plans.append((asset, report, members))
    return {'plans': plans, 'terrain': ground.metadata, 'source_diagnostics': evidence['diagnostics']}


def export_families(plan, writer, feature):
    """Emit a prevalidated structural plan after slabs and filtered railings, retaining source and inference separately."""
    features, reports, counts = [], [], Counter()
    steel = next((i for i, m in enumerate(writer.doc['materials']) if m['name'] == 'galvanized steel'), None)
    if steel is None:
        steel = len(writer.doc['materials'])
        writer.doc['materials'].append({'name': 'galvanized steel', 'doubleSided': False,
            'pbrMetallicRoughness': {'baseColorFactor': [.52, .56, .59, 1],
                'metallicFactor': .8, 'roughnessFactor': .38}})
    for asset, report, members in plan['plans']:
        report['first_mesh'] = len(writer.doc['meshes'])
        report['mesh_count'], report['member_count'] = 0, 0
        if members is None:
            reports.append(report)
            continue
        for material, triangles in members.triangles.items():
            count = writer.add('Bridge family '+asset['id']+' '+material, np.asarray(triangles),
                1 if material == 'concrete' else steel)
            counts[material] += count
        for n, record in enumerate(members.records):
            props = {k: v for k, v in record.items() if k not in ('a', 'b', 'footprint')}
            props.update(asset_id=asset['id'], family=asset['family'], family_status=asset['family_status'],
                template_variant=report['template_variant'], template_variant_status=report['template_variant_status'],
                source_ways=asset['source_ways'], elevation_datum='WGS84 ellipsoid meters',
                dimensions_basis='Illustrative template; retained ReadyMap DEM for support bases',
                evidence='bridge-families.json')
            features.append(feature(LineString([record['a'], record['b']]), props, asset['id']+'/member/'+str(n)))
        report.update(mesh_count=len(writer.doc['meshes'])-report['first_mesh'], member_count=len(members.records),
            member_kinds=dict(Counter(r['kind'] for r in members.records)), rejected_candidates=members.rejected)
        if not members.records:
            report['deferred'] = 'No supported placement; see rejected candidates'
        reports.append(report)
    result = {'assets': reports, 'member_count': len(features), 'triangles': dict(counts),
        'terrain': plan['terrain'], 'source_diagnostics': plan['source_diagnostics'],
        'policy': 'Mapped road/outline families; untagged bridges use inferred beam; uncertain families deferred',
        'edge_policy': 'Successful truss/cable families own structural edges; deferred families retain generic rails',
        'truss_policy': 'Inferred Parker default, Pratt webs; variant never presented as source attribution',
        'placement': 'Retained base DEM; reject road-clearance collisions, rail corridors, and unmapped water foundations',
        'limitations': ['Straight-span visual templates; dimensions and tower stations inferred',
            'Mapped support association, movable mechanisms, foundations and water clearance remain unresolved',
            'Base DEM can include existing bridges; new supports are not a revalidation of composed terrain clearance']}
    return result, features, counts
