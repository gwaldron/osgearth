"""Retain source traffic controls and construct explicitly inferred stop bars for the bounded road experiment."""

import gzip
import xml.etree.ElementTree as ET

import numpy as np
from shapely.geometry import LineString, Point
from shapely.ops import unary_union

from intersections import frame


def read_controls(path, roads, xy, region):
    """Read signal/stop/yield nodes and explicit stop-line ways; keep unsupported or ambiguous records for review."""
    with gzip.open(path, 'rb') as stream:
        root = ET.parse(stream).getroot()
    records = []
    for element in root:
        tags = {t.get('k'): t.get('v') for t in element.findall('tag')}
        kind = tags.get('highway')
        explicit = tags.get('road_marking') in ('stop_line', 'solid_stop_line')
        if kind not in ('traffic_signals', 'stop', 'give_way') and not explicit:
            continue
        if element.tag == 'node':
            node = int(element.get('id'))
            position = xy[node]
            if not region.covers(Point(position)):
                continue
            associated = [r['id'] for r in roads if node in r['refs'] and r['tags']['highway'] != 'cycleway']
            direction = tags.get('traffic_signals:direction', tags.get('stop:direction', tags.get('direction', 'unknown')))
            records.append({'node': node, 'element': 'node', 'kind': kind or 'stop_line', 'tags': tags,
                            'roads': associated, 'position': position, 'direction': direction,
                            'explicit_stop_line': explicit, 'rendered': False})
        elif element.tag == 'way' and explicit:
            refs = [int(n.get('ref')) for n in element.findall('nd')]
            if any(n not in xy for n in refs):
                raise ValueError('Mapped stop line references missing source nodes')
            line = LineString([xy[n] for n in refs])
            if not line.intersects(region):
                continue
            associated = [r['id'] for r in roads if r['line'].intersects(line) and r['tags']['highway'] != 'cycleway']
            records.append({'way': int(element.get('id')), 'element': 'way', 'kind': 'stop_line', 'tags': tags,
                            'roads': associated, 'line_xy': list(line.coords), 'direction': tags.get('direction', 'unknown'),
                            'explicit_stop_line': True, 'rendered': False})
    return records


def layout_section(record, layouts):
    """Find a stop section on the resolved incoming lane ribbons; retain source position as placement evidence."""
    point = Point(record['position'])
    for layout in layouts:
        if not layout['domain'].buffer(.5).covers(point):
            continue
        group = sorted([c for c in layout['connectors'] if c['from'] == record['roads'][0]],
                       key=lambda c: c['from_lane'])
        if not group:
            continue
        center = np.mean([c['points'] for c in group], axis=0)
        line = LineString(center)
        station = line.project(point)
        cumulative = np.r_[0, np.cumsum(np.linalg.norm(np.diff(center, axis=0), axis=1))]
        index = min(len(center)-2, max(0, int(np.searchsorted(cumulative, station)-1)))
        t = (station-cumulative[index])/max(cumulative[index+1]-cumulative[index], 1e-9)
        left, right = np.asarray(group[0]['left_edge'].coords), np.asarray(group[-1]['right_edge'].coords)
        ends = [edge[index]*(1-t)+edge[index+1]*t for edge in (left, right)]
        bar = LineString(ends)
        record['layout'] = layout['name']
        record['placement_displacement_m'] = point.distance(bar.interpolate(.5, normalized=True))
        return bar
    return None


def build_stop_bars(records, roads, layouts, ground, crossing_exclusions, urban=False, driving_side='right'):
    """Construct supported directional stop bars, recording inferred shape separately from mapped control evidence."""
    by_id = {r['id']: r for r in roads}
    patches, exclusions = [], []
    inset = None
    for record in records:
        record['rendered'] = False
        if len(record['roads']) != 1:
            record['unresolved'] = 'No unique selected carriageway; control retained without a guessed bar'
            continue
        road = by_id[record['roads'][0]]
        if road['bridge'] or road['tunnel']:
            record['unresolved'] = 'Structure control needs its own supporting deck'
            continue
        if record['kind'] == 'give_way' or record['tags'].get('stroke', 'solid') != 'solid':
            record['unresolved'] = 'Yield/non-solid marking needs a separate regional style'
            continue
        if any('conditional' in key for key in record['tags']):
            record['unresolved'] = 'Conditional control retained without unconditional paint inference'
            continue
        crossing = record['tags'].get('crossing')
        combined_crossing = crossing not in ('no', 'separate') and (
            crossing in ('traffic_signals', 'uncontrolled', 'marked', 'unmarked', 'zebra')
            or record['tags'].get('crossing:markings', 'no') != 'no')
        if record['element'] == 'node' and not record['explicit_stop_line'] and combined_crossing:
            record['unresolved'] = 'Combined signal/crossing evidence needs a separate stop position'
            continue
        if record['element'] == 'way':
            bar = LineString(record['line_xy'])
            record['placement_displacement_m'] = 0.0
            record['geometry_basis'] = 'Mapped stop-line alignment; stroke width remains inferred'
        else:
            direction = record['direction']
            forward = road['tags'].get('oneway') in ('yes', '1', 'true')
            backward = road['tags'].get('oneway') == '-1'
            if direction not in ('forward', 'backward'):
                record['unresolved'] = 'No unambiguous approach direction; no stop bar inferred'
                continue
            if not urban and ((forward and direction != 'forward') or (backward and direction != 'backward')):
                record['unresolved'] = 'Control direction opposes this one-way carriageway'
                continue
            bar = layout_section(record, layouts) if forward or backward else None
            if bar is None:
                station = road['line'].project(Point(record['position']))
                center, _, normal = frame(road['line'], station)
                half = road['width']/2
                low, high = (-half, half) if forward or backward else ((-half, 0) if direction == 'forward' else (0, half))
                if urban:
                    from urban import lane_pitch, lane_schema
                    from intersections import LayoutError
                    try:
                        forward_count, backward_count = lane_schema(road)
                        pitch = lane_pitch(road, forward_count, backward_count)
                    except LayoutError as error:
                        record['unresolved'] = str(error)
                        continue
                    count, opposing = ((forward_count, backward_count) if direction == 'forward'
                                       else (backward_count, forward_count))
                    if not count:
                        record['unresolved'] = 'Control has no resolved physical lanes in this direction'
                        continue
                    if (forward and direction == 'backward') or (backward and direction == 'forward'):
                        record['physical_control_section'] = {
                            'direction': direction, 'lanes': count, 'opposing_lanes': opposing,
                            'basis': 'SOURCE directional control on validated physical bus contraflow; stop paint inferred'}
                    shared = road['lanes'] == 1 and forward_count and backward_count
                    side = road['tags'].get('driving_side', driving_side)
                    if side not in ('right', 'left'):
                        record['unresolved'] = 'Unsupported traffic side for directional stop line'
                        continue
                    hand = 1 if side == 'right' else -1
                    travel = 1 if direction == 'forward' else -1
                    middle = 0 if shared else -travel*hand*opposing*pitch/2
                    low, high = middle-count*pitch/2+.1, middle+count*pitch/2-.1
                bar = LineString([center+normal*low, center+normal*high])
                record['placement_displacement_m'] = 0.0
            record['geometry_basis'] = ('Mapped stop-line node; inferred transverse shape' if record['explicit_stop_line']
                                        else 'INFERRED stop bar at mapped directional control; paint presence unverified')
        if inset is None:
            inset = ground.buffer(-.20)
        paint = bar.buffer(.20, cap_style=2).intersection(inset)
        if paint.is_empty or paint.area < .2:
            record['unresolved'] = 'No supported marking area'
            continue
        if paint.intersection(crossing_exclusions).area > .001:
            record['unresolved'] = 'Control overlaps a crossing exclusion; exact stop position needs review'
            continue
        record['rendered'] = True
        record['bar_xy'] = list(bar.coords)
        record['paint_area_m2'] = paint.area
        record['stroke_width_m'] = .40
        record['signal_phase'] = 'unknown' if record['kind'] == 'traffic_signals' else 'not applicable'
        patches.append(paint)
        exclusions.append(bar.buffer(1.0, cap_style=2))
    return unary_union(patches), unary_union(exclusions)


def contraflow_control_views(records, height, to_geo, region):
    """Select a rendered contraflow stop bar away from the crop edge; return no view when none is supported."""
    candidates = [r for r in records if r['rendered'] and 'physical_control_section' in r
                  and region.boundary.distance(Point(r['position'])) > 20]
    if not candidates:
        return {}
    target = max(candidates, key=lambda r: (region.boundary.distance(Point(r['position'])), -r['node']))
    point = LineString(target['bar_xy']).centroid
    lon, lat = to_geo.transform(point.x, point.y)
    return {'contraflow-control': (lon, lat, -15, -60, 65, float(height(point.x, point.y)[0]))}
