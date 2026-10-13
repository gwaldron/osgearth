"""Separate physical road lanes from the bounded private-car profile and retained bus-lane evidence."""

from intersections import LayoutError
from semantics import oneway_value


ALLOWED = {'yes', 'permissive', 'destination', 'permit', 'private', 'customers', 'delivery', 'designated'}
ACCESS_MODES = ('access', 'vehicle', 'motor_vehicle', 'motorcar')


def physical_counts(road):
    """Return physical directional counts, retaining a proven bus contraflow instead of assigning it to cars."""
    tags, total = road['tags'], road['lanes']
    if 'lanes' in tags and not tags['lanes'].isdigit():
        raise LayoutError('Nonintegral source lane count cannot use the prototype default')
    if total < 1 or total > 6:
        raise LayoutError('Urban lane count outside the supported 1..6 range')
    one = oneway_value(tags)
    if one not in ('yes', '1', 'true', '-1', 'no', '0', 'false'):
        raise LayoutError('Unsupported one-way value')
    canonical = {'yes': 1, '1': 1, 'true': 1, '-1': -1, 'no': 0, '0': 0, 'false': 0}
    for mode in ('bus', 'psv'):
        value = tags.get('oneway:'+mode)
        if value is not None and (value not in canonical or canonical[value] not in (0, canonical[one])):
            raise LayoutError('Opposite-only or unsupported bus/PSV direction needs a separate modal profile')
    counts = [tags.get('lanes:'+d) for d in ('forward', 'backward')]
    if any(v is not None and not v.isdigit() for v in counts):
        raise LayoutError('Nonintegral directional lane count')
    main = 1 if one == '-1' else 0
    single = one in ('yes', '1', 'true', '-1')
    contraflow = single and any(tags.get('oneway:'+mode) in ('no', '0', 'false') for mode in ('bus', 'psv'))
    if single and not contraflow:
        result = [0, 0]
        result[main] = total
        if any(v is not None and int(v) != result[i] for i, v in enumerate(counts)):
            raise LayoutError('One-way direction contradicts physical directional lane counts')
        return tuple(result)
    if contraflow and all(v is None for v in counts):
        direction = ('forward', 'backward')[1-main]
        values = {tags[k] for k in ('lanes:bus:'+direction, 'lanes:psv:'+direction) if k in tags}
        if len(values) == 1 and next(iter(values)).isdigit():
            counts[1-main] = next(iter(values))
        else:
            raise LayoutError('Bus contraflow needs explicit directional lane counts')
    if all(v is None for v in counts):
        if total == 1:
            return 1, 1
        if total % 2:
            raise LayoutError('Odd two-way lane count lacks a directional split')
        return total//2, total//2
    forward = int(counts[0]) if counts[0] is not None else total-int(counts[1])
    backward = int(counts[1]) if counts[1] is not None else total-forward
    if forward < 1 or backward < 1 or forward+backward != total:
        raise LayoutError('Directional lane counts contradict total lanes')
    return forward, backward


def lane_fields(tags, mode, travel, count, single):
    """Read one physical-lane vector in travel order; reject ambiguous generic two-way or conflicting vectors."""
    generic, directed = mode+':lanes', mode+':lanes:'+travel
    if generic in tags and not single:
        raise LayoutError('Generic per-lane access on a two-way road needs a directional vector')
    key = directed if directed in tags else generic if generic in tags else None
    if key is None:
        return ['']*count
    if generic in tags and directed in tags and tags[generic] != tags[directed]:
        raise LayoutError('Conflicting generic and directional per-lane access')
    fields = tags[key].split('|')
    if len(fields) != count:
        raise LayoutError(f'{key} has {len(fields)} entries for {count} physical lanes')
    return fields


def lane_model(road, driving_side=None):
    """Retain physical lane numbers and source roles; withhold unresolved legacy-busway car eligibility explicitly."""
    tags = road['tags']
    side = tags.get('driving_side', driving_side or road.get('driving_side', 'right'))
    if side not in ('right', 'left'):
        raise LayoutError('Unsupported driving-side value')
    counts = physical_counts(road)
    if road['lanes'] == 1 and all(counts) and any(':lanes' in k or k.startswith(('lanes:', 'busway'))
                                               for k in tags):
        raise LayoutError('Vehicle roles on a shared physical lane need a directional sharing policy')
    one = oneway_value(tags)
    single = 0 in counts
    allowed_keys = {mode+':lanes'+suffix for mode in (*ACCESS_MODES, 'bus', 'psv')
                    for suffix in ('', ':forward', ':backward')}
    allowed_keys |= {'lanes:'+mode+suffix for mode in ('bus', 'psv') for suffix in ('', ':forward', ':backward')}
    allowed_keys |= {'busway', 'busway:left', 'busway:right', 'busway:both', 'busway:left:oneway', 'busway:right:oneway'}
    for key in tags:
        relevant = key.startswith(('turn:', 'lanes', 'oneway', *ACCESS_MODES, 'busway', 'bus:', 'psv:'))
        if relevant and ('conditional' in key or key.startswith('lanes:both_ways')):
            raise LayoutError('Conditional/shared/reserved lane needs a vehicle and time policy')
        qualified = (key.startswith('busway') or any(key.startswith(mode+':lanes') or key.startswith('lanes:'+mode)
                     for mode in (*ACCESS_MODES, 'bus', 'psv', 'taxi', 'hgv')))
        if qualified and key not in allowed_keys:
            raise LayoutError('Unsupported vehicle-qualified lane role or side-count syntax')
        if key.startswith('oneway:') and key not in ('oneway:bus', 'oneway:psv', 'oneway:bicycle'):
            raise LayoutError('Vehicle-specific one-way direction needs its own profile')
    result = {'counts': counts, 'roles': {}, 'source_tags': {k: v for k, v in tags.items()
              if k in allowed_keys or k.startswith('oneway')}, 'vehicle_profile': 'private passenger car'}
    for travel, count in zip(('forward', 'backward'), counts):
        if not count:
            if any(k.endswith(':lanes:'+travel) for k in tags):
                raise LayoutError('Directional lane vector has no matching physical travel lanes')
            result['roles'][travel] = []
            continue
        access = ['yes']*count
        explicit = [False]*count
        motor_values = access.copy()
        for mode in ACCESS_MODES:
            if mode in tags:
                access = [tags[mode]]*count
            fields = lane_fields(tags, mode, travel, count, single)
            for i, value in enumerate(fields):
                if value:
                    access[i], explicit[i] = value, True
            if mode == 'motor_vehicle':
                motor_values = access.copy()
        bus_access = motor_values.copy()
        bus_specific = [False]*count
        designated = [set() for _ in range(count)]
        for mode in ('psv', 'bus'):
            if mode in tags:
                bus_access = [tags[mode]]*count
                bus_specific = [True]*count
            fields = lane_fields(tags, mode, travel, count, single)
            for i, value in enumerate(fields):
                if value:
                    bus_access[i] = value
                    bus_specific[i] = True
                value = value or tags.get(mode, '')
                if value not in ('', 'yes', 'no', 'designated', 'permissive', 'private', 'destination'):
                    raise LayoutError('Unsupported bus/PSV lane access value')
                if value == 'designated':
                    designated[i].add(mode)
        roles = []
        for i, value in enumerate(access):
            if value != 'no' and not set(value.split(';')) <= ALLOWED:
                raise LayoutError('Restricted motor access needs an explicit policy')
            contra = (one in ('yes', '1', 'true') and travel == 'backward') or (one == '-1' and travel == 'forward')
            roles.append({'lane': i+1, 'car_access': 'no' if contra else value,
                          'car_eligible': value != 'no' and not contra, 'designated_modes': sorted(designated[i]),
                          'bus_access': bus_access[i], 'explicit_bus_access': bus_specific[i],
                          'basis': 'SOURCE car one-way excludes contraflow' if contra else 'SOURCE access hierarchy',
                          'explicit_lane_access': explicit[i]})
        result['roles'][travel] = roles
    for key, value in tags.items():
        if not key.startswith('busway') or key.endswith(':oneway') or value in ('no', 'none'):
            continue
        if key == 'busway' or value not in ('lane', 'opposite_lane'):
            raise LayoutError('Legacy busway needs an explicit side and supported lane type')
        sides = ('left', 'right') if key == 'busway:both' else (key.split(':')[1],)
        for edge in sides:
            travel = ('forward' if counts[0] else 'backward') if single else (
                'forward' if (edge == 'right') == (side == 'right') else 'backward')
            stated = tags.get('busway:'+edge+':oneway')
            if stated is not None:
                if stated not in ('yes', '1', '-1'):
                    raise LayoutError('Legacy busway direction is not a single directed lane')
                expected = 'backward' if stated == '-1' else 'forward'
                if expected != travel:
                    raise LayoutError('Legacy busway side contradicts its physical travel direction')
            if value == 'opposite_lane':
                if one not in ('yes', '1', 'true', '-1') or travel == ('backward' if one == '-1' else 'forward'):
                    raise LayoutError('Contraflow busway lacks a compatible physical directional split')
            roles = result['roles'][travel]
            index = 0 if (edge == 'left') == (travel == 'forward') else len(roles)-1
            role = roles[index]
            role['designated_modes'] = sorted(set(role['designated_modes']) | {'bus'})
            if role['bus_access'] == 'no' and role['explicit_bus_access']:
                raise LayoutError('Legacy busway conflicts with explicit bus/PSV access')
            role['legacy_busway_source'] = key
            role['bus_access'] = 'designated'
            if role['car_eligible'] and not role['explicit_lane_access']:
                role.update(car_access='unknown', car_eligible=False,
                            basis='INFERRED position from legacy busway side; car eligibility withheld pending access evidence')
    for mode in ('bus', 'psv'):
        for suffix, travels in [(':forward', ('forward',)), (':backward', ('backward',)), ('', ('forward', 'backward'))]:
            key = 'lanes:'+mode+suffix
            if key not in tags:
                continue
            if not tags[key].isdigit():
                raise LayoutError('Nonintegral reserved lane count')
            expected = int(tags[key])
            roles = [r for travel in travels for r in result['roles'][travel]]
            known = [r for r in roles if mode in r['designated_modes'] or (mode == 'psv' and 'bus' in r['designated_modes'])]
            # An entirely one-way-excluded direction has an unambiguous physical location for its bus lane count.
            if not known and len(travels) == 1 and expected == len(roles) and expected and all(
                    r['basis'] == 'SOURCE car one-way excludes contraflow' for r in roles):
                for role in roles:
                    role['designated_modes'] = sorted(set(role['designated_modes']) | {mode})
                known = roles
            if mode == 'bus' and any(r['bus_access'] == 'no' for r in known):
                raise LayoutError('Reserved bus count contradicts explicit bus access')
            if len(known) != expected:
                raise LayoutError('Reserved lane count lacks matching physical positions or contradicts lane roles')
    return result


def role_signature(model):
    """Compare physical role eligibility for chaining; source-chain way IDs retain each access condition."""
    return tuple(tuple((r['car_eligible'], r['bus_access'] != 'no', tuple(r['designated_modes']))
                       for r in model['roles'][travel])
                 for travel in ('forward', 'backward'))
