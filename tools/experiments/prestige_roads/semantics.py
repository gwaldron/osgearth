"""Small source-tag interpretations shared by the experimental road reader and urban layout solver."""


def oneway_value(tags):
    """Honor an explicit one-way value; roundabouts imply forward one-way traffic when the tag is absent."""
    return tags.get('oneway', 'yes' if tags.get('junction') == 'roundabout' else 'no')


def motor_access(tags):
    """Return the most specific passenger-car access value without erasing its authorization conditions."""
    return next((tags[key] for key in ('motorcar', 'motor_vehicle', 'vehicle', 'access') if key in tags), 'yes')


def restriction_for_car(tags):
    """Select an unconditional restriction for a private passenger car; reject unknown qualifiers or relevant conditions."""
    hierarchy = ('restriction:motorcar', 'restriction:motor_vehicle', 'restriction:vehicle', 'restriction')
    other_modes = {'bicycle', 'foot', 'bus', 'psv', 'taxi', 'hgv', 'goods', 'motorcycle', 'moped', 'mofa',
                   'emergency', 'agricultural', 'forestry', 'caravan', 'hazmat', 'delivery'}
    exceptions = set(tags.get('except', '').split(';'))-{''}
    if not exceptions <= other_modes | {'motorcar', 'motor_vehicle', 'vehicle'}:
        raise ValueError('Unknown restriction exception for the passenger-car profile')
    if exceptions & {'motorcar', 'motor_vehicle', 'vehicle'}:
        return None, 'source exception excludes the passenger-car profile'
    for key in tags:
        if any(key == scope+':conditional' for scope in hierarchy):
            raise ValueError('Conditional passenger-car restriction needs a time policy')
        if key.startswith('restriction:'):
            mode = key.split(':')[1]
            if mode not in other_modes | {'motorcar', 'motor_vehicle', 'vehicle', 'conditional'}:
                raise ValueError('Unknown vehicle-qualified restriction')
    selected = next((key for key in hierarchy if key in tags), None)
    if selected is None:
        if any(key.startswith('restriction:') for key in tags):
            return None, 'restriction applies to another vehicle profile'
        raise ValueError('Restriction relation has no supported restriction value')
    return tags[selected], selected
