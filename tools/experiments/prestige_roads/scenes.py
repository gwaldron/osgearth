"""Bounded source/render regions and explicit locale defaults for offline road experiments."""

SCENES = {
    'boston-region': {'origin': (-71.067, 42.353), 'bbox': (-71.0785, 42.3475, -71.056, 42.359),
                      'driving_side': 'right', 'center_line': 'double_yellow',
                      'additional_classes': ['service', 'tertiary_link', 'living_street'], 'source_grid': 2},
    'pittsburgh-region': {'origin': (-80.0045, 40.445), 'bbox': (-80.014, 40.4385, -79.995, 40.452),
                          'driving_side': 'right', 'center_line': 'double_yellow',
                          'additional_classes': ['service', 'tertiary_link', 'living_street'], 'source_grid': 2},
    'alewife': {'origin': (-71.1425, 42.3978), 'bbox': (-71.148, 42.3966, -71.1385, 42.3998),
                'driving_side': 'right', 'center_line': 'double_yellow', 'source': 'alewife-expanded'},
    'manhattan': {'origin': (-74.008, 40.7184), 'bbox': (-74.01125, 40.7162, -74.00545, 40.72035),
                  'driving_side': 'right', 'center_line': 'double_yellow'},
    'boston': {'origin': (-71.0565, 42.3535), 'bbox': (-71.0605, 42.3512, -71.0525, 42.3564),
               'driving_side': 'right', 'center_line': 'double_yellow'},
    'pittsburgh': {'origin': (-80.002, 40.4425), 'bbox': (-80.0065, 40.4397, -79.9975, 40.445),
                   'driving_side': 'right', 'center_line': 'double_yellow'},
    'london': {'origin': (-.119, 51.518), 'bbox': (-.124, 51.515, -.114, 51.521),
               'driving_side': 'left', 'center_line': 'broken_white'},
    'paris': {'origin': (2.366, 48.853), 'bbox': (2.360, 48.8505, 2.371, 48.8565),
              'driving_side': 'right', 'center_line': 'broken_white'},
    'sion': {'origin': (7.360, 46.232), 'bbox': (7.355, 46.2285, 7.366, 46.2355),
             'driving_side': 'right', 'center_line': 'broken_white',
             'crossing_colour': 'yellow', 'unspecified_crossing_style': 'zebra'}
}
