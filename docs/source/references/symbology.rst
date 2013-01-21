Symbology Reference Guide
=========================

Basic Types
~~~~~~~~~~~

These are the basic value types. Where applicable, the default value for a
property appears in parentheses.

:HTML Color:            Color string in hex format, as used in HTML; in the
                        format #RRGGBB or #RRGGBBAA. (Example: #FFCC007F)
:float:                 Floating-point number
:float with units:      Floating-point number with unit designator, e.g.
                        20px (20 pixels) or 10m (10 meters)
:integer:               Integral number


Geometry Symbols
----------------

+-----------------------+---------------------------------------+----------------------------+
| Property              | Description                           | Value(s)                   |
+=======================+=======================================+============================+
| fill                  | Fill color for a polygon.             | HTML color                 |
+-----------------------+---------------------------------------+----------------------------+
| stroke                | Line color (or polygon outline color, | HTML color                 |
|                       | if ``fill`` is present)               |                            |
+-----------------------+---------------------------------------+----------------------------+
| stroke-width          | Line width                            | float with units           |
+-----------------------+---------------------------------------+----------------------------+
| stroke-tessellation   | Number of times to subdivide a line   | integer                    |
+-----------------------+---------------------------------------+----------------------------+
| stroke-linejoin       | Join style for polygonized lines.     | miter, round               |
|                       | Only applies with ``stroke-width``    |                            |
|                       | is in world units (and not pixels)    |                            |
+-----------------------+---------------------------------------+----------------------------+
| stroke-linecap        | Cap style for polygonized lines.      | square, flat, round        |
|                       | Only applies with ``stroke-width``    |                            |
|                       | is in world units (and not pixels)    |                            |
+-----------------------+---------------------------------------+----------------------------+
| stroke-rounding-ratio | For joins and caps that are set to    | float (0.4)                |
|                       | ``round``, the resolution of the      |                            |
|                       | rounded corner. Value is the ratio of |                            |
|                       | line width to corner segment length.  |                            |
+-----------------------+---------------------------------------+----------------------------+
| point-size            | Size for a GL point geometry          | float (1.0)                |
+-----------------------+---------------------------------------+----------------------------+


Altitude Symbol
---------------

The *altitude symbol* controls a feature's interaction with the terrain.

+-----------------------+--------------------------------------------------------------------+
| Property              | Description                                                        |
+==================== ==+====================================================================+
| altitude-clamping     | Controls terrain following behavior.                               |
|                       |   :none:     no clamping                                           |
|                       |   :terrain:  clamp to terrain and discard Z values                 |
|                       |   :relative: clamp to terrain and retain Z value                   |
|                       |   :absolute: feature's Z contains its absolute Z.                  |
+-----------------------+--------------------------------------------------------------------+
| altitude-technique    | When ``altitude-clamping`` is set to ``terrain``, chooses a        |
|                       | terrain following technique:                                       |
|                       |   :map:    clamp geometry to the map's elevation data tiles        |
|                       |   :drape:  clamp geometry using a projective texture               |
|                       |   :gpu:    clamp geometry to the terrain on the GPU                |
|                       |   :scene:  re-clamp geometry to new paged tiles (annotations only) |
+-----------------------+--------------------------------------------------------------------+
| altitude-binding      | Granulatiry at which to sample the terrain when                    |
|                       | ``altitude-technique`` is ``map``:                                 |
|                       |   :vertex:   clamp every vertex                                    |
|                       |   :centroid: only clamp the centroid of each feature               |
+-----------------------+--------------------------------------------------------------------+
| altitude-resolution   | Elevation data resolution at which to sample terrain height when   |
|                       | ``altitude-technique`` is ``map`` (float)                          |
+-----------------------+--------------------------------------------------------------------+
| altitude-offset       | Vertical offset to apply to geometry Z                             |
+-----------------------+--------------------------------------------------------------------+
| altitude-scale        | Scale factor to apply to geometry Z                                |
+-----------------------+--------------------------------------------------------------------+

