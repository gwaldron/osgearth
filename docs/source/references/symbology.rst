Symbology Reference
===================

osgEarth renders *features* and *annotations* using *stylesheets*.
This document lists all the symbol properties available for use in a
stylesheet. Not every symbol is applicable to every situation; this
is just a master list.

Jump to a symbol:

 * Geometry_
 * Altitude_
 * Extrusion_
 * Icon_
 * Model_
 * Render_
 * Skin_
 * Text_
 
**Developer Note**:

    *In the SDK, symbols are in the ``osgEarth::Symbology`` namespace, and each
    symbol class is in the form ``AltitudeSymbol`` for example. Properties below
    are as they appear in the earth file; in the SDK, properties are available
    via accessors in the form ``LineSymbol::strokeWidth()`` etc.
 

Value Types
-----------

These are the basic value types. In the symbol tables on this page, each
property includes the value type in parantheses following its description.

  :float:                 Floating-point number
  :float with units:      Floating-point number with unit designator, e.g.
                          20px (20 pixels) or 10m (10 meters)
  :HTML_Color:            Color string in hex format, as used in HTML; in the
                          format #RRGGBB or #RRGGBBAA. (Example: #FFCC007F)
  :integer:               Integral number
  :numeric_expr:          Expression (simple or JavaScript) resolving to a number
  :string:                Simple text string
  :string_expr:           Expression (simple or JacaScript) resolving to a text string
  :uri_string:            String denoting a resource location (like a URL or file path).
                          URIs can be absolute or relative; relative URIs are always
                          relative to the location of the *referrer*, i.e. the entity
                          that requested the resource. (For example, a relative URI within
                          an earth file will be relative to the location of the earth file
                          itself.)

                          
Geometry
--------

Basic *geometry symbols* (SDK: ``LineSymbol``, ``PolygonSymbol``, ``PointSymbol``)
control the color and style of the vector data.

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
| stroke-min-pixels     | Minimum rendering width; Prevents a   | float (pixels)             |
|                       | line from getting thinner than this   |                            |
|                       | value in pixels. Only applies when    |                            |
|                       | the ``stroke-width`` is NOT in pixels |                            |
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
| stroke-stipple-pattern| Stippling pattern bitmask. Each set   | integer (65535)            |
|                       | bit represents an "on" pixel in the   |                            |
|                       | pattern.                              |                            |
+-----------------------+---------------------------------------+----------------------------+
| stroke-stipple-factor | Stipple factor for pixel-width lines. | integer (1)                |
|                       | Number of times to repeat each bit in |                            |
|                       | the stippling pattern                 |                            |
+-----------------------+---------------------------------------+----------------------------+
| point-fill            | Fill color for a point.               | HTML color                 |
+-----------------------+---------------------------------------+----------------------------+
| point-size            | Size for a GL point geometry          | float (1.0)                |
+-----------------------+---------------------------------------+----------------------------+


Altitude
--------

The *altitude symbol* (SDK: ``AltitudeSymbol``) controls a feature's interaction with
the terrain under its location.

+-----------------------+--------------------------------------------------------------------+
| Property              | Description                                                        |
+=======================+====================================================================+
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

Tip: You can also use a shortcut to activate draping or GPU clamping; set ``altitude-clamping``
to either ``terrain-drape`` or ``terrain-gpu``.


Extrusion
---------

The *extrusion symbol* (SDK: ``ExtrusionSymbol``) directs osgEarth to create *extruded*
geometry from the source vector data; Extrusion turns a 2D vector into a 3D shape.
**Note:** The simple *presence* of an *extrusion* property will enable extrusion.

+-------------------------+--------------------------------------------------------------------+
| Property                | Description                                                        |
+=========================+====================================================================+
| extrusion-height        | How far to extrude the vector data (numeric-expr)                  |
+-------------------------+--------------------------------------------------------------------+
| extrusion-flatten       | Whether to force all extruded vertices to the same Z value (bool). |
|                         | For example, if you are extruding polygons to make 3D buildings,   |
|                         | setting this to ``true`` will force the rooftops to be flat even   |
|                         | if the underlying terrain is not. (boolean)                        |
+-------------------------+--------------------------------------------------------------------+
| extrusion-wall-gradient | Factor by which to multiply the ``fill`` color of the extruded     |
|                         | geometry at the *base* of the 3D shape. This results in the 3D     |
|                         | shape being darker at the bottom than at the top, a nice effect.   |
|                         | (float [0..1]; try 0.75)                                           |
+-------------------------+--------------------------------------------------------------------+
| extrusion-wall-style    | Name of another style in the same stylesheet that osgEarth should  |
|                         | apply to the *walls* of the extruded shape. (string)               |
+-------------------------+--------------------------------------------------------------------+
| extrusion-roof-style    | Name of another style in the same stylesheet that osgEarth should  |
|                         | apply to the *roof* of the extruded shape. (string)                |
+-------------------------+--------------------------------------------------------------------+


Skin
----

The *skin symbol* (SDK: ``SkinSymbol``) applies texture mapping to a geometry, when applicable.
(At the moment this only applies to *extruded* geometry.)

+-------------------------+--------------------------------------------------------------------+
| Property                | Description                                                        |
+=========================+====================================================================+
| skin-library            | Name of the *resource library* containing the skin(s)              |
+-------------------------+--------------------------------------------------------------------+
| skin-tags               | Set of strings (separated by whitespace containing one or more     |
|                         | *resource tags*. When selecting a texture skin to apply, osgEarth  |
|                         | will limit the selection to skins with one of these tags. If you   |
|                         | omit this property, all skins are considered. For example, if you  |
|                         | are extruding buildings, you may only want to consider textures    |
|                         | with the ``building`` tag. (string)                                |
+-------------------------+--------------------------------------------------------------------+
| skin-tiled              | When set to ``true``, osgEarth will only consider selecting a skin |
|                         | that has its ``tiled`` attribute set to ``true``. The ``tiled``    |
|                         | attribute indicates that the skin may be used as a repeating       |
|                         | texture. (boolean)                                                 |
+-------------------------+--------------------------------------------------------------------+
| skin-object-height      | *Numeric expression* resolving to the feature's real-world height  |
|                         | (in meters). osgEarth will use this value to narrow down the       |
|                         | selection to skins appropriate to that height (i.e., skins for     |
|                         | which the value falls between the skin's min/max object height     |
|                         | range. (numeric-expr)                                              |
+-------------------------+--------------------------------------------------------------------+
| skin-min-object-height  | Tells osgEarth to only consider skins whose minimum object height  |
|                         | is greater than or equal to this value. (numeric-expr)             |
+-------------------------+--------------------------------------------------------------------+
| skin-max-object-height  | Tells osgEarth to only consider skins whose maximum object height  |
|                         | is less than or equal to this value. (numeric-expr)                |
+-------------------------+--------------------------------------------------------------------+
| skin-random-seed        | Once the filtering is done (according to the properties above,     |
|                         | osgEarth determines the minimal set of appropriate skins from      |
|                         | which to choose and chooses one at random. By setting this seed    |
|                         | value you can ensure that the same "random" selection happens each |
|                         | time you run the appplication.  (integer)                          |
+-------------------------+--------------------------------------------------------------------+


Icon
----

The *icon symbol* (SDK: ``IconSymbol``) describes the appearance of 2D icons.
Icons are used for different things, the most common being:

 * Point model substitution - replace geometry with icons
 * Place annotations

+--------------------------------+--------------------------------------------------------------------+
| Property                       | Description                                                        |
+================================+====================================================================+
| icon                           | URI of the icon image. (uri-string)                                |
+--------------------------------+--------------------------------------------------------------------+
| icon-library                   | Name of a *resource library* containing the icon (optional)        |
+--------------------------------+--------------------------------------------------------------------+
| icon-placement                 | For model substitution, describes how osgEarth should replace      |
|                                | geometry with icons:                                               |
|                                |    :vertex:   Replace each vertex in the geometry with an icon.    |
|                                |    :interval: Place icons at regular intervals along the geometry, |
|                                |               according to the ``icon-density`` property.          |
|                                |    :random:   Place icons randomly within the geometry, according  |
|                                |               to the ``icon-density`` property.                    |
|                                |    :centroid: Place a single icon at the centroid of the geometry. |
+--------------------------------+--------------------------------------------------------------------+
| icon-density                   | For ``icon-placement`` settings of ``interval`` or ``random``,     |
|                                | this property is hint as to how many instances osgEarth should     |
|                                | place. The unit is approximately "units per km" (for linear data)  |
|                                | or "units per square km" for polygon data. (float)                 |
+--------------------------------+--------------------------------------------------------------------+
| icon-scale                     | Scales the icon by this amount (float)                             |
+--------------------------------+--------------------------------------------------------------------+
| icon-heading                   | Rotates the icon along its central axis (float, degrees)           |
+--------------------------------+--------------------------------------------------------------------+
| icon-declutter                 | Activate *decluttering* for this icon. osgEarth will attempt to    |
|                                | automatically show or hide things so they don't overlap on the     |
|                                | screen. (boolean)                                                  |
+--------------------------------+--------------------------------------------------------------------+
| icon-align                     | Sets the icon's location relative to its anchor point. The valid   |
|                                | values are in the form "horizontal-vertical", and are:             |
|                                |   * ``left-top``                                                   |
|                                |   * ``left-center``                                                |
|                                |   * ``left-bottom``                                                |
|                                |   * ``center-top``                                                 |
|                                |   * ``center-center``                                              |
|                                |   * ``center-bottom``                                              |
|                                |   * ``right-top``                                                  |
|                                |   * ``right-center``                                               |
|                                |   * ``right-bottom``                                               |
+--------------------------------+--------------------------------------------------------------------+
| icon-random-seed               | For random placement operations, set this seed so that the         |
|                                | randomization is repeatable each time you run the app. (integer)   |
+--------------------------------+--------------------------------------------------------------------+
| icon-occlusion-cull            | Whether to occlusion cull the text so they do not display          |
|                                | when line of sight is obstructed by terrain                        | 
+--------------------------------+--------------------------------------------------------------------+
| icon-occlusion-cull-altitude   | The viewer altitude (MSL) to start occlusion culling               |
|                                | when line of sight is obstructed by terrain                        |
+--------------------------------+--------------------------------------------------------------------+
 

Model
-----

The *model symbol* (SDK: ``ModelSymbol``) describes external 3D models.
Like icons, models are typically used for:

 * Point model substitution - replace geometry with 3D models
 * Model annotations

+-------------------------+--------------------------------------------------------------------+
| Property                | Description                                                        |
+=========================+====================================================================+
| model                   | URI of the 3D model (uri-string). Use this *OR* the                |
|                         | ``model-library`` property, but not both.                          |
+-------------------------+--------------------------------------------------------------------+
| model-library           | Name of a *resource library* containing the model. Use this *OR*   |
|                         | the ``model`` property, but not both.                              |
+-------------------------+--------------------------------------------------------------------+
| model-placement         | For model substitution, describes how osgEarth should replace      |
|                         | geometry with models:                                              |
|                         |    :vertex:   Replace each vertex in the geometry with a model.    |
|                         |    :interval: Place models at regular intervals along the geometry,|
|                         |               according to the ``model-density`` property.         |
|                         |    :random:   Place models randomly within the geometry, according |
|                         |               to the ``model-density`` property.                   |
|                         |    :centroid: Place a single model at the centroid of the geometry.|
+-------------------------+--------------------------------------------------------------------+
| model-density           | For ``model-placement`` settings of ``interval`` or ``random``,    |
|                         | this property is hint as to how many instances osgEarth should     |
|                         | place. The unit is approximately "units per km" (for linear data)  |
|                         | or "units per square km" for polygon data. (float)                 |
+-------------------------+--------------------------------------------------------------------+
| model-scale             | Scales the model by this amount along all axes (float)             |
+-------------------------+--------------------------------------------------------------------+
| model-heading           | Rotates the about its +Z axis (float, degrees)                     |
+-------------------------+--------------------------------------------------------------------+
| icon-random-seed        | For random placement operations, set this seed so that the         |
|                         | randomization is repeatable each time you run the app. (integer)   |
+-------------------------+--------------------------------------------------------------------+
 
 
Render
------

The *render symbol* (SDK: ``RenderSymbol``) applies general OpenGL rendering settings as well
as some osgEarth-specific settings that are not specific to any other symbol type.

+-------------------------------+--------------------------------------------------------------+
| Property                      | Description                                                  |
+===============================+==============================================================+
| render-depth-test             | Enable or disable GL depth testing. (boolean)                |
+-------------------------------+--------------------------------------------------------------+
| render-lighting               | Enable or disable GL lighting. (boolean)                     |
+-------------------------------+--------------------------------------------------------------+
| render-depth-offset           | Enable or disable Depth Offseting. Depth offsetting is a     |
|                               | GPU technique that modifies a fragment's depth value,        |
|                               | simulating the rendering of that object closer or farther    |
|                               | from the viewer than it actually is. It is a mechanism for   |
|                               | mitigating z-fighting. (boolean)                             |
+-------------------------------+--------------------------------------------------------------+
| render-depth-offset-min-bias  | Sets the minimum bias (distance-to-viewer offset) for depth  |
|                               | offsetting. If is usually sufficient to set this property;   |
|                               | all the others will be set automatically. (float, meters)    |
+-------------------------------+--------------------------------------------------------------+
| render-depth-offset-max-bias  | Sets the minimum bias (distance-to-viewer offset) for depth  |
|                               | offsetting.                                                  |
+-------------------------------+--------------------------------------------------------------+
| render-depth-offset-min-range | Sets the range (distance from viewer) at which to apply the  |
|                               | minimum depth offsetting bias. The bias graduates between its|
|                               | min and max values over the specified range.                 |
+-------------------------------+--------------------------------------------------------------+
| render-depth-offset-max-range | Sets the range (distance from viewer) at which to apply the  |
|                               | maximum depth offsetting bias. The bias graduates between its|
|                               | min and max values over the specified range.                 |
+-------------------------------+--------------------------------------------------------------+



Text
----

The *text symbol* (SDK: ``TextSymbol``) controls the existance and appearance of text labels.

+--------------------------------+--------------------------------------------------------------------+
| Property                       | Description                                                        |
+================================+====================================================================+
| text-fill                      | Foreground color of the text (HTML color)                          |
+--------------------------------+--------------------------------------------------------------------+
| text-size                      | Size of the text (float, pixels)                                   |
+--------------------------------+--------------------------------------------------------------------+
| text-font                      | Name of the font to use (system-dependent). For example, use       |
|                                | "arialbd" on Windows for Arial Bold.                               |
+--------------------------------+--------------------------------------------------------------------+
| text-halo                      | Outline color of the text; Omit this propery altogether for no     |
|                                | outline. (HTML Color)                                              |
+--------------------------------+--------------------------------------------------------------------+
| text-halo-offset               | Outline thickness (float, pixels)                                  |
+--------------------------------+--------------------------------------------------------------------+
| text-align                     | Alignment of the text string relative to its anchor point:         |
|                                |   * ``left-top``                                                   |
|                                |   * ``left-center``                                                |
|                                |   * ``left-bottom``                                                |
|                                |   * ``left-base-line``                                             |
|                                |   * ``left-bottom-base-line``                                      |
|                                |   * ``center-top``                                                 |
|                                |   * ``center-center``                                              |
|                                |   * ``center-bottom``                                              |
|                                |   * ``center-base-line``                                           |
|                                |   * ``center-bottom-base-line``                                    |
|                                |   * ``right-top``                                                  |
|                                |   * ``right-center``                                               |
|                                |   * ``right-bottom``                                               |
|                                |   * ``right-base-line``                                            |
|                                |   * ``right-bottom-base-line``                                     |
|                                |   * ``base-line``                                                  |
+--------------------------------+--------------------------------------------------------------------+
| text-layout                    | Layout of text:                                                    |
|                                |   * ``ltr``                                                        |
|                                |   * ``rtl``                                                        |
|                                |   * ``vertical``                                                   |
+--------------------------------+--------------------------------------------------------------------+
| text-content                   | The actual text string to display (string-expr)                    |
+--------------------------------+--------------------------------------------------------------------+
| text-encoding                  | Character encoding of the text content:                            |
|                                |   * ``utf-8``                                                      |
|                                |   * ``utf-16``                                                     |
|                                |   * ``utf-32``                                                     |
|                                |   * ``ascii``                                                      |
+--------------------------------+--------------------------------------------------------------------+
| text-declutter                 | Activate *decluttering* for this icon. osgEarth will attempt to    |
|                                | automatically show or hide things so they don't overlap on the     |
|                                | screen. (boolean)                                                  |
+--------------------------------+--------------------------------------------------------------------+
| text-occlusion-cull            | Whether to occlusion cull the text so they do not display          |
|                                | when line of sight is obstructed by terrain                        | 
+--------------------------------+--------------------------------------------------------------------+
| text-occlusion-cull-altitude   | The viewer altitude (MSL) to start occlusion culling               |
|                                | when line of sight is obstructed by terrain                        |
+--------------------------------+--------------------------------------------------------------------+

