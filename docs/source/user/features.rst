Features & Symbology
====================

Understanding Features
----------------------

**Features** are vector geometry. Unlike imagery and elevation data (which are *rasters*),
feature does not have a discrete display resolution. osgEarth can render features at any
level of detail.

A **Feature** is a combination of three components:

  * Vector geometry (a collection of points, lines, or polygons)
  * Attributes (a collection of name/value pairs)
  * Spatial Reference (describing the geometry coordinates)

Creating a Feature Layer
------------------------

osgEarth can render features in two different ways:

  * Rasterized as an *image layer*
  * Tessellated as a *model layer*

Rasterization
~~~~~~~~~~~~~

*Rasterized features* are the simplest - osgEarth will "draw" the vectors to an image tile
and then use that image tile in a normal image layer.

osgEarth has one rasterizing feature driver: the ``agglite`` driver. Here's an example
that renders an ESRI Shapefile as a rasterized image layer::

    <model name="my layer" driver="agglite">
        <features name="states" driver="ogr">
            <url>states.shp</url>
        </features>
        <styles>
            <style type="text/css">
                states {
                    stroke:       #ffff00;
                    stroke-width: 2.0;
                }
            </style>
        </styles>
    </model>

Tessellation
~~~~~~~~~~~~

*Tessellated features* go through a compilation process that turns the input vectors
into OSG geometry (points, lines, triangles, or substituted 3D models). The primary
feature tessellation plugin is the ``feature_geom`` driver - you will see this in use
in most of osgEarth's earth files that demonstrate the use of feature data.

Here is a *model layer* that renders an ESRI Shapefile as a series of yellow lines,
rendered as OSG line geometry::

    <model name="my layer" driver="feature_geom">
        <features name="states" driver="ogr">
            <url>states.shp</url>
        </features>
        <styles>
            <style type="text/css">
                states {
                    stroke:       #ffff00;
                    stroke-width: 2.0;
                }
            </style>
        </styles>
    </model>

Components of a Feature Layer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As you can see from the examples above, there are a few necessary components 
to any feature layer

  * The ``<features>`` block describes the actual feature source; i.e., where osgEarth
    should go to find the input data.
    
  * The ``<styles>`` block describes how osgEarth should render the features, i.e., 
    their appearance in the scene. We call this the *stylesheet* or the *symbology*.
    The makeup of the *stylesheet* can radically alter the appearance of the feature
    data.
  
Both of these elements are required.

Styling
-------

In an earth file, you may see a ``<styles>`` block that looks like this::

    <styles>
        <style type="text/css">
            buildings {
                altitude-clamping: terrain;
                extrusion-height:  15;
                extrusion-flatten: true;
                fill:              #ff7f2f;
            }    
        </style>
    </styles>
    
That is a *stylesheet* block. You will find this inside a ``<model>`` layer that is
rendering feature data, paired with a ``<features>`` block. (The ``<features>`` block
defines the source of the actual content.)

In this case, the ``<style>`` element holds CSS-formatted data. A CSS style block can
hold multiple styles, each of which has a name. In this case we only have one style:
``buildings``. This style tells the geometry engine to do the following:

    * Clamp the feature geometry to the terrain elevation data;
    * Extrude shapes to a height of 15m above the terrain;
    * Flatten the top of the extruded shape; and
    * Color the shape orange.

osgEarth takes a "model/view" approach to rendering features. It separates the
concepts of *content* and *style*, much in the same way that a web application will
use *CSS* to style the web content.

osgEarth takes each input feature and subjects it to a styling process. The output
will depend entirely on the combination of symbols in the stylesheet. This includes:

  * Fill and Stroke - whether to draw the data as lines or polygons
  * Extrusion - extruding 2D geometry into a 3D shape
  * Substitution - replacing the geometry with external 3D models (e.g., trees) or icons
  * Altitude - how the geometry interacts with the map's terrain
  * Text - controls labeling
  * Rendering - application of lighting, blending, and depth testing
  
Stylesheets
~~~~~~~~~~~

Each feature layer requires a *stylesheet*. The stylesheet appears as a ``<styles>``
block in the earth file. Here's an example::

    <model name="test" driver="feature_geom">
        <features driver="ogr">
            <geometry>POLYGON( (0 0, 1 0, 1 1, 0 1) )</geometry>
            <profile>global-geodetic</profile>
        </features>
        <styles>
            <style type="text/css">
                default {
                    fill:               #ff7f009f;
                    stroke:             #ffffff;
                    stroke-width:       2.0;
                    altitude-clamping:  terrain;
                    altitude-technique: drape;
                    render-lighting:    false;
                }
            </style>
        </styles>
    </model>
    
The *stylesheet* contains one *style* called ``default``. Since there is only one
style, osgEarth will apply it to all the input features. (To apply different styles 
to different features, use *selectors* - more information below.)

The style contains a set of *symbols* what describe how osgEarth should render
the feature geometry. In this case:

    :fill:   Draw a filled polygon in the specified HTML-style color (orange in this case).
    :stroke: Outline the polygon in white.
    :stroke-width: Draw the outline 2 pixels wide.
    :altitude-clamping: Clamp the polygon to the terrain.
    :altitude-technique: Use a "draping" technique to clamp the polygon (projective texturing).
    :render-lighting: Disable OpenGL lighting on the polygon.

This is only a small sample of available symbology.
For a complete listing, please refer to: :doc:`/references/symbology`.


Expressions
~~~~~~~~~~~

Some symbol properties support *expression*. An expression is a simple in-line calculation
that uses feature attribute values to calculate a property dynamically.

In an expression, you access a feature attribute value by enclosing its name
in square brackets, like this: ``[name]``

Example::

    mystyle {
        extrusion-height:  [hgt]*0.3048;           - read the "hgt" attribute, and convert it from feet to meters
        altitude-offset:   max([base_offset], 1);  - use the greater of the "base_offset" attribute, and 1.0
        text-content:      "Name: [name]";         - sets the text label to the concatenation of a literal and an attribute value 
    }
    
The numeric expression evaluator supports basic arithmetic (+, -, *, / %), some utility
functions (min, max), and grouping with parentheses. It also works for string values. 
There are no operators, but you can still embed attributes.

If simple expressions are not enough, you can use Javascript::

    <styles>
        <script language="javascript">
            function getOffset() {
                return feature.properties.base_offset + 1.0;
            }
        </script>

        <style type="text/css">
            mystyle {
                extrusion-height: feature.properties.hgt * 0.3048;
                altitude-offset:  getOffset();
            }
        </style>
    </styles>


Terrain Following
-----------------

It is fairly common for features to interact with the terrain in some way.
Requirements for this include things like:

  * Streets that follow the contours of the terrain
  * Trees planted on the ground
  * Thematic mapping, like coloring a country's area based on its population
  
osgEarth offers a variety of terrain following approaches, because no single
approach is best for every situation.


Map Clamping
~~~~~~~~~~~~

*Map Clamping* is the simplest approach. When compiling the features for display,
osgEarth will sample the *elevation layers* in the map, find the height of the terrian,
and apply that to the resulting feature geometry. It will test each point along the
geometry.

Map clamping results in high-quality rendering; the trade-off is performance:

    * It can be slow sampling the elevation data in the map, depending on the 
      resolution you select. For a large number of features, it can be CPU-intensive
      and time-consuming.
    * Sampling is accurate, and done for every point in the geometry. You can opt to
      sample at the *centroid* of each feature to improve compilation speed.
    * Depending on the resolution of the feature geometry, you may need to tessellate
      your data to achieve better quality.
    * The rendering quality is good compared to other methods.
    
You can activate map clamping in your stylesheet like so::

    altitude-clamping:   terrain;        // terrain-following on
    altitude-technique:  map;            // clamp features to the map data
    altitude-resolution: 0.005;          // [optional] resolution of map data to clamp to

    
Draping
~~~~~~~

*Draping* is the process of overlaying compiled geometry on the terrain skin, much
like "draping" a blanket over an uneven surface. osgEarth does this be rendering the
feature to a texture (RTT) and then projecting that texture down onto the terrain.

Draping has its advantages and disadvantages:

    * Draping will conform features perfectly to the terrain; there is no worrying
      about resolution or tessellation.
    * You may get jagged artificats when rendering lines or polygon edges. 
      The projected texture is of limited size, and the larger of an area it must
      cover, the lower the resolution of the image being projected. This means
      that in practice draping is more useful for polygons than for lines.
    * Unexpected blending artifacts may result from draping many transparent
      geometries atop each other.
      
You can activate draping like so::

    altitude-clamping:   terrain;        // terrain-following on
    altitude-technique:  drape;          // drape features with a projective texture
    
    
GPU Clamping
~~~~~~~~~~~~

*GPU Clamping* implements approximate terrain following using GPU shaders. It uses
a two-phase technique: first it uses depth field sampling to clamp each vertex to
the terrain skin in a vertex shader; secondly it applies a depth-offsetting 
algorithm in the fragment shader to mitigate z-fighting.

GPU clamping also has its trade-offs:

    * It is very well suited to lines (or even triangulated lines), but less so
      to polygons because it needs to tessellate the interior of a polygon in order
      to do a good approximate clamping.
    * It is fast, happens completely at runtime, and takes advantage of the GPU's
      parallel processing.
    * There are no jagged-edge effects as there are in draping.
    
Set up GPU clamping like this::

    altitude-clamping:   terrain;        // terrain-following on
    altitude-technique:  gpu;            // clamp and offset feature data on the GPU
    

Rendering Large Datasets
------------------------

The simplest way to load feature data into osgEarth is like this::

   <model name="shapes">
      <features name="data" driver="ogr">
         <url>data.shp</url>
      </features>
      <styles>
         data {
             fill: #ffff00;
         }
      </styles>
   </model>

We just loaded every feature in the shapefile and colored them all yellow.

This works fine up to a point -- the point at which osgEarth (and OSG) become
overloaded with too much geometry. Even with the optimizations that osgEarth's
geometry compiler employs, a large enough dataset can exhaust system resources.

The solution to that is feature tiling and paging. Here is how to configure it.

Feature display layouts
~~~~~~~~~~~~~~~~~~~~~~~

The feature display layout activates paging and tiles of feature data.
Let's modify the previous example::

   <model name="shapes">
      <features name="data" driver="ogr">
         <url>data.shp</url>
      </features>

      <layout>
          <tile_size>250000</tile_size>
          <level name="data" max_range="100000"/>
      </layout>

      <styles>
         data {
             fill: #ffff00;
         }
      </styles>
   </model>
   
The mere presence of the ``<layout>`` element activates paging. This means that
instead of being loaded and compiled at load time, the feature data will load
and compile in the background once the application has started. There may be a
delay before the feature data shows up in the scene, depending on its complexity.

The presence of ``<level>`` elements within the layout actives tiling and levels of
detail. If you OMIT levels, the data will still load in the background, but it will
all load at once. With one or more levels, osgEarth will break up the feature data
into tiles at one or more levels of detail and page those tiles in individually.
More below.

Paging breaks the data up into tiles. The ``tile_size`` is the width (in meters)
of each paged tile.

Cropping features
~~~~~~~~~~~~~~~~~

By default, if a feature intersects the tile, it will be included even if it extends outside 
extents of the tile.  This is useful for things like extruded buildings where it doesn't make sense
to try to chop them to fit exactly in the tiles because you don't want to see half a building page in.
Buildings are also generally small, so the distance that they will extend outside the tile is relatively small.

For things like roads or country borders that are linear features, it might make more sense to crop
them to fit the tile exactly.  Visually a line won't look that bad if you see part if it page in.
You can enable feature cropping on a layout by setting the ``crop_features`` attribute to true on the layout.

For example::

  <model name="roads" driver="feature_geom">
        <features name="roads" driver="ogr" build_spatial_index="true">
              <url>roads.shp</url>
        </features>
          
        <layout crop_features="true" tile_size="1000">
            <level max_range="5000"/>
        </layout>
          
        <styles>
            <style type="text/css">
                roads {
                    stroke:  #ffff7f7f;
                  }
            </style>
        </styles>        
  </model>



Levels
~~~~~~

Each level describes a level of detail. This is a camera range (between ``min_range``
and ``max_range``) at which tiles in this level of detail are rendered. But how
big is each tile? This is calculated based on the *tile range factor*.

The ``tile_size`` sets the size of a tile (in meters).

Why do you care about tile size? Because the density of your data will affect
how much geometry is in each tile. And since OSG (OpenGL, really) benefits from
sending large batches of similar geometry to the graphics card, tweaking the
tile size can help with performance and throughput. Unfortunately there's no way
for osgEarth to know exactly what the "best" tile size will be in advance;
so, you have the opportunity to tweak using this setting.

Layout Settings
~~~~~~~~~~~~~~~

    :tile_size:         The size (in one dimension) of each tile of features in the layout
                        at the maximum range. Maximum range must be set for this to take effect.                                                      
    :tile_size_factor:  The ratio of visibility range to feature tile radius. Default is 15.
                        Increase this to produce more, smaller tiles at a given visibility
                        range; decrease this to produce fewer, larger tiles.
                        For example, for factor=15, at a visibility range of (say) 120,000m
                        the system will attempt to create tiles that are approximately
                        8,000m in radius. (120,000 / 15 = 8,000).
    :max_range:         The desired max range for pre-tiled feature sources like TFS.  The tileSizeFactor will be automatically computed
                        based on the first level of the feature profile so that it shows up at that range.
    :min_range:         Minimum visibility range for all tiles in the layout.
    :crop_features:     Whether to crop geometry to fit within the cell extents when chopping
                        a feature level up into grid cells. By default, this is false, meaning 
                        that a feature whose centroid falls within the cell will be included.
                        Setting this to true means that if any part of the feature falls within
                        the working cell, it will be cropped to the cell extents and used.
    :priority_offset:   Sets the offset that will be applied to the computed paging priority
                        of tiles in this layout. Adjusting this can affect the priority of this
                        data with respect to other paged data in the scene (like terrain or other
                        feature layers).
                        Default = 0.0
    :priority_scale:    Sets the scale factor to be applied to the computed paging priority
                        of tiles in this layout. Adjusting this can affect the priority of this
                        data with respect to other paged data in the scene (like terrain or other
                        feature layers).
                        Default = 1.0.
    :min_expiry_time:   Minimum time, in second, before a feature tile is eligible for pageout.
                        Set this to a negative number to disable expiration altogether (i.e., tiles
                        will never page out).