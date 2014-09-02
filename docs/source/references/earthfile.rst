Earth File Reference
====================

Map
~~~
The *map* is the top-level element in an earth file.

.. parsed-literal::

    <map name    = "my map"
         type    = "geocentric"
         version = "2" >
         
        <:ref:`options   <MapOptions>`>
        <:ref:`image     <ImageLayer>`>
        <:ref:`elevation <ElevationLayer>`>
        <:ref:`model     <ModelLayer>`>
        <:ref:`mask      <MaskLayer>`>
        

+------------------------+--------------------------------------------------------------------+
| Property               | Description                                                        |
+========================+====================================================================+
| name                   | Readable name. No effect on rendering.                             |
+------------------------+--------------------------------------------------------------------+
| type                   | Coordinate system type.                                            |
|                        |   :geocentric:  Render an ellipsoidal globe.                       |
|                        |   :projected:   Render a "flat", projected map.                    |
+------------------------+--------------------------------------------------------------------+
| version                | Earth file version. default = "2". Set this to load an older earth |
|                        | file format.                                                       |
+------------------------+--------------------------------------------------------------------+


.. _MapOptions:

Map Options
~~~~~~~~~~~
These options control both the Map Model and the rendering properties associated with
the entire map.

.. parsed-literal::

    <map>
        <options lighting                 = "true"
                 elevation_interpolation  = "bilinear"
                 elevation_tile_size      = "17"
                 overlay_texture_size     = "4096"
                 overlay_blending         = "true"
                 overlay_resolution_ratio = "3.0" >

            <:ref:`profile <Profile>`>
            <:ref:`proxy <ProxySettings>`>
            <:ref:`cache <Cache>`>
            <:ref:`cache_policy <CachePolicy>`>
            <:ref:`terrain <TerrainOptions>`>

+--------------------------+--------------------------------------------------------------------+
| Property                 | Description                                                        |
+==========================+====================================================================+
| lighting                 | Whether to enable GL_LIGHTING on the entire map. By default this is|
|                          | unset, meaning it will inherit the lighting mode of the scene.     |
+--------------------------+--------------------------------------------------------------------+
| elevation_interpolation  | Algorithm to use when resampling elevation source data:            |
|                          |   :nearest:     Nearest neighbor                                   |
|                          |   :average:     Averages the neighoring values                     |
|                          |   :bilinear:    Linear interpolation in both axes                  |
|                          |   :triangulate: Interp follows triangle slope                      |
+--------------------------+--------------------------------------------------------------------+
| elevation_tile_size      | Sets the number of samples to render for each terrain tile         |
|                          | (width and height)                                                 |
+--------------------------+--------------------------------------------------------------------+
| overlay_texture_size     | Sets the texture size to use for draping (projective texturing)    |
+--------------------------+--------------------------------------------------------------------+
| overlay_blending         | Whether overlay geometry blends with the terrain during draping    |
|                          | (projective texturing                                              |
+--------------------------+--------------------------------------------------------------------+
| overlay_resolution_ratio | For draped geometry, the ratio of the resolution of the projective |
|                          | texture near the camera versus the resolution far from the camera. |
|                          | Increase the value to improve appearance close to the camera while |
|                          | sacrificing appearance of farther geometry. NOTE: If you're using  |
|                          | a camera manipulator that support roll, you will probably need to  |
|                          | set this to 1.0; otherwise you will get draping artifacts! This is |
|                          | a known issue.                                                     |
+--------------------------+--------------------------------------------------------------------+


.. _TerrainOptions:

Terrain Options
~~~~~~~~~~~~~~~
These options control the rendering of the terrain surface.

.. parsed-literal::

    <map>
        <options>
            <terrain driver                = "mp"
                     lighting              = "true"
                     min_tile_range_factor = "6"
                     min_lod               = "0"
                     max_lod               = "23"
                     first_lod             = "0"
                     cluster_culling       = "true"
                     mercator_fast_path    = "true"
                     blending              = "false"
                     color                 = "#ffffffff" >

+-----------------------+--------------------------------------------------------------------+
| Property              | Description                                                        |
+=======================+====================================================================+
| driver                | Terrain engine plugin to load. Default = "mp".                     |
|                       | Please refer to the driver reference guide for properties specific |
|                       | to each individual plugin.                                         |
+-----------------------+--------------------------------------------------------------------+
| lighting              | Whether to enable GL_LIGHTING on the terrain. By default this is   |
|                       | unset, meaning it will inherit the lighting mode of the scene.     |
+-----------------------+--------------------------------------------------------------------+
| min_tile_range_factor | Determines how close you need to be to a terrain tile for it to    |
|                       | display. The value is the ratio of a tile's extent to its          |
|                       | For example, if a tile is 10km is radius, and the MTRF=6, then the |
|                       | tile will become visible at a range of about 60km.                 |
+-----------------------+--------------------------------------------------------------------+
| min_lod               | The lowest level of detail that the terrain is guaranteed to       |
|                       | display, even if no source data is available at that LOD. The      |
|                       | terrain will continue to subdivide up to this LOD even if it runs  |
|                       | out of data.                                                       |
+-----------------------+--------------------------------------------------------------------+
| max_lod               | The highest level of detail at which the terrain will render, even |
|                       | if there is higher resolution source data available.               |
+-----------------------+--------------------------------------------------------------------+
| first_lod             | The lowest level of detail at which the terrain will display tiles.|
|                       | I.e., the terrain will never display a lower LOD than this.        |
+-----------------------+--------------------------------------------------------------------+
| cluster_culling       | Disable "cluster culling" by setting this to ``false``. You may    |
|                       | wish to do this is you are placing the camera underground.         |
+-----------------------+--------------------------------------------------------------------+
| mercator_fast_path    | The *mercator fast path* allows the renderer to display Mercator   |
|                       | projection imagery without reprojecting it. You can disable this   |
|                       | technique (and allow reprojection as necessary) by setting this    |
|                       | to ``false``.                                                      |
+-----------------------+--------------------------------------------------------------------+
| blending              | Set this to ``true`` to enable GL blending on the terrain's        |
|                       | underlying geometry. This lets you make the globe partially        |
|                       | transparent. This is handy for seeing underground objects.         |
+-----------------------+--------------------------------------------------------------------+
| color                 | Color on the underlying (untextures) terrain.                      |
+-----------------------+--------------------------------------------------------------------+



.. _ImageLayer:

Image Layer
~~~~~~~~~~~
An *image layer* is a raster image overlaid on the map's geometry.

.. parsed-literal::

    <map>
        <image name           = "my image layer"
               driver         = "gdal"
               nodata_image   = "http://readymap.org/nodata.png"
               opacity        = "1.0"
               min_range      = "0"
               max_range      = "100000000"
               min_level      = "0"
               max_level      = "23"
               min_resolution = "100.0"
               max_resolution = "0.0"
               enabled        = "true"
               visible        = "true"
               shared         = "false"
               feather_pixels = "false"
               min_filter     = "LINEAR"
               mag_filter     = "LINEAR" 
               texture_compression = "auto" >

            <:ref:`cache_policy <CachePolicy>`>
            <:ref:`color_filters <ColorFilterChain>`>
            <:ref:`proxy <ProxySettings>`>


+-----------------------+--------------------------------------------------------------------+
| Property              | Description                                                        |
+=======================+====================================================================+
| name                  | Readable layer name. Not used in the engine.                       |
+-----------------------+--------------------------------------------------------------------+
| driver                | Plugin to use to create tiles for this layer.                      |
|                       | Please refer to the driver reference guide for properties specific |
|                       | to each individual plugin.                                         |
+-----------------------+--------------------------------------------------------------------+
| nodata_image          | URI of an image that represents "no data" in the source. If        |
|                       | osgEarth matches a tile to this image, it will act as if it found  |
|                       | no data at that location and it will *not* render the tile.        |
+-----------------------+--------------------------------------------------------------------+
| opacity               | The layer's opacity, [0..1].                                       |
+-----------------------+--------------------------------------------------------------------+
| min_range             | Minimum visibility range, in meters from the camera. If the camera |
|                       | gets closer than this, the tile will not be visible.               |
+-----------------------+--------------------------------------------------------------------+
| max_range             | Maximum visibility range, in meters from the camera. The tile will |
|                       | not be drawn beyond this range.                                    |
+-----------------------+--------------------------------------------------------------------+
| min_level             | Minimum visibility level of detail.                                |
+-----------------------+--------------------------------------------------------------------+
| max_level             | Maximum visibility level of detail.                                |
+-----------------------+--------------------------------------------------------------------+
| min_resolution        | Minimum source data resolution at which to draw tiles. Value is    |
|                       | units per pixel, in the native units of the source data.           |
+-----------------------+--------------------------------------------------------------------+
| max_resolution        | Maximum source data resolution at which to draw tiles. Value is    |
|                       | units per pixel, in the native units of the source data.           |
+-----------------------+--------------------------------------------------------------------+
| max_data_level        | Maximum level of detail at which new source data is available to   |
|                       | this image layer. Usually the driver will report this information. |
|                       | But you may wish to limit it yourself. This is especially true for |
|                       | some drivers that have no resolution limit, like a rasterization   |
|                       | driver (agglite) for example.                                      |
+-----------------------+--------------------------------------------------------------------+
| enabled               | Whether to include this layer in the map. You can only set this at |
|                       | load time; it is just an easy way of "commenting out" a layer in   |
|                       | the earth file.                                                    |
+-----------------------+--------------------------------------------------------------------+
| visible               | Whether to draw the layer.                                         |
+-----------------------+--------------------------------------------------------------------+
| shared                | Generates a secondary, dedicated sampler for this layer so that it |
|                       | may be accessed globally by custom shaders.                        |
+-----------------------+--------------------------------------------------------------------+
| feather_pixels        | Whether to feather out alpha regions for this image layer with the |
|                       | featherAlphaRegions function. Used to get proper blending when you |
|                       | have datasets that abutt exactly with no overlap.                  |
+-----------------------+--------------------------------------------------------------------+
| min_filter            | OpenGL texture minification filter to use for this layer.          |
|                       | Options are NEAREST, LINEAR, NEAREST_MIPMAP_NEAREST,               |
|                       | NEAREST_MIPMIP_LINEAR, LINEAR_MIPMAP_NEAREST, LINEAR_MIPMAP_LINEAR |
+-----------------------+--------------------------------------------------------------------+
| mag_filter            | OpenGL texture magnification filter to use for this layer.         |
|                       | Options are the same as for ``min_filter`` above.                  |
+-----------------------+--------------------------------------------------------------------+
| texture_compression   | "auto" to compress textures on the GPU; "none" to disable.         |
+-----------------------+--------------------------------------------------------------------+


.. _ElevationLayer:

Elevation Layer
~~~~~~~~~~~~~~~
An *Elevation Layer* provides heightmap grids to the terrain engine. The osgEarth engine
will composite all elevation data into a single heightmap and use that to build a terrain tile.

.. parsed-literal::

    <map>
        <elevation name           = "text"
                   driver         = "gdal"
                   min_level      = "0"
                   max_level      = "23"
                   min_resolution = "100.0"
                   max_resolution = "0.0"
                   enabled        = "true"
                   offset         = "false"
                   nodata_policy  = "interpolate" >


+-----------------------+--------------------------------------------------------------------+
| Property              | Description                                                        |
+=======================+====================================================================+
| name                  | Readable layer name. Not used in the engine.                       |
+-----------------------+--------------------------------------------------------------------+
| driver                | Plugin to use to create tiles for this layer.                      |
|                       | Please refer to the driver reference guide for properties specific |
|                       | to each individual plugin.                                         |
+-----------------------+--------------------------------------------------------------------+
| min_level             | Minimum visibility level of detail.                                |
+-----------------------+--------------------------------------------------------------------+
| max_level             | Maximum visibility level of detail.                                |
+-----------------------+--------------------------------------------------------------------+
| min_resolution        | Minimum source data resolution at which to draw tiles. Value is    |
|                       | units per pixel, in the native units of the source data.           |
+-----------------------+--------------------------------------------------------------------+
| max_resolution        | Maximum source data resolution at which to draw tiles. Value is    |
|                       | units per pixel, in the native units of the source data.           |
+-----------------------+--------------------------------------------------------------------+
| enabled               | Whether to include this layer in the map. You can only set this at |
|                       | load time; it is just an easy way of "commenting out" a layer in   |
|                       | the earth file.                                                    |
+-----------------------+--------------------------------------------------------------------+
| offset                | Indicates that the height values in this layer are relative        |
|                       | offsets rather than true terrain height samples.                   |
+-----------------------+--------------------------------------------------------------------+
| nodata_policy         | What to do with "no data" values. Default is "interpolate" which   |
|                       | will interpolate neighboring values to fill holes. Set it to "msl" |
|                       | to replace "no data" samples with the current sea level value.     |
+-----------------------+--------------------------------------------------------------------+


.. _ModelLayer:

Model Layer
~~~~~~~~~~~
A *Model Layer* renders non-terrain data, like vector features or external 3D models.

.. parsed-literal::

    <map>
        <model name    = "my model layer"
               driver  = "feature_geom"
               enabled = true
               visible = true >


+-----------------------+--------------------------------------------------------------------+
| Property              | Description                                                        |
+=======================+====================================================================+
| name                  | Readable layer name. Not used in the engine.                       |
+-----------------------+--------------------------------------------------------------------+
| driver                | Plugin to use to create tiles for this layer.                      |
|                       | Please refer to the driver reference guide for properties specific |
|                       | to each individual plugin.                                         |
+-----------------------+--------------------------------------------------------------------+
| enabled               | Whether to include this layer in the map. You can only set this at |
|                       | load time; it is just an easy way of "commenting out" a layer in   |
|                       | the earth file.                                                    |
+-----------------------+--------------------------------------------------------------------+
| visible               | Whether to draw the layer.                                         |
+-----------------------+--------------------------------------------------------------------+

The Model Layer also allows you to define a cut-out mask. The terrain engine will cut a hole
in the terrain surface matching a *boundary geometry* that you supply. You can use the tool
*osgearth_boundarygen* to create such a geometry.

This is useful if you have an external terrain model and you want to insert it into the 
osgEarth terrain. The model MUST be in the same coordinate system as the terrain.

.. parsed-literal::

    <map>
        <model ...>
            <mask driver="feature">
                <features driver="ogr">
                    ...

The Mask can take any polygon feature as input. You can specify masking geometry inline
by using an inline geometry:

.. parsed-literal::
    
    <features ...>
        <geometry>POLYGON((120 42 0, 121 41 0, 121 40 0))</geometry>

Or you use a shapefile or other feature source, in which case osgEarth will use the 
*first* feature in the source.

Refer to the *mask.earth* sample for an example.



.. _Profile:

Profile
~~~~~~~
The profile tells osgEarth the spatial reference system, the geospatial extents, and the
tiling scheme that it should use to render map tiles.

.. parsed-literal::

    <profile srs    = "+proj=utm +zone=17 +ellps=GRS80 +datum=NAD83 +units=m +no_defs"
             vdatum = "egm96"
             xmin   = "560725.500"
             xmax   = "573866.500"
             ymin   = "4385762.500"
             ymax   = "4400705.500"
             num_tiles_wide_at_lod_0 = "1"
             num_tiles_high_at_lod_0 = "1">

+-----------------------+--------------------------------------------------------------------+
| Property              | Description                                                        |
+=======================+====================================================================+
| srs                   | Spatial reference system of the map. This can be a WKT string, an  |
|                       | ESPG code, a PROJ4 initialization string, or a stock profile name. |
|                       | Please refer to :doc:`/user/spatialreference` for details.         |
+-----------------------+--------------------------------------------------------------------+
| vdatum                | Vertical datum of the profile, which describes how to treat        |
|                       | Z values. Please refer to :doc:`/user/spatialreference` for        |
|                       | details.                                                           |
+-----------------------+--------------------------------------------------------------------+
| xmin, xmax, ymin, ymax| Geospatial extent of the map. The units are those defined by the   |
|                       | SRS above (usually meters for a projected map, degrees for a       |
|                       | geocentric map).                                                   |
+-----------------------+--------------------------------------------------------------------+
| num_tiles_*_at_lod_0  | Size of the tile hierarchy's top-most level. Default is "1" in both|
|                       | directions. (*optional*)                                           |
+-----------------------+--------------------------------------------------------------------+


.. _Cache:

Cache
~~~~~
Configures a cache for tile data.

.. parsed-literal::

    <cache driver = "filesystem"
           path   = "c:/osgearth_cache" >


+-----------------------+--------------------------------------------------------------------+
| Property              | Description                                                        |
+=======================+====================================================================+
| driver                | Plugin to use for caching.                                         |
|                       | At the moment there is only one caching plugin that comes with     |
|                       | osgEarth, the ``filesystem`` plugin.                               |
+-----------------------+--------------------------------------------------------------------+
| path                  | Path (relative or absolute) or the root of a ``filesystem`` cache. |
+-----------------------+--------------------------------------------------------------------+


.. _CachePolicy:

CachePolicy
~~~~~~~~~~~
Policy that determines how a given element will interact with a configured cache.

.. parsed-literal::

    <cache_policy usage="no_cache">


+-----------------------+--------------------------------------------------------------------+
| Property              | Description                                                        |
+=======================+====================================================================+
| usage                 | Policy towards the cache.                                          |
|                       |   :read_write:  Use a cache if one is configured. The default.     |
|                       |   :cache_only:  ONLY read data from the cache, ignoring the actual |
|                       |                 data source. This is nice for offline rendering.   |
|                       |   :no_cache:    Ignore caching and always read from the data       |
|                       |                 source.                                            |
+-----------------------+--------------------------------------------------------------------+



.. _ProxySettings:

Proxy Settings
~~~~~~~~~~~~~~
*Proxy settings* let you configure a network proxy for remote data sources.

.. parsed-literal::

    <proxy host     = "hostname"
           port     = "8080"
           username = "jason"
           password = "helloworld" >
           
Hopefully the properties are self-explanatory.



.. _ColorFilterChain:

Color Filters
~~~~~~~~~~~~~
A *color filter* is a pluggable shader that can alter the appearance of the
color data in a layer before the osgEarth engine composites it into the terrain.

.. parsed-literal::

    <image>
        <color_filters>
            <gamma rgb="1.3">
            ...
            
You can chain multiple color filters together. Please refer to :doc:`/references/colorfilters` for
details on color filters.
