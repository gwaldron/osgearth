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
        <:ref:`libraries <Libraries>`>

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
| lighting                 | Whether to allow lighting shaders to affect the map.               |
+--------------------------+--------------------------------------------------------------------+
| elevation_interpolation  | Algorithm to use when resampling elevation source data:            |
|                          |   :nearest:     Nearest neighbor                                   |
|                          |   :average:     Averages the neighoring values                     |
|                          |   :bilinear:    Linear interpolation in both axes                  |
|                          |   :triangulate: Interp follows triangle slope                      |
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
            <terrain driver                = "rex"
                     lighting              = "true"
                     min_tile_range_factor = "6"
                     first_lod             = "0"
                     blending              = "false"
                     color                 = "#ffffffff"
                     tile_size             = "17"
                     normalize_edges       = "false"
                     compress_normal_maps  = "false"
                     normal_maps           = "true"
                     min_expiry_frames     = "0"
                     min_expiry_time       = "0" >

+-----------------------+--------------------------------------------------------------------+
| Property              | Description                                                        |
+=======================+====================================================================+
| driver                | Terrain engine plugin to load. Default = "rex".                    |
|                       | Please refer to the driver reference guide for properties specific |
|                       | to each individual plugin.                                         |
+-----------------------+--------------------------------------------------------------------+
| lighting              | Whether the terrain will accept lighting if present. Default=true  |
+-----------------------+--------------------------------------------------------------------+
| min_tile_range_factor | Determines how close you need to be to a terrain tile for it to    |
|                       | display. The value is the ratio of a tile's extent to its          |
|                       | For example, if a tile has a 10km radius, and the MTRF=7, then the |
|                       | tile will become visible at a range of about 70km. Default=6.0     |
+-----------------------+--------------------------------------------------------------------+
| first_lod             | The lowest level of detail at which the terrain will display tiles.|
|                       | I.e., the terrain will never display a lower LOD than this.        |
+-----------------------+--------------------------------------------------------------------+
| blending              | Set this to ``true`` to enable GL blending on the terrain's        |
|                       | underlying geometry. This lets you make the globe partially        |
|                       | transparent. This is handy for seeing underground objects.         |
+-----------------------+--------------------------------------------------------------------+
| tile_size             | The dimensions of each terrain tile. Each terrain tile will have   |
|                       | ``tile_size`` X ``tile_size`` vertices. Default=17                 |
+-----------------------+--------------------------------------------------------------------+
| normalize_edges       | Calculate normal vectors along the edges of terrain tiles so that  |
|                       | lighting appears smoother from one tile to the next. Default=false |
+-----------------------+--------------------------------------------------------------------+
| normal_maps           | Whether to generate and use normal maps in place of geometry       |
|                       | normals. Normal maps are used with lighting to create the          |
|                       | appearance of higher-resolution terrain than can be represented    |
|                       | with triangles alone. Default is engine-dependent.                 |
+-----------------------+--------------------------------------------------------------------+
| compress_normal_maps  | Whether to compress normal maps before sending them to the GPU.    |
|                       | You must have the NVIDIA Texture Tools image processor plugin      |
|                       | built in your OpenSceneGraph build.  Default is false              |
+-----------------------+--------------------------------------------------------------------+
| min_expiry_frames     | The number of frames that a terrain tile hasn't been seen before   |
|                       | it can be considered for expiration. Default = 0                   |
+-----------------------+--------------------------------------------------------------------+
| min_expiry_time       | The number of seconds that a terrain tile hasn't been culled before|
|                       | it can be considered for expiration. Default = 0                   |
+-----------------------+--------------------------------------------------------------------+


.. _ImageLayer:

Image Layer
~~~~~~~~~~~
An *image layer* is a raster image overlaid on the map's geometry.

.. parsed-literal::

    <map>
        <image name              = "my image layer"
               driver            = "gdal"
               nodata_image      = "http://readymap.org/nodata.png"
               opacity           = "1.0"
               min_range         = "0"
               max_range         = "100000000"
               attenuation_range = "0"
               min_level         = "0"
               max_level         = "23"
               min_resolution    = "100.0"
               max_resolution    = "0.0"
               max_data_level    = "23"
               enabled           = "true"
               visible           = "true"
               shared            = "false"
               shared_sampler    = "string"
               shared_matrix     = "string"
               coverage          = "false"
               min_filter        = "LINEAR"
               mag_filter        = "LINEAR"
               blend             = "interpolate"
               altitude          = "0"
               texture_compression = "none" >

            <:ref:`cache_policy <CachePolicy>`>
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
| attenuation_range     | Distance over which to blend towards min_range or max_range.       |
|                       | (not supported for text or icons, only geometry)                   |
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
| shared_sampler        | For a shared layer, the uniform name of the sampler that will be   |
|                       | available in GLSL code.                                            |
+-----------------------+--------------------------------------------------------------------+
| shared_matrix         | For a shared layer, the uniform name of the texture matrix that    |
|                       | will be available in GLSL code that you can use to access          |
|                       | the proper texture coordinate for the ``shared_sampler`` above.    |
+-----------------------+--------------------------------------------------------------------+
| coverage              | Indicates that this is a coverage layer, i.e. a layer that conveys |
|                       | discrete values with particular semantics. An example would be a   |
|                       | "land use" layer in which each pixel holds a value that indicates  |
|                       | whether the area is grassland, desert, etc. Marking a layer as a   |
|                       | coverage disables any interpolation, filtering, or compression as  |
|                       | these will corrupt the sampled data values on the GPU.             |
+-----------------------+--------------------------------------------------------------------+
| min_filter            | OpenGL texture minification filter to use for this layer.          |
|                       | Options are NEAREST, LINEAR, NEAREST_MIPMAP_NEAREST,               |
|                       | NEAREST_MIPMIP_LINEAR, LINEAR_MIPMAP_NEAREST, LINEAR_MIPMAP_LINEAR |
+-----------------------+--------------------------------------------------------------------+
| mag_filter            | OpenGL texture magnification filter to use for this layer.         |
|                       | Options are the same as for ``min_filter`` above.                  |
+-----------------------+--------------------------------------------------------------------+
| texture_compression   | "auto" to compress textures on the GPU;                            |
|                       | "none" to disable.                                                 |
|                       | "fastdxt" to use the FastDXT real time DXT compressor              |
+-----------------------+--------------------------------------------------------------------+
| blend                 | "modulate" to multiply pixels with the framebuffer;                |
|                       | "interpolate" to blend with the framebuffer based on alpha (def)   |
+-----------------------+--------------------------------------------------------------------+
| altitude              | Meters above sea level at which to render this image layer. You    |
|                       | can use this to render a weather or cloud layer above the ground,  |
|                       | for example, as a visual aide. Default=0                           |
+-----------------------+--------------------------------------------------------------------+


.. _ElevationLayer:

Elevation Layer
~~~~~~~~~~~~~~~
An *Elevation Layer* provides heightmap grids to the terrain engine. The osgEarth engine
will composite all elevation data into a single heightmap and use that to build a terrain tile.

.. parsed-literal::

    <map>
        <elevation name            = "text"
                   driver          = "gdal"
                   min_level       = "0"
                   max_level       = "23"
                   min_resolution  = "100.0"
                   max_resolution  = "0.0"
                   enabled         = "true"
                   offset          = "false"
                   nodata_value    = "-32768"
                   min_valid_value = "-32768"
                   max_valid_value = "32768"
                   nodata_policy   = "interpolate" >


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
| nodata_value          | Treat this value as "no data".                                     |
+-----------------------+--------------------------------------------------------------------+
| min_valid_value       | Treat anything less than this value as "no data".                  |
+-----------------------+--------------------------------------------------------------------+
| max_valid_value       | Treat anything greater than this value as "no data".               |
+-----------------------+--------------------------------------------------------------------+


.. _ModelLayer:

Model Layer
~~~~~~~~~~~
A *Model Layer* renders an external 3D model as a map layer.

.. parsed-literal::

    <map>
        <model name    = "my model layer"
               driver  = "simple" >


+-----------------------+--------------------------------------------------------------------+
| Property              | Description                                                        |
+=======================+====================================================================+
| name                  | Readable layer name. Not used in the engine.                       |
+-----------------------+--------------------------------------------------------------------+
| driver                | Plugin to use to create tiles for this layer.                      |
|                       | Please refer to the driver reference guide for properties specific |
|                       | to each individual plugin.                                         |
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
|                       | EPSG code, a PROJ4 initialization string, or a stock profile name. |
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
| driver                | Plugin to use for caching, ``filesystem`` or ``leveldb``.          |
+-----------------------+--------------------------------------------------------------------+
| path                  | Path (relative or absolute) or the cache folder or file.           |
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
| max_age               | Treat cache entries older than this value (in seconds) as expired. |
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

.. _Libraries:

Libraries
~~~~~~~~~
Preload any libraries.

.. parsed-literal::

    <libraries>a</libraries>

Multiple library names could be listed by using ';' as separator.

    <libraries>a;b;c;d;e</libraries>

The libraries are searched in the osg library path and library name needs to follow the osg nodekit library name convention (postfixed with osg library version)
