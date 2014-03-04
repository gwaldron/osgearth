Tools
=====

osgEarth comes with many tools that help you work with earth files and geospatial data.

osgearth_viewer
---------------
osgearth_viewer can load and display a map from and command line.  The osgEarth EarthManipulator is
used to control the camera and is optimized for viewing geospatial data.

**Sample Usage**
::
    osgearth_viewer earthfile.earth [options]


+----------------------------+--------------------------------------------------------------------+
| Option                     | Description                                                        |
+============================+====================================================================+
| ``--sky``                  | Installs a SkyNode (sun, moon, stars and atmosphere..globe only)   |
+----------------------------+--------------------------------------------------------------------+
| ``--ocean``                | Installs a sample ocean surface node                               |
+----------------------------+--------------------------------------------------------------------+
| ``--kml [file.kml]``       | Loads a KML or KMZ file                                            |
+----------------------------+--------------------------------------------------------------------+
| ``--coords``               | Displays map coords under mouse                                    |
+----------------------------+--------------------------------------------------------------------+
| ``--dms``                  | Displays map coords as degrees/mins/seconds                        |
+----------------------------+--------------------------------------------------------------------+
| ``--dd``                   | Displays map coords as decimal degrees                             |
+----------------------------+--------------------------------------------------------------------+
| ``--mgrs``                 | Displays map coords as MGRS                                        |
+----------------------------+--------------------------------------------------------------------+
| ``--ortho``                | Installs an orthographic camera projection                         |
+----------------------------+--------------------------------------------------------------------+
| ``--autoclip``             | Installs an automatic clip plane handler                           |
+----------------------------+--------------------------------------------------------------------+
| ``--images [path]``        | Finds images in [path] and loads them as image layers              |
+----------------------------+--------------------------------------------------------------------+
| ``--image-extensions [*]`` | With ``--images``, only considers the listed extensions            |
+----------------------------+--------------------------------------------------------------------+
| ``--out-earth [out.earth]``| With ``--images``, writes out an earth file                        |
+----------------------------+--------------------------------------------------------------------+


osgearth_version
----------------
**osgearth_version** displays the current version of osgEarth.

+----------------------------+--------------------------------------------------------------------+
| Argument                   | Description                                                        |
+============================+====================================================================+
| ``--caps``                 | Print out system capabilities                                      |
+----------------------------+--------------------------------------------------------------------+
| ``--major-number``         | Print out major version number only                                |
+----------------------------+--------------------------------------------------------------------+
| ``--minor-number``         | Print out minor version number only                                |
+----------------------------+--------------------------------------------------------------------+
| ``--patch-number``         | Print out patch version number only                                |
+----------------------------+--------------------------------------------------------------------+
| ``--so-number``            | Print out shared object version number only                        |
+----------------------------+--------------------------------------------------------------------+
| ``--version-number``       | Print out version number only                                      |
+----------------------------+--------------------------------------------------------------------+

osgearth_cache
--------------
osgearth_cache can be used to manage osgEarth's cache.  See :doc:`/user/caching` for more information on caching.
The most common usage of osgearth_cache is to populate a cache in a non-interactive manner using the ``--seed`` argument.

**Sample Usage**
::
    osgearth_cache --seed file.earth

+-------------------------------------+--------------------------------------------------------------------+
| Argument                            | Description                                                        |
+=====================================+====================================================================+
| ``--list``                          | Lists info about the cache in a .earth file                        |
+-------------------------------------+--------------------------------------------------------------------+
| ``--seed``                          | Seeds the cache in a .earth file                                   |
+-------------------------------------+--------------------------------------------------------------------+
| ``--estimate``                      | Print out an estimation of the number of tiles, disk space and     |
|                                     | time it will take to perform this seed operation                   |
+-------------------------------------+--------------------------------------------------------------------+
| ``--threads``                       |The number of threads to use for the seed operation (default=1)     |
+-------------------------------------+--------------------------------------------------------------------+
| ``--min-level level``               | Lowest LOD level to seed (default=0)                               |
+-------------------------------------+--------------------------------------------------------------------+
| ``--max-level level``               | Highest LOD level to seed (default=highest available)              |
+-------------------------------------+--------------------------------------------------------------------+
| ``--bounds xmin ymin xmax ymax``    | Geospatial bounding box to seed                                    |
|                                     | (in map coordinates; default=entire map                            |
+-------------------------------------+--------------------------------------------------------------------+
| ``--index shapefile``               | Loads a shapefile (.shp) and uses the feature extents to set the   |
|                                     | cache seeding bounding box(es). For each feature in the shapefile, |
|                                     | adds a bounding box (similar to ``--bounds``) to constrain the     |
|                                     | region you wish to cache.                                          |
+-------------------------------------+--------------------------------------------------------------------+
| ``--cache-path path``               | Overrides the cache path in the .earth file                        |
+-------------------------------------+--------------------------------------------------------------------+
| ``--cache-type type``               | Overrides the cache type in the .earth file                        |
+-------------------------------------+--------------------------------------------------------------------+
| ``--purge``                         | Purges a layer cache in a .earth file                              |
+-------------------------------------+--------------------------------------------------------------------+

osgearth_package
----------------
osgearth_package creates a redistributable `TMS`_ based package from an earth file.

**Sample Usage**
::
    osgearth_package --tms file.earth --out package

+------------------------------------+--------------------------------------------------------------------+
| Argument                           | Description                                                        |
+====================================+====================================================================+
| ``--tms``                          | make a TMS repo                                                    |
+------------------------------------+--------------------------------------------------------------------+
| ``--out path``                     | root output folder of the TMS repo (required)                      |
+------------------------------------+--------------------------------------------------------------------+
| ``--bounds xmin ymin xmax ymax``   | bounds to package (in map coordinates; default=entire map)         |
|                                    | You can provide multiple bounds                                    |
+------------------------------------+--------------------------------------------------------------------+
| ``--max-level level``              | max LOD level for tiles (all layers; default=inf)                  |
+------------------------------------+--------------------------------------------------------------------+
| ``--min-level level``              | min LOD level for tiles (all layers; default=0)                    |
+------------------------------------+--------------------------------------------------------------------+
| ``--root-key z/x/y``               | initial tile to start processing                                   |
+------------------------------------+--------------------------------------------------------------------+
| ``--out-earth earthfile``          | export an earth file referencing the new repo                      |
+------------------------------------+--------------------------------------------------------------------+
| ``--ext extension``                | overrides the image file extension (e.g. jpg)                      |
+------------------------------------+--------------------------------------------------------------------+
| ``--overwrite``                    | overwrite existing tiles                                           |
+------------------------------------+--------------------------------------------------------------------+
| ``--keep-empties``                 | writes out fully transparent image tiles (normally discarded)      |
+------------------------------------+--------------------------------------------------------------------+
| ``--continue-single-color``        | continues to subdivide single color tiles,                         |
|                                    | subdivision typicaly stops on single color images                  |
+------------------------------------+--------------------------------------------------------------------+
| ``--db-options``                   | db options string to pass to the image writer                      |
|                                    | in quotes (e.g., "JPEG_QUALITY 60")                                |
+------------------------------------+--------------------------------------------------------------------+

osgearth_tfs
------------
osgearth_tfs generates a TFS dataset from a feature source such as a shapefile.  By pre-processing your features
into the gridded structure provided by TFS you can significantly increase performance of large datasets.
In addition, the TFS package generated can be served by any standard web server, web enabling your dataset.

**Sample Usage**
::
    osgearth_tfs filename

+----------------------------------+--------------------------------------------------------------------+
| Argument                         | Description                                                        |
+==================================+====================================================================+
| ``filename``                     | Shapefile (or other feature source data file )                     |
+----------------------------------+--------------------------------------------------------------------+
| ``--first-level level``          | The first level where features will be added to the quadtree       |
+----------------------------------+--------------------------------------------------------------------+
| ``--max-level level``            | The maximum level of the feature quadtree                          | 
+----------------------------------+--------------------------------------------------------------------+
| ``--max-features``               | The maximum number of features per tile                            |
+----------------------------------+--------------------------------------------------------------------+
| ``--out``                        | The destination directory                                          |
+----------------------------------+--------------------------------------------------------------------+
| ``--layer``                      | The name of the layer to be written to the metadata document       |
+----------------------------------+--------------------------------------------------------------------+
| ``--description``                | The abstract/description of the layer to be written                |
|                                  | to the metadata document                                           |
+----------------------------------+--------------------------------------------------------------------+
| ``--expression``                 | The expression to run on the feature source,                       |
|                                  | specific to the feature source                                     |
+----------------------------------+--------------------------------------------------------------------+
| ``--order-by``                   | Sort the features, if not already included in the expression.      |
|                                  | Append DESC for descending order!                                  |
+----------------------------------+--------------------------------------------------------------------+
| ``--crop``                       | Crops features instead of doing a centroid check.                  |
|                                  | Features can be added to multiple tiles when cropping is enabled   |
+----------------------------------+--------------------------------------------------------------------+
| ``--dest-srs``                   | The destination SRS string in any format osgEarth can              |
|                                  | understand (wkt, proj4, epsg).                                     |
|                                  | If none is specific the source data SRS will be used.              |
+----------------------------------+--------------------------------------------------------------------+

osgearth_backfill
-----------------
osgearth_backfill is a specialty tool that is used to post-process `TMS`_ datasets.  Some web mapping services use different completely different datasets 
at different zoom levels.  For example, they may use NASA BlueMarble imagery until they reach level 4, then abruptly switch to LANDSAT data.  This is fine for
2D slippy map visualization but can be visually distracting when viewed in 3D because neighboring tiles at different LODs look completely different.

osgearth_backfill lets you generate a TMS dataset like you normally would (using osgearth_package or another tool) and then "backfill" lower levels of detail from
a specified higher level of detail.  For example, you can specify a max level of 10 and lods 0-9 will be regenerated based on the data found in level 10.

**Sample Usage**
::
    osgearth_backfill tms.xml

+----------------------------------+--------------------------------------------------------------------+
| Argument                         | Description                                                        |
+==================================+====================================================================+
| ``--bounds xmin ymin xmax ymax`` | bounds to backfill (in map coordinates; default=entire map         |
+----------------------------------+--------------------------------------------------------------------+
| ``--min-level level``            | The minimum level to stop backfilling to. (default=0)              |
+----------------------------------+--------------------------------------------------------------------+
| ``--max-level level``            | The level to start backfilling from(default=inf)                   |
+----------------------------------+--------------------------------------------------------------------+
| ``--db-options``                 | db options string to pass to the                                   |
|                                  | image writer in quotes (e.g., "JPEG_QUALITY 60")                   |
+----------------------------------+--------------------------------------------------------------------+


osgearth_boundarygen
-----------------
osgearth_boundarygen generates boundary geometry that you can use with an osgEarth <mask> layer in order to 
stich an external model into the terrain.

**Sample Usage**
::
    osgearth_boundarygen model_file

+----------------------------------+--------------------------------------------------------------------+
| Argument                         | Description                                                        |
+==================================+====================================================================+
| ``--out file_name``              | output file for boundary geometry( default is boundary.txt)        |
+----------------------------------+--------------------------------------------------------------------+
| ``--no-geocentric``              | Skip geocentric reprojection (for flat databases)                  |
+----------------------------------+--------------------------------------------------------------------+
| ``--convex-hull``                | calculate a convex hull instead of a full boundary                 |
+----------------------------------+--------------------------------------------------------------------+
| ``--verbose``                    | print progress to console                                          |
+----------------------------------+--------------------------------------------------------------------+
| ``--view``                       | show result in 3D window                                           |
+----------------------------------+--------------------------------------------------------------------+



osgearth_overlayviewer
----------------------
**osgearth_overlayviewer** is a utility for debugging the overlay decorator capability in osgEarth.  It shows two windows, one with the normal
view of the map and another that shows the bounding frustums that are used for the overlay computations.

.. _TMS: http://en.wikipedia.org/wiki/Tile_Map_Service




