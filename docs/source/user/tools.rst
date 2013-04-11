Tools
============

osgEarth comes with many tools that help you work with earth files and geospatial data.

osgearth_viewer
---------------
osgearth_viewer can load and display a map from and command line.  The osgEarth EarthManipulator is
used to control the camera and is optimized for viewing geospatial data.

**Sample Usage**
::
    osgearth_viewer earthfile.earth


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
| ``--min-level level``               | Lowest LOD level to seed (default=0)                               |
+-------------------------------------+--------------------------------------------------------------------+
| ``--max-level level``               | Highest LOD level to seed (default=highest available)              |
+-------------------------------------+--------------------------------------------------------------------+
| ``--bounds xmin ymin xmax ymax``    | Geospatial bounding box to seed                                    |
|                                     | (in map coordinates; default=entire map                            |
+-------------------------------------+--------------------------------------------------------------------+
| ``--cache-path path``               | Overrides the cache path in the .earth file                        |
+-------------------------------------+--------------------------------------------------------------------+
| ``--cache-type type``               | Overrides the cache type in the .earth file                        |
+-------------------------------------+--------------------------------------------------------------------+
| ``--purge``                         | Purges a layer cache in a .earth file                              |
+-------------------------------------+--------------------------------------------------------------------+       

osgearth_package
----------------
todo

osgearth_tfs
------------
todo

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
todo


osgearth_featureinfo
--------------------
todo

osgearth_overlayviewer
----------------------
**osgearth_overlayviewer** is a utility for debugging the overlay decorator capability in osgEarth.  It shows two windows, one with the normal
view of the map and another that shows the bounding frustums that are used for the overlay computations.

.. _TMS: http://en.wikipedia.org/wiki/Tile_Map_Service




