Using Earth Files
=================

An *Earth File* is an XML description of a map. Creating an *earth file* is the
easiest way to configure a map and get up and running quickly. In the osgEarth
repository you will find dozens of sample earth files in the ``tests`` folder,
covering various topics and demonstrating various features. We encourage you to
explore and try them out!

    Also see: :doc:`/references/earthfile`


Contents of an Earth File
-------------------------
osgEarth uses an XML based file format called an *Earth File* to specify exactly
how source data turns into an OSG scene graph. An Earth File has a ``.earth``
extension, but it is XML.

Fundamentally the Earth File allows you to specify:

* The type of map to create (geocentric or projected)
* The image, elevation, vector and model sources to use
* Where the data will be cached


A Simple Earth File
-------------------
Here is a very simple example that reads data from a GeoTIFF file on the local
file system and renders it as a geocentric round Earth scene::

    <map name="MyMap" type="geocentric" version="2">
        <image name="bluemarble" driver="gdal">
            <url>world.tif</url>
        </image>
    </map>

This Earth File creates a geocentric Map named ``MyMap`` with a single
GeoTIFF image source called ``bluemarble``. The ``driver`` attribute
tells osgEarth which of its plugins to use to use to load the image.
(osgEarth uses a plug-in framework to load different types of data
from different sources.)

Some of the sub-elements (under ``image``) are particular to the selected
driver. To learn more about drivers and how to configure each one, please
refer to the `Driver Reference Guide`_.

    *Note: the ``version`` number is required!*


Multiple Image Layers
---------------------
osgEarth supports maps with multiple image sources.
This allows you to create maps such as base layer with a transportation
overlay or provide high resolution insets for specific areas that sit
atop a lower resolution base map.

To add multiple images to a Earth File, simply add multiple "image" blocks
to your Earth File::

    <map name="Transportation" type="geocentric" version="2">
    
        <!--Add a base map of the blue marble data-->
        <image name="bluemarble" driver="gdal">
            <url>c:/data/bluemarble.tif</url>
        </image>

        <!--Add a high resolution inset of Washington, DC-->
        <image name="dc" driver="gdal">
            <url>c:/data/dc_high_res.tif</url>
        </image>
        
    </map>

The above map provides two images from local data sources using the GDAL driver.
Order is important when defining multiple image sources: osgEarth renders them
in the order in which they appear in the Earth File.

    *Tip: relative paths within an Earth File are interpreted
    as being relative to the Earth File itself.*


Adding Elevation Data
---------------------
Adding elevation data (sometimes called "terrain data") to an Earth File is 
very similar to adding images. Use an ``elevation`` block like so::

    <map name="Elevation" type="geocentric" version="2">
    
        <!--Add a base map of the blue marble data-->
        <image name="bluemarble" driver="gdal">
            <url>c:/data/bluemarble.tif</url>
        </image>

        <!--Add SRTM data-->
        <elevation name="srtm" driver="gdal">
            <url>c:/data/SRTM.tif</url>
        </elevation>
        
    </map>

This Earth File has a base ``bluemarble`` image as well as a elevation
grid that is loaded from a local GeoTIFF file. You can add as many elevation
layers as you like; osgEarth will combine them into a single mesh.

As with images, order is important - For example, if you have a base
elevation data source with low-resolution coverage of the entire world and
a high-resolution inset of a city, you need specify the base data FIRST,
followed by the high-resolution inset.

Some osgEarth drivers can generate elevation grids as well as imagery.

    *Note: osgEarth only supports single-channel 16-bit integer or 32-bit
    floating point data for use in elevation layers.*


Caching
-------
Since osgEarth renders data on demand, it sometimes needs to do some work in
order to prepare a tile for display. The *cache* exists so that osgEarth can
save the results of this work for next time, instead of processing the tile
anew each time. This increases performance and avoids multiple downloads of
the same data. 

Here's an example cache setup::

    <map name="TMS Example" type="geocentric" version="2">
    
        <image name="metacarta blue marble" driver="tms">
            <url>http://labs.metacarta.com/wms-c/Basic.py/1.0.0/satellite/</url>
        </image>

        <options>
            <!--Specify where to cache the data-->
            <cache type="filesystem">
                <path>c:/osgearth_cache</path>
            </cache>
        </options>
        
    </map>
    
This Earth File shows the most basic way to specify a cache for osgEarth.
This tells osgEarth to enable caching and to cache to the folder ``c:/osgearth_cache``.
The cache path can be relative or absolute; relative paths are relative to the 
Earth File itself.

There are many ways to configure caching; please refer to the section on Caching_ for
more details.

.. _Driver Reference Guide:     #
