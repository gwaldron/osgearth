Working with Data
=================

Where to Find Data
------------------

Help us add useful sources of Free data to this list.

**Raster data**

    * `ReadyMap.org`_ - Free 15m imagery, elevation, and street tiles for osgEarth developers
    
    * `USGS National Map`_ - Elevation, orthoimagery, hydrography, geographic names, boundaries,
      transportation, structures, and land cover products for the US.
    
    * `NASA BlueMarble`_ - NASA's whole-earth imagery (including topography and bathymetry maps)
    
    * `Natural Earth`_ - Free vector and raster map data at various scales
    
    * `Virtual Terrain Project`_ - Various sources for whole-earth imagery
    
    * `Bing Maps`_ - Microsoft's worldwide imagery and map data ($)
        
        
**Elevation data**

    * `CGIAR`_ - World 90m elevation data derived from SRTM and ETOPO (`CGIAR European mirror`_)
    
    * `SRTM30+`_ - Worldwide elevation coverage (including bathymetry)
    
    * `GLCF`_ - UMD's Global Land Cover Facility (they also have mosaiced LANDSAT data)
    
    * `GEBCO`_ - Genearl Batymetry Chart of the Oceans

**Feature data**

    * `OpenStreetMap`_ - Worldwide, community-sources street and land use data (vectors and rasterized tiles)
    
    * `Natural Earth`_ - Free vector and raster map data at various scales
    
    * `DIVA-GIS`_ - Free low-resolution vector data for any country
    

.. _CGIAR:                      http://srtm.csi.cgiar.org/
.. _CGIAR European mirror:      ftp://xftp.jrc.it/pub/srtmV4/
.. _DIVA-GIS:                   http://www.diva-gis.org/gData
.. _GEBCO:                      http://www.gebco.net/
.. _GLCF:                       http://glcf.umiacs.umd.edu/data/srtm/
.. _OpenStreetMap:              http://openstreetmap.org
.. _NASA BlueMarble:            http://visibleearth.nasa.gov/view_cat.php?categoryID=1484
.. _Natural Earth:              http://www.naturalearthdata.com/
.. _SRTM30+:                    ftp://topex.ucsd.edu/pub/srtm30_plus/
.. _USGS National Map:          http://nationalmap.gov/viewer.html
.. _Virtual Terrain Project:    http://vterrain.org/Imagery/WholeEarth/
.. _Bing Maps:                  http://www.microsoft.com/maps/choose-your-bing-maps-API.aspx
.. _ReadyMap.org:               http://readymap.org

----

Tips for Preparing your own Data
--------------------------------

**Processing Local Source Data**

    If you have geospatial data that you would like to view in osgEarth, you can usually use the GDAL driver.
    If you plan on doing this, try loading it as-is first.
    If you find that it's too slow, here are some tips for optimizing your data for tiled access.
    
    **Reproject your data**

    osgEarth will reproject your data on-the-fly if it does not have the necessary
    coordinate system.  For instance, if you are trying to view a UTM image on a
    geodetic globe (epsg:4326).  However, osgEarth will run much faster if your data
    is already in the correct coordinate system.  You can use any tool you want to 
    reproject your data such as GDAL, Global Mapper or ArcGIS.
    
    For example, to reproject a UTM image to geodetic using gdal_warp::

        gdalwarp -t_srs epsg:4326 my_utm_image.tif my_gd_image.tif

    **Build internal tiles**
    
    Typically formats such as GeoTiff store their pixel data in scanlines.
    However, using a tiled dataset will be more efficient for osgEarth because
    of how it uses tiles internally.
    
    To create a tiled GeoTiff using gdal_translate, issue the following command::
    
        gdal_translate -of GTiff -co TILED=YES input.tif output.tif
        
    Take it a step further and use compression to save space. You can use internal
    JPEG compression if your data contains no transparency::
    
        gdal_translate -of GTiff -co TILED=YES -co COMPRESS=JPG input.tif output.tif   
    

    **Build overviews**
    
    Adding overviews (also called ''pyramids'' or ''rsets'') can sometimes increase
    the performance of a large data source in osgEarth.  You can use the
    `gdaladdo <http://gdal.org/gdaladdo.html>`_ utility to add overviews to a dataset::
    
        gdaladdo -r average myimage.tif 2 4 8 16

        
**Spatial indexing for feature data**

    Large vector feature datasets (e.g., shapefiles) will benefit greatly from a spatial index.
    Using the *ogrinfo* tool (included with GDAL/OGR binary distributions) you can create a 
    spatial index for your vector data like so::

        ogrinfo -sql "CREATE SPATIAL INDEX ON myfile" myfile.shp

    For shapefiles, this will generate a ".qix" file that contains the spatial index information.


**Building tile sets with osgearth_conv**

   Pre-tiling your imagery can speed up load time dramatically, especially over the network.   
   In fact, if you want to serve your data over the network, this is the only way!

   *osgearth_conv* is a low-level conversion tool that comes with osgEarth. One useful 
   application of the tool is tile up a large GeoTIFF (or other input) in a tiled format.   
   Note: this approach only works with drivers that support writing (MBTiles, TMS).

   To make a portable MBTiles file::

       osgearth_conv --in driver GDALImage --in url myLargeFile.tif
                     --out driver MBTilesImage --out filename myData.mbtiles
                     --out format jpg

   If you want to serve tiles from a web server, use TMS::

       osgearth_conv --in driver GDALImage --in url myLargeData.tif
                     --out driver TMSImage --out url myLargeData/tms.xml
                     --out format jpg

   That will yield a folder (called "myLargeData" in this case) that you can deploy on the web
   behind any standard web server (e.g. Apache).
   
   **Tip:** The ``jpg`` format does NOT support transparency. If your data was an alpha
   channel, use ``png`` instead.
   
   Just type *osgearth_conv* for a full list of options. The ``--in`` and ``--out`` options
   correspond directly to properties you would normally include in an Earth file.
