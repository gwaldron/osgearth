Working with Data
=================

Where to Find Data
------------------

Help us add useful sources of Free data to this list.

**Raster data**

    * `ReadyMap.org`_ - Free 15m imagery, elevation, and street tiles for osgEarth developers
    
    * MapQuest_ - MapQuest open aerial imagery and rasterized OpenStreetMap layers
    
    * `Bing Maps`_ - Microsoft's worldwide imagery and map data ($)
    
    * `USGS National Map`_ - Elevation, orthoimagery, hydrography, geographic names, boundaries,
      transportation, structures, and land cover products for the US.
    
    * `NASA EOSDIS`_ - NASA's Global Imagery Browse Services (GIBS) replaces the agency's old
      `JPL OnEarth`_ site for global imagery products like MODIS.
       
    * `NASA BlueMarble`_ - NASA's whole-earth imagery (including topography and bathymetry maps)
    
    * `NRL GIDB`_ - US Naval Research Lab's GIDB OpenGIS Web Services
    
    * `Natural Earth`_ - Free vector and raster map data at various scales
    
    * `Virtual Terrain Project`_ - Various sources for whole-earth imagery
        
        
**Elevation data**

    * `CGIAR`_ - World 90m elevation data derived from SRTM and ETOPO (`CGIAR European mirror`_)
    
    * `SRTM30+`_ - Worldwide elevation coverage (including batymetry)
    
    * `GLCF`_ - UMD's Global Land Cover Facility (they also have mosaiced LANDSAT data)
    
    * `GEBCO`_ - Genearl Batymetry Chart of the Oceans

**Feature data**

    * `OpenStreetMap`_ - Worldwide, community-sources street and land use data (vectors and rasterized tiles)
    
    * `DIVA-GIS`_ - Free low-resolution vector data for any country
    
    * `Natural Earth`_ - Free vector and raster map data at various scales
    

.. _CGIAR:                      http://srtm.csi.cgiar.org/
.. _CGIAR Europoean mirror:     ftp://xftp.jrc.it/pub/srtmV4/
.. _DIVA-GIS:                   http://www.diva-gis.org/gData
.. _GEBCO:                      http://www.gebco.net/
.. _GLCF:                       http://glcf.umiacs.umd.edu/data/srtm/
.. _OpenStreetMap:              http://openstreetmap.org
.. _MapQuest:                   http://developer.mapquest.com/web/products/open/map
.. _NASA EOSDIS:                http://earthdata.nasa.gov/about-eosdis/system-description/global-imagery-browse-services-gibs
.. _NASA BlueMarble:            http://visibleearth.nasa.gov/view_cat.php?categoryID=1484
.. _Natural Earth:              http://www.naturalearthdata.com/
.. _NRL GIDB:                   http://columbo.nrlssc.navy.mil/ogcwms/servlet/WMSServlet
.. _+SRTM30+:                   ftp://topex.ucsd.edu/pub/srtm30_plus/
.. _USGS National Map:          http://nationalmap.gov/viewer.html
.. _Virtual Terrain Project:    http://vterrain.org/Imagery/WholeEarth/
.. _Bing Maps:                  http://www.microsoft.com/maps/choose-your-bing-maps-API.aspx
.. _ReadyMap.org:               http://readymap.org/index_orig.html

----

Tips for Preparing your own Data
--------------------------------

**Processing Local Source Data**

    If you have geospatial data that you would like to view in osgEarth, you can usually use the GDAL driver.
    If you plan on doing this, try loading it as-is first.
    If you find that it's too slow, here are some tips for optimizing your data for tiled access.
    
    **Reproject your data**

    osgEarth will reproject your data on your fly if it does not have the necessary
    coordinate system.  For instance, if you are trying to view a UTM image on a
    geodetic globe (epsg:4326).  However, osgEarth will run much faster if your data
    is already in the correct coordinate system.  You can use any tool you want to 
    reproject your data such as GDAL, Global Mapper or ArcGIS.
    
    For example, to reproject a UTM image to geodetic using gdal_warp::

        gdalwarp -t_srs epsg:4326 my_utm_image.tif my_gd_image.tif

    **Build internal tiles**
    
    Typically formats such as GeoTiff store their pixel data in scanlines.
    This generally works well, but because of the tiled approach that osgEarth
    uses to access the data, you may find that using a tiled dataset will be more
    efficient as osgEarth doens't need to read nearly as much data from disk to
    extract a tile.
    
    To create a tiled GeoTiff using gdal_translate, issue the following command::
    
        gdal_translate -of GTiff -co "TILED=YES" myfile.tif myfile_tiled.tif

    **Build overviews**
    
    Adding overviews (also called ''pyramids'' or ''rsets'') can sometimes increase
    the performance of a datasource in osgEarth.  You can use the
    `gdaladdo <http://gdal.org/gdaladdo.html>`_ utility to add overviews to a dataset.
    
    For example::

        gdaladdo -r average myimage.tif 2 4 8 16

        
**Building tile sets**

    Another way to speed up imagery and elevation loading in osgEarth is to build **tile sets**.
    In fact, if you want to serve your data over the network, this is the only way!
    
    This process takes the source data and chops it up into a quad-tree hierarchy of discrete
    *tiles* that osgEarth can load very quickly. Normally, if you load a GeoTIFF (for example),
    osgEarth has to create the tiles at runtime in order to build the globe; Doing this beforehand
    means less work for osgEarth when you run your application.

    **osgearth_package**

    *osgearth_package* is a utility that prepares source data for use in osgEarth. 
    It is **optional** - you can run osgEarth against your raw source data 
    and it will work fine - but you can use *osgearth_package* to build optimized 
    tile sets that will maximize performance in most cases. Usage::
    
        osgearth_package file.earth --tms --out output_folder

    This will load each of the data sources in the the earth file 
    (``file.earth`` in this case) and generate a TMS repository for each under the
    folder ``output_folder``. You can also specify options:
    
        --out path                          Root output folder of the TMS repo
        --ext extension                     Output file extension
        --max-level level                   Maximum level of detail
        --bounds xmin ymin xmax ymax        Bounds to package (in map coordinates; default=entire map)
        --out-earth                         Generate an output earth file referencing the new repo
        --overwrite                         Force overwriting of existing files
        --keep-empties                      Writes fully transparent image tiles (normally discarded)
        --db-options                        An optional OSG options string
        --quiet                             Suppress progress reporting
