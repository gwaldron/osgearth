GDAL (Geospatial Data Abstraction Library)
==========================================
The GDAL plugin will read most geospatial file types. This is the most
common driver that you will use to read data on your local filesystem.

The GDAL_ library support a huge `list of formats`_, among the most common
being GeoTIFF, JPEG, and ECW. It can also read from databases and web
services if properly configured.

Example usage::

    <image driver="gdal">
        <url>data/world.tif</url>
    </image>
    
Loading multiple files from a folder (they must all be in the same projection)::

    <image driver="gdal">
        <url>data</url>
        <extensions>tif</extensions>
    </image>
    
Properties:

    :url:               Location of the file to load, or the location of a folder if
                        you intend to load multiple files in the same projection.
    :connection:        If the data source is a database (e.g., PostGIS), the connection
                        string to use to open the database table.
    :extensions:        One or more file extensions, separated by semicolons, to load when
                        ``url`` points to a folder and you are trying to load multiple files.
    :black_extensions:  Set of file extensions to ignore (opposite of ``extensions``)
    :interpolation:     Interpolation method to use when resampling source data; options are
                        ``nearest``, ``average``, and ``bilinear``.  Only effects elevation data
                        unless interp_imagery is also set to true.
    :max_data_level:    Maximum level of detail of available data
    :subdataset:        Some GDAL-supported formats support sub-datasets; use this property
                        to specify such a data source
    :interp_imagery:    Set to true to also sample imagery using the method specified by "interpolation"
                        By default imagery is sampled using nearest sampling.  This takes advantage of
                        any built in overviews or wavelet compression in the source file but can 
                        cause artifacts on neighboring tiles.  Interpolating the imagery can look nicer
                        but will be much slower.
    :warp_profile:      The "warp profile" is a way to tell the GDAL driver to keep the original SRS and geotransform of the source data
                        but use a Warped VRT to make the data appear to conform to the given profile.  This is useful for merging multiple 
                        files that may be in different projections using the composite driver.
    
Also see:

    ``gdal_tiff.earth`` sample in the repo ``tests`` folder.



.. _GDAL:               http://www.gdal.org
.. _list of formats:    http://www.gdal.org/formats_list.html