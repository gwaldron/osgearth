OGR
===
This plugin reads vector data from any of the formats supported by the
`OGR Simple Feature Library`_ (which is quite a lot). Most common among
these includes ESRI Shapefiles, GML, and PostGIS.

Example usage::

    <model driver="feature_geom">
        <features driver="ogr">
            <url>data/world_boundaries.shp</url>
        </features>
        ...
    
Properties:

    :url:                   Location from which to load feature data
    :connection:            If the feature data is in a database, use this to specify the
                            DB connection string instead of using the ``url``.
    :geometry:              Specify *inline* geometry in `OGC WKT format`_ instead of using
                            ``url`` or ``connection``.
    :geometry_url:          Same as ``geometry`` except that the WKT string is in a file.
    :ogr_driver:            ``OGR driver``_ to use. (default = "ESRI Shapefile")
    :build_spatial_index:   Set to ``true`` to build a spatial index for the feature data,
                            which will dramatically speed up access for larger datasets.
    :layer:                 Some datasets require an addition layer identifier for sub-datasets;
                            Set that here (integer).

*Special Note on PostGIS usage:*

PostGIS uses a ``connection`` string instead of a ``url`` to make its database connection.
It is common to include a tables reference such as ``table=something``. In this driver,
however, that can lead to problems; instead specify your table in the ``layer`` property.
For example::

    <features driver="ogr">
        <connection>PG:dbname=mydb host=127.0.0.1 ...</connection>
        <layer>myTableName</layer>
    </features>
   

.. _OGR Simple Feature Library:  http://www.gdal.org/ogr
.. _OGR driver:                  http://www.gdal.org/ogr/ogr_formats.html
