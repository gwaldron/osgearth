Mapnik Vector Tiles
===========================
This plugin reads vector data from an `MBTiles`_ file which contains `vector tiles<https://github.com/mapbox/vector-tile-spec>`_.

Note:  This driver does not currently support multi-level mbtiles files.  It will only load the maximum level in the database.  This will change in the future when
osgEarth has better support for non-additive feature datasources.

This driver requires that you build osgEarth with SQLite3 support and Protobuf support.

Example usage::

    <model driver="feature_geom">
        <features name="osm" driver="mapnikvectortiles">
            <url>../data/osm.mbtiles</url>
        </features>
        ...

Properties:

    :url:      Location of the mbtiles file.

.. _MBTiles:  https://www.mapbox.com/developers/mbtiles/
