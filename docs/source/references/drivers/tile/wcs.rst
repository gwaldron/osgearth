WCS (OGC Web Coverage Service)
==============================
This plugin reads raster coverage data in a limited fashion based on the
OGC `Web Coverage Service`_ specification. In osgEarth it is only really useful
for fetching elevation grid data tiles. We support a subset of WCS 1.1.

Example usage::

    <elevation driver="wcs">
        <url>http://server</url>
        <identifier>elevation</identifier>
        <format>image/GeoTIFF<format>
    </elevation>
    
Properties:

    :url:            Location of the WCS resource
    :identifier:     WCS identifier (i.e., layer to read)
    :format:         Format of the data to return (usually ``tif``)
    :elevation_unit: Unit to use when interpreting elevation grid height values (defaults to ``m``)
    :range_subset:   WCS range subset string (see the WCS docs)


.. _Web Coverage Service:  http://en.wikipedia.org/wiki/Web_Coverage_Service
