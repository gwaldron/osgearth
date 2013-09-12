TMS (Tile Map Service)
======================
This plugin reads data stored according to the widely-used
OSGeo `Tile Map Service`_ specification.

Example usage::

    <image driver="tms">
        <url>http://readymap.org:8080/readymap/tiles/1.0.0/79/</url>
    </image>
    
Properties:

    :url:      Root URL (or pathname) of the TMS repository
    :tmsType:  Set to ``google`` to invert the Y axis of the tile index
    :format:   Override the format reported by the service (e.g., jpg, png)


.. _Tile Map Service:  http://wiki.osgeo.org/wiki/Tile_Map_Service_Specification
