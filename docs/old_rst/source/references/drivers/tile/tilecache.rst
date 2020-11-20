TileCache
=========
TileCache_ (MetaCarta Labs) is a web map tile caching system with its own
layout for encoding tile hierarchies. This plugin will read tiles from that
file layout.

Example usage::

    <image driver="tilecache">
        <url>http://server/tiles/root</url>
        <layer>landuse</layer>
        <format>jpg</format>
    </image>
    
Properties:

    :url:      Root URL (or pathname) of the tilecache repository
    :layer:    Which TileCache layer to access
    :format:   Format of the individual tiles (e.g., jpg, png)


.. _TileCache:      http://tilecache.org/
