WorldWind TileService
=====================
This plugin reads tiles stored in the NASA WorldWind TileService_ layout.

Example usage::

    <image driver="tileservice">
        <url>http://server/tileservice/tiles</url>
        <dataset>weather</dataset>
        <format>png</format>
    </image>
    
Properties:

    :url:      Root URL (or pathname) of the TileService repository
    :dataset:  Which WW dataset (layer) to access
    :format:   Format of the individual tiles (e.g., jpg, png)


.. _TileService:  http://www.worldwindcentral.com/wiki/TileService
