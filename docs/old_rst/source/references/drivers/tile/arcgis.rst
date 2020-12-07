ArcGIS Server
=============
This plugin reads image tiles form an ESRI ArcGIS server REST API.

Example usage::

    <image driver="arcgis">
        <url>http://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer</url>
    </image>
    
Properties:

    :url:   URL or the ArcGIS Server REST API entry point for the map service
    :token: ArcGIS Server security token (optional)

Also see:

    ``arcgisonline.earth`` in the ``tests`` folder.


*ArcGIS is a registered copyright of ESRI.*
