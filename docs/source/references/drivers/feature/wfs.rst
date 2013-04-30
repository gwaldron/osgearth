WFS (OGC Web Feature Service)
===========================
This plugin reads vector data from an OGC `Web Feature Service`_ resource.

Example usage::

    <model driver="feature_geom">
        <features name="states" driver="wfs">
            <url> http://demo.opengeo.org/geoserver/wfs</url>
            <typename>states</typename>
            <outputformat>json</outputformat>
        </features>
        ...
    
Properties:

    :url:             Location from which to load feature data
    :typename:        WFS type name to access (i.e., the layer)
    :outputformat:    Format to return from the service; ``json`` or ``gml``
    :maxfeatures:     Maximum number of features to return for a query


.. _Web Feature Service:    http://en.wikipedia.org/wiki/Web_Feature_Service
