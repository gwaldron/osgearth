TFS (Tiled Feature Service)
===========================
This plugin reads vector data from a *Tiled Feature Service* repository.
TFS is a tiled layout similar to :doc:`/references/drivers/tile/tms` but 
for cropped feature data.

Example usage::

    <model driver="feature_geom">
        <features driver="tfs">
            <url>http://readymap.org/features/1/tfs/</url>
            <format>json</format>
        </features>
        ...
    
Properties:

    :url:      Location from which to load feature data
    :format:   Format of the TFS data; options are ``json`` (default) or ``gml``.
