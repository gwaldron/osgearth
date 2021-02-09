Feature Geometry
================
This plugin renders vector feature data into OSG geometry using style sheets.

Example usage::

    <model driver="feature_geom">
        <features driver="ogr">
            <url>world.shp</url>
        </features>
        <styles>
            <style type="text/css">
                default {
                    stroke:       #ffff00;
                    stroke-width: 2;
                }
            </style>
        </styles>
        <fading duration="1.0"/>
    </model>
    
Properties:

    :geo_interpolation:     How to interpolate geographic lines; options are ``great_circle`` or ``rhumb_line``
    :instancing:            For point model substitution, whether to use GL draw-instanced (default is ``false``)

.. include:: feature_model_shared_props.rst

Also see:

    ``feature_rasterize.earth`` sample in the repo


.. include:: feature_model_shared_blocks.rst
