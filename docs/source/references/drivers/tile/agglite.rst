AGGLite Rasterizer
==================
This plugin uses the *agglite* library to rasterize feature data to image
tiles. It is a simple yet powerful way to render vector graphics on to
the map.

Example usage::

    <image driver="agglite">
        <features driver="ogr">
            <url>world.shp</url>
        </features>
        <styles>
            <style type="text/css">
                default {
                    stroke:       #ffff00;
                    stroke-width: 500m;
                }
            </style>
        </styles>
    </image>
    
Properties:

    :optimize_line_sampling: Downsample the line data so that it is no higher
                             resolution than to image to which we intend to rasterize
                             it. If you don't do this, you run the risk of the buffer 
                             operation taking forever on very high-resolution input data.
                             (optional)

Also see:

    ``feature_rasterize.earth`` sample in the repo

