Feature Stencil
===============
This plugin "drapes" vector feature data over the terrain using a
stencil buffering technique.

Example usage::

    <model driver="feature_stencil">
        <features name="world" driver="ogr">
            <url>../data/world.shp</url>
        </features>
             
        <styles>
            <style type="text/css">
                world {
                   stroke:         #ffff007f;
                   stroke-width:   0.1;
                }            
            </style>
        </styles>
    </model>
    
Properties:

    :extrusion_distance:  How far to extrude stencil volumes (meters)
    :inverted:            Whether to stencil the inversion of the feature data (true/false)
    :mask:                Whether to use the stenciled region as a terrain mask (true/false)
    :show_volumes:        For debugging; draws the actual stencil volume geometry

.. include:: feature_model_shared_props.rst

Also see:

    ``feature_stencil_line_draping.earth`` sample in the repo
    
Notes:

* This plugin does NOT support paging (display layouts).


.. include:: feature_model_shared_blocks.rst
