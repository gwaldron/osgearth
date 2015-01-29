MP
==
The default terrain engine for osgEarth renders an unlimited number of
image layers using a tile-level multipass blending technique.

Example usage::

    <map>
        <options>
            <terrain driver                   = "mp"
                     skirt_ratio              = "0.05"
                     color                    = "#ffffffff"
                     normalize_edges          = "false"
                     incremental_update       = "false"
                     quick_release_gl_objects = "true"
                     min_tile_range_factor    = "6.0"
                     cluster_culling          = "true" />

Properties:

    :skirt_ratio:               The "skirt" is a piece of vertical geometry that hides
                                gaps between adjacent tiles with different levels of
                                detail. This property sets the ratio of skirt height to
                                the width of the tile.
    :color:                     Color of the underlying terrain (without imagery) in
                                HTML format. Default = "#ffffffff" (opaque white). You
                                can adjust the alpha to get transparency.
    :normalize_edges:           Post-process the normal vectors on tile boundaries to 
                                smooth them across tiles, making the tile boundaries
                                less visible when not using imagery.
    :incremental_update:        When enabled, only visible tiles update when the map
                                model changes (i.e., when layers are added or removed).
                                Non-visible terrain tiles (like those at lower LODs)
                                don't update until they come into view.
    :quick_release_gl_objects:  When true, installs a module that releases GL resources
                                immediately when a tile pages out. This can prevent
                                memory run-up when traversing a paged terrain at high
                                speed. Disabling quick-release may help achieve a more
                                consistent frame rate.
    
.. include:: terrain_options_shared.rst
