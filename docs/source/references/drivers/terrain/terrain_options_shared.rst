Common Properties:

    :min_tile_range_factor:     The "maximum visible distance" ratio for all tiles. The 
                                maximum visible distance is computed as tile radius * 
                                this value. (default = 6.0)
    :cluster_culling:           Cluster culling discards back-facing tiles by default. You
                                can disable it be setting this to ``false``, for example if
                                you want to go underground and look up at the surface.
