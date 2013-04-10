Common Properties:

    :sample_ratio:              Ratio at which to resample the number of height values
                                in each elevation tile. (The preferred approach is to use
                                MapOptions.elevation_tile_size instead.)
    :min_tile_range_factor:     The "maximum visible distance" ratio for all tiles. The 
                                maximum visible distance is computed as tile radius * 
                                this value. (default = 6.0)
    :cluster_culling:           Cluster culling discards back-facing tiles by default. You
                                can disable it be setting this to ``false``, for example if
                                you want to go underground and look up at the surface.
