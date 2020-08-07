INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIRS} )
# For colorscheme includes
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OSGUTIL_LIBRARY OSGVIEWER_LIBRARY OPENTHREADS_LIBRARY)

SET(COLOR_SCHEMES
        colorschemes/Blues.c
        colorschemes/BrBG.c
        colorschemes/BuGn.c
        colorschemes/BuPu.c
        colorschemes/GnBu.c
        colorschemes/gray.c
        colorschemes/Greens.c
        colorschemes/Greys.c
        colorschemes/Oranges.c
        colorschemes/OrRd.c
        colorschemes/PiYG.c
        colorschemes/PRGn.c
        colorschemes/PuBu.c
        colorschemes/PuBuGn.c
        colorschemes/PuOr.c
        colorschemes/PuRd.c
        colorschemes/Purples.c
        colorschemes/RdBu.c
        colorschemes/RdGy.c
        colorschemes/RdPu.c
        colorschemes/RdYlBu.c
        colorschemes/RdYlGn.c
        colorschemes/Reds.c
        colorschemes/Spectral.c
        colorschemes/YlGn.c
        colorschemes/YlGnBu.c
        colorschemes/YlOrBr.c
        colorschemes/YlOrRd.c
    )
SET(TARGET_SRC osgearth_heatmap.cpp
               heatmap.c
               ${COLOR_SCHEMES}
               )

#### end var setup  ###
SETUP_APPLICATION(osgearth_heatmap)