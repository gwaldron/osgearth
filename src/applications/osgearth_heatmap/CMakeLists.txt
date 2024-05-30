INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

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

add_osgearth_app(
    TARGET osgearth_heatmap
    SOURCES ${TARGET_SRC}
    HEADERS heatmap.h
    FOLDER Examples)
