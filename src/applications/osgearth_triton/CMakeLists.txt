# Triton example
# Later, if we want to link in the Triton library directly
# to use the Triton API, uncomment the include and lib 
# below.

INCLUDE_DIRECTORIES(
    ${OSG_INCLUDE_DIRS}
)

# 3rd party:
SET(TARGET_LIBRARIES_VARS 
    OSG_LIBRARY 
    OSGDB_LIBRARY 
    OSGGA_LIBRARY
    OSGUTIL_LIBRARY 
    OSGVIEWER_LIBRARY 
    OPENTHREADS_LIBRARY
)

# osgEarth:
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthTriton)

SET(TARGET_SRC osgearth_triton.cpp)

#### end var setup  ###
SETUP_APPLICATION(osgearth_triton)
