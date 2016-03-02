IF(TRITON_FOUND)

SET(TARGET_SRC TritonDriver.cpp
)
               
SET(TARGET_H 
)

INCLUDE_DIRECTORIES( 
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
    osgEarthUtil
    osgEarthTriton
)

SETUP_PLUGIN(osgearth_ocean_triton)

# to install public driver includes:
SET(LIB_NAME ocean_triton)

INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

ENDIF(TRITON_FOUND)
