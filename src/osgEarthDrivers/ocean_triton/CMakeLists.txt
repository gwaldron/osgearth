
SET(TARGET_SRC TritonDriver.cpp
               TritonNode.cpp
               TritonContext.cpp
               TritonDrawable.cpp
)
               
SET(TARGET_H   TritonNode
               TritonOptions
               TritonContext
               TritonDrawable
)

INCLUDE_DIRECTORIES( 
    ${TRITON_INCLUDE_DIR}
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES}
	${TRITON_LIBRARY}
    osgEarthUtil
)

SETUP_PLUGIN(osgearth_ocean_triton)

# to install public driver includes:
SET(LIB_NAME ocean_triton)

SET(LIB_PUBLIC_HEADERS TritonOptions)

INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
