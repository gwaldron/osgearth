
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures)

SET(TARGET_SRC
	AGGLiteRasterizerTileSource.cpp
	agg.cpp)
	
SET(TARGET_H
	agg.h
	AGGLiteOptions
)

SETUP_PLUGIN(osgearth_agglite)

# to install public driver includes:
SET(LIB_NAME agglite)
SET(LIB_PUBLIC_HEADERS AGGLiteOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
