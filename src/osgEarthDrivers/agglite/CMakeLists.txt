
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures)

SET(TARGET_SRC
	AGGLiteRasterizerTileSource.cpp
	agg.cpp)
	
SET(TARGET_H
	agg.h)

SETUP_PLUGIN(osgearth_agglite)
