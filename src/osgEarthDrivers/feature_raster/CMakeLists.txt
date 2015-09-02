SET(TARGET_SRC
    FeatureSourceRaster.cpp    
)

SET(TARGET_H   	
    RasterFeatureOptions
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology osgEarthUtil)
SETUP_PLUGIN(osgearth_feature_raster)

# to install public driver includes:
SET(LIB_NAME feature_raster)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
