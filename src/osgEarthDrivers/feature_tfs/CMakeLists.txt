SET(TARGET_SRC
    FeatureSourceTFS.cpp    
)

SET(TARGET_H   	
    TFSFeatureOptions
)

INCLUDE_DIRECTORIES( ${GDAL_INCLUDE_DIR} )
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology osgEarthUtil)
SET(TARGET_LIBRARIES_VARS GDAL_LIBRARY)
SETUP_PLUGIN(osgearth_feature_tfs)


# to install public driver includes:
SET(LIB_NAME feature_tfs)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
