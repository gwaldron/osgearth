SET(TARGET_SRC
    JoinFeatureFilter.cpp    
)

SET(TARGET_H   	
    JoinFeatureFilterOptions
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)
SETUP_PLUGIN(osgearth_featurefilter_join)

# to install public driver includes:
SET(LIB_NAME featurefilter_join)
SET(LIB_PUBLIC_HEADERS ${TARGET_H})
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
