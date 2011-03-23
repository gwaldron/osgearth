SET(TARGET_SRC FeatureMaskSource.cpp)
SET(TARGET_H FeatureMaskOptions)
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology)
SETUP_PLUGIN(osgearth_mask_feature)


# to install public driver includes:
SET(LIB_NAME mask_feature)
SET(LIB_PUBLIC_HEADERS FeatureMaskOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

