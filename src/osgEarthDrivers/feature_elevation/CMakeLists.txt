SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthUtil)

SET(TARGET_SRC
    ReaderWriterFeatureElevation.cpp
)
SET(TARGET_H
    FeatureElevationOptions
)

SETUP_PLUGIN(osgearth_feature_elevation)


# to install public driver includes:
SET(LIB_NAME feature_elevation)
SET(LIB_PUBLIC_HEADERS FeatureElevationOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

