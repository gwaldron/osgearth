SET(TARGET_SRC
    FeatureSourceOGR.cpp
    FeatureCursorOGR.cpp
)

SET(TARGET_H
    FeatureCursorOGR
    GeometryUtils
    OGRFeatureOptions
)

INCLUDE_DIRECTORIES( ${GDAL_INCLUDE_DIR} )
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures2 osgEarthSymbology)
SET(TARGET_LIBRARIES_VARS GDAL_LIBRARY)
SETUP_PLUGIN(osgearth_feature_ogr2)


# to install public driver includes:
SET(LIB_NAME feature_ogr2)
SET(LIB_PUBLIC_HEADERS OGRFeatureOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

