SET(TARGET_SRC OverlayLabelSource.cpp)
#SET(TARGET_H OverlayLabelOptions)
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology osgEarthUtil)
SETUP_PLUGIN(osgearth_label_overlay)

# to install public driver includes:
SET(LIB_NAME label_overlay)
#SET(LIB_PUBLIC_HEADERS OverlayLabelOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
