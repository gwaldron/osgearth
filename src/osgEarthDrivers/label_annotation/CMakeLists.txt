SET(TARGET_SRC AnnotationLabelSource.cpp)
#SET(TARGET_H AnnotationLabelOptions)
SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthFeatures osgEarthSymbology osgEarthAnnotation)
SETUP_PLUGIN(osgearth_label_annotation)

# to install public driver includes:
SET(LIB_NAME label_annotation)
#SET(LIB_PUBLIC_HEADERS AnnotationLabelOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
