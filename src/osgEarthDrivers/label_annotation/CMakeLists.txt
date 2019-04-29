SET(TARGET_SRC AnnotationLabelSource.cpp)

SETUP_PLUGIN(osgearth_label_annotation)

# to install public driver includes:
SET(LIB_NAME label_annotation)
#SET(LIB_PUBLIC_HEADERS AnnotationLabelOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
