SET(TARGET_SRC EGM96.cpp)
SET(TARGET_H EGM96Grid.h)
SETUP_PLUGIN(osgearth_vdatum_egm96)


# to install public driver includes:
SET(LIB_NAME vdatum_egm96)
#SET(LIB_PUBLIC_HEADERS EGM96Grid.h)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
