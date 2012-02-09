SET(TARGET_SRC EGM84.cpp)
SET(TARGET_H EGM84Grid.h)
SETUP_PLUGIN(osgearth_vdatum_egm84)


# to install public driver includes:
SET(LIB_NAME vdatum_egm84)
#SET(LIB_PUBLIC_HEADERS EGM84Grid.h)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
