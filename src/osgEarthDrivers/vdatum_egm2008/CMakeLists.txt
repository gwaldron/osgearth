SET(TARGET_SRC EGM2008.cpp)
SET(TARGET_H EGM2008Grid.h)
SETUP_PLUGIN(osgearth_vdatum_egm2008)


# to install public driver includes:
SET(LIB_NAME vdatum_egm2008)
#SET(LIB_PUBLIC_HEADERS EGM2008Grid.h)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
