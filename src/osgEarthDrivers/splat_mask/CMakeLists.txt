INCLUDE_DIRECTORIES( ${LIBNOISE_INCLUDE_DIR} )

SET(TARGET_SRC SplatMaskDriver.cpp)
SET(TARGET_H SplatMaskOptions)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} ${LIBNOISE_LIBRARY})

SETUP_PLUGIN(osgearth_splat_mask)

# to install public driver includes:
SET(LIB_NAME splat_mask)
SET(LIB_PUBLIC_HEADERS SplatMaskOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

