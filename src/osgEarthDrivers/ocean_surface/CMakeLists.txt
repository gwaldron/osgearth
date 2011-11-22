
SET(TARGET_SRC ElevationProxyImageLayer.cpp
               OceanCompositor.cpp
               OceanSurfaceContainer.cpp
               ReaderWriterOceanSurface.cpp
)
               
SET(TARGET_H   ElevationProxyImageLayer
               OceanCompositor
               OceanSurface
               OceanSurfaceContainer
               OceanShaders
)

SET(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} osgEarthSymbology)

SETUP_PLUGIN(osgearth_ocean_surface)

# to install public driver includes:
SET(LIB_NAME ocean_surface)
SET(LIB_PUBLIC_HEADERS OceanSurface)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
