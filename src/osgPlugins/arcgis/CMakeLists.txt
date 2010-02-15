
SET(TARGET_SRC
    ReaderWriterArcGIS.cpp
    MapService.cpp
)

# headers to show in IDE
SET(TARGET_H
    Extent.h
    MapService.h
    ArcGISMapLayerFactory
)

SETUP_PLUGIN(osgearth_arcgis)


# to install public driver includes:
# SET(LIB_NAME osgEarthDrivers)
# SET(LIB_PUBLIC_HEADERS ArcGISMapLayerFactory)
# INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
