
SET(TARGET_SRC
    ReaderWriterArcGIS.cpp
    MapService.cpp
)

# headers to show in IDE
SET(TARGET_H
    Extent.h
    MapService.h
    ArcGISOptions
)

SETUP_PLUGIN(osgearth_arcgis)


# to install public driver includes:
SET(LIB_NAME arcgis)
SET(LIB_PUBLIC_HEADERS ArcGISOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
