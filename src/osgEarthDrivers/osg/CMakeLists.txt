SET(TARGET_H
    OSGOptions
)
SET(TARGET_SRC 
    OSGTileSource.cpp
)
SETUP_PLUGIN(osgearth_osg)


# to install public driver includes:
SET(LIB_NAME osg)
SET(LIB_PUBLIC_HEADERS OSGOptions)
INCLUDE_DIRECTORIES(${OSG_INCLUDE_DIR} )
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)

