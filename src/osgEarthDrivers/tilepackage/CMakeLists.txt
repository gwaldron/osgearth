
SET(TARGET_SRC
    BundleReader.cpp
    ReaderWriterTilePackage.cpp
)

# headers to show in IDE
SET(TARGET_H    
    BundleReader
    TilePackageOptions
)

SETUP_PLUGIN(osgearth_tilepackage)

# to install public driver includes:
SET(LIB_NAME tilepackage)
SET(LIB_PUBLIC_HEADERS TilePackageOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
