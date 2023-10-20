
SET(TARGET_H
    FileSystemCache
)
SET(TARGET_SRC 
    FileSystemCache.cpp
)

SET(TARGET_LIBRARIES_VARS OSG_LIBRARY OSGDB_LIBRARY OPENTHREADS_LIBRARY) 

SETUP_PLUGIN(osgearth_cache_filesystem)

# to install public driver includes:
SET(LIB_NAME cache_filesystem)
SET(LIB_PUBLIC_HEADERS FileSystemCache)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
