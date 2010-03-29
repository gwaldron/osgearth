#the old construct SUBDIRS( was substituded by ADD_SUBDIRECTORY that is to be preferred according on CMake docs.
FOREACH( myfolder
        osgEarth
        osgEarthFeatures
        osgEarthUtil
        osgEarthDrivers
        osgEarthSymbology
        applications
    )

    ADD_SUBDIRECTORY(${myfolder})

ENDFOREACH( myfolder)


IF(MSVC80)
  OPTION(OSGEARTH_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS "Generate or not manifests files under VS8 for dynamically loaded dlls" ON)
ENDIF(MSVC80)


