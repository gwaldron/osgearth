#the old construct SUBDIRS( was substituded by ADD_SUBDIRECTORY that is to be preferred according on CMake docs.
FOREACH( lib
         osgEarth
         osgEarthFeatures
         osgEarthUtil
         osgEarthSymbology )

    ADD_SUBDIRECTORY(${lib})
    
    SET_PROPERTY(TARGET ${lib} PROPERTY FOLDER "Libs")

ENDFOREACH( lib )

ADD_SUBDIRECTORY( osgEarthDrivers )
ADD_SUBDIRECTORY( applications )

IF(MSVC80)
  OPTION(OSGEARTH_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS "Generate or not manifests files under VS8 for dynamically loaded dlls" ON)
ENDIF(MSVC80)


