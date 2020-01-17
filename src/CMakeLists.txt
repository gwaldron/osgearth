#the old construct SUBDIRS( was substituded by ADD_SUBDIRECTORY that is to be preferred according on CMake docs.
FOREACH( lib
         osgEarth
         osgEarthAnnotation
         osgEarthFeatures
         osgEarthSymbology
         osgEarthUtil )

    ADD_SUBDIRECTORY(${lib})

    SET_PROPERTY(TARGET ${lib} PROPERTY FOLDER "Core")

ENDFOREACH( lib )


FOREACH( lib
         osgEarthSplat
         osgEarthSilverLining
         osgEarthTriton )
    add_subdirectory( ${lib} )
ENDFOREACH( lib )

ADD_SUBDIRECTORY( osgEarthDrivers )

IF(BUILD_APPLICATIONS AND NOT ANDROID)
    ADD_SUBDIRECTORY( applications )
ENDIF()

IF(BUILD_TESTS AND NOT OSGEARTH_BUILD_PLATFORM_IPHONE)
    ADD_SUBDIRECTORY( tests )
ENDIF()


IF(MSVC80)
  OPTION(OSGEARTH_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS "Generate or not manifests files under VS8 for dynamically loaded dlls" ON)
ENDIF(MSVC80)


