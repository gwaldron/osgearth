add_subdirectory(osgEarth)
set_property(TARGET osgEarth PROPERTY FOLDER "Core")

FOREACH( lib
         osgEarthSplat
         osgEarthSilverLining
         osgEarthTriton
         osgEarthDrivers )
    add_subdirectory( ${lib} )
ENDFOREACH( lib )

IF(BUILD_APPLICATIONS AND NOT ANDROID)
    ADD_SUBDIRECTORY( applications )
ENDIF()

IF(BUILD_TESTS AND NOT OSGEARTH_BUILD_PLATFORM_IPHONE)
    ADD_SUBDIRECTORY( tests )
ENDIF()
