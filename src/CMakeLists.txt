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
         osgEarthQt
         osgEarthSplat
         osgEarthSilverLining
         osgEarthTriton )
    add_subdirectory( ${lib} )    
ENDFOREACH( lib )

ADD_SUBDIRECTORY( osgEarthDrivers )

IF(NOT OSG_BUILD_PLATFORM_IPHONE AND NOT OSG_BUILD_PLATFORM_IPHONE_SIMULATOR AND NOT ANDROID)
    ADD_SUBDIRECTORY( applications )
ENDIF()

#IF (Qt5Widgets_FOUND OR QT4_FOUND AND NOT ANDROID AND OSGEARTH_USE_QT)
#    ADD_SUBDIRECTORY(osgEarthQt)
#    SET_PROPERTY(TARGET osgEarthQt PROPERTY FOLDER "Libs")
#ENDIF()

#IF (SILVERLINING_FOUND)
#    ADD_SUBDIRECTORY(osgEarthSilverLining)
    #SET_PROPERTY(TARGET osgEarthSilverLining PROPERTY FOLDER "Extensions")
#ENDIF()

#IF (TRITON_FOUND)
    #ADD_SUBDIRECTORY(osgEarthTriton)
    #SET_PROPERTY(TARGET osgEarthTriton PROPERTY FOLDER "Extensions")
#ENDIF()

IF(MSVC80)
  OPTION(OSGEARTH_MSVC_GENERATE_PLUGINS_AND_WRAPPERS_MANIFESTS "Generate or not manifests files under VS8 for dynamically loaded dlls" ON)
ENDIF(MSVC80)


