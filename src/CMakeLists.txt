# required:
add_subdirectory(osgEarth)
add_subdirectory(osgEarthDrivers)
set_property(TARGET osgEarth PROPERTY FOLDER "Core")

#optionals:
if(OSGEARTH_BUILD_PROCEDURAL_NODEKIT)
    add_definitions(-DHAVE_OSGEARTHPROCEDURAL)
    set(OSGEARTHPROCEDURAL_LIBRARY osgEarthProcedural)
    add_subdirectory(osgEarthProcedural)
endif()

if(OSGEARTH_BUILD_TRITON_NODEKIT)
    add_subdirectory(osgEarthTriton)
endif()

if(OSGEARTH_BUILD_SILVERLINING_NODEKIT)
    add_subdirectory(osgEarthSilverLining)
endif()

if(OSGEARTH_BUILD_TOOLS OR OSGEARTH_BUILD_EXAMPLES)
    add_subdirectory( applications )
endif()

if(OSGEARTH_BUILD_TESTS AND NOT OSGEARTH_BUILD_PLATFORM_IPHONE)
    add_subdirectory( tests )
endif()

if(OSGEARTH_BUILD_LEGACY_SPLAT_NODEKIT)
    add_subdirectory(osgEarthSplat)
endif()
