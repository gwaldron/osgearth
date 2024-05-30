

#optionals:
if(OSGEARTH_BUILD_PROCEDURAL_NODEKIT)
    add_subdirectory(osgEarthProcedural)
endif()

if(OSGEARTH_BUILD_CESIUM_NODEKIT)
    add_subdirectory(osgEarthCesium)
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

# plugins:
add_subdirectory(osgEarthDrivers)

# Do this LAST since we are generating the BuildConfig header
# based on everything that's happened so far
add_subdirectory(osgEarth)
set_property(TARGET osgEarth PROPERTY FOLDER "Core")
