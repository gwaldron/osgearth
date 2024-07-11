add_osgearth_app(
    TARGET osgearth_pick
    SOURCES osgearth_pick.cpp
    LIBRARIES osgEarthImGui
    FOLDER Examples )
    
if(OSGEARTH_BUILD_PROCEDURAL_NODEKIT)
    target_link_libraries(osgearth_pick PRIVATE osgEarthProcedural)
endif()

if(OSGEARTH_BUILD_CESIUM_NODEKIT)
    target_link_libraries(osgearth_pick PRIVATE osgEarthCesium)
endif()

