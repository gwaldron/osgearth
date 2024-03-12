PROJECT(OSGEARTH_APPLICATIONS)

SET(OSGCORE_BUNDLED TRUE)

IF(NOT OSGCORE_BUNDLED)
	FIND_PACKAGE(OSGCORE)
	IF(OSGCORE_FOUND)
		INCLUDE(${OSGCORE_USE_FILE})
	ELSE(OSGCORE_FOUND)
		MESSAGE(ERROR "OSGCORE needed but NOT FOUND")
	ENDIF(OSGCORE_FOUND)
	SET(CMAKE_MODULE_PATH  ${PROJECT_SOURCE_DIR}/../../Macros)
ENDIF(NOT OSGCORE_BUNDLED)

SET(OPENSCENEGRAPH_APPLICATION_DIR ${PROJECT_SOURCE_DIR})


#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
SET(TARGET_COMMON_LIBRARIES
    osgEarth
)

SET(TARGET_DEFAULT_PREFIX "application_")

IF(NOT OSGEARTH_BUILD_PLATFORM_IPHONE)

    # Tools:
    IF(OSGEARTH_BUILD_TOOLS)
        set(TARGET_DEFAULT_LABEL_PREFIX "Tool")
        set(TARGET_DEFAULT_APPLICATION_FOLDER "Tools")
        add_subdirectory(osgearth_viewer)
        add_subdirectory(osgearth_imgui)
        add_subdirectory(osgearth_tfs)
        add_subdirectory(osgearth_boundarygen)
        add_subdirectory(osgearth_version)
        add_subdirectory(osgearth_atlas)
        add_subdirectory(osgearth_bakefeaturetiles)
        add_subdirectory(osgearth_conv)
        add_subdirectory(osgearth_3pv)
        add_subdirectory(osgearth_clamp)
        
        if(OSGEARTH_BUILD_PROCEDURAL_NODEKIT)
            add_subdirectory(osgearth_exportvegetation)
            add_subdirectory(osgearth_biome)
            add_subdirectory(osgearth_imposterbaker)
        endif()

        IF (Protobuf_FOUND AND SQLITE3_FOUND)
            add_subdirectory(osgearth_mvtindex)
        ENDIF()   
        
        if(OSGEARTH_BUILD_LEGACY_CONTROLS_API)
            add_subdirectory(osgearth_createtile)
        endif()
        
        if(OSGEARTH_BUILD_LEGACY_SPLAT_NODEKIT)
            add_subdirectory(osgearth_exportgroundcover_splat)
        endif()     
    ENDIF()

    # Examples:
    IF(OSGEARTH_BUILD_EXAMPLES)
        set(TARGET_DEFAULT_LABEL_PREFIX "Example")
        set(TARGET_DEFAULT_APPLICATION_FOLDER "Examples")
        add_subdirectory(osgearth_simple)
        add_subdirectory(osgearth_features)
        add_subdirectory(osgearth_featurefilter)
        add_subdirectory(osgearth_los)
        add_subdirectory(osgearth_terrainprofile)
        add_subdirectory(osgearth_map)
        add_subdirectory(osgearth_annotation)
        add_subdirectory(osgearth_occlusionculling)
        add_subdirectory(osgearth_minimap)
        add_subdirectory(osgearth_mrt)
        add_subdirectory(osgearth_pick)
        add_subdirectory(osgearth_skyview)
        add_subdirectory(osgearth_lights)
        add_subdirectory(osgearth_infinitescroll)
        add_subdirectory(osgearth_video)
        add_subdirectory(osgearth_heatmap)
        add_subdirectory(osgearth_collecttriangles)
        add_subdirectory(osgearth_roads)
        
        if(OSGEARTH_BUILD_LEGACY_CONTROLS_API)
            add_subdirectory(osgearth_cluster)
            add_subdirectory(osgearth_eci)
            add_subdirectory(osgearth_ephemeris)
            add_subdirectory(osgearth_graticule)
            add_subdirectory(osgearth_magnify)
            add_subdirectory(osgearth_manip)
            add_subdirectory(osgearth_tracks)
            add_subdirectory(osgearth_transform)
        endif()

    ENDIF(OSGEARTH_BUILD_EXAMPLES)
    
    if(OSGEARTH_BUILD_TESTS)
        set(TARGET_DEFAULT_LABEL_PREFIX "Test")
        set(TARGET_DEFAULT_APPLICATION_FOLDER "Tests")
        add_subdirectory(osgearth_bindless)
        add_subdirectory(osgearth_drawables)
        add_subdirectory(osgearth_horizon)
        add_subdirectory(osgearth_overlayviewer)
        add_subdirectory(osgearth_windows)
        
        if(OSGEARTH_BUILD_LEGACY_CONTROLS_API)
            add_subdirectory(osgearth_shadercomp)
            
            if(SILVERLINING_FOUND)
                add_subdirectory(osgearth_silverlining)
            endif(SILVERLINING_FOUND)

            if(TRITON_FOUND)
                add_subdirectory(osgearth_triton)
            endif(TRITON_FOUND)
        endif()
    endif(OSGEARTH_BUILD_TESTS)

ELSE()

    # ios
    IF(OSGEARTH_BUILD_EXAMPLES)
        SET(TARGET_DEFAULT_LABEL_PREFIX "Example")
        SET(TARGET_DEFAULT_APPLICATION_FOLDER "Examples")
        add_subdirectory(osgearth_viewerIOS)
    ENDIF()

ENDIF()
