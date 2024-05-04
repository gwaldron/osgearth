# Required Vars:
# ${LIB_NAME}
# ${LIB_PUBLIC_HEADERS}

# Optional Vars:
# ${HEADER_INSTALL_DIR}

macro(install_osgearth_library)

    # custom header installation folder...?
    if(HEADER_INSTALL_DIR)
        set(INSTALL_INCDIR include/${HEADER_INSTALL_DIR})
    else()
        set(INSTALL_INCDIR include/${LIB_NAME})
    endif()

    # apply postfixes for non-windows...?
    set(INSTALL_BINDIR bin)
    if(WIN32)
        set(INSTALL_LIBDIR bin)
        set(INSTALL_ARCHIVEDIR lib)
    else()
        set(INSTALL_LIBDIR lib${LIB_POSTFIX})
        set(INSTALL_ARCHIVEDIR lib${LIB_POSTFIX})
    endif()


    # IDE setup
    source_group("Headers" FILES ${LIB_PUBLIC_HEADERS})
    source_group("Shaders" FILES ${TARGET_GLSL} )
    source_group("Templates" FILES ${TARGET_IN} )

    # library install and target exports for the cmake config packaging.
    install(
        TARGETS ${LIB_NAME}
        EXPORT ${LIB_NAME}Targets
        RUNTIME DESTINATION ${INSTALL_BINDIR}
        LIBRARY DESTINATION ${INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${INSTALL_ARCHIVEDIR}
    )

    # deploy the shaders for this library, if requested.
    if(OSGEARTH_INSTALL_SHADERS)
        install(
            FILES ${TARGET_GLSL}
            DESTINATION resources/shaders )
    endif()


    # "Frameworks" is an OSX thing.
    IF(NOT OSGEARTH_BUILD_FRAMEWORKS)

        # normal path to install the public header files:
        install(
            FILES ${LIB_PUBLIC_HEADERS}
            DESTINATION ${INSTALL_INCDIR})
            
    else()
        # MAC OSX stuff.
        set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
        set(CMAKE_INSTALL_RPATH "${OSGEARTH_BUILD_FRAMEWORKS_INSTALL_NAME_DIR}")
        
        set_target_properties(${LIB_NAME} PROPERTIES
             FRAMEWORK TRUE
             FRAMEWORK_VERSION ${OSGEARTH_MAJOR_VERSION}
             PUBLIC_HEADER  "${LIB_PUBLIC_HEADERS}"
             INSTALL_NAME_DIR "${OSGEARTH_BUILD_FRAMEWORKS_INSTALL_NAME_DIR}"
        )
    endif()


    # custom install for ImGui headers.
    # inputs: LIB_PUBLIC_HEADERS_IMGUI = list of ImGui headers (each with ImGui/ prefix)
    #         LIB_NAME = library that includes these headers
    if(OSGEARTH_ENABLE_IMGUI)
        source_group("Headers\\ImGui" FILES ${LIB_PUBLIC_HEADERS_IMGUI})
        
        if(HEADER_INSTALL_DIR)
            set(_IMGUI_INSTALL_INCDIR include/${HEADER_INSTALL_DIR}/ImGui)
        else()
            set(_IMGUI_INSTALL_INCDIR include/${LIB_NAME}/ImGui)
        endif()

        install(
            FILES ${LIB_PUBLIC_HEADERS_IMGUI}
            DESTINATION ${_IMGUI_INSTALL_INCDIR})
    endif()
    
endmacro()
