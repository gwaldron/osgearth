function(osgearth_install_package_config_files TARGET TARGET_VERSION INCLUDE_INSTALL_DIR LIBRARY_INSTALL_DIR)

    include(CMakePackageConfigHelpers)

    set(PACKAGE_INSTALL_DIR share/osgEarth)
    #set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
    #set(LIBRARY_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})

    configure_package_config_file(
        "${TARGET}-config.cmake.in"
        "${TARGET}-config.cmake"
        INSTALL_DESTINATION ${PACKAGE_INSTALL_DIR}
        PATH_VARS INCLUDE_INSTALL_DIR LIBRARY_INSTALL_DIR) 

    write_basic_package_version_file(
        "${TARGET}-configversion.cmake"
        VERSION ${TARGET_VERSION}
        COMPATIBILITY AnyNewerVersion)

    install(
        EXPORT ${TARGET}Targets
        FILE ${TARGET}-targets.cmake
        NAMESPACE ${TARGET}::
        DESTINATION ${PACKAGE_INSTALL_DIR} )
        
    install(
        FILES
            "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}-config.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}-configVersion.cmake"
        DESTINATION
             ${PACKAGE_INSTALL_DIR} )
    
endfunction()
