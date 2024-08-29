# osgearth_package_install_config_file
#
# Creates and installs the top-level LIBNAME-config.cmake file for a library
#
function(osgearth_package_install_config_files INCLUDE_INSTALL_DIR LIBRARY_INSTALL_DIR)

    set(PACKAGE_INSTALL_DIR share/osgEarth)

    include(CMakePackageConfigHelpers)

    configure_package_config_file(
        "${CMAKE_SOURCE_DIR}/cmake/osgEarth-config.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/osgEarth-config.cmake"
        INSTALL_DESTINATION ${PACKAGE_INSTALL_DIR}
        PATH_VARS INCLUDE_INSTALL_DIR LIBRARY_INSTALL_DIR) 

    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/osgEarth-configVersion.cmake"
        VERSION ${OSGEARTH_VERSION}
        COMPATIBILITY AnyNewerVersion)
        
    install(
        FILES
            "${CMAKE_CURRENT_BINARY_DIR}/osgEarth-config.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/osgEarth-configVersion.cmake"
        DESTINATION
             ${PACKAGE_INSTALL_DIR} )
    
endfunction()


# osgearth_package_install_library_target
#
# Installs the -targets.cmake file for the library "MY_TARGET".
# Each -targets.cmaks file corresond to one "component" or "nodekit" library,
# and is included from the top-level osgEarth-config.cmake file.
#
function(osgearth_package_install_library_target MY_TARGET)

    set(PACKAGE_INSTALL_DIR share/osgEarth)

    install(
        EXPORT ${MY_TARGET}Targets
        FILE ${MY_TARGET}-targets.cmake
        NAMESPACE osgEarth::
        DESTINATION ${PACKAGE_INSTALL_DIR} )
    
endfunction()
