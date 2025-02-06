# osgearth_package_install_config_file
#
# Creates and installs the top-level LIBNAME-config.cmake file for a library
#
function(osgearth_package_install_config_files)
    include(CMakePackageConfigHelpers)

    # main target include dir
    set(INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_INCLUDEDIR}")

    configure_package_config_file(
        "${PROJECT_SOURCE_DIR}/cmake/osgearth-config.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/osgearth-config.cmake"
        INSTALL_DESTINATION ${OSGEARTH_INSTALL_CMAKEDIR}
        PATH_VARS INCLUDE_INSTALL_DIR OSGEARTH_INSTALL_DATADIR) 

    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/osgearth-config-version.cmake"
        VERSION ${OSGEARTH_VERSION}
        COMPATIBILITY AnyNewerVersion)
        
    install(
        FILES
            "${CMAKE_CURRENT_BINARY_DIR}/osgearth-config.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/osgearth-config-version.cmake"
        DESTINATION
             ${OSGEARTH_INSTALL_CMAKEDIR} )
    
endfunction()


# osgearth_package_install_library_target
#
# Installs the -targets.cmake file for the library "MY_TARGET".
# Each -targets.cmaks file corresond to one "component" or "nodekit" library,
# and is included from the top-level osgEarth-config.cmake file.
#
function(osgearth_package_install_library_target MY_TARGET)
    install(
        EXPORT ${MY_TARGET}Targets
        FILE ${MY_TARGET}-targets.cmake
        NAMESPACE osgEarth::
        DESTINATION ${OSGEARTH_INSTALL_CMAKEDIR} )
    
endfunction()
