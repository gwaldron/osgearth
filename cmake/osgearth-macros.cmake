#
#  Detect osg version and setup variables accordingly
#
macro(detect_osg_version)

    # Fall back to OSG_DIR if OSG_INCLUDE_DIR is not defined
    if(OSG_DIR AND NOT OSG_INCLUDE_DIR AND EXISTS "${OSG_DIR}/include/osg/Version")
        set(OSG_INCLUDE_DIR "${OSG_DIR}/include")
    endif()

    OPTION(APPEND_OPENSCENEGRAPH_VERSION "Append the OSG version number to the osgPlugins directory" ON)

    # Try to ascertain the version...
    # (Taken from CMake's FindOpenSceneGraph.cmake)
    if(OSG_INCLUDE_DIR)
        if(OpenSceneGraph_DEBUG)
            message(STATUS "[ FindOpenSceneGraph.cmake:${CMAKE_CURRENT_LIST_LINE} ] "
                "Detected OSG_INCLUDE_DIR = ${OSG_INCLUDE_DIR}")
        endif()

        set(_osg_Version_file "${OSG_INCLUDE_DIR}/osg/Version")
        if("${OSG_INCLUDE_DIR}" MATCHES "\\.framework$" AND NOT EXISTS "${_osg_Version_file}")
            set(_osg_Version_file "${OSG_INCLUDE_DIR}/Headers/Version")
        endif()

        if(EXISTS "${_osg_Version_file}")
          file(STRINGS "${_osg_Version_file}" _osg_Version_contents
               REGEX "#define (OSG_VERSION_[A-Z]+|OPENSCENEGRAPH_[A-Z]+_VERSION)[ \t]+[0-9]+")
        else()
          set(_osg_Version_contents "unknown")
        endif()

        string(REGEX MATCH ".*#define OSG_VERSION_MAJOR[ \t]+[0-9]+.*"
            _osg_old_defines "${_osg_Version_contents}")
        string(REGEX MATCH ".*#define OPENSCENEGRAPH_MAJOR_VERSION[ \t]+[0-9]+.*"
            _osg_new_defines "${_osg_Version_contents}")
        if(_osg_old_defines)
            string(REGEX REPLACE ".*#define OSG_VERSION_MAJOR[ \t]+([0-9]+).*"
                "\\1" _osg_VERSION_MAJOR ${_osg_Version_contents})
            string(REGEX REPLACE ".*#define OSG_VERSION_MINOR[ \t]+([0-9]+).*"
                "\\1" _osg_VERSION_MINOR ${_osg_Version_contents})
            string(REGEX REPLACE ".*#define OSG_VERSION_PATCH[ \t]+([0-9]+).*"
                "\\1" _osg_VERSION_PATCH ${_osg_Version_contents})
        elseif(_osg_new_defines)
            string(REGEX REPLACE ".*#define OPENSCENEGRAPH_MAJOR_VERSION[ \t]+([0-9]+).*"
                "\\1" _osg_VERSION_MAJOR ${_osg_Version_contents})
            string(REGEX REPLACE ".*#define OPENSCENEGRAPH_MINOR_VERSION[ \t]+([0-9]+).*"
                "\\1" _osg_VERSION_MINOR ${_osg_Version_contents})
            string(REGEX REPLACE ".*#define OPENSCENEGRAPH_PATCH_VERSION[ \t]+([0-9]+).*"
                "\\1" _osg_VERSION_PATCH ${_osg_Version_contents})
        else()
            message(WARNING "[ FindOpenSceneGraph.cmake:${CMAKE_CURRENT_LIST_LINE} ] "
                "Failed to parse version number, please report this as a bug")
        endif()
        unset(_osg_Version_contents)

        set(OPENSCENEGRAPH_VERSION "${_osg_VERSION_MAJOR}.${_osg_VERSION_MINOR}.${_osg_VERSION_PATCH}"
                                    CACHE INTERNAL "The version of OSG which was detected")
        if(OpenSceneGraph_DEBUG)
            message(STATUS "[ FindOpenSceneGraph.cmake:${CMAKE_CURRENT_LIST_LINE} ] "
                "Detected version ${OPENSCENEGRAPH_VERSION}")
        endif()
    endif()

	MARK_AS_ADVANCED(OPENSCENEGRAPH_VERSION)


    IF (APPEND_OPENSCENEGRAPH_VERSION AND OPENSCENEGRAPH_VERSION)
        SET(OSG_PLUGINS "osgPlugins-${OPENSCENEGRAPH_VERSION}"  CACHE STRING "" FORCE)
        #MESSAGE(STATUS "Plugins will be installed under osgPlugins-${OPENSCENEGRAPH_VERSION} directory.")
	else (APPEND_OPENSCENEGRAPH_VERSION AND OPENSCENEGRAPH_VERSION)
		SET(OSG_PLUGINS  CACHE STRING "" FORCE)
    ENDIF(APPEND_OPENSCENEGRAPH_VERSION AND OPENSCENEGRAPH_VERSION)

	MARK_AS_ADVANCED(OSG_PLUGINS)

endmacro(detect_osg_version)





# ..........................................................
# Create a library for an osgdb plugin.
#
# example usage:
#
#   add_osgearth_library_as_plugin(
#       TARGET [name of plugin with osgdb_ or osgdb_osgearth_ prefix]
#       HEADERS [list of header files to include in the project]
#       PUBLIC_HEADERS [list of header files to install]
#       SOURCES [list of source files]
#       TEMPLATES [list of cmake .in files]
#       SHADERS [list of GLSL shader files]
#       LIBRARIES [list of additional libraries to link with]
#       INCLUDE_DIRECTORIES [list of additional include folders to use]
#
# Note: osgEarth and OpenScenegraph libraries will automatically link to all plugins.
#
macro(add_osgearth_library_as_plugin)

    set(options "")
    set(oneValueArgs TARGET)
    set(multiValueArgs SOURCES HEADERS PUBLIC_HEADERS SHADERS TEMPLATES LIBRARIES INCLUDE_DIRECTORIES)
    cmake_parse_arguments(MY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    set(ALL_HEADERS ${MY_HEADERS} ${MY_PUBLIC_HEADERS})
    
    include_directories(${MY_INCLUDE_DIRECTORIES})
    
    if(OSGEARTH_BUILD_SHARED_LIBS)
        add_library(${MY_TARGET} MODULE ${MY_SOURCES} ${ALL_HEADERS} ${MY_SHADERS} ${MY_TEMPLATES})
    else()
        add_library(${MY_TARGET} STATIC ${MY_SOURCES} ${ALL_HEADERS} ${MY_SHADERS} ${MY_TEMPLATES})
    endif()

    if (${MY_TARGET} MATCHES "^osgdb_osgearth_")
        string(SUBSTRING ${MY_TARGET} 15 -1 MY_TARGET_SUFFIX)
        set_target_properties(${MY_TARGET} PROPERTIES PROJECT_LABEL "osgEarth ${MY_TARGET_SUFFIX}")
    elseif(${MY_TARGET} MATCHES "^osgdb_")
        string(SUBSTRING ${MY_TARGET} 6 -1 MY_TARGET_SUFFIX)
        set_target_properties(${MY_TARGET} PROPERTIES PROJECT_LABEL "osg ${MY_TARGET_SUFFIX}")
    else()
        message(FATAL_ERROR "add_library_as_plugin: name '${MY_TARGET}' must begin with 'osgdb_' or 'osgdb_osgearth_'")
    endif()
   
    # install the shader source files, if requested:
    if(OSGEARTH_INSTALL_SHADERS)
        install(FILES ${MY_SHADERS} DESTINATION resources/shaders)
    endif()
    
    # install the dynamic libraries.
    install(TARGETS ${MY_TARGET}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib/${OSG_PLUGINS}
        LIBRARY DESTINATION bin/${OSG_PLUGINS})
    
    # organize the files in the IDE
    source_group( "Header Files"   FILES ${ALL_HEADERS} )
    source_group( "Shader Files"   FILES ${MY_SHADERS} )
    source_group( "Template Files" FILES ${MY_TEMPLATES} )
    set_property(TARGET ${MY_TARGET} PROPERTY FOLDER "Plugins")
    
    # We always need these so just link them here.
    target_link_libraries(${MY_TARGET} PRIVATE osgEarth ${OPENSCENEGRAPH_LIBRARIES} ${MY_LIBRARIES})
    
    # Install any public headers
    if (MY_PUBLIC_HEADERS)
        install(FILES ${MY_PUBLIC_HEADERS} DESTINATION include/osgEarthDrivers/${MY_TARGET_SUFFIX})
    endif()

endmacro()





# ..........................................................
# Create a library for an osgEarth application.
#
# example usage:
#
#   add_osgearth_app(
#       TARGET [name of the application]
#       HEADERS [list of header files to include in the project]
#       SOURCES [list of source files]
#       TEMPLATES [list of cmake .in files]
#       SHADERS [list of GLSL shader files]
#       LIBRARIES [list of additional libraries to link with]
#       INCLUDE_DIRECTORIES [list of additional include folders to use]
#
# Note: osgEarth and OpenScenegraph libraries will automatically link to all apps.
#
macro(add_osgearth_app)
    set(options "")
    set(oneValueArgs TARGET FOLDER)
    set(multiValueArgs SOURCES HEADERS SHADERS TEMPLATES LIBRARIES INCLUDE_DIRECTORIES)
    cmake_parse_arguments(MY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    set(ALL_HEADERS ${MY_HEADERS})
    
    include_directories(${MY_INCLUDE_DIRECTORIES})

    add_executable(${MY_TARGET} ${MY_SOURCES} ${ALL_HEADERS} ${MY_SHADERS} ${MY_TEMPLATES})
        
    # We always need these so just link them here.
    target_link_libraries(${MY_TARGET} PRIVATE osgEarth ${OPENSCENEGRAPH_LIBRARIES} ${MY_LIBRARIES})
    
    set_target_properties(${MY_TARGET} PROPERTIES PROJECT_LABEL "${MY_TARGET}")

    install(TARGETS ${MY_TARGET} RUNTIME DESTINATION bin)

	if(NOT MY_FOLDER)
	    set(MY_FOLDER "Ungrouped")
	endif()

	SET_PROPERTY(TARGET ${MY_TARGET} PROPERTY FOLDER "${MY_FOLDER}")
endmacro(add_osgearth_app)




# ..........................................................
# Create a library for an osgEarth ImGUi application.
#
# example usage:
#
#   add_osgearth_app(
#       TARGET [name of the application]
#       HEADERS [list of header files to include in the project]
#       SOURCES [list of source files]
#       TEMPLATES [list of cmake .in files]
#       SHADERS [list of GLSL shader files]
#       LIBRARIES [list of additional libraries to link with]
#       INCLUDE_DIRECTORIES [list of additional include folders to use]
#
# Note: osgEarth and OpenScenegraph libraries will automatically link to all apps.
#
macro(add_osgearth_imgui_app)
    set(options "")
    set(oneValueArgs TARGET FOLDER)
    set(multiValueArgs SOURCES HEADERS SHADERS TEMPLATES LIBRARIES INCLUDE_DIRECTORIES)
    cmake_parse_arguments(MY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    if(OSGEARTH_HAVE_IMGUI)
        include_directories(
            ${GLEW_INCLUDE_DIR}
            ${OSGEARTH_EMBEDDED_THIRD_PARTY_DIR}/imgui
            ${OSGEARTH_EMBEDDED_THIRD_PARTY_DIR}/imgui/examples
        )

        set(MY_SOURCES ${MY_SOURCES}
            ${OSGEARTH_EMBEDDED_THIRD_PARTY_DIR}/imgui/imgui.cpp
            ${OSGEARTH_EMBEDDED_THIRD_PARTY_DIR}/imgui/imgui_demo.cpp
            ${OSGEARTH_EMBEDDED_THIRD_PARTY_DIR}/imgui/imgui_draw.cpp
            ${OSGEARTH_EMBEDDED_THIRD_PARTY_DIR}/imgui/imgui_widgets.cpp
            ${OSGEARTH_EMBEDDED_THIRD_PARTY_DIR}/imgui/imgui_tables.cpp
            ${OSGEARTH_EMBEDDED_THIRD_PARTY_DIR}/imgui/backends/imgui_impl_opengl3.cpp
            ${OSGEARTH_SOURCE_DIR}/src/osgEarth/ImGui/ImGui.cpp
        )

        set(MY_LIBRARIES ${MY_LIBRARIES} ${GLEW_LIBRARIES} ${CMAKE_DL_LIBS} OpenGL::GL)

        # define for conditional compilation (esp for imgui headers)
        if(OSGEARTHPROCEDURAL_LIBRARY)
            add_definitions(-DHAVE_OSGEARTHPROCEDURAL)
            list(APPEND MY_LIBRARIES osgEarthProcedural)
        endif()

        if(OSGEARTHCESIUM_LIBRARY)
            add_definitions(-DHAVE_OSGEARTHCESIUM)
            list(APPEND MY_LIBRARIES osgEarthCesium)
        endif()
        
        add_osgearth_app(
            TARGET ${MY_TARGET}
            SOURCES ${MY_SOURCES}
            HEADERS ${MY_HEADERS}
            INCLUDE_DIRECTORIES ${MY_INCLUDE_DIRECTORIES}
            LIBRARIES ${MY_LIBRARIES}
            FOLDER ${MY_FOLDER} )
            
    else()
    
        message(STATUS "ImGui application '${MY_TARGET}' skipped")
        if (NOT GLEW_FOUND)
            message(STATUS "   ...because GLEW not found")
        elseif (NOT EXISTS "${OSGEARTH_EMBEDDED_THIRD_PARTY_DIR}/imgui/imgui.cpp")
            message(STATUS "   ...because ImGui git submodule not found")
        endif()
        
    endif()
    
endmacro(add_osgearth_imgui_app)



# ...........................................................
# Adds the targets for an osgEarth library.
#
# example usage:
#
#   add_osgearth_library(
#       TARGET [name of the application]
#       HEADERS [list of header files to include in the project]
#       IMGUI_HEADERS [list of header files with imgui code]
#       PUBLIC_HEADERS [list of headers to be installed]
#       SOURCES [list of source files]
#       TEMPLATES [list of cmake .in files]
#       SHADERS [list of GLSL shader files]
#       LIBRARIES [list of additional libraries to link with]
#       INCLUDE_DIRECTORIES [list of additional include folders to use]
#
# Note: osgEarth and OpenScenegraph libraries will automatically link to all apps.
#
macro(add_osgearth_library)

    set(options "")
    set(oneValueArgs TARGET FOLDER MY_DYNAMIC_OR_STATIC)
    set(multiValueArgs SOURCES HEADERS PUBLIC_HEADERS IMGUI_HEADERS SHADERS TEMPLATES LIBRARIES INCLUDE_DIRECTORIES)
    cmake_parse_arguments(MY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    set(INSTALL_INCDIR include/${MY_TARGET})

    # apply postfixes for non-windows...?
    set(INSTALL_BINDIR bin)
    if(WIN32)
        set(INSTALL_LIBDIR bin)
        set(INSTALL_ARCHIVEDIR lib)
    else()
        set(INSTALL_LIBDIR lib${LIB_POSTFIX})
        set(INSTALL_ARCHIVEDIR lib${LIB_POSTFIX})
    endif()
    
    set(ALL_HEADERS ${MY_HEADERS} ${MY_IMGUI_HEADERS} ${MY_PUBLIC_HEADERS})
    include_directories(${MY_INCLUDE_DIRECTORIES})        

    # IDE setup
    source_group("Sources" FILES ${MY_SOURCES})
    source_group("Headers" FILES ${ALL_HEADERS})
    source_group("Shaders" FILES ${MY_SHADERS} )
    source_group("Templates" FILES ${MY_TEMPLATES} )
    
    # create the library.
    add_library(
        ${MY_TARGET}
        ${OSGEARTH_DYNAMIC_OR_STATIC}
        ${ALL_HEADERS}
        ${MY_SOURCES}
        ${MY_SHADERS}
        ${MY_TEMPLATES} )
        
    include_directories(${MY_INCLUDE_DIRECTORIES})

    # Link:
    if(NOT ${MY_TARGET} MATCHES "osgEarth")
        target_link_libraries(${MY_TARGET} PRIVATE osgEarth ${OPENSCENEGRAPH_LIBRARIES} ${MY_LIBRARIES})
    else()
        target_link_libraries(${MY_TARGET} PRIVATE ${OPENSCENEGRAPH_LIBRARIES} ${MY_LIBRARIES})
    endif()

    # library install and target exports for the cmake config packaging.
    install(
        TARGETS ${MY_TARGET}
        EXPORT ${MY_TARGET}Targets
        RUNTIME DESTINATION ${INSTALL_BINDIR}
        LIBRARY DESTINATION ${INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${INSTALL_ARCHIVEDIR}
    )

    # deploy the shaders for this library, if requested.
    if(OSGEARTH_INSTALL_SHADERS)
        install(
            FILES ${MY_SHADERS}
            DESTINATION resources/shaders )
    endif()


    # Public Header File install:
    IF(NOT OSGEARTH_BUILD_FRAMEWORKS)
        # normal path to install the public header files:
        install(
            FILES ${MY_PUBLIC_HEADERS}
            DESTINATION ${INSTALL_INCDIR})
    else()
        # MAC OSX stuff.
        set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
        set(CMAKE_INSTALL_RPATH "${OSGEARTH_BUILD_FRAMEWORKS_INSTALL_NAME_DIR}")
        
        set_target_properties(${LIB_NAME} PROPERTIES
             FRAMEWORK TRUE
             FRAMEWORK_VERSION ${OSGEARTH_MAJOR_VERSION}
             PUBLIC_HEADER "${MY_PUBLIC_HEADERS}"
             INSTALL_NAME_DIR "${OSGEARTH_BUILD_FRAMEWORKS_INSTALL_NAME_DIR}"
        )
    endif()


    # custom install for ImGui headers.
    if(OSGEARTH_HAVE_IMGUI)
        source_group("Headers\\ImGui" FILES ${MY_IMGUI_HEADERS})        
        install(
            FILES ${MY_IMGUI_HEADERS}
            DESTINATION include/${MY_TARGET}/ImGui)
    endif()
    
endmacro()




# -----------------------------------------------------------------------
# configure_shaders -gw
#
# Bakes GLSL shaders to make into a CPP file at runtime.
# Example:
#
#   configure_shaders( MyTemplate.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/AutoGen.cpp file1.glsl file2.glsl )
#
macro(configure_shaders templateFile autoGenCppFile)

	# set up configure variables:
	set(TEMPLATE_FILE   ${templateFile} )
	set(GLSL_FILES      ${ARGN} )
	set(OUTPUT_CPP_FILE ${autoGenCppFile})

	# generate the build-time script that will create out cpp file with inline shaders:
	configure_file(
		"${CMAKE_SOURCE_DIR}/cmake/ConfigureShaders.cmake.in"
		"${CMAKE_CURRENT_BINARY_DIR}/ConfigureShaders.cmake"
		@ONLY)

	# add the custom build-time command to run the script:
	add_custom_command(
		OUTPUT
			"${autoGenCppFile}"
		COMMAND
			"${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/ConfigureShaders.cmake"
		DEPENDS
			${GLSL_FILES}
			"${TEMPLATE_FILE}"
			"${CMAKE_SOURCE_DIR}/cmake/ConfigureShaders.cmake.in" )

endmacro(configure_shaders)

