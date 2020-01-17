#######################################################################################################
#  macro to detect osg version and setup variables accordingly
#######################################################################################################
MACRO(DETECT_OSG_VERSION)

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
        MESSAGE(STATUS "Plugins will be installed under osgPlugins-${OPENSCENEGRAPH_VERSION} directory.")
	else (APPEND_OPENSCENEGRAPH_VERSION AND OPENSCENEGRAPH_VERSION)
		SET(OSG_PLUGINS  CACHE STRING "" FORCE)
    ENDIF(APPEND_OPENSCENEGRAPH_VERSION AND OPENSCENEGRAPH_VERSION)
	
	MARK_AS_ADVANCED(OSG_PLUGINS)
	
	#MESSAGE("OSG_PLUGINS=${OSG_PLUGINS}")

ENDMACRO(DETECT_OSG_VERSION)



#######################################################################################################
#  macro for linking libraries that come from Findxxxx commands, so there is a variable that contains the
#  full path of the library name. in order to differentiate release and debug, this macro get the
#  NAME of the variables, so the macro gets as arguments the target name and the following list of parameters
#  is intended as a list of variable names each one containing  the path of the libraries to link to
#  The existence of a variable name with _DEBUG appended is tested and, in case it's value is used
#  for linking to when in debug mode
#  the content of this library for linking when in debugging
#######################################################################################################

MACRO(LINK_WITH_VARIABLES TRGTNAME)
    FOREACH(varname ${ARGN})
        IF(${varname}_DEBUG)
            IF(${varname}_RELEASE)
                TARGET_LINK_LIBRARIES(${TRGTNAME} optimized "${${varname}_RELEASE}" debug "${${varname}_DEBUG}")
            ELSE(${varname}_RELEASE)
                TARGET_LINK_LIBRARIES(${TRGTNAME} optimized "${${varname}}" debug "${${varname}_DEBUG}")
            ENDIF(${varname}_RELEASE)
        ELSE(${varname}_DEBUG)
            TARGET_LINK_LIBRARIES(${TRGTNAME} ${${varname}} )
        ENDIF(${varname}_DEBUG)
    ENDFOREACH(varname)
ENDMACRO(LINK_WITH_VARIABLES TRGTNAME)

MACRO(LINK_INTERNAL TRGTNAME)
    TARGET_LINK_LIBRARIES(${TRGTNAME} ${ARGN})
ENDMACRO(LINK_INTERNAL TRGTNAME)

MACRO(LINK_EXTERNAL TRGTNAME)
    FOREACH(LINKLIB ${ARGN})
        TARGET_LINK_LIBRARIES(${TRGTNAME} "${LINKLIB}" )
    ENDFOREACH(LINKLIB)
ENDMACRO(LINK_EXTERNAL TRGTNAME)


#######################################################################################################
#  macro for common setup of core libraries: it links OPENGL_LIBRARIES in undifferentiated mode
#######################################################################################################

MACRO(LINK_CORELIB_DEFAULT CORELIB_NAME)
    LINK_EXTERNAL(${CORELIB_NAME} ${OPENGL_LIBRARIES})
    LINK_WITH_VARIABLES(${CORELIB_NAME} OPENTHREADS_LIBRARY)
    IF(OSGEARTH_SONAMES)
      SET_TARGET_PROPERTIES(${CORELIB_NAME} PROPERTIES VERSION ${OSGEARTH_VERSION} SOVERSION ${OSGEARTH_SOVERSION})
    ENDIF(OSGEARTH_SONAMES)
ENDMACRO(LINK_CORELIB_DEFAULT CORELIB_NAME)


#######################################################################################################
#  macro for common setup of plugins, examples and applications it expect some variables to be set:
#  either within the local CMakeLists or higher in hierarchy
#  TARGET_NAME is the name of the folder and of the actually .exe or .so or .dll
#  TARGET_TARGETNAME  is the name of the target , this get buit out of a prefix, if present and TARGET_TARGETNAME
#  TARGET_SRC  are the sources of the target
#  TARGET_H are the eventual headers of the target
#  TARGET_LIBRARIES are the libraries to link to that are internal to the project and have d suffix for debug
#  TARGET_EXTERNAL_LIBRARIES are external libraries and are not differentiated with d suffix
#  TARGET_LABEL is the label IDE should show up for targets
##########################################################################################################

MACRO(SETUP_LINK_LIBRARIES)
    ######################################################################
    #
    # This set up the libraries to link to, it assumes there are two variable: one common for a group of examples or plagins
    # kept in the variable TARGET_COMMON_LIBRARIES and an example or plugin specific kept in TARGET_ADDED_LIBRARIES
    # they are combined in a single list checked for unicity
    # the suffix ${CMAKE_DEBUG_POSTFIX} is used for differentiating optimized and debug
    #
    # a second variable TARGET_EXTERNAL_LIBRARIES hold the list of  libraries not differentiated between debug and optimized
    ##################################################################################
    SET(TARGET_LIBRARIES ${TARGET_COMMON_LIBRARIES})

    FOREACH(LINKLIB ${TARGET_ADDED_LIBRARIES})
      SET(TO_INSERT TRUE)
      FOREACH (value ${TARGET_COMMON_LIBRARIES})
            IF (${value} STREQUAL ${LINKLIB})
                  SET(TO_INSERT FALSE)
            ENDIF (${value} STREQUAL ${LINKLIB})
        ENDFOREACH (value ${TARGET_COMMON_LIBRARIES})
      IF(TO_INSERT)
          LIST(APPEND TARGET_LIBRARIES ${LINKLIB})
      ENDIF(TO_INSERT)
    ENDFOREACH(LINKLIB)

#    FOREACH(LINKLIB ${TARGET_LIBRARIES})
#            TARGET_LINK_LIBRARIES(${TARGET_TARGETNAME} optimized ${LINKLIB} debug "${LINKLIB}${CMAKE_DEBUG_POSTFIX}")
#    ENDFOREACH(LINKLIB)
    LINK_INTERNAL(${TARGET_TARGETNAME} ${TARGET_LIBRARIES})

    IF(TARGET_LIBRARIES_VARS)
            LINK_WITH_VARIABLES(${TARGET_TARGETNAME} ${TARGET_LIBRARIES_VARS})
    ENDIF(TARGET_LIBRARIES_VARS)

    FOREACH(LINKLIB ${TARGET_EXTERNAL_LIBRARIES})
            TARGET_LINK_LIBRARIES(${TARGET_TARGETNAME} ${LINKLIB})
    ENDFOREACH(LINKLIB)
ENDMACRO(SETUP_LINK_LIBRARIES)

############################################################################################
# this is the common set of command for all the plugins


MACRO(SETUP_PLUGIN PLUGIN_NAME)

    SET(TARGET_NAME ${PLUGIN_NAME} )

    #MESSAGE("in -->SETUP_PLUGIN<-- ${TARGET_NAME}-->${TARGET_SRC} <--> ${TARGET_H}<--")
    
    SOURCE_GROUP( "Header Files"   FILES ${TARGET_H} )
    SOURCE_GROUP( "Shader Files"   FILES ${TARGET_GLSL} )
    SOURCe_GROUP( "Template Files" FILES ${TARGET_IN} )

    ## we have set up the target label and targetname by taking into account global prfix (osgdb_)

    IF(NOT TARGET_TARGETNAME)
            SET(TARGET_TARGETNAME "${TARGET_DEFAULT_PREFIX}${TARGET_NAME}")
    ENDIF(NOT TARGET_TARGETNAME)
    IF(NOT TARGET_LABEL)
            SET(TARGET_LABEL "${TARGET_DEFAULT_LABEL_PREFIX} ${TARGET_NAME}")
    ENDIF(NOT TARGET_LABEL)

# here we use the command to generate the library

    IF   (DYNAMIC_OSGEARTH)
        ADD_LIBRARY(${TARGET_TARGETNAME} MODULE ${TARGET_SRC} ${TARGET_H} ${TARGET_GLSL} ${TARGET_IN})
    ELSE (DYNAMIC_OSGEARTH)
        ADD_LIBRARY(${TARGET_TARGETNAME} STATIC ${TARGET_SRC} ${TARGET_H} ${TARGET_GLSL} ${TARGET_IN})
    ENDIF(DYNAMIC_OSGEARTH)

    SET_TARGET_PROPERTIES(${TARGET_TARGETNAME} PROPERTIES PROJECT_LABEL "${TARGET_LABEL}")

    SETUP_LINK_LIBRARIES()

#the installation path are differentiated for win32 that install in bib versus other architecture that install in lib${LIB_POSTFIX}/${VPB_PLUGINS}
    IF(WIN32)
        INSTALL(TARGETS ${TARGET_TARGETNAME} RUNTIME DESTINATION bin ARCHIVE DESTINATION lib/${OSG_PLUGINS} LIBRARY DESTINATION bin/${OSG_PLUGINS} )
	    
		#Install to the OSG_DIR as well
		IF(OSGEARTH_INSTALL_TO_OSG_DIR AND OSG_DIR)
		  INSTALL(TARGETS ${TARGET_TARGETNAME} RUNTIME DESTINATION ${OSG_DIR}/bin/${OSG_PLUGINS} LIBRARY DESTINATION ${OSG_DIR}/bin/${OSG_PLUGINS} )
		ENDIF(OSGEARTH_INSTALL_TO_OSG_DIR AND OSG_DIR)
		
    ELSE(WIN32)
        INSTALL(TARGETS ${TARGET_TARGETNAME} RUNTIME DESTINATION bin ARCHIVE DESTINATION lib${LIB_POSTFIX}/${OSG_PLUGINS} LIBRARY DESTINATION lib${LIB_POSTFIX}/${OSG_PLUGINS} )

		#Install to the OSG_DIR as well
		IF(OSGEARTH_INSTALL_TO_OSG_DIR AND OSG_DIR)
		  INSTALL(TARGETS ${TARGET_TARGETNAME} RUNTIME DESTINATION ${OSG_DIR}/bin LIBRARY DESTINATION lib${LIB_POSTFIX}/bin)
		ENDIF(OSGEARTH_INSTALL_TO_OSG_DIR AND OSG_DIR)
		
    ENDIF(WIN32)

    IF(OSG_BUILD_PLATFORM_IPHONE)
        SET_TARGET_PROPERTIES(${TARGET_TARGETNAME} PROPERTIES XCODE_ATTRIBUTE_ENABLE_BITCODE ${IPHONE_ENABLE_BITCODE})
    ENDIF()

    # install the shader source files
    if(OSGEARTH_INSTALL_SHADERS)
        INSTALL(
            FILES ${TARGET_GLSL} 
            DESTINATION resources/shaders )
    endif(OSGEARTH_INSTALL_SHADERS)
    
#finally, set up the solution folder -gw
    SET_PROPERTY(TARGET ${TARGET_TARGETNAME} PROPERTY FOLDER "Plugins")    
    
ENDMACRO(SETUP_PLUGIN)





MACRO(SETUP_EXTENSION PLUGIN_NAME)

    SET(TARGET_NAME ${PLUGIN_NAME} )

    #MESSAGE("in -->SETUP_EXTENSION<-- ${TARGET_NAME}-->${TARGET_SRC} <--> ${TARGET_H}<--")
    
    SOURCE_GROUP( "Header Files"   FILES ${TARGET_H} )
    SOURCE_GROUP( "Shader Files"   FILES ${TARGET_GLSL} )
    SOURCe_GROUP( "Template Files" FILES ${TARGET_IN} )

    ## we have set up the target label and targetname by taking into account global prefix (osgdb_)

    IF(NOT TARGET_TARGETNAME)
            SET(TARGET_TARGETNAME "${TARGET_DEFAULT_PREFIX}${TARGET_NAME}")
    ENDIF(NOT TARGET_TARGETNAME)
    IF(NOT TARGET_LABEL)
            SET(TARGET_LABEL "${TARGET_DEFAULT_LABEL_PREFIX} ${TARGET_NAME}")
    ENDIF(NOT TARGET_LABEL)

# here we use the command to generate the library

    IF   (DYNAMIC_OSGEARTH)
        ADD_LIBRARY(${TARGET_TARGETNAME} MODULE ${TARGET_SRC} ${TARGET_H} ${TARGET_GLSL} ${TARGET_IN})
    ELSE (DYNAMIC_OSGEARTH)
        ADD_LIBRARY(${TARGET_TARGETNAME} STATIC ${TARGET_SRC} ${TARGET_H} ${TARGET_GLSL} ${TARGET_IN})
    ENDIF(DYNAMIC_OSGEARTH)

    SET_TARGET_PROPERTIES(${TARGET_TARGETNAME} PROPERTIES PROJECT_LABEL "${TARGET_LABEL}")

    SETUP_LINK_LIBRARIES()

#the installation path are differentiated for win32 that install in bib versus other architecture that install in lib${LIB_POSTFIX}/${VPB_PLUGINS}
    IF(WIN32)
        INSTALL(
            TARGETS ${TARGET_TARGETNAME}
            RUNTIME DESTINATION bin
            ARCHIVE DESTINATION lib/${OSG_PLUGINS}
            LIBRARY DESTINATION bin/${OSG_PLUGINS} )
	    
		#Install to the OSG_DIR as well
		IF(OSGEARTH_INSTALL_TO_OSG_DIR AND OSG_DIR)
		    INSTALL(
                TARGETS ${TARGET_TARGETNAME} 
                RUNTIME DESTINATION ${OSG_DIR}/bin/${OSG_PLUGINS}
                LIBRARY DESTINATION ${OSG_DIR}/bin/${OSG_PLUGINS} )
		ENDIF(OSGEARTH_INSTALL_TO_OSG_DIR AND OSG_DIR)
		
    ELSE(WIN32)
        INSTALL(
            TARGETS ${TARGET_TARGETNAME} 
            RUNTIME DESTINATION bin 
            ARCHIVE DESTINATION lib${LIB_POSTFIX}/${OSG_PLUGINS} 
            LIBRARY DESTINATION lib${LIB_POSTFIX}/${OSG_PLUGINS} )

		#Install to the OSG_DIR as well
		IF(OSGEARTH_INSTALL_TO_OSG_DIR AND OSG_DIR)
		    INSTALL(
                TARGETS ${TARGET_TARGETNAME}
                RUNTIME DESTINATION ${OSG_DIR}/bin
                LIBRARY DESTINATION lib${LIB_POSTFIX}/bin )
		ENDIF(OSGEARTH_INSTALL_TO_OSG_DIR AND OSG_DIR)
		
    ENDIF(WIN32)
    
    # install the shader source files
    if(OSGEARTH_INSTALL_SHADERS)
        INSTALL(
            FILES ${TARGET_GLSL} 
            DESTINATION resources/shaders )
    endif(OSGEARTH_INSTALL_SHADERS)

    IF(OSG_BUILD_PLATFORM_IPHONE)
        SET_TARGET_PROPERTIES(${TARGET_TARGETNAME} PROPERTIES XCODE_ATTRIBUTE_ENABLE_BITCODE ${IPHONE_ENABLE_BITCODE})
    ENDIF()
    
#finally, set up the solution folder -gw
    SET_PROPERTY(TARGET ${TARGET_TARGETNAME} PROPERTY FOLDER "Extensions")    
    
ENDMACRO(SETUP_EXTENSION)





#################################################################################################################
# this is the macro for example and application setup
###########################################################

MACRO(SETUP_EXE IS_COMMANDLINE_APP)
    #MESSAGE("in -->SETUP_EXE<-- ${TARGET_NAME}-->${TARGET_SRC} <--> ${TARGET_H}<--")
    IF(NOT TARGET_TARGETNAME)
            SET(TARGET_TARGETNAME "${TARGET_DEFAULT_PREFIX}${TARGET_NAME}")
    ENDIF(NOT TARGET_TARGETNAME)
    IF(NOT TARGET_LABEL)
            SET(TARGET_LABEL "${TARGET_DEFAULT_LABEL_PREFIX} ${TARGET_NAME}")
    ENDIF(NOT TARGET_LABEL)

    IF(${IS_COMMANDLINE_APP})

        ADD_EXECUTABLE(${TARGET_TARGETNAME} ${TARGET_SRC} ${TARGET_H})

    ELSE(${IS_COMMANDLINE_APP})

        IF(APPLE)
            # SET(MACOSX_BUNDLE_LONG_VERSION_STRING "${VIRTUALPLANETBUILDER_MAJOR_VERSION}.${VIRTUALPLANETBUILDER_MINOR_VERSION}.${VIRTUALPLANETBUILDER_PATCH_VERSION}")
            # Short Version is the "marketing version". It is the version
            # the user sees in an information panel.
            SET(MACOSX_BUNDLE_SHORT_VERSION_STRING "${OSGEARTH_MAJOR_VERSION}.${OSGEARTH_MINOR_VERSION}.${OSGEARTH_PATCH_VERSION}")
            # Bundle version is the version the OS looks at.
            SET(MACOSX_BUNDLE_BUNDLE_VERSION "${OSGEARTH_MAJOR_VERSION}.${OSGEARTH_MINOR_VERSION}.${OSGEARTH__PATCH_VERSION}")
            SET(MACOSX_BUNDLE_GUI_IDENTIFIER "org.osgearth.${TARGET_TARGETNAME}" )
            SET(MACOSX_BUNDLE_BUNDLE_NAME "${TARGET_NAME}" )
            # SET(MACOSX_BUNDLE_ICON_FILE "myicon.icns")
            # SET(MACOSX_BUNDLE_COPYRIGHT "")
            # SET(MACOSX_BUNDLE_INFO_STRING "Info string, localized?")
        ENDIF(APPLE)

        IF(WIN32)
            IF (REQUIRE_WINMAIN_FLAG)
                SET(PLATFORM_SPECIFIC_CONTROL WIN32)
            ENDIF(REQUIRE_WINMAIN_FLAG)
        ENDIF(WIN32)

        IF(APPLE)
            IF(VPB_BUILD_APPLICATION_BUNDLES)
                SET(PLATFORM_SPECIFIC_CONTROL MACOSX_BUNDLE)
            ENDIF(VPB_BUILD_APPLICATION_BUNDLES)
        ENDIF(APPLE)

        ADD_EXECUTABLE(${TARGET_TARGETNAME} ${PLATFORM_SPECIFIC_CONTROL} ${TARGET_SRC} ${TARGET_H})

    ENDIF(${IS_COMMANDLINE_APP})
    
    SET_TARGET_PROPERTIES(${TARGET_TARGETNAME} PROPERTIES PROJECT_LABEL "${TARGET_LABEL}")
    SET_TARGET_PROPERTIES(${TARGET_TARGETNAME} PROPERTIES OUTPUT_NAME ${TARGET_NAME})
    SET_TARGET_PROPERTIES(${TARGET_TARGETNAME} PROPERTIES DEBUG_OUTPUT_NAME "${TARGET_NAME}${CMAKE_DEBUG_POSTFIX}")
    SET_TARGET_PROPERTIES(${TARGET_TARGETNAME} PROPERTIES RELEASE_OUTPUT_NAME "${TARGET_NAME}${CMAKE_RELEASE_POSTFIX}")
    SET_TARGET_PROPERTIES(${TARGET_TARGETNAME} PROPERTIES RELWITHDEBINFO_OUTPUT_NAME "${TARGET_NAME}${CMAKE_RELWITHDEBINFO_POSTFIX}")
    SET_TARGET_PROPERTIES(${TARGET_TARGETNAME} PROPERTIES MINSIZEREL_OUTPUT_NAME "${TARGET_NAME}${CMAKE_MINSIZEREL_POSTFIX}")

    IF(OSG_BUILD_PLATFORM_IPHONE)
        SET_TARGET_PROPERTIES(${TARGET_TARGETNAME} PROPERTIES XCODE_ATTRIBUTE_ENABLE_BITCODE ${IPHONE_ENABLE_BITCODE})
    ENDIF()

    SETUP_LINK_LIBRARIES()

ENDMACRO(SETUP_EXE)

# Taked optional second arg: APPLICATION_FOLDER
# Takes optional third arg:  (is_commandline_app?) in ARGV2
MACRO(SETUP_APPLICATION APPLICATION_NAME)

    SET(TARGET_NAME ${APPLICATION_NAME} )
    
    # 2nd arguemnt: application folder name for IDE?
    IF(${ARGC} GREATER 1)
        SET(APPLICATION_FOLDER ${ARGV1})
    ELSE(${ARGC} GREATER 1)
        SET(APPLICATION_FOLDER ${TARGET_DEFAULT_APPLICATION_FOLDER})
    ENDIF(${ARGC} GREATER 1)

    # 3rd argument: is it a command-line app?
    IF(${ARGC} GREATER 2)
        SET(IS_COMMANDLINE_APP ${ARGV2})
    ELSE(${ARGC} GREATER 2)
        SET(IS_COMMANDLINE_APP 0)
    ENDIF(${ARGC} GREATER 2)

    SETUP_EXE(${IS_COMMANDLINE_APP})
        
    INSTALL(TARGETS ${TARGET_TARGETNAME} RUNTIME DESTINATION bin  )
	#Install to the OSG_DIR as well
	IF(OSGEARTH_INSTALL_TO_OSG_DIR AND OSG_DIR)
	  INSTALL(TARGETS ${TARGET_TARGETNAME} RUNTIME DESTINATION ${OSG_DIR}/bin)
	ENDIF(OSGEARTH_INSTALL_TO_OSG_DIR AND OSG_DIR)
	
	IF(NOT APPLICATION_FOLDER)
	    SET(APPLICATION_FOLDER "Examples")
	ENDIF(NOT APPLICATION_FOLDER)
	
	SET_PROPERTY(TARGET ${TARGET_TARGETNAME} PROPERTY FOLDER ${APPLICATION_FOLDER})

ENDMACRO(SETUP_APPLICATION)

MACRO(SETUP_COMMANDLINE_APPLICATION APPLICATION_NAME)

    SETUP_APPLICATION(${APPLICATION_NAME} 1)

ENDMACRO(SETUP_COMMANDLINE_APPLICATION)

# Takes optional second argument (is_commandline_app?) in ARGV1
MACRO(SETUP_EXAMPLE EXAMPLE_NAME)

        SET(TARGET_NAME ${EXAMPLE_NAME} )

        IF(${ARGC} GREATER 1)
            SET(IS_COMMANDLINE_APP ${ARGV1})
        ELSE(${ARGC} GREATER 1)
            SET(IS_COMMANDLINE_APP 0)
        ENDIF(${ARGC} GREATER 1)

        SETUP_EXE(${IS_COMMANDLINE_APP})

    INSTALL(TARGETS ${TARGET_TARGETNAME} RUNTIME DESTINATION share/OpenSceneGraph/bin BUNDLE DESTINATION share/OpenSceneGraph/bin  )

ENDMACRO(SETUP_EXAMPLE)


MACRO(SETUP_COMMANDLINE_EXAMPLE EXAMPLE_NAME)

    SETUP_EXAMPLE(${EXAMPLE_NAME} 1)

ENDMACRO(SETUP_COMMANDLINE_EXAMPLE)


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
		"${CMAKE_SOURCE_DIR}/CMakeModules/ConfigureShaders.cmake.in"
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
			"${CMAKE_SOURCE_DIR}/CMakeModules/ConfigureShaders.cmake.in" )
	
endmacro(configure_shaders)

# http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
        LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()
