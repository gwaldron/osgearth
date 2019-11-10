include(SelectLibraryConfigurations)
include(FindPackageHandleStandardArgs)

find_path(OSG_INCLUDE_DIR "osg/Node")
find_path(OSG_GEN_INCLUDE_DIR "osg/Config") 

macro(add_lib LIB_NAME)
	STRING(TOUPPER "${LIB_NAME}" LIB_NAME_U) 
	
	find_library(${LIB_NAME_U}_LIBRARY_RELEASE NAMES ${LIB_NAME}${CMAKE_RELEASE_POSTFIX} )
	find_library(${LIB_NAME_U}_LIBRARY_DEBUG NAMES ${LIB_NAME}${CMAKE_DEBUG_POSTFIX} )
	select_library_configurations(${LIB_NAME_U})
	mark_as_advanced(${LIB_NAME_U}_LIBRARY_DEBUG ${LIB_NAME_U}_LIBRARY_RELEASE )
	if( NOT "${LIB_NAME_U}" STREQUAL "OSG")
		set(OSG_LIBRARIES ${OSG_LIBRARIES} ${${LIB_NAME_U}_LIBRARY})
	endif()
endmacro()

add_lib(osg)
add_lib(osgUtil)
add_lib(osgDB)
add_lib(osgGA)
add_lib(osgText)
add_lib(osgSim)
add_lib(osgViewer)
add_lib(osgManipulator)
add_lib(osgShadow)
add_lib(OpenThreads)

find_package_handle_standard_args(OSG
	FOUND_VAR 
	  OSG_FOUND
	REQUIRED_VARS
	  OSG_LIBRARIES
	  OSG_INCLUDE_DIR
	VERSION_VAR
	  OSG_VERSION
	FAIL_MESSAGE
	  "Could NOT find OSG, try to set the path to OSG root folder"
)

if(OSG_FOUND)
	SET( OSG_INCLUDE_DIRS ${OSG_INCLUDE_DIR} ${OSG_GEN_INCLUDE_DIR} )
endif()
