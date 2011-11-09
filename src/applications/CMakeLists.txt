PROJECT(OSGEARTH_APPLICATIONS)

SET(OSGCORE_BUNDLED TRUE)

IF(NOT OSGCORE_BUNDLED)
	FIND_PACKAGE(OSGCORE)
	IF(OSGCORE_FOUND)
		INCLUDE(${OSGCORE_USE_FILE})
	ELSE(OSGCORE_FOUND)
		MESSAGE(ERROR "OSGCORE neeeded but NOT FOUND")
	ENDIF(OSGCORE_FOUND)
	SET(CMAKE_MODULE_PATH  ${PROJECT_SOURCE_DIR}/../../Macros)
ENDIF(NOT OSGCORE_BUNDLED)
SET(OPENSCENEGRAPH_APPLICATION_DIR ${PROJECT_SOURCE_DIR})

#OpenThreads, osg, osgDB and osgUtil are included elsewhere.
SET(TARGET_COMMON_LIBRARIES
    osgEarth
    osgEarthFeatures
    osgEarthUtil
    osgEarthSymbology
    osgEarthAnnotation
)

SET(TARGET_DEFAULT_PREFIX "application_")
SET(TARGET_DEFAULT_LABEL_PREFIX "Sample")

ADD_SUBDIRECTORY(osgearth_viewer)
ADD_SUBDIRECTORY(osgearth_manip)
ADD_SUBDIRECTORY(osgearth_seed)
ADD_SUBDIRECTORY(osgearth_composite)
ADD_SUBDIRECTORY(osgearth_clouds)
ADD_SUBDIRECTORY(osgearth_toc)
ADD_SUBDIRECTORY(osgearth_elevation)
ADD_SUBDIRECTORY(osgearth_features)
ADD_SUBDIRECTORY(osgearth_featureinfo)
ADD_SUBDIRECTORY(osgearth_map)
ADD_SUBDIRECTORY(osgearth_annotation)

IF(NOT ${OPENSCENEGRAPH_VERSION} VERSION_LESS "2.9.6")
    ADD_SUBDIRECTORY(osgearth_featureeditor)
ENDIF()

ADD_SUBDIRECTORY(osgearth_measure)
ADD_SUBDIRECTORY(osgearth_version)
ADD_SUBDIRECTORY(osgearth_controls)
ADD_SUBDIRECTORY(osgearth_shadercomp)
ADD_SUBDIRECTORY(osgearth_tilesource)
ADD_SUBDIRECTORY(osgearth_labels)
ADD_SUBDIRECTORY(osgearth_imageoverlay)


#ADD_SUBDIRECTORY(osgearth_symbology)

#testing
#add_subdirectory(osgearth_resources)
