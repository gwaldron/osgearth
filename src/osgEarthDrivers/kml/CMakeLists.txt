OPTION(CURL_IS_STATIC "on if curl is a static lib " ON)
MARK_AS_ADVANCED(CURL_IS_STATIC)

IF(WIN32)
    IF(MSVC)
        SET(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:MSVCRT")
    ENDIF(MSVC)
    IF(CURL_IS_STATIC)
        ADD_DEFINITIONS(-DCURL_STATICLIB)
        SET(TARGET_EXTERNAL_LIBRARIES ws2_32 winmm)
    ENDIF(CURL_IS_STATIC)
ENDIF(WIN32)
    
set(TARGET_COMMON_LIBRARIES ${TARGET_COMMON_LIBRARIES} 
    osgEarthFeatures 
    osgEarthSymbology 
    osgEarthUtil
    osgEarthAnnotation
)

SET(TARGET_H
    KML
    KMLOptions
    KMLReader
    KML_Common
    KML_Container
    KML_Document
    KML_Feature
    KML_Folder
    KML_Geometry
    KML_GroundOverlay
    KML_IconStyle
    KML_LabelStyle
    KML_LinearRing
    KML_LineString
    KML_LineStyle
    KML_Model
    KML_MultiGeometry
    KML_NetworkLink
    KML_NetworkLinkControl
    KML_Object
    KML_Overlay
    KML_Placemark
    KML_PhotoOverlay
    KML_Point
    KML_Polygon
    KML_PolyStyle
    KML_Root
    KML_Schema
    KML_ScreenOverlay
    KML_Style
    KML_StyleMap
    KML_StyleSelector
    KMZArchive
    rapidxml_ext.hpp
)

SET(TARGET_SRC
    ReaderWriterKML.cpp
    KMLReader.cpp
    KML_Document.cpp
    KML_Feature.cpp
    KML_Folder.cpp
    KML_Geometry.cpp
    KML_GroundOverlay.cpp
    KML_IconStyle.cpp
    KML_LabelStyle.cpp
    KML_LinearRing.cpp
    KML_LineString.cpp
    KML_LineStyle.cpp
    KML_Model.cpp
    KML_MultiGeometry.cpp
    KML_NetworkLink.cpp
    KML_NetworkLinkControl.cpp
    KML_Object.cpp
    KML_Overlay.cpp
    KML_PhotoOverlay.cpp
    KML_Placemark.cpp
    KML_Point.cpp
    KML_Polygon.cpp
    KML_PolyStyle.cpp
    KML_Root.cpp
    KML_Schema.cpp
    KML_ScreenOverlay.cpp
    KML_Style.cpp
    KML_StyleMap.cpp
    KMZArchive.cpp
)
    
SETUP_PLUGIN(kml)

SET(LIB_NAME kml)
SET(LIB_PUBLIC_HEADERS KML KMLOptions)
INCLUDE(ModuleInstallOsgEarthDriverIncludes OPTIONAL)
