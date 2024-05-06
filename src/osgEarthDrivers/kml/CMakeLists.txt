set(TARGET_H
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

add_osgearth_plugin(
    TARGET osgdb_kml
    HEADERS ${TARGET_H}
    PUBLIC_HEADERS KML KMLOptions
    SOURCES ${TARGET_SRC} )
    