/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_LineString"

using namespace osgEarth_kml;

void
KML_LineString::parseStyle( xml_node<>* node, KMLContext& cs, Style& style )
{
    KML_Geometry::parseStyle(node, cs, style);

    // need a line symbol minimally
    LineSymbol* line = style.get<LineSymbol>();
    if ( !line )
    {
        line = style.getOrCreate<LineSymbol>();
        line->stroke().mutable_value().color() = osg::Vec4f(1,1,1,1);
    }

    if ( getValue(node, "tessellate") == "1" )
    {
        line->tessellation() = 20; // KML default
    }
}

void
KML_LineString::parseCoords( xml_node<>* node, KMLContext& cx )
{
    _geom = new LineString();
    KML_Geometry::parseCoords( node, cx );
}
