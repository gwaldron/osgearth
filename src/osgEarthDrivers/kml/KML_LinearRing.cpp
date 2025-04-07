/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_LinearRing"

using namespace osgEarth_kml;

void
KML_LinearRing::parseStyle( xml_node<>* node, KMLContext& cs, Style& style )
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
        line->tessellation() = 20;
    }
}

void
KML_LinearRing::parseCoords( xml_node<>* node, KMLContext& cx )
{
    _geom = new Ring();
    KML_Geometry::parseCoords( node, cx );
}
