/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_LineStyle"

using namespace osgEarth_kml;

void 
KML_LineStyle::scan( xml_node<>* node, Style& style, KMLContext& cx )
{
    if ( node )
    {
        LineSymbol* line = style.getOrCreate<LineSymbol>();
		std::string color = getValue(node, "color");
        if ( !color.empty() )
        {
            line->stroke().mutable_value().color() = Color( Stringify() << "#" << color, Color::ABGR );
        }
		std::string widthStr = getValue(node, "width");
        if ( !widthStr.empty() )
        {
            float width = as<float>(widthStr, 1.0f);
            line->stroke().mutable_value().width() = Distance(width, Units::PIXELS);
        }
    }
}
