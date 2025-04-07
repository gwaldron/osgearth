/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_LabelStyle"

using namespace osgEarth_kml;

void 
KML_LabelStyle::scan( xml_node<>* node, Style& style, KMLContext& cx )
{
    if (!node)
      return;
    TextSymbol* text = style.getOrCreate<TextSymbol>();
    std::string color = getValue(node, "color");
    if (!color.empty())
    {
      text->fill().mutable_value().color() = Color(Stringify() << "#" << color, Color::ABGR);
    } 
}
