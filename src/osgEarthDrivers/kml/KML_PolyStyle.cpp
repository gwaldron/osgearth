/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "KML_PolyStyle"

using namespace osgEarth_kml;

void
KML_PolyStyle::scan( xml_node<>* node, Style& style, KMLContext& cx )
{
	if (node)
	{
        PolygonSymbol* poly = style.getOrCreate<PolygonSymbol>();

		Color color(Color::White);
		std::string colorVal = getValue(node, "color");
		if (!colorVal.empty())
		{
			color = Color(Stringify() << "#" << colorVal, Color::ABGR);
            poly->fill().mutable_value().color() = color;
		}

        if (poly)
        {
		    bool fill = true;	// By default it is true
		    std::string fillVal = getValue(node, "fill");
		    if (!fillVal.empty())
		    {
    			fill = (as<int>(fillVal, 1) == 1);
			    if (!fill)
			    {
                    poly->fill().mutable_value().color().a() = 0.0f;
			    }
		    }

		    std::string outlineVal = getValue(node, "outline");
		    if (!outlineVal.empty())
		    {
			    bool outline = (as<int>(outlineVal, 0) == 1);
                poly->outline() = outline;
		    }
        }
	}
}
