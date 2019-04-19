/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
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
            poly->fill()->color() = color;
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
                    poly->fill()->color().a() = 0.0f;
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
