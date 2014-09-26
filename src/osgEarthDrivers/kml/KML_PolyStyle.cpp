/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
KML_PolyStyle::scan( const Config& conf, Style& style, KMLContext& cx )
{
	if (!conf.empty())
	{
		Color color(Color::White);
		bool colorSpecified = conf.hasValue("color");
		if (colorSpecified)
		{
			color = Color(Stringify() << "#" << conf.value("color"), Color::ABGR);
		}

		bool fill = true;	// By default it is true
		if (conf.hasValue("fill"))
		{
			fill = (as<int>(conf.value("fill"), 1) == 1);
			if (!fill)
			{
				color.a() = 0;
			}
		}

		if (colorSpecified || !style.has<PolygonSymbol>())
		{
			PolygonSymbol* poly = style.getOrCreate<PolygonSymbol>();
			poly->fill()->color() = color;
		}

		bool outline = true;	// By default it is true
		if (conf.hasValue("outline"))
		{
			outline = (as<int>(conf.value("outline"), 0) == 1);
		}
		if (!outline)
		{
			LineSymbol* line = style.getOrCreate<LineSymbol>();
			line->stroke()->color().a() = 0;
		}
	}
}
