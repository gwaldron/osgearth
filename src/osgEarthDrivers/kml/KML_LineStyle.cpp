/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
            line->stroke()->color() = Color( Stringify() << "#" << color, Color::ABGR );
        }
		std::string width = getValue(node, "width");
        if ( !width.empty() )
        {
            line->stroke()->width() = as<float>( width, 1.0f );

            if ( line->stroke()->width() == 0.0f )
            {
                line->stroke()->width() = 1.0f;
            }
        }
    }
}
