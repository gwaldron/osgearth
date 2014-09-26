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
#include "KML_LineStyle"

using namespace osgEarth_kml;

void 
KML_LineStyle::scan( const Config& conf, Style& style, KMLContext& cx )
{
    if ( !conf.empty() )
    {
        LineSymbol* line = style.getOrCreate<LineSymbol>();
        if ( conf.hasValue("color") )
        {
            line->stroke()->color() = Color( Stringify() << "#" << conf.value("color"), Color::ABGR );
        }
        if ( conf.hasValue("width") )
        {
            line->stroke()->width() = as<float>( conf.value("width"), 1.0f );
        }
    }
}
