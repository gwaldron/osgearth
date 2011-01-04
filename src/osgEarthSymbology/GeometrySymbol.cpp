/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthSymbology/GeometrySymbol>

using namespace osgEarth::Symbology;

Stroke::Stroke() :
_color( 1, 1, 1, 1 ),
_lineCap( LINECAP_DEFAULT ),
_lineJoin( LINEJOIN_DEFAULT ),
_width( 1.0f )
{
    //nop
}

Stroke::Stroke( float r, float g, float b, float a ) :
_color( r, g, b, a ),
_lineCap( LINECAP_DEFAULT ),
_lineJoin( LINEJOIN_DEFAULT ),
_width( 1.0f )
{
    //nop
}

Fill::Fill( float r, float g, float b, float a ) :
_color( r, g, b, a )
{
    //nop
}

Fill::Fill() :
_color( 1, 1, 1, 1 )
{
    //nop
}

LineSymbol::LineSymbol() :
_stroke( Stroke() )
{
    //nop
}


PolygonSymbol::PolygonSymbol() :
_fill( Fill() )
{
    //nop
}

PointSymbol::PointSymbol() :
_fill( Fill() ), _size(1.0)
{
    //nop
}

