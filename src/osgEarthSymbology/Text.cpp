/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthSymbology/Text>

using namespace osgEarth::Symbology;

TextSymbol::TextSymbol() :
_fill( Fill( 1, 1, 1, 1 ) ),
_halo( Stroke( 0.3, 0.3, 0.3, 1) ),
_size( 16.0f ),
_sizeMode( SIZEMODE_SCREEN ),
_contentAttributeDelimiter( "[]" )
//_font( "fonts/arial.ttf" )
{
    //nop
}
