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
#include <osgEarthSymbology/Symbol>

using namespace osgEarth;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

Symbol::Symbol( const Config& conf )
{
    _uriContext = conf.uriContext();
}

//------------------------------------------------------------------------

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

Config 
Stroke::getConfig() const {
    Config conf("stroke");
    conf.add( "color", vec4fToHtmlColor(_color) );
    conf.addIfSet("linecap", "butt", _lineCap, LINECAP_BUTT);
    conf.addIfSet("linecap", "square", _lineCap, LINECAP_SQUARE);
    conf.addIfSet("linecap", "round", _lineCap, LINECAP_ROUND);
    conf.addIfSet("width", _width);
    conf.addIfSet("stipple", _stipple);
    return conf;
}

void 
Stroke::mergeConfig( const Config& conf ) {
    _color = htmlColorToVec4f( conf.value("color") );
    conf.getIfSet("linecap", "butt", _lineCap, LINECAP_BUTT);
    conf.getIfSet("linecap", "square", _lineCap, LINECAP_SQUARE);
    conf.getIfSet("linecap", "round", _lineCap, LINECAP_ROUND);
    conf.getIfSet("width", _width);
    conf.getIfSet("stipple", _stipple);
}

//------------------------------------------------------------------------

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

Config
Fill::getConfig() const
{
    Config conf("fill");
    conf.add("color", vec4fToHtmlColor(_color));
    return conf;
}

void
Fill::mergeConfig( const Config& conf )
{
    _color = htmlColorToVec4f(conf.value("color"));
}
