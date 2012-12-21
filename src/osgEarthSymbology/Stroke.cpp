/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthSymbology/Stroke>

using namespace osgEarth;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

Stroke::Stroke() :
_color        ( 1, 1, 1, 1 ),
_lineCap      ( LINECAP_ROUND ),
_lineJoin     ( LINEJOIN_ROUND ),
_width        ( 1.0f ),
_roundingRatio( 0.4f )
{
    //nop
}

Stroke::Stroke( float r, float g, float b, float a ) :
_color        ( r, g, b, a ),
_lineCap      ( LINECAP_ROUND ),
_lineJoin     ( LINEJOIN_ROUND ),
_width        ( 1.0f ),
_roundingRatio( 0.4f )
{
    //nop
}

Stroke::Stroke(const Color& color) :
_color        ( color ),
_lineCap      ( LINECAP_ROUND ),
_lineJoin     ( LINEJOIN_ROUND ),
_width        ( 1.0f ),
_roundingRatio( 0.4f )
{
    //nop
}

Config 
Stroke::getConfig() const {
    Config conf("stroke");
    conf.add( "color", _color.toHTML() );
    conf.addIfSet("linecap", "flat",   _lineCap, LINECAP_FLAT);
    conf.addIfSet("linecap", "square", _lineCap, LINECAP_SQUARE);
    conf.addIfSet("linecap", "round",  _lineCap, LINECAP_ROUND);
    conf.addIfSet("linejoin", "mitre", _lineJoin, LINEJOIN_MITRE);
    conf.addIfSet("linejoin", "round", _lineJoin, LINEJOIN_ROUND);
    conf.addIfSet("width", _width);
    conf.addIfSet("stipple", _stipple);
    conf.addIfSet("rounding_ratio", _roundingRatio);
    return conf;
}

void 
Stroke::mergeConfig( const Config& conf ) {
    _color = Color( conf.value("color") );
    conf.getIfSet("linecap", "flat",   _lineCap, LINECAP_FLAT);
    conf.getIfSet("linecap", "square", _lineCap, LINECAP_SQUARE);
    conf.getIfSet("linecap", "round",  _lineCap, LINECAP_ROUND);
    conf.getIfSet("linejoin", "mitre", _lineJoin, LINEJOIN_MITRE);
    conf.getIfSet("linejoin", "miter", _lineJoin, LINEJOIN_MITRE); // alternate spelling
    conf.getIfSet("linejoin", "round", _lineJoin, LINEJOIN_ROUND);
    conf.getIfSet("width", _width);
    conf.getIfSet("stipple", _stipple);
    conf.getIfSet("rounding_ratio", _roundingRatio);
}
