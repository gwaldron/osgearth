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
#include <osgEarthSymbology/LineSymbol>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Symbology;

LineSymbol::LineSymbol( const Config& conf ) :
Symbol       ( conf ),
_stroke      ( Stroke() ),
_tessellation( 0 )
{
    mergeConfig(conf);
}

Config 
LineSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "line";
    conf.addObjIfSet("stroke",       _stroke);
    conf.addIfSet   ("tessellation", _tessellation);
    return conf;
}

void 
LineSymbol::mergeConfig( const Config& conf )
{
    conf.getObjIfSet("stroke",       _stroke);
    conf.getIfSet   ("tessellation", _tessellation);
}

void
LineSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "stroke") ) {
        style.getOrCreate<LineSymbol>()->stroke()->color() = Color(c.value());
    }
    else if ( match(c.key(), "stroke-opacity") ) {
        style.getOrCreate<LineSymbol>()->stroke()->color().a() = as<float>( c.value(), 1.0f );
    }
    else if ( match(c.key(), "stroke-width") ) {
        style.getOrCreate<LineSymbol>()->stroke()->width() = as<float>( c.value(), 1.0f );
    }
    else if ( match(c.key(), "stroke-linecap") ) {
        style.getOrCreate<LineSymbol>()->stroke()->lineCap() =
            c.value() == "round"  ?   Stroke::LINECAP_ROUND  :
            c.value() == "square" ?   Stroke::LINECAP_SQUARE :
          /*value == "butt"   ?*/ Stroke::LINECAP_BUTT;
    }
    else if ( match(c.key(), "stroke-tessellation") ) {
        style.getOrCreate<LineSymbol>()->tessellation() = as<unsigned>( c.value(), 0 );
    }
}
