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
#include <osgEarthSymbology/PolygonSymbol>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Symbology;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(polygon, PolygonSymbol);

PolygonSymbol::PolygonSymbol(const PolygonSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_fill(rhs._fill),
_outline(rhs._outline)
{
    //nop
}

PolygonSymbol::PolygonSymbol( const Config& conf ) :
Symbol( conf ),
_fill ( Fill() ),
_outline( true )
{
    mergeConfig(conf);
}

Config 
PolygonSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "polygon";
    conf.set( "fill", _fill );
    conf.set("outline", _outline);
    return conf;
}

void 
PolygonSymbol::mergeConfig(const Config& conf )
{
    conf.get( "fill", _fill );
    conf.get("outline", _outline);
}

void
PolygonSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "fill") ) {
        style.getOrCreate<PolygonSymbol>()->fill()->color() = Color(c.value());
    }
    else if ( match(c.key(), "fill-opacity") ) {
        style.getOrCreate<PolygonSymbol>()->fill()->color().a() = as<float>( c.value(), 1.0f );
    }
    else if ( match(c.key(), "fill-script") ) {
        style.getOrCreate<PolygonSymbol>()->script() = StringExpression(c.value());
    }
}
