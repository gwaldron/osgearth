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
#include <osgEarthSymbology/PointSymbol>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Symbology;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(point, PointSymbol);

PointSymbol::PointSymbol(const PointSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_fill(rhs._fill),
_size(rhs._size)
{
}

PointSymbol::PointSymbol( const Config& conf ) :
Symbol( conf ),
_fill ( Fill() ), 
_size ( 1.0 )
{
    mergeConfig(conf);
}

Config 
PointSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "point";
    conf.addObjIfSet( "fill", _fill );
    conf.addIfSet( "size", _size );
    return conf;
}

void 
PointSymbol::mergeConfig( const Config& conf )
{
    conf.getObjIfSet( "fill", _fill );
    conf.getIfSet( "size", _size );
}


void
PointSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "point-fill") ) {
        style.getOrCreate<PointSymbol>()->fill()->color() = Color(c.value());
    }
    else if ( match(c.key(), "point-fill-opacity") ) {
        style.getOrCreate<PointSymbol>()->fill()->color().a() = as<float>( c.value(), 1.0f );
    }
    else if ( match(c.key(), "point-size") ) {
        style.getOrCreate<PointSymbol>()->size() = as<float>(c.value(), 1.0f);
    }
    else if ( match(c.key(), "point-script") ) {
        style.getOrCreate<PointSymbol>()->script() = StringExpression(c.value());
    }
}

