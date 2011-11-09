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
#include <osgEarthSymbology/PointSymbol>

using namespace osgEarth;
using namespace osgEarth::Symbology;

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
