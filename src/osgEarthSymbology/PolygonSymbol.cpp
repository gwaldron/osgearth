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
#include <osgEarthSymbology/PolygonSymbol>

using namespace osgEarth;
using namespace osgEarth::Symbology;

PolygonSymbol::PolygonSymbol( const Config& conf ) :
Symbol( conf ),
_fill ( Fill() )
{
    mergeConfig(conf);
}

Config 
PolygonSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "polygon";
    conf.addObjIfSet( "fill", _fill );
    return conf;
}

void 
PolygonSymbol::mergeConfig(const Config& conf )
{
    conf.getObjIfSet( "fill", _fill );
}
