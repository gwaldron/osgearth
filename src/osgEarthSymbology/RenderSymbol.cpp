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
#include <osgEarthSymbology/RenderSymbol>
#include <osgEarthSymbology/Style>

using namespace osgEarth;
using namespace osgEarth::Symbology;

RenderSymbol::RenderSymbol(const Config& conf) :
Symbol    ( conf ),
_depthTest( true )
{
    mergeConfig(conf);
}

Config 
RenderSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "render";
    conf.addIfSet( "depth_test", _depthTest );
    return conf;
}

void 
RenderSymbol::mergeConfig( const Config& conf )
{
    conf.getIfSet( "depth_test", _depthTest );
}

void
RenderSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "render-depth-test") ) {
        style.getOrCreate<RenderSymbol>()->depthTest() = as<bool>(c.value(), true);
    }
}
