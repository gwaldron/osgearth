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
#include <osgEarth/DriverOptions>

using namespace osgEarth;

static PluginOptions* dummyPO = new PluginOptions();


/****************************************************************/

DriverOptions::DriverOptions( const PluginOptions* rhs ) :
PluginOptions( rhs ? *rhs : *dummyPO )
{
    if ( rhs )
        fromConfig( rhs->config() );
}

DriverOptions::DriverOptions( const Config& conf ) :
PluginOptions( conf )
{
    fromConfig( conf );
}

void
DriverOptions::fromConfig( const Config& conf )
{
    _name = conf.value("name");
    _driver = conf.value("driver");
}

Config
DriverOptions::toConfig() const
{
    Config conf = config();
    conf.attr("name") = _name;
    conf.attr("driver") = _driver;
    return conf;
}




