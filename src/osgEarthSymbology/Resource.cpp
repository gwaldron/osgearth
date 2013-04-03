/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthSymbology/Resource>
#include <osgEarth/StringUtils>

using namespace osgEarth;
using namespace osgEarth::Symbology;

Resource::Resource( const Config& conf )
{
    mergeConfig( conf );
}

void
Resource::mergeConfig( const Config& conf )
{
    _name = conf.value("name");
    addTags( conf.value("tags") );
}

Config
Resource::getConfig() const
{
    Config conf( "resource" );
    conf.set("name", _name );

    std::string tags = tagString();
    if ( !tags.empty() )
        conf.add( "tags", tags );

    return conf;
}
