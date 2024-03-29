/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/InstanceResource>

#define LC "[InstanceResource] "

using namespace osgEarth;

//---------------------------------------------------------------------------

InstanceResource::InstanceResource( const Config& conf ) :
Resource( conf )
{
    mergeConfig( conf );
}

void
InstanceResource::mergeConfig( const Config& conf )
{
    conf.get( "url", _uri );
}

Config
InstanceResource::getConfig() const
{
    Config conf = Resource::getConfig();
    conf.key() = "instance";

    conf.set( "url", _uri );

    return conf;
}

osg::Node*
InstanceResource::createNode( const osgDB::Options* dbOptions ) const
{
    if (_node.valid())
        return _node.get();
    else
        return createNodeFromURI( _uri.value(), dbOptions );
}
