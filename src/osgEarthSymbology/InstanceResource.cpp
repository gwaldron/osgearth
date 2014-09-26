/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthSymbology/InstanceResource>
#include <osgEarth/StringUtils>
#include <osgEarth/ImageUtils>

#include <osg/AutoTransform>
#include <osg/Depth>
#include <osg/Geometry>
#include <osg/TextureRectangle>
#include <osg/Program>

#define LC "[InstanceResource] "

using namespace osgEarth;
using namespace osgEarth::Symbology;

//---------------------------------------------------------------------------

InstanceResource::InstanceResource( const Config& conf ) :
Resource( conf )
{
    mergeConfig( conf );
}

void
InstanceResource::mergeConfig( const Config& conf )
{
    conf.getIfSet( "url", _uri );
}

Config
InstanceResource::getConfig() const
{
    Config conf = Resource::getConfig();
    conf.key() = "instance";

    conf.updateIfSet( "url", _uri );

    return conf;
}

osg::Node*
InstanceResource::createNode( const osgDB::Options* dbOptions ) const
{
    return createNodeFromURI( _uri.value(), dbOptions );
}
