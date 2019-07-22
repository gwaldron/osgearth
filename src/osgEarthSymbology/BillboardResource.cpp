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
#include <osgEarthSymbology/BillboardResource>

#define LC "[BillboardResource] "

using namespace osgEarth;
using namespace osgEarth::Symbology;


BillboardResource::BillboardResource(const Config& conf) :
InstanceResource( conf )
{
    mergeConfig( conf );
}

void
BillboardResource::mergeConfig(const Config& conf)
{
    conf.get( "width", _width );
    conf.get( "height", _height );
}

Config
BillboardResource::getConfig() const
{
    Config conf = InstanceResource::getConfig();
    conf.key() = "billboard";
    conf.set( "width", _width );
    conf.set( "height", _height );
    //nop
    return conf;
}

osg::Node*
BillboardResource::createNodeFromURI(const URI& uri, const osgDB::Options* dbOptions) const
{
    // unsupported atm.
    osg::Node* node = 0L;
    return node;
}
