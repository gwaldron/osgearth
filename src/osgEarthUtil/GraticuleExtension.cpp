/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include "GraticuleExtension"
#include "GraticuleTerrainEffect"
#include "GraticuleNode"

#include <osgEarth/MapNode>
#include <osgDB/FileNameUtils>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[GraticuleExtension] "


GraticuleExtension::GraticuleExtension()
{
    //nop
}

GraticuleExtension::GraticuleExtension(const GraticuleOptions& options) :
_options( options )
{
    //nop
}

GraticuleExtension::~GraticuleExtension()
{
    //nop
}

void
GraticuleExtension::setDBOptions(const osgDB::Options* dbOptions)
{
    _dbOptions = dbOptions;
}

bool
GraticuleExtension::connect(MapNode* mapNode)
{
    if ( !mapNode )
    {
        OE_WARN << LC << "Illegal: MapNode cannot be null." << std::endl;
        return false;
    }

   _node = new GraticuleNode(mapNode, _options);
    mapNode->addChild(_node.get());
    
    OE_INFO << LC << "Installed!\n";

    return true;
}

bool
GraticuleExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode )
    {
        mapNode->removeChild(_node.get());
    }
    _node = 0L;
    return true;
}



// Register the GraticuleExtension as a plugin
class GraticulePlugin : public osgDB::ReaderWriter
{
public: // Plugin stuff

    GraticulePlugin() {
        supportsExtension( "osgearth_graticule", "osgEarth Graticule Extension" );
    }

    const char* className() {
        return "osgEarth Graticule Extension";
    }

    virtual ~GraticulePlugin() { }

    ReadResult readObject(const std::string& filename, const osgDB::Options* dbOptions) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension(filename)) )
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new GraticuleExtension(Extension::getConfigOptions(dbOptions)) );
    }
};

REGISTER_OSGPLUGIN(osgearth_graticule, GraticulePlugin)

