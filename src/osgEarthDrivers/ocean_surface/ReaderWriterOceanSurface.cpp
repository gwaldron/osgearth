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
#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/ThreadingUtils>

#include "OceanSurface"
#include "OceanSurfaceContainer"

#undef  LC
#define LC "[ReaderWriterOceanSurface] "

using namespace osgEarth_ocean_surface;
using namespace osgEarth;
using namespace osgEarth::Drivers;

//---------------------------------------------------------------------------

struct ReaderWriterOceanSurface : public osgDB::ReaderWriter
{
    ReaderWriterOceanSurface()
    {
        supportsExtension( "osgearth_ocean_surface", "Ocean Surface" );
    }

    ReadResult readObject(const std::string& url, const Options* options) const
    {
        return readNode( url, options );
    }

    ReadResult readNode(const std::string& url, const Options* options) const
    {
        std::string ext = osgDB::getLowerCaseFileExtension(url);
        if ( !acceptsExtension(ext) )
            return ReadResult::FILE_NOT_HANDLED;

        MapNode*             mapNode    = 0L;
        OceanSurfaceOptions* osOptions  = 0L;

        if ( options )
        {
            mapNode    = static_cast<MapNode*>( const_cast<void*>(options->getPluginData("mapNode")) );
            osOptions  = static_cast<OceanSurfaceOptions*>( const_cast<void*>(options->getPluginData("options")) );
        }

        if ( !mapNode )
            return ReadResult::ERROR_IN_READING_FILE;

        osg::observer_ptr<OceanSurfaceContainer>& node = const_cast<ReaderWriterOceanSurface*>(this)->_oceans.get(mapNode);
        if ( !node.valid() )
        {
            node = new OceanSurfaceContainer( mapNode, osOptions? *osOptions : OceanSurfaceOptions() );
            return ReadResult( node.get() );
        }
        else if ( osOptions )
        {
            node->apply( *osOptions );
            return ReadResult( node.get() );
        }
        else
        {
            return ReadResult();
        }
    }

    Threading::PerObjectMap<MapNode*, osg::observer_ptr<OceanSurfaceContainer> > _oceans;
};

REGISTER_OSGPLUGIN( osgearth_ocean_surface, ReaderWriterOceanSurface )
