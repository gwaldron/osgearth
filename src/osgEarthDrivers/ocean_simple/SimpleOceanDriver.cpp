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
#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/FileUtils>

#include "SimpleOceanNode"

#undef  LC
#define LC "[SimpleOceanDriver] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace osgEarth { namespace Drivers { namespace SimpleOcean
{
    struct SimpleOceanDriver : public OceanDriver
    {
        SimpleOceanDriver()
        {
            supportsExtension( "osgearth_ocean_simple", "Simple Ocean" );
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

            MapNode*           mapNode( getMapNode(options) );
            SimpleOceanOptions oceanOptions( getOceanOptions(options) );

            if ( !mapNode )
            {
                OE_WARN << LC << "Internal error - no MapNode marshalled" << std::endl;
                return ReadResult::ERROR_IN_READING_FILE;
            }

            return new SimpleOceanNode( oceanOptions, mapNode );
        }
    };

    REGISTER_OSGPLUGIN( osgearth_ocean_simple, SimpleOceanDriver )

} } } // namespace osgEarth::Drivers::SimpleOcean
