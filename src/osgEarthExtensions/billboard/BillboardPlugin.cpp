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
#include "BillboardOptions"
#include "BillboardExtension"

#include <osgDB/ReaderWriter>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

namespace osgEarth { namespace Billboard
{
    /**
     * Plugin entry point
     */
    class BillboardPlugin : public osgDB::ReaderWriter
    {
    public: // Plugin stuff

        BillboardPlugin() {
            supportsExtension( "osgearth_billboard", "osgEarth Billboard Extension Plugin" );
        }
        
        const char* className() {
            return "osgEarth Billboard Extension Plugin";
        }

        virtual ~BillboardPlugin() { }

        ReadResult readObject(const std::string& filename, const osgDB::Options* dbOptions) const
        {
          if ( !acceptsExtension(osgDB::getLowerCaseFileExtension(filename)) )
                return ReadResult::FILE_NOT_HANDLED;

          return ReadResult( new BillboardExtension(Extension::getConfigOptions(dbOptions)) );
        }
    };

    REGISTER_OSGPLUGIN(osgearth_billboard, BillboardPlugin)

} } // namespace osgEarth::Billboard
