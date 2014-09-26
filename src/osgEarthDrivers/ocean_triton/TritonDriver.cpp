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
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarth/ThreadingUtils>
#include <osgEarthUtil/Ocean>

#include "TritonNode"

#undef  LC
#define LC "[TritonDriver] "

using namespace osgEarth;
using namespace osgEarth::Util;

//---------------------------------------------------------------------------

namespace osgEarth { namespace Drivers { namespace Triton
{
    class TritonDriver : public OceanDriver
    {
    public:
        TritonDriver()
        {
            supportsExtension(
                "osgearth_ocean_triton",
                "osgEarth Triton Ocean plugin" );
        }

        const char* className()
        {
            return "osgEarth Triton Ocean plugin";
        }

        ReadResult readNode(const std::string& file_name, const Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            TritonOptions tritonOptions = getOceanOptions(options);

            // if the Resource Path isn't set, attempt to set it from 
            // the SL environment variable.
            if ( !tritonOptions.resourcePath().isSet() )
            {
                const char* ev = ::getenv("TRITON_PATH");
                if ( ev )
                {
                    tritonOptions.resourcePath() = osgDB::concatPaths(
                        std::string(ev),
                        "Resources" );

                    OE_INFO << LC 
                        << "Setting resource path to << " << tritonOptions.resourcePath().get()
                        << std::endl;
                }
                else
                {
                    OE_WARN << LC
                        << "No resource path! Triton might not initialize properly. "
                        << "Consider setting the TRITON_PATH environment variable."
                        << std::endl;
                }
            }

            MapNode* mapNode = getMapNode(options);
            return new TritonNode( mapNode, tritonOptions );
        }

    protected:
        virtual ~TritonDriver() { }
    };

    REGISTER_OSGPLUGIN(osgearth_ocean_triton, TritonDriver)

} } } // namespace osgEarth::Drivers::Triton
