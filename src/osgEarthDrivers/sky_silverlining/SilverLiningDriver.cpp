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
#include "SilverLiningOptions"
#include "SilverLiningNode"
#include <osgEarth/MapNode>
#include <osgEarthUtil/Sky>
#include <osgDB/FileNameUtils>

#define LC "[SilverLiningDriver] "

using namespace osgEarth::Util;

namespace osgEarth { namespace Drivers { namespace SilverLining
{
    class SilverLiningDriver : public SkyDriver
    {
    public:
        SilverLiningDriver()
        {
            supportsExtension(
                "osgearth_sky_silverlining",
                "osgEarth SilverLining Plugin" );
        }

        const char* className()
        {
            return "osgEarth SilverLining Plugin";
        }

        ReadResult readNode(const std::string& file_name, const osgDB::Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            SilverLiningOptions slOptions = getSkyOptions(options);

            // if the Resource Path isn't set, attempt to set it from 
            // the SL environment variable.
            if ( !slOptions.resourcePath().isSet() )
            {
                const char* ev = ::getenv("SILVERLINING_PATH");
                if ( ev )
                {
                    slOptions.resourcePath() = osgDB::concatPaths(
                        std::string(ev),
                        "Resources" );
                }
                else
                {
                    OE_WARN << LC
                        << "No resource path! SilverLining might not initialize properly. "
                        << "Consider setting the SILVERLINING_PATH environment variable."
                        << std::endl;
                }
            }

            MapNode* mapnode = getMapNode(options);
            const Map* map = mapnode ? mapnode->getMap() : 0L;
            return new SilverLiningNode( map, slOptions );
        }

    protected:
        virtual ~SilverLiningDriver() { }
    };

    REGISTER_OSGPLUGIN(osgearth_sky_silverlining, SilverLiningDriver)

} } } // namespace osgEarth::Drivers::SilverLining
