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
#include "GLSkyOptions"
#include "GLSkyNode"
#include <osgDB/FileNameUtils>
#include <osgEarth/Map>
#include <osgEarth/MapNode>

#define LC "[GLSkyDriver] "

using namespace osgEarth::Util;

namespace osgEarth { namespace Drivers { namespace GLSky
{
    class GLSkyDriver : public SkyDriver
    {
    public:
        GLSkyDriver()
        {
            supportsExtension(
                "osgearth_sky_gl",
                "osgEarth GL Sky Plugin" );
        }

        const char* className()
        {
            return "osgEarth GL Sky Plugin";
        }

        ReadResult readNode(const std::string& file_name, const osgDB::Options* options) const
        {
            if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
                return ReadResult::FILE_NOT_HANDLED;

            MapNode* mapNode = getMapNode(options);
            const Profile* profile = mapNode ? mapNode->getMap()->getProfile() : 0L;
            return new GLSkyNode(profile, getSkyOptions(options));
        }

    protected:
        virtual ~GLSkyDriver() { }
    };

    REGISTER_OSGPLUGIN(osgearth_sky_gl, GLSkyDriver)

} } } // namespace osgEarth::Drivers::GLSky
