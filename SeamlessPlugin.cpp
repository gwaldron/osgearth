/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osg/Notify>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgEarth/EarthFile>
#include <osgEarth/Profile>

#include <string>
#include <iostream>

using namespace osgEarth;

class SeamlessPlugin : public osgDB::ReaderWriter
{
public:
    SeamlessPlugin() {}

    virtual const char* className()
    {
        return "OSG Earth Seamless Engine";
    }

    virtual bool acceptsExtension(const std::string& extension) const
    {
        return osgDB::equalCaseInsensitive( extension, "engine_seamless" );
    }

    virtual ReadResult readNode(const std::string& uri, const Options* options) const
    {
        std::string earthFile = osgDB::getNameLessExtension( uri );
        osgEarth::EarthFile ef;
        if (ef.readXML(earthFile))
        {
            Map* map = ef.getMap();
            if (map->getCoordinateSystemType() != Map::CSTYPE_PROJECTED)
            {
                OSG_WARN << "map is not projected\n";
                return ReadResult::FILE_NOT_FOUND;
            }
            const Profile* profile = map->getProfile();
            if (!profile)
            {
                OSG_WARN << "no map profile\n";
                return ReadResult::FILE_NOT_FOUND;
            }
            unsigned xTiles, yTiles;
            profile->getNumTiles(0, xTiles, yTiles);
            std::cout << "x tiles: " << xTiles << " y tiles: " << yTiles
                      << "\n";
            std::cout << "LOD for 10m resolution: "
                      << profile->getLevelOfDetailForHorizResolution(10, 128)
                      << "\n";
            return ReadResult::FILE_NOT_FOUND;

        }
        else
        {
            return ReadResult::FILE_NOT_FOUND;
        }
    }           
};

REGISTER_OSGPLUGIN(engine_seamless, SeamlessPlugin)
