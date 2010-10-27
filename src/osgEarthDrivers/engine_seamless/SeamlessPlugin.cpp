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
#include <osgEarth/Profile>

#include "SeamlessEngineNode"

#include <string>
#include <iostream>

using namespace osgEarth;

namespace seamless
{
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
        return osgDB::equalCaseInsensitive( extension, "osgearth_engine_seamless" );
    }

    virtual ReadResult readObject(const std::string& uri, const Options* options) const
    {
        if ("osgearth_engine_seamless" == osgDB::getFileExtension(uri))
        {
            if ("earth"
                == osgDB::getNameLessExtension(osgDB::getFileExtension(uri)))
                return readNode(uri, options);
            else
                return ReadResult(new SeamlessEngineNode);
        }
    }
    
    virtual ReadResult readNode(const std::string& uri, const Options* options) const
    {
        // The pseudo-loader form for engines isn't supported anymore.
        return ReadResult::FILE_NOT_FOUND;
    }
};

REGISTER_OSGPLUGIN(osgearth_engine_seamless, SeamlessPlugin)
}
