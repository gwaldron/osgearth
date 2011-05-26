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

#include "PatchGroup"
#include "PatchSet"
#include "SeamlessEngineNode"

#include <string>
#include <iostream>

namespace seamless
{
using namespace osg;
using namespace osgEarth;

class SeamlessPlugin : public osgDB::ReaderWriter
{
public:
    SeamlessPlugin()
    {
        supportsExtension("osgearth_engine_seamless",
                          "osgEarth seamless engine plugin");
        supportsExtension("osgearth_engine_seamless_patch",
                          "seamless engine patch pseudo loader");
    }

    virtual const char* className()
    {
        return "OSG Earth Seamless Engine";
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
        else
          return ReadResult::FILE_NOT_HANDLED;
    }
    
    virtual ReadResult
    readNode(const std::string& uri, const Options* options) const
    {
        if ("osgearth_engine_seamless_patch" == osgDB::getFileExtension(uri))
        {
            Vec2d lowerLeft(0.0, 1.0);
            Vec2d upperRight(1.0, 1.0);
            const PatchOptions* poptions
                = dynamic_cast<const PatchOptions*>(options);
            if (!poptions)
            {
                OE_FATAL
                    << "PatchGroup reader: Options object is not PatchOptions\n";
                return osgDB::ReaderWriter::ReadResult::ERROR_IN_READING_FILE;
            }
            PatchSet* pset = poptions->getPatchSet();
            Group* result = new Group;
            for (int i = 0; i < 4; ++i)
                result->addChild(pset->createChild(poptions, i));
            return result;

        }
        else
        {
            // The pseudo-loader form for engines isn't supported anymore.
            return ReadResult::FILE_NOT_FOUND;
        }
    }
};

REGISTER_OSGPLUGIN(osgearth_engine_seamless, SeamlessPlugin)
}
