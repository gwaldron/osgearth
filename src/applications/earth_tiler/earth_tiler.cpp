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

#include <osgEarth/TileKey>
#include <osgEarth/Mercator>
#include <osgEarth/MapConfig>

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>

using namespace osg;
using namespace osgDB;
using namespace osgEarth;


typedef std::vector<ref_ptr<TileKey>> KeyList;


//Get the children for the tilekey
KeyList getChildren(TileKey* key)
{
    KeyList children;

    children.push_back(key->getSubkey(0));
    children.push_back(key->getSubkey(1));

    if (key->getLevelOfDetail() > 0 || dynamic_cast<MercatorTileKey*>(key))
    {
        children.push_back(key->getSubkey(2));
        children.push_back(key->getSubkey(3));
    }

    return children;
}

void addTiles(TileKey *key, KeyList &tiles, unsigned int maxLevel)
{
    //Add this tile to the list of tiles
    tiles.push_back(key);

    if (key->getLevelOfDetail() < maxLevel)
    {
        //Add all of the children
        KeyList children = getChildren(key);
        for (unsigned int i = 0; i < children.size(); ++i)
        {
            addTiles(children[i].get(), tiles, maxLevel);
        }
    }
}


int main(int argc, char **argv)
{
    //The file to tile
    std::string filename;
    unsigned int tile_width = 256;
    unsigned int tile_height = 256;

    std::string dataset = "t";

    int maxLevel = 5;

    osg::ArgumentParser args(&argc, argv);

    while (args.read("--tile-width", tile_width));
    while (args.read("--tile-height", tile_height));
    while (args.read("--max-level", maxLevel));
    while (args.read("--dataset", dataset));

    //Find the input filename
    for(int pos=1;pos<args.argc();++pos)
    {
        if (!args.isOption(pos))
        {
            filename = args[pos];
        }
    }

    if (filename.empty())
    {
        osg::notify(osg::NOTICE) << "Please specify a filename " << std::endl;
        return 1;
    }


    osg::notify(osg::NOTICE) << "Dim " << tile_width << " " << tile_height << std::endl;
    osg::notify(osg::NOTICE) << "MaxLOD " << maxLevel << std::endl;
    osg::notify(osg::NOTICE) << "Filename " << filename << std::endl;


    //Create the root TileKey
    ref_ptr<TileKey> rootKey = new MercatorTileKey("");

    KeyList tiles;

    addTiles(rootKey.get(), tiles, maxLevel);

    osg::notify(osg::NOTICE) << "Generating " << tiles.size() << " tiles " << std::endl;

    for (unsigned int i = 0; i < tiles.size(); ++i)
    {
        TileKey *key = tiles[i].get();

        double minx, miny, maxx, maxy;

        //Get the GeoExtents of the file
        if (!key->getGeoExtents(minx, miny, maxx, maxy))
        {
            osg::notify(osg::WARN) << "getGeoExtents failed" << std::endl;
            return 1;
        }

        std::stringstream ss;

        //Use gdalwarp to create an appropriate tile for this TileKey's bounds from the source file
        char destFilename[128];
        strcpy(destFilename, "./tiles/");
        strcat(destFilename, dataset.c_str());
        strcat(destFilename, key->str().c_str());
        strcat(destFilename, ".tif");

        osg::notify(osg::NOTICE) << "Generating Tile " << key->str().c_str() << std::endl;

        std::string dest_srs = "EPSG:4326";

        //Use gdalwarp to generate the tile
        ss << "gdalwarp -t_srs " << dest_srs << " -te " << minx << " " << miny << " " << maxx << " " << maxy
            << " -ts " << tile_width << " " << tile_height
            << " -multi " //Use multiple threads
            << " \"" << filename << "\"  \"" << destFilename << "\"";

        //If file exists, skip it
        if (!osgDB::fileExists(destFilename))
        {
            //osg::notify(osg::NOTICE) << "Command is " << ss.str() << std::endl;

            if (system(ss.str().c_str()))
            {
                return 1;
            }
        }
    }
    return 0;
}