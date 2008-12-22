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

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

#include <osg/io_utils>

#include <osgEarth/Common>
#include <osgEarth/MapConfig>
#include <osgEarth/CacheSeed>

#include <iostream>
#include <sstream>

using namespace osgEarth;

int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser args(&argc,argv);
    args.getApplicationUsage()->setApplicationName(args.getApplicationName());
    args.getApplicationUsage()->setDescription(args.getApplicationName() + " is an application used to export a mapfile to a local cache.");
    args.getApplicationUsage()->setCommandLineUsage(args.getApplicationName()+" [options] filename ...");
    args.getApplicationUsage()->addCommandLineOption("--max-level level","The maximum level to export down to.");
    args.getApplicationUsage()->addCommandLineOption("-l","Shorthand for --max-level.");
    args.getApplicationUsage()->addCommandLineOption("--bounds minlon minlat maxlon maxlat","The geospatial extents to export.");
    args.getApplicationUsage()->addCommandLineOption("-b","Shorthand for --bounds.");
    args.getApplicationUsage()->addCommandLineOption("--export export_directory","The directory to export to.");
    args.getApplicationUsage()->addCommandLineOption("-e","Shorthand for --export.");

    // if user request help write it out to cout.
    if (args.read("-h") || args.read("--help") || args.argc() <= 1)
    {
        args.getApplicationUsage()->write(std::cout);
        return 1;
    }


    args.getApplicationUsage()->setApplicationName(args.getApplicationName());

    unsigned int maxLevel = 5;

    Bounds bounds(0, 0, 0, 0);
    while (args.read("--max-level", maxLevel));
    while (args.read("-l", maxLevel));
    while (args.read("--bounds", bounds._min.x(), bounds._min.y(), bounds._max.x(), bounds._max.y()));
    while (args.read("-b", bounds._min.x(), bounds._min.y(), bounds._max.x(), bounds._max.y()));

    std::string export_dir("export");
    while (args.read("--export", export_dir));
    while (args.read("-e", export_dir));

    std::string filename;

    //Find the input filename
    for(int pos=1;pos<args.argc();++pos)
    {
        if (!args.isOption(pos))
        {
            filename = args[pos];
        }
    } 

    //Make sure they specified an input file
    if (filename.size() == 0 )
    {
        osg::notify(osg::NOTICE) << "Please specify a .earth file to export" << std::endl;
        return 1;
    }

    //Load the map file
    osg::ref_ptr<MapConfig> map = MapConfigReaderWriter::readXml( filename );
    if ( map.valid() )
    {
        //Write out the destination map file
        std::string export_map_file = osgDB::concatPaths(export_dir, osgDB::getSimpleFileName( filename) );
        osg::notify(osg::NOTICE) << "Exporting file to " << export_map_file << std::endl;

        std::string path = osgDB::getFilePath( export_map_file );
        //If the path doesn't currently exist or we can't create the path, don't cache the file
        if (!osgDB::fileExists( path ) && !osgDB::makeDirectory( path ) )
        {
            osg::notify(osg::NOTICE) << "Couldn't create path " << path << std::endl;
            return 1;
        }

        //The cache will be relative to the output location of the mapfile
        map->setCachePath("cache");
        //Set the map to be offline before writing
        map->setOfflineHint( true );
        //Actually write the mapfile
        MapConfigReaderWriter::writeXml(map.get(), export_map_file); 
        //Set the filename of the map so relative paths will be relative to the location of the mapfile
        map->setFilename( osgDB::getRealPath( export_map_file ) );
        //Set the offline hint to false so that new tiles can be downloading during the export operation
        map->setOfflineHint( false );
 
        osg::ref_ptr<CacheSeed> seed = new CacheSeed();
        seed->setMaxLevel(maxLevel);
        seed->setBounds(bounds);
        seed->seed( map.get() );

        return 0;
    }
    else
    {
        osg::notify(osg::NOTICE) << "Could not load earth file from " << filename << std::endl;
        return 1;
    }
}
