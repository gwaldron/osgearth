/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/GUIEventHandler>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarth/Utils>
#include <osgEarth/CompositeTileSource>

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osgEarthDrivers/gdal/GDALOptions>

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Util;

static void
getFiles(const std::string &file, const std::vector<std::string> &exts, std::vector<std::string> &files)
{
    if (osgDB::fileType(file) == osgDB::DIRECTORY)
    {
        osgDB::DirectoryContents contents = osgDB::getDirectoryContents(file);
        for (osgDB::DirectoryContents::iterator itr = contents.begin(); itr != contents.end(); ++itr)
        {
            if (*itr == "." || *itr == "..") continue;
            std::string f = osgDB::concatPaths(file, *itr);
            getFiles(f, exts, files);
        }
    }
    else
    {
        bool fileValid = false;
        //If we have no _extensions specified, assume we should try everything
        if (exts.size() == 0)
        {
            fileValid = true;
        }
        else
        {
            //Only accept files with the given _extensions
            std::string ext = osgDB::getFileExtension(file);
            for (unsigned int i = 0; i < exts.size(); ++i)
            {
                if (osgDB::equalCaseInsensitive(ext, exts[i]))
                {
                    fileValid = true;
                    break;
                }
            }
        }
        
        if (fileValid)
        {
          files.push_back(osgDB::convertFileNameToNativeStyle(file));
        }
    }
}

//
// Loads a directory of images, demonstrates the programatic use of the CompositeTileSource
// usage:  osgearth_composite --dir DIRECTORY_OF_IMAGES --ext EXTENSION [--ext EXTENSION]
// example: osgearth_composite --dir c:\myimages --ext jpg --ext png
// NOTE: run this sample from the repo/tests directory.
//
int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    std::vector<std::string> files;
    std::vector< std::string > exts;    

    //Specify a directory
    std::string directory = ".";    
    while (arguments.read("--dir", directory) );

    //Specify the extensions that are valid
    std::string ext;
    while (arguments.read("--ext", ext)) { exts.push_back( ext ); };

    OE_NOTICE << "directory=" << directory << std::endl;

    getFiles(directory, exts, files);

    // Start by creating the map:
    Map* map = new Map();

    //Add a base layer
    GDALOptions basemapOpt;
    basemapOpt.url() = "../data/world.tif";
    map->addImageLayer( new ImageLayer( ImageLayerOptions("basemap", basemapOpt) ) );    

    osgEarth::CompositeTileSourceOptions compositeOpt; 
    for (unsigned int i = 0; i < files.size(); i++)
    { 
        GDALOptions gdalOpt; 
        gdalOpt.url() = files[i];
        ImageLayerOptions ilo(files[i], gdalOpt);
        //Set the transparent color on each image        
        //ilo.transparentColor() = osg::Vec4ub(255, 255, 206, 0); 
        compositeOpt.add( ilo );
        OE_NOTICE << "Added file " << files[i] << std::endl;
    } 

    map->addImageLayer( new ImageLayer( ImageLayerOptions("composite", compositeOpt) ) );

    MapNode* mapNode = new MapNode( map );

    viewer.setSceneData( mapNode );
    viewer.setCameraManipulator( new EarthManipulator() );


    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
