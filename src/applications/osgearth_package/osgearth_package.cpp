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

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

#include <osg/io_utils>

#include <osgEarth/Common>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarth/StringUtils>
#include <osgEarth/HTTPClient>
#include <osgEarthUtil/TMSPackager>

#include <iostream>
#include <sstream>
#include <iterator>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[osgearth_package] "


/** Prints an error message, usage information, and returns -1. */
int
usage( const std::string& msg = "" )
{
    if ( !msg.empty() )
    {
        std::cout << msg << std::endl;
    }

    std::cout
        << std::endl
        << "USAGE: osgearth_package <earth_file>" << std::endl
        << std::endl
        << "         --tms                   : make a TMS repo" << std::endl
        << "            [--out <path>]       : root output folder of the TMS repo" << std::endl
        << "            [--ext <extension>]  : custom image file extension (e.g. jpg)" << std::endl
        << "            [--max-level <num>]  : max LOD level for tiles (all layers)" << std::endl
#if 0
        << std::endl
        << "         --tfs                   : make a TFS repo" << std::endl
        << "            [--out  <path>]      : root output folder of the TFS repo" << std::endl
        << "            [--sort <attr>]      : name of attribute by which to sort features" << std::endl
        << "            [--max  <num> ]      : target maximum # of features per tile" << std::endl
#endif
        << std::endl
        << "         [--quiet]               : suppress progress output" << std::endl;

    return -1;
}


/** Prints a message and returns a non-error return code. */
int
message( const std::string& msg )
{
    if ( !msg.empty() )
    {
        std::cout << msg << std::endl << std::endl;
    }
    return 0;
}


/** Finds an argument with the specified extension. */
std::string
findArgumentWithExtension( osg::ArgumentParser& args, const std::string& ext )
{
    for( int i=0; i<args.argc(); ++i )
    {
        std::string arg( args.argv()[i] );
        if ( endsWith( toLower(trim(arg)), ".earth" ) )
            return arg;
    }
    return "";
}


/** Packages an image layer as a TMS folder. */
int
makeTMS( osg::ArgumentParser& args )
{
    // see if the user wants to override the type extension (imagery only)
    std::string extension = "png";
    args.read( "--ext", extension );

    // verbosity?
    bool verbose = !args.read( "--quiet" );

    // find a .earth file on the command line
    std::string earthFile = findArgumentWithExtension(args, ".earth");
    if ( earthFile.empty() )
        return usage( "Missing required .earth file" );

    // folder to which to write the TMS archive.
    std::string rootFolder;
    if ( !args.read( "--out", rootFolder ) )
        rootFolder = Stringify() << earthFile << ".tms_repo";

    // max level to which to generate
    unsigned maxLevel = ~0;
    args.read( "--max-level", maxLevel );

    // load up the map
    osg::ref_ptr<MapNode> mapNode = MapNode::load( args );
    if ( !mapNode.valid() )
        return usage( "Failed to load a valid .earth file" );

    // create a folder for the output
    osgDB::makeDirectory(rootFolder);
    if ( !osgDB::fileExists(rootFolder) )
        return usage("Failed to create root output folder" );

    Map* map = mapNode->getMap();

    // fire up a packager:
    TMSPackager packager( map->getProfile() );
    packager.setVerbose( verbose );
    if ( maxLevel != ~0 )
        packager.setMaxLevel( maxLevel );
    

    // package any image layers that are enabled:
    ImageLayerVector imageLayers;
    map->getImageLayers( imageLayers );

    unsigned counter = 0;
    
    for( ImageLayerVector::iterator i = imageLayers.begin(); i != imageLayers.end(); ++i, ++counter )
    {
        ImageLayer* layer = i->get();
        if ( layer->getImageLayerOptions().enabled() == true )
        {
            std::string layerFolder = toLegalFileName( layer->getName() );
            if ( layerFolder.empty() )
                layerFolder = Stringify() << "image_layer_" << counter;

            if ( verbose )
            {
                OE_NOTICE << LC << "Packaging image layer \"" << layerFolder << "\"" << std::endl;
            }

            std::string layerRoot = osgDB::concatPaths( rootFolder, layerFolder );
            packager.package( layer, layerRoot, extension );
        }
        else if ( verbose )
        {
            OE_NOTICE << LC << "Skipping disabled layer \"" << layer->getName() << "\"" << std::endl;
        }
    }

    // package any elevation layers that are enabled:
    counter = 0;
    ElevationLayerVector elevationLayers;
    map->getElevationLayers( elevationLayers );

    for( ElevationLayerVector::iterator i = elevationLayers.begin(); i != elevationLayers.end(); ++i, ++counter )
    {
        ElevationLayer* layer = i->get();
        if ( layer->getElevationLayerOptions().enabled() == true )
        {
            std::string layerFolder = toLegalFileName( layer->getName() );
            if ( layerFolder.empty() )
                layerFolder = Stringify() << "elevation_layer_" << counter;

            if ( verbose )
            {
                OE_NOTICE << LC << "Packaging elevation layer \"" << layerFolder << "\"" << std::endl;
            }

            std::string layerRoot = osgDB::concatPaths( rootFolder, layerFolder );
            packager.package( layer, layerRoot );
        }
        else if ( verbose )
        {
            OE_NOTICE << LC << "Skipping disabled layer \"" << layer->getName() << "\"" << std::endl;
        }
    }

    return 0;
}

/**
 * Data packaging tool for osgEarth.
 */
int
main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc,argv);

    HTTPClient::setUserAgent( "osgearth_package/2.2" );

    if ( args.read("--tms") )
        return makeTMS(args);

    else
        return usage();
}
