/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/MapFrame>
#include <osgEarth/Cache>
#include <osgEarth/CacheEstimator>
#include <osgEarth/CacheSeed>
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>

#include <iostream>
#include <sstream>
#include <iterator>

using namespace osgEarth;
using namespace osgEarth::Drivers;

#define LC "[osgearth_cache] "

int list( osg::ArgumentParser& args );
int seed( osg::ArgumentParser& args );
int purge( osg::ArgumentParser& args );
int usage( const std::string& msg );
int message( const std::string& msg );
std::string prettyPrintTime( double seconds );
std::string prettyPrintSize( double mb );


int
main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc,argv);

    if ( args.read( "--seed") )
        return seed( args );
    else if ( args.read( "--list" ) )
        return list( args );
    else if ( args.read( "--purge" ) )
        return purge( args );
    else
        return usage("");
}

int
usage( const std::string& msg )
{
    if ( !msg.empty() )
    {
        std::cout << msg << std::endl;
    }

    std::cout
        << std::endl
        << "USAGE: osgearth_cache" << std::endl
        << std::endl
        << "    --list file.earth                   ; Lists info about the cache in a .earth file" << std::endl
        << std::endl
        << "    --seed file.earth                   ; Seeds the cache in a .earth file"  << std::endl
        << "        [--estimate]                    ; Print out an estimation of the number of tiles, disk space and time it will take to perform this seed operation" << std::endl
        << "        [--min-level level]             ; Lowest LOD level to seed (default=0)" << std::endl
        << "        [--max-level level]             ; Highest LOD level to seed (defaut=highest available)" << std::endl
        << "        [--bounds xmin ymin xmax ymax]* ; Geospatial bounding box to seed (in map coordinates; default=entire map)" << std::endl
        << "        [--index shapefile]             ; Use the feature extents in a shapefile to set the bounding boxes for seeding" << std::endl
        << "        [--cache-path path]             ; Overrides the cache path in the .earth file" << std::endl
        << "        [--cache-type type]             ; Overrides the cache type in the .earth file" << std::endl
        << "        [--threads]                     ; The number of threads to use for the seed operation (default=1)" << std::endl
        << std::endl
        << "    --purge file.earth                  ; Purges a layer cache in a .earth file (interactive)" << std::endl
        << std::endl;

    return -1;
}

int message( const std::string& msg )
{
    if ( !msg.empty() )
    {
        std::cout << msg << std::endl << std::endl;
    }
    return 0;
}

int
seed( osg::ArgumentParser& args )
{    

    //Read the min level
    unsigned int minLevel = 0;
    while (args.read("--min-level", minLevel));
    
    //Read the max level
    unsigned int maxLevel = 5;
    while (args.read("--max-level", maxLevel));

    bool estimate = args.read("--estimate");        

    unsigned int threads = 1;
    while (args.read("--threads", threads));
    

    std::vector< Bounds > bounds;
    // restrict packaging to user-specified bounds.    
    double xmin=DBL_MAX, ymin=DBL_MAX, xmax=DBL_MIN, ymax=DBL_MIN;
    while (args.read("--bounds", xmin, ymin, xmax, ymax ))
    {        
        Bounds b;
        b.xMin() = xmin, b.yMin() = ymin, b.xMax() = xmax, b.yMax() = ymax;
        bounds.push_back( b );
    }    

    //Read the cache override directory
    std::string cachePath;
    while (args.read("--cache-path", cachePath));

    //Read the cache type
    std::string cacheType;
    while (args.read("--cache-type", cacheType));

    bool verbose = args.read("--verbose");

    //Read in the earth file.
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFiles( args );
    if ( !node.valid() )
        return usage( "Failed to read .earth file." );

    MapNode* mapNode = MapNode::findMapNode( node.get() );
    if ( !mapNode )
        return usage( "Input file was not a .earth file" );
    
    // Read in an index shapefile
    std::string index;
    while (args.read("--index", index))
    {        
        //Open the feature source
        OGRFeatureOptions featureOpt;
        featureOpt.url() = index;        

        osg::ref_ptr< FeatureSource > features = FeatureSourceFactory::create( featureOpt );
        features->initialize();
        features->getFeatureProfile();

        osg::ref_ptr< FeatureCursor > cursor = features->createFeatureCursor();
        while (cursor.valid() && cursor->hasMore())
        {
            osg::ref_ptr< Feature > feature = cursor->nextFeature();
            osgEarth::Bounds featureBounds = feature->getGeometry()->getBounds();
            GeoExtent ext( feature->getSRS(), featureBounds );
            ext = ext.transform( mapNode->getMapSRS() );
            bounds.push_back( ext.bounds() );            
        }
    }

    // If they requested to do an estimate then don't do the the seed, just print out the estimated values.
    if (estimate)
    {        
        CacheEstimator est;
        est.setMinLevel( minLevel );
        est.setMaxLevel( maxLevel );
        est.setProfile( mapNode->getMap()->getProfile() );

        for (unsigned int i = 0; i < bounds.size(); i++)
        {
            GeoExtent extent(mapNode->getMapSRS(), bounds[i]);
            OE_DEBUG << "Adding extent " << extent.toString() << std::endl;
            est.addExtent( extent );
        } 

        unsigned int numTiles = est.getNumTiles();
        double size = est.getSizeInMB();
        double time = est.getTotalTimeInSeconds();
        std::cout << "Cache Estimation " << std::endl
                  << "---------------- " << std::endl
                  << "Total number of tiles: " << numTiles << std::endl
                  << "Size on disk:          " << prettyPrintSize( size ) << std::endl
                  << "Total time:            " << prettyPrintTime( time ) << std::endl;

        return 0;
    }

    CacheSeed seeder;
    seeder.setMinLevel( minLevel );
    seeder.setMaxLevel( maxLevel );
    seeder.setNumThreads( threads );


    for (unsigned int i = 0; i < bounds.size(); i++)
    {
        GeoExtent extent(mapNode->getMapSRS(), bounds[i]);
        OE_DEBUG << "Adding extent " << extent.toString() << std::endl;
        seeder.addExtent( extent );
    }    

    if (verbose)
    {
        seeder.setProgressCallback(new ConsoleProgressCallback);
    }


    osg::Timer_t start = osg::Timer::instance()->tick();

    seeder.seed( mapNode->getMap() );

    osg::Timer_t end = osg::Timer::instance()->tick();

    OE_NOTICE << "Completed seeding in " << prettyPrintTime( osg::Timer::instance()->delta_s( start, end ) ) << std::endl;

    return 0;
}

int
list( osg::ArgumentParser& args )
{
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFiles( args );
    if ( !node.valid() )
        return usage( "Failed to read .earth file." );

    MapNode* mapNode = MapNode::findMapNode( node.get() );
    if ( !mapNode )
        return usage( "Input file was not a .earth file" );

    Map* map = mapNode->getMap();
    const Cache* cache = map->getCache();

    if ( !cache )
        return message( "Earth file does not contain a cache." );

    std::cout 
        << "Cache config: " << std::endl
        << cache->getCacheOptions().getConfig().toJSON(true) << std::endl;

    MapFrame mapf( mapNode->getMap() );

    TerrainLayerVector layers;
    std::copy( mapf.imageLayers().begin(), mapf.imageLayers().end(), std::back_inserter(layers) );
    std::copy( mapf.elevationLayers().begin(), mapf.elevationLayers().end(), std::back_inserter(layers) );

    for( TerrainLayerVector::iterator i =layers.begin(); i != layers.end(); ++i )
    {
        TerrainLayer* layer = i->get();
        TerrainLayer::CacheBinMetadata meta;

        bool useMFP =
            layer->getProfile() &&
            layer->getProfile()->getSRS()->isSphericalMercator() &&
            mapNode->getMapNodeOptions().getTerrainOptions().enableMercatorFastPath() == true;

        const Profile* cacheProfile = useMFP ? layer->getProfile() : map->getProfile();

        if ( layer->getCacheBinMetadata( cacheProfile, meta ) )
        {
            Config conf = meta.getConfig();
            std::cout << "Layer \"" << layer->getName() << "\", cache metadata =" << std::endl
                << conf.toJSON(true) << std::endl;
        }
        else
        {
            std::cout << "Layer \"" << layer->getName() << "\": no cache information" 
                << std::endl;
        }
    }

    return 0;
}

struct Entry
{
    bool                   _isImage;
    std::string            _name;
    osg::ref_ptr<CacheBin> _bin;
};


int
purge( osg::ArgumentParser& args )
{
    //return usage( "Sorry, but purge is not yet implemented." );
    
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFiles( args );
    if ( !node.valid() )
        return usage( "Failed to read .earth file." );

    MapNode* mapNode = MapNode::findMapNode( node.get() );
    if ( !mapNode )
        return usage( "Input file was not a .earth file" );

    Map* map = mapNode->getMap();

    if ( !map->getCache() )
        return message( "Earth file does not contain a cache." );

    std::vector<Entry> entries;


    ImageLayerVector imageLayers;
    map->getImageLayers( imageLayers );
    for( ImageLayerVector::const_iterator i = imageLayers.begin(); i != imageLayers.end(); ++i )
    {
        ImageLayer* layer = i->get();

        bool useMFP =
            layer->getProfile() &&
            layer->getProfile()->getSRS()->isSphericalMercator() &&
            mapNode->getMapNodeOptions().getTerrainOptions().enableMercatorFastPath() == true;

        const Profile* cacheProfile = useMFP ? layer->getProfile() : map->getProfile();

        CacheBin* bin = layer->getCacheBin( cacheProfile );
        if ( bin )
        {
            entries.push_back(Entry());
            entries.back()._isImage = true;
            entries.back()._name = i->get()->getName();
            entries.back()._bin = bin;
        }
    }

    ElevationLayerVector elevationLayers;
    map->getElevationLayers( elevationLayers );
    for( ElevationLayerVector::const_iterator i = elevationLayers.begin(); i != elevationLayers.end(); ++i )
    {
        ElevationLayer* layer = i->get();

        bool useMFP =
            layer->getProfile() &&
            layer->getProfile()->getSRS()->isSphericalMercator() &&
            mapNode->getMapNodeOptions().getTerrainOptions().enableMercatorFastPath() == true;

        const Profile* cacheProfile = useMFP ? layer->getProfile() : map->getProfile();

        CacheBin* bin = i->get()->getCacheBin( cacheProfile );
        if ( bin )
        {
            entries.push_back(Entry());
            entries.back()._isImage = false;
            entries.back()._name = i->get()->getName();
            entries.back()._bin = bin;
        }
    }

    if ( entries.size() > 0 )
    {
        std::cout << std::endl;

        for( unsigned i=0; i<entries.size(); ++i )
        {
            std::cout << (i+1) << ") " << entries[i]._name << " (" << (entries[i]._isImage? "image" : "elevation" ) << ")" << std::endl;
        }

        std::cout 
            << std::endl
            << "Enter number of cache to purge, or <enter> to quit: "
            << std::flush;

        std::string input;
        std::getline( std::cin, input );

        if ( !input.empty() )
        {
            unsigned k = as<unsigned>(input, 0L);
            if ( k > 0 && k <= entries.size() )
            {
                Config meta = entries[k-1]._bin->readMetadata();
                if ( !meta.empty() )
                {
                    std::cout
                        << std::endl
                        << "Cache METADATA:" << std::endl
                        << meta.toJSON() 
                        << std::endl << std::endl;
                }

                std::cout
                    << "Are you sure (y/N)? "
                    << std::flush;

                std::getline( std::cin, input );
                if ( input == "y" || input == "Y" )
                {
                    std::cout << "Purging.." << std::flush;
                    entries[k-1]._bin->purge();
                }
                else
                {
                    std::cout << "No action taken." << std::endl;
                }
            }
            else
            {
                std::cout << "Invalid choice." << std::endl;
            }
        }
        else
        {
            std::cout << "No action taken." << std::endl;
        }
    }

    return 0;
}

/**
 * Gets the total number of seconds formatted as H:M:S
 */
std::string prettyPrintTime( double seconds )
{
    int hours = (int)floor(seconds / (3600.0) );
    seconds -= hours * 3600.0;

    int minutes = (int)floor(seconds/60.0);
    seconds -= minutes * 60.0;

    std::stringstream buf;
    buf << hours << ":" << minutes << ":" << seconds;
    return buf.str();
}

/**
 * Gets a pretty printed version of the given size in MB.
 */
std::string prettyPrintSize( double mb )
{
    std::stringstream buf;
    // Convert to terabytes
    if ( mb > 1024 * 1024 )
    {
        buf << (mb / (1024.0*1024.0)) << " TB";
    }
    else if (mb > 1024)
    {
        buf << (mb / 1024.0) << " GB";
    }
    else 
    {
        buf << mb << " MB";
    }
    return buf.str();
}
