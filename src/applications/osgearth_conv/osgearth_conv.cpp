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
#define LC "[osgearth_conv] "

#include <osgEarth/Notify>
#include <osgEarth/Profile>
#include <osgEarth/TileSource>
#include <osgEarth/TileHandler>
#include <osgEarth/TileVisitor>
#include <osg/ArgumentParser>
#include <osg/Timer>
#include <iomanip>

using namespace osgEarth;

// documentation
int usage(char** argv)
{
    std::cout
        << "Converts tiles from one format to another.\n\n"
        << argv[0]
        << "\n    --in [prop_name] [prop_value]       : set an input property"
        << "\n    --out [prop_name] [prop_value]      : set an output property"
        << "\n    --profile [profile def]             : set an output profile (optional; default = same as input)"
        << "\n    --min-level [int]                   : minimum level of detail"
        << "\n    --max-level [int]                   : maximum level of detail"
        << std::endl;
        
    return 0;
}


// TileHandler that copies images from one tilesource to another.
struct TileSourceToTileSource : public TileHandler
{
    TileSourceToTileSource(TileSource* source, ReadWriteTileSource* dest)
        : _source(source), _dest(dest)
    {
        //nop
    }

    bool handleTile(const TileKey& key)
    {
        bool ok = false;
        osg::ref_ptr<osg::Image> image = _source->createImage(key);
        if ( image.valid() )
            ok = _dest->storeImage(key, image.get(), 0L);
        return ok;
    }
    
    bool hasData(const TileKey& key) const
    {
        return _source->hasData(key);
    }

    TileSource*          _source;
    ReadWriteTileSource* _dest;
};


// TileHandler that copies images from an ImageLayer to a TileSource.
// This will automatically handle any mosaicing and reprojection that is
// necessary to translate from one Profile/SRS to another.
struct ImageLayerToTileSource : public TileHandler
{
    ImageLayerToTileSource(ImageLayer* source, ReadWriteTileSource* dest)
        : _source(source), _dest(dest)
    {
        //nop
    }

    bool handleTile(const TileKey& key)
    {
        bool ok = false;
        GeoImage image = _source->createImage(key);
        if ( image.valid() )
            ok = _dest->storeImage(key, image.getImage(), 0L);
        return ok;
    }
    
    bool hasData(const TileKey& key) const
    {
        return _source->getTileSource()->hasData(key);
    }

    osg::ref_ptr<ImageLayer> _source;
    ReadWriteTileSource* _dest;
};


// Custom progress reporter
struct ProgressReporter : public osgEarth::ProgressCallback
{
    bool reportProgress(double             current, 
                        double             total, 
                        unsigned           currentStage,
                        unsigned           totalStages,
                        const std::string& msg )
    {
        _mutex.lock();

        float percentage = current/total*100.0f;
        std::cout 
            << std::fixed
            << std::setprecision(1) << "\r" 
            << (int)current << "/" << (int)total
            << " (" << percentage << "%)"
            << "                        "
            << std::flush;

        if ( percentage >= 100.0f )
            std::cout << std::endl;

        _mutex.unlock();

        return false;
    }

    Threading::Mutex _mutex;
};


/**
 * Command-line tool that copies the contents of one TileSource
 * to another. All arguments are Config name/value pairs, so you need
 * to look in the header file for each driver's Options structure for
 * options :)
 *
 * Example: copy a GDAL file to an MBTiles repo:
 *
 *   osgearth_conv
 *      --in driver gdal
 *      --in url world.tif
 *      --out driver mbtiles
 *      --out filename world.db
 *
 * The "in" properties come from the GDALOptions getConfig method. The
 * "out" properties come from the MBTilesOptions getConfig method.
 *
 * Other arguments:
 *
 *      --profile [profile]   : reproject to the target profile, e.g. "wgs84"
 *      --min-level [int]     : min level of detail to copy
 *      --max-level [int]     : max level of detail to copy
 *      --threads [n]         : threads to use (may crash. Careful.)
 *
 * Of course, the output driver must support writing (by implementing
 * the ReadWriteTileSource interface).
 */
int
main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc,argv);

    if ( argc == 1 )
        return usage(argv);

    typedef std::map<std::string,std::string> KeyValue;
    std::string key, value;

    // collect input configuration:
    Config inConf;
    while( args.read("--in", key, value) )
        inConf.set(key, value);

    TileSourceOptions inOptions(inConf);
    osg::ref_ptr<TileSource> input = TileSourceFactory::openReadOnly(inOptions);
    if ( !input.valid() )
    {
        OE_WARN << LC << "Failed to open input" << std::endl;
        return -1;
    }

    TileSource::Status inputStatus = input->startup(0L);
    if ( inputStatus.isError() )
    {
        OE_WARN << LC << "Error initializing input" << std::endl;
        return -1;
    }

    // collect output configuration:
    Config outConf;
    while( args.read("--out", key, value) )
        outConf.set(key, value);

    // are we changing profiles?
    osg::ref_ptr<const Profile> outputProfile = input->getProfile();
    std::string profileString;
    bool isSameProfile = true;

    if ( args.read("--profile", profileString) )
    {
        outputProfile = Profile::create(profileString);
        if ( !outputProfile.valid() || !outputProfile->isOK() )
        {
            OE_WARN << LC << "Output profile is not recognized" << std::endl;
            return -1;
        }
        isSameProfile = outputProfile->isHorizEquivalentTo(input->getProfile());
    }

    // set the output profile.
    ProfileOptions profileOptions = outputProfile->toProfileOptions();
    outConf.add("profile", profileOptions.getConfig());

    // open the output tile source:
    TileSourceOptions outOptions(outConf);
    osg::ref_ptr<ReadWriteTileSource> output = TileSourceFactory::openReadWrite(outOptions);
    if ( !output.valid() )
    {
        OE_WARN << LC << "Failed to open output" << std::endl;
        return -1;
    }

    TileSource::Status outputStatus = output->startup(0L);
    if ( outputStatus.isError() )
    {
        OE_WARN << LC << "Error initializing output" << std::endl;
        return -1;
    }

    // Dump out some stuff...
    OE_NOTICE << LC << "FROM:\n"
        << inConf.toJSON(true)
        << std::endl;

    OE_NOTICE << LC << "TO:\n"
        << outConf.toJSON(true)
        << std::endl;

    // create the visitor.
    osg::ref_ptr<TileVisitor> visitor;

    unsigned numThreads = 1;
    if (args.read("--threads", numThreads))
    {
        MultithreadedTileVisitor* mtv = new MultithreadedTileVisitor();
        mtv->setNumThreads( numThreads < 1 ? 1 : numThreads );
        visitor = mtv;
    }
    else
    {
        visitor = new TileVisitor();
    }

    // If the profiles are identical, just use a tile copier.
    if ( isSameProfile )
    {
        OE_NOTICE << LC << "Profiles match - initiating simple tile copy" << std::endl;
        visitor->setTileHandler( new TileSourceToTileSource(input.get(), output.get()) );
    }
    else
    {
        OE_NOTICE << LC << "Profiles differ - initiating tile transformation" << std::endl;
        ImageLayer* layer = new ImageLayer(ImageLayerOptions(), input.get());
        if ( !layer->getProfile() || !layer->getProfile()->isOK() )
        {
            OE_WARN << LC << "Input profile is not valid" << std::endl;
            return -1;
        }
        visitor->setTileHandler( new ImageLayerToTileSource(layer, output.get()) );
    }
    
    // Set the level limits:
    unsigned minLevel = ~0;
    bool minLevelSet = args.read("--min-level", minLevel);

    unsigned maxLevel = 0;
    bool maxLevelSet = args.read("--max-level", maxLevel);

    // figure out the max source level:
    if ( !minLevelSet || !maxLevelSet )
    {
        for(DataExtentList::const_iterator i = input->getDataExtents().begin();
            i != input->getDataExtents().end();
            ++i)
        {
            if ( !maxLevelSet && i->maxLevel().isSet() && i->maxLevel().value() > maxLevel )
                maxLevel = i->maxLevel().value();
            if ( !minLevelSet && i->minLevel().isSet() && i->minLevel().value() < minLevel )
                minLevel = i->minLevel().value();
        }
    }
       
    if ( minLevel < ~0 )
    {
        visitor->setMinLevel( minLevel );
    }

    if ( maxLevel > 0 )
    {
        maxLevel = outputProfile->getEquivalentLOD( input->getProfile(), maxLevel );
        visitor->setMaxLevel( maxLevel );
        OE_NOTICE << LC << "Calculated max level = " << maxLevel << std::endl;
    }

    std::cout << "Working..." << std::endl;

    visitor->setProgressCallback( new ProgressReporter() );

    osg::Timer_t t0 = osg::Timer::instance()->tick();

    visitor->run( outputProfile.get() );

    osg::Timer_t t1 = osg::Timer::instance()->tick();

    std::cout
        << "Time = " 
        << std::fixed
        << std::setprecision(1)
        << osg::Timer::instance()->delta_s(t0, t1)
        << " seconds." << std::endl;

    return 0;
}
