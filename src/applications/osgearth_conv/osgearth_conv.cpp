/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
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
        << "\n    --elevation                         : convert as elevation data (default is image)"
        << "\n    --profile [profile def]             : set an output profile (optional; default = same as input)"
        << "\n    --min-level [int]                   : minimum level of detail"
        << "\n    --max-level [int]                   : maximum level of detail"
        << "\n    --osg-options [OSG options string]  : options to pass to OSG readers/writers"
        << "\n    --extents [minLat] [minLong] [maxLat] [maxLong] : Lat/Long extends to copy"
        << std::endl;
        
    return 0;
}


// TileHandler that copies images from one tilesource to another.
struct TileSourceToTileSource : public TileHandler
{
    TileSourceToTileSource(TileSource* source, TileSource* dest, bool heightFields)
        : _source(source), _dest(dest), _heightFields(heightFields)
    {
        //nop
    }

    bool handleTile(const TileKey& key, const TileVisitor& tv)
    {
        bool ok = false;
        if (_heightFields)
        {
            osg::ref_ptr<osg::HeightField> hf = _source->createHeightField(key);
            if ( hf.valid() )
                ok = _dest->storeHeightField(key, hf.get(), 0L);
        }
        else
        {
            osg::ref_ptr<osg::Image> image = _source->createImage(key);
            if ( image.valid() )
                ok = _dest->storeImage(key, image.get(), 0L);
        }
        return ok;
    }
    
    bool hasData(const TileKey& key) const
    {
        return _source->hasData(key);
    }

    TileSource* _source;
    TileSource* _dest;
    bool        _heightFields;
};


// TileHandler that copies images from an ImageLayer to a TileSource.
// This will automatically handle any mosaicing and reprojection that is
// necessary to translate from one Profile/SRS to another.
struct ImageLayerToTileSource : public TileHandler
{
    ImageLayerToTileSource(ImageLayer* source, TileSource* dest)
        : _source(source), _dest(dest)
    {
        //nop
    }

    bool handleTile(const TileKey& key, const TileVisitor& tv)
    {
        bool ok = false;
        GeoImage image = _source->createImage(key);
        if (image.valid())
            ok = _dest->storeImage(key, image.getImage(), 0L);        

        return ok;
    }
    
    bool hasData(const TileKey& key) const
    {
        return _source->getTileSource()->hasData(key);
    }

    osg::ref_ptr<ImageLayer> _source;
    TileSource*              _dest;
};


// TileHandler that copies images from an ElevationLayer to a TileSource.
// This will automatically handle any mosaicing and reprojection that is
// necessary to translate from one Profile/SRS to another.
struct ElevationLayerToTileSource : public TileHandler
{
    ElevationLayerToTileSource(ElevationLayer* source, TileSource* dest)
        : _source(source), _dest(dest)
    {
        //nop
    }

    bool handleTile(const TileKey& key, const TileVisitor& tv)
    {
        bool ok = false;
        GeoHeightField hf = _source->createHeightField(key, 0L);
        if ( hf.valid() )
            ok = _dest->storeHeightField(key, hf.getHeightField(), 0L);
        return ok;
    }
    
    bool hasData(const TileKey& key) const
    {
        return _source->getTileSource()->hasData(key);
    }

    osg::ref_ptr<ElevationLayer> _source;
    TileSource*                  _dest;
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
 *      --out format image/png
 *      --out filename world.db
 *
 * The "in" properties come from the GDALOptions getConfig method. The
 * "out" properties come from the MBTilesOptions getConfig method.
 *
 * Other arguments:
 *
 *      --elevation           : convert as elevation data (instead of image data)
 *      --profile [profile]   : reproject to the target profile, e.g. "wgs84"
 *      --min-level [int]     : min level of detail to copy
 *      --max-level [int]     : max level of detail to copy
 *      --threads [n]         : threads to use (may crash. Careful.)
 *
 *      --extents [minLat] [minLong] [maxLat] [maxLong] : Lat/Long extends to copy (*)
 *
 * OSG arguments:
 *
 *      -O <string>           : OSG Options string (plugin options)
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

    osg::ref_ptr<osgDB::Options> dbo = new osgDB::Options();
    
    // plugin options, if the user passed them in:
    std::string str;
    while(args.read("--osg-options", str) || args.read("-O", str))
    {
        dbo->setOptionString( str );
    }

    TileSourceOptions inOptions(inConf);
    osg::ref_ptr<TileSource> input = TileSourceFactory::create(inOptions);
    if ( !input.valid() )
    {
        OE_WARN << LC << "Failed to open input" << std::endl;
        return -1;
    }

    Status inputStatus = input->open( input->MODE_READ, dbo.get() );
    if ( inputStatus.isError() )
    {
        OE_WARN << LC << "Error initializing input" << std::endl;
        return -1;
    }

    // collect output configuration:
    Config outConf;
    while( args.read("--out", key, value) )
        outConf.set(key, value);

    // heightfields?
    bool heightFields = args.read("--heightfield") || args.read("--hf") || args.read("--elevation");
    if ( heightFields )
        OE_INFO << LC << "Converting heightfield tiles" << std::endl;
    else
        OE_INFO << LC << "Converting image tiles" << std::endl;

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
    osg::ref_ptr<TileSource> output = TileSourceFactory::create(outOptions);
    if ( !output.valid() )
    {
        OE_WARN << LC << "Failed to open output" << std::endl;
        return -1;
    }

    Status outputStatus = output->open(
        TileSource::MODE_WRITE | TileSource::MODE_CREATE,
        dbo.get() );

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
        visitor->setTileHandler( new TileSourceToTileSource(input.get(), output.get(), heightFields) );
    }
    else
    {
        OE_NOTICE << LC << "Profiles differ - initiating tile transformation" << std::endl;

        if (heightFields)
        {
            ElevationLayer* layer = new ElevationLayer(ElevationLayerOptions(), input.get());
            Status layerStatus = layer->open();
            if (layerStatus.isError())
            {
                OE_WARN << "Failed to create input ElevationLayer " << layerStatus.message() << std::endl;
                return -1;
            }
            if ( !layer->getProfile() || !layer->getProfile()->isOK() )
            {
                OE_WARN << LC << "Input profile is not valid" << std::endl;
                return -1;
            }
            visitor->setTileHandler( new ElevationLayerToTileSource(layer, output.get()) );
        }
        else
        {
            ImageLayer* layer = new ImageLayer(ImageLayerOptions(), input.get());
            Status layerStatus = layer->open();
            if (layerStatus.isError())
            {
                OE_WARN << "Failed to create input ImageLayer " << layerStatus.message() << std::endl;
                return -1;
            }
            if ( !layer->getProfile() || !layer->getProfile()->isOK() )
            {
                OE_WARN << LC << "Input profile is not valid" << std::endl;
                return -1;
            }
            visitor->setTileHandler( new ImageLayerToTileSource(layer, output.get()) );
        }
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

    // set the extents:
    double minlat, minlon, maxlat, maxlon;
    while( args.read("--extents", minlat, minlon, maxlat, maxlon) )
    {
        GeoExtent extent(SpatialReference::get("wgs84"), minlon, minlat, maxlon, maxlat);
        visitor->addExtent( extent );
    }

    // Ready!!!
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
