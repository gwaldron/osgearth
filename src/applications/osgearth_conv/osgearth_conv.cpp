/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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
#include <osgEarth/ImageLayer>
#include <osgEarth/ElevationLayer>
#include <osgEarth/MapNode>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/ImageUtils>

#include <osg/ArgumentParser>
#include <osg/Timer>
#include <osgDB/ReadFile>

#include <iomanip>
#include <algorithm>
#include <iterator>

using namespace osgEarth;

// documentation
int usage(char** argv)
{
    std::cout
        << "Converts tiles from one format to another.\n\n"
        << argv[0]
        << "\n    --in [prop_name] [prop_value]       : set an input property (instead of using --in-earth)"
        << "\n    --in-earth [earthfile]              : earth file from which to load input layer (instead of using --in)"
        << "\n    --in-layer [layer name]             : with --in-earth, name of layer to convert"
        << "\n    --out [prop_name] [prop_value]      : set an output property"
        << "\n    --profile [profile def]             : set an output profile (optional; default = same as input)"
        << "\n    --min-level [int]                   : minimum level of detail"
        << "\n    --max-level [int]                   : maximum level of detail"
        << "\n    --osg-options [OSG options string]  : options to pass to OSG readers/writers"
        << "\n    --extents [minLat] [minLong] [maxLat] [maxLong] : Lat/Long extends to copy"
        << "\n    --no-overwrite                      : skip tiles that already exist in the destination"
        << "\n    --threads [int]                     : go faster by using [n] working threads"
        << std::endl;

    return 0;
}

// Visitor that converts image tiles
struct ImageLayerTileCopy : public TileHandler
{
    ImageLayerTileCopy(ImageLayer* source, ImageLayer* dest, bool overwrite, bool compress)
        : _source(source), _dest(dest), _overwrite(overwrite), _compress(compress)
    {
        //nop
    }

    bool handleTile(const TileKey& key, const TileVisitor& tv)
    {
        bool ok = false;

        // if overwriting is disabled, check to see whether the destination
        // already has data for the key
        if (_overwrite == false)
        {
            if (_dest->createImage(key).valid())
            {
                return true;
            }
        }

        GeoImage image = _source->createImage(key);
        if (image.valid())
        {
            osg::ref_ptr<const osg::Image> imageToWrite = image.getImage();
            if (_compress)
                imageToWrite = ImageUtils::compressImage(image.getImage(), "cpu");

            Status status = _dest->writeImage(key, imageToWrite.get(), 0L);
            ok = status.isOK();
            if (!ok)
            {
                OE_WARN << key.str() << ": " << status.message() << std::endl;
            }
        }

        return ok;
    }

    bool hasData(const TileKey& key) const
    {
        return _source->mayHaveData(key);
    }

    osg::ref_ptr<ImageLayer> _source;
    osg::ref_ptr<ImageLayer> _dest;
    bool _overwrite;
    bool _compress;
};

// Visitor that converts elevation tiles
struct ElevationLayerTileCopy : public TileHandler
{
    ElevationLayerTileCopy(ElevationLayer* source, ElevationLayer* dest, bool overwrite)
        : _source(source), _dest(dest), _overwrite(overwrite)
    {
        //nop
    }

    bool handleTile(const TileKey& key, const TileVisitor& tv)
    {
        bool ok = false;

        // if overwriting is disabled, check to see whether the destination
        // already has data for the key
        if (_overwrite == false)
        {
            if (_dest->createHeightField(key).valid())
            {
                return true;
            }
        }

        GeoHeightField hf = _source->createHeightField(key, 0L);
        if ( hf.valid() )
        {
            Status s = _dest->writeHeightField(key, hf.getHeightField(), 0L);
            ok = s.isOK();
            if (!ok)
            {
                OE_WARN << key.str() << ": " << s.message() << std::endl;
            }
        }
        else
        {
            //OE_WARN << key.str() << " : " << hf.getStatus().message() << std::endl;
        }
        return ok;
    }

    bool hasData(const TileKey& key) const
    {
        return _source->mayHaveData(key);
    }

    osg::ref_ptr<ElevationLayer> _source;
    osg::ref_ptr<ElevationLayer> _dest;
    bool _overwrite;
};


// Custom progress reporter
struct ProgressReporter : public osgEarth::ProgressCallback
{
    ProgressReporter() : _first(true), _start(0) { }

    bool reportProgress(double             current,
                        double             total,
                        unsigned           currentStage,
                        unsigned           totalStages,
                        const std::string& msg )
    {
        ScopedMutexLock lock(_mutex);

        if (_first)
        {
            _first = false;
            _start = osg::Timer::instance()->tick();
        }
        osg::Timer_t now = osg::Timer::instance()->tick();



        float percentage = current/total;

        double timeSoFar = osg::Timer::instance()->delta_s(_start, now);
        double projectedTotalTime = timeSoFar/percentage;
        double timeToGo = projectedTotalTime - timeSoFar;
        double minsToGo = timeToGo/60.0;
        double secsToGo = fmod(timeToGo,60.0);
        double minsTotal = projectedTotalTime/60.0;
        double secsTotal = fmod(projectedTotalTime,60.0);

        std::cout
            << std::fixed
            << std::setprecision(1) << "\r"
            << (int)current << "/" << (int)total
            << " " << int(100.0f*percentage) << "% complete, "
            << (int)minsTotal << "m" << (int)secsTotal << "s projected, "
            << (int)minsToGo << "m" << (int)secsToGo << "s remaining          "
            << std::flush;

        if ( percentage >= 100.0f )
            std::cout << std::endl;

        return false;
    }

    Threading::Mutex _mutex;
    bool _first;
    osg::Timer_t _start;
};


/**
 * Command-line tool that copies the contents of one TileSource
 * to another. All arguments are Config name/value pairs, so you need
 * to look in the header file for each driver's Options structure for
 * options :)
 *
 * Example #2: copy a GDAL file to an MBTiles repo using --in:
 *
 *   osgearth_conv
 *      --in driver gdalimage
 *      --in url world.tif
 *      --out driver mbtilesimage
 *      --out format image/png
 *      --out filename world.mbtiles
 *
 *   The "in" properties come from the GDALOptions getConfig method. The
 *   "out" properties come from the MBTilesOptions getConfig method.
 *
 * Example #2: copy an earth file layer to an MBTiles repo using --in-earth:
 *
 *   osgearth_conv
 *      --in-earth myfile.earth
 *      --in-layer my_image_layer_name
 *      --out driver mbtilesimage
 *      --out format image/png
 *      --out filename world.mbtiles
 *
 * Other arguments:
 *
 *      --profile [profile]   : reproject to the target profile, e.g. "wgs84"
 *      --min-level [int]     : min level of detail to copy
 *      --max-level [int]     : max level of detail to copy
 *      --extents [minLat] [minLong] [maxLat] [maxLong] : Lat/Long extends to copy (*)
 *      --no-overwrite        : don't overwrite data that already exists
 *      --threads [int]       : number of threads to launch
 *
 * OSG arguments:
 *
 *      -O <string>           : OSG Options string (plugin options)
 *
 * Of course, the output layer must support writing.
 * Of course, the output layer must support writing.
 */
int
main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc,argv);

    if ( argc == 1 )
        return usage(argv);

    osgDB::readCommandLine(args);

    if (args.read("--pause"))
    {
        std::cout << "Press enter to continue" << std::endl;
        getchar();
    }

    // plugin options, if the user passed them in:
    osg::ref_ptr<osgDB::Options> dbo = new osgDB::Options();
    std::string str;
    while (args.read("--osg-options", str) || args.read("-O", str))
    {
        dbo->setOptionString(str);
    }

    typedef std::unordered_map<std::string,std::string> KeyValue;
    std::string key, value;

    // There are two paths. Either the user defines a source layer on
    // the command line using "--in" options, or the user specifies
    // an earth file and layer name.
    osg::ref_ptr<TileLayer> input;
    Config inConf;

    // earth file option:
    std::string earthFile;
    osg::ref_ptr<MapNode> mapNode;
    osg::ref_ptr<const Map> map;
    if (args.read("--in-earth", earthFile))
    {
        std::string layerName;
        if (!args.read("--in-layer", layerName))
        {
            OE_WARN << "Missing required argument --in-layer" << std::endl;
            return -1;
        }

        osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFile(earthFile, dbo.get());
        mapNode = MapNode::get(node.get());
        if (mapNode.valid())
        {
            mapNode->open();
            map = mapNode->getMap();
        }

        input = map->getLayerByName<TileLayer>(layerName);
        if (!input.valid())
        {
            OE_WARN << "Layer \"" << layerName << "\" not found in the earth file" << std::endl;
            return -1;
        }

        inConf = input->getConfig();
    }

    // command line input option:
    else
    {
        // collect input configuration:
        while (args.read("--in", key, value))
            inConf.set(key, value);

        inConf.key() = inConf.value("driver");

        input = dynamic_cast<TileLayer*>(Layer::create(ConfigOptions(inConf)));
        if (!input.valid())
        {
            OE_WARN << LC << "Failed to open input for " << inConf.toJSON(false) << std::endl;
            return -1;
        }
    }

    // Assign a custom tile size to the input source, if possible:
    unsigned tileSize = input->getTileSize();
    if (args.read("--tile-size", tileSize))
    {
        input->setTileSize(tileSize);
    }

    // Open the input layer:
    input->setReadOptions(dbo.get());
    Status inputStatus = input->open();
    if ( inputStatus.isError() )
    {
        OE_WARN << LC << "Error initializing input: " << inputStatus.message() << std::endl;
        return -1;
    }

    // collect output configuration:
    bool compress = false;
    Config outConf;
    while (args.read("--out", key, value))
    {
        outConf.set(key, value);

        // special case: turn on compression when using dds at the output format
        // and disable the built-in image flipping logic
        if (key == "format" && value == "dds")
        {
            compress = true;
            //dbo->setOptionString("ddsNoAutoFlipWrite " + dbo->getOptionString());
        }
    }
    outConf.key() = outConf.value("driver");

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
    osg::ref_ptr<TileLayer> output = dynamic_cast<TileLayer*>(Layer::create(ConfigOptions(outConf)));
    if ( !output.valid() )
    {
        OE_WARN << LC << "Failed to create output layer" << std::endl;
        return -1;
    }

    output->setReadOptions(dbo.get());
    Status outputStatus = output->openForWriting();
    if (outputStatus.isError())
    {
        OE_WARN << LC << "Error initializing output: " << outputStatus.message() << std::endl;
        return -1;
    }

    // Transfomr and copy over the data extents to the output datasource.
    DataExtentList outputExtents;
    for (DataExtentList::const_iterator itr = input->getDataExtents().begin(); itr != input->getDataExtents().end(); ++itr)
    {
        // Convert the data extent to the profile that is actually used by the output tile source
        const DataExtent& inputExtent = *itr;
        GeoExtent outputExtent = outputProfile->clampAndTransformExtent(inputExtent);
        unsigned int minLevel = 0;
        unsigned int maxLevel = outputProfile->getEquivalentLOD(input->getProfile(), inputExtent.maxLevel().get());
        DataExtent result(outputExtent, minLevel, maxLevel);
        outputExtents.push_back(result);
    }
    if (!outputExtents.empty())
    {
        output->setDataExtents(outputExtents);
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

    bool overwrite = true;
    if (args.read("--no-overwrite"))
        overwrite = false;

    if (dynamic_cast<ImageLayer*>(input.get()) && dynamic_cast<ImageLayer*>(output.get()))
    {
        visitor->setTileHandler(new ImageLayerTileCopy(
            dynamic_cast<ImageLayer*>(input.get()),
            dynamic_cast<ImageLayer*>(output.get()),
            overwrite,
            compress));
    }
    else if (dynamic_cast<ElevationLayer*>(input.get()) && dynamic_cast<ElevationLayer*>(output.get()))
    {
        visitor->setTileHandler(new ElevationLayerTileCopy(
            dynamic_cast<ElevationLayer*>(input.get()),
            dynamic_cast<ElevationLayer*>(output.get()),
            overwrite));
    }

    // set the manula extents, if specified:
    bool userSetExtents = false;
    double minlat, minlon, maxlat, maxlon;
    while( args.read("--extents", minlat, minlon, maxlat, maxlon) )
    {
        GeoExtent extent(SpatialReference::get("wgs84"), minlon, minlat, maxlon, maxlat);
        visitor->addExtent( extent );
        userSetExtents = true;
    }

    // Read in an index shapefile to drive where to tile
    std::string index;
    while (args.read("--index", index))
    {
        osg::ref_ptr< OGRFeatureSource > indexFeatures = new OGRFeatureSource;
        indexFeatures->setURL(index);
        if (indexFeatures->open().isError())
        {
            OE_WARN <<  "Failed to open index " << index << ": " << indexFeatures->getStatus().toString() << std::endl;
            return -1;
        }

        osg::ref_ptr< FeatureCursor > cursor = indexFeatures->createFeatureCursor(0);
        while (cursor.valid() && cursor->hasMore())
        {
            osg::ref_ptr< Feature > feature = cursor->nextFeature();
            osgEarth::Bounds featureBounds = feature->getGeometry()->getBounds();
            GeoExtent ext(feature->getSRS(), featureBounds);
            ext = ext.transform(mapNode->getMapSRS());
            visitor->addDataExtent(ext);
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

            if (userSetExtents == false)
            {
                visitor->addExtent(*i);
            }
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

    // Ready!!!
    std::cout << "Working..." << std::endl;

    visitor->setProgressCallback( new ProgressReporter() );

    osg::Timer_t t0 = osg::Timer::instance()->tick();

    visitor->run( outputProfile.get() );

    osg::Timer_t t1 = osg::Timer::instance()->tick();

    std::cout
        << std::endl
        << "Complete. Time = "
        << std::fixed
        << std::setprecision(1)
        << osg::Timer::instance()->delta_s(t0, t1)
        << " seconds." << std::endl;

    return 0;
}
