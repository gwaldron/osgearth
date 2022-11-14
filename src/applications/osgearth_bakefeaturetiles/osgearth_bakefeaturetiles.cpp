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
#define LC "[osgearth_bakefeaturetiles] "

#include <osgEarth/Notify>
#include <osgEarth/Profile>
#include <osgEarth/Registry>
#include <osgEarth/TileHandler>
#include <osgEarth/TileVisitor>
#include <osgEarth/MapNode>
#include <osgEarth/TileEstimator>
#include <osgEarth/SimplePager>
#include <osgEarth/NodeUtils>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osg/TextureBuffer>
#include <osgEarth/MVT>

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
        << "Converts feature tiles to prebaked scene graph tiles.\n\n"
        << argv[0]        
        << "\n    --in-earth [earthfile]              : earth file from which to load input layer (instead of using --in)"
        << "\n    --in-layer [layer name]             : with --in-earth, name of layer to convert"
        << "\n    --path [output path]                : output path (default out)"
        << "\n    --ext  [output extension]           : output extension (default osgb)"
        << "\n    --tile-key-file [tile key file]     : process tiles from a file in z x y format"
        << "\n    --invert-tilekeys                   : invert the y values in tile keys specified in the tile key file"
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

struct PrepareForWriting : public osg::NodeVisitor
{
    PrepareForWriting() : osg::NodeVisitor()
    {
        setTraversalMode(TRAVERSE_ALL_CHILDREN);
        setNodeMaskOverride(~0);
    }

    void apply(osg::Node& node)
    {
        apply(node.getStateSet());
        applyUserData(node);
        traverse(node);
    }

    void apply(osg::Drawable& drawable)
    {
        apply(drawable.getStateSet());
        applyUserData(drawable);

        osg::Geometry* geom = drawable.asGeometry();
        if (geom)
            apply(geom);
    }

    void apply(osg::Geometry* geom)
    {
        // This detects any NULL vertex attribute arrays and then populates them.
        // Do this because a NULL VAA will crash the OSG serialization reader (osg 3.4.0)
        osg::Geometry::ArrayList& arrays = geom->getVertexAttribArrayList();
        for (osg::Geometry::ArrayList::iterator i = arrays.begin(); i != arrays.end(); ++i)
        {
            if (i->get() == 0L)
            {
                *i = new osg::FloatArray();
                i->get()->setBinding(osg::Array::BIND_OFF);
            }
        }

        // Get rid of any kdtree since osg can't serialize it.
        geom->setShape(nullptr);
    }

    void apply(osg::StateSet* ss)
    {
        if (!ss) return;

        osg::StateSet::AttributeList& a0 = ss->getAttributeList();
        for (osg::StateSet::AttributeList::iterator i = a0.begin(); i != a0.end(); ++i)
        {
            osg::StateAttribute* sa = i->second.first.get();
            applyUserData(*sa);
        }

        // Disable the texture image-unref feature so we can share the resource 
        // across cached tiles.
        osg::StateSet::TextureAttributeList& a = ss->getTextureAttributeList();
        for (osg::StateSet::TextureAttributeList::iterator i = a.begin(); i != a.end(); ++i)
        {
            osg::StateSet::AttributeList& b = *i;
            for (osg::StateSet::AttributeList::iterator j = b.begin(); j != b.end(); ++j)
            {
                osg::StateAttribute* sa = j->second.first.get();
                if (sa)
                {
                    osg::Texture* tex = dynamic_cast<osg::Texture*>(sa);
                    if (tex)
                    {
                        tex->setUnRefImageDataAfterApply(false);
                    }
                    else
                    {
                        applyUserData(*sa);
                    }
                }
            }
        }

        applyUserData(*ss);
    }

    void applyUserData(osg::Object& object)
    {
        object.setUserDataContainer(0L);
    }
};

struct WriteExternalImages : public osgEarth::TextureAndImageVisitor
{
    std::string _destinationPath;

    WriteExternalImages(const std::string& destinationPath)
        : TextureAndImageVisitor(),
        _destinationPath(destinationPath)
    {
        setTraversalMode(TRAVERSE_ALL_CHILDREN);
        setNodeMaskOverride(~0L);
    }

    void apply(osg::Texture & tex)
    {
        if (dynamic_cast<osg::TextureBuffer*>(&tex) != 0L)
        {
            // skip texture buffers, they need no prep and 
            // will be inlined as long as they have a write hint
            // set to STORE_INLINE.
        }
        else
        {
            osgEarth::TextureAndImageVisitor::apply(tex);
        }
    }

    void apply(osg::Image & image)
    {
        std::string path = image.getFileName();
        if (path.empty())
        {
            OE_WARN << "ERROR image with blank filename.\n";
            return;
        }

        if (image.getWriteHint() != osg::Image::EXTERNAL_FILE)
        {
            std::string format = "dds";
            unsigned int hash = osgEarth::hashString(path);

            std::string relativeName = Stringify() << "../../images/" << hash << "." << format;
            //OE_NOTICE << "Hashed " << path << " to " << relativeName << std::endl;
            std::string filename = osgDB::concatPaths(_destinationPath, relativeName);

            image.setFileName(relativeName);
            image.setWriteHint(osg::Image::EXTERNAL_FILE);
            osg::ref_ptr < osgDB::Options > options = new osgDB::Options;
            options->setOptionString("ddsNoAutoFlipWrite");
            osgDB::makeDirectoryForFile(filename);
            if (!osgDB::fileExists(filename))
            {
                osgDB::writeImageFile(image, filename, options.get());
            }
        }
    }
};

struct CreateTileHandler : public TileHandler
{
    CreateTileHandler(SimplePager* simplePager, bool overwrite, std::string& path, std::string& ext)
        :_simplePager(simplePager),
         _overwrite(overwrite),
        _path(path),
        _ext(ext)
    {
        if (::getenv(OSGEARTH_ENV_DEFAULT_COMPRESSOR) != 0L)
        {
            _compressorName = ::getenv(OSGEARTH_ENV_DEFAULT_COMPRESSOR);
        }
        else
        {
            _compressorName = "zlib";
        }        
    }

    bool handleTile(const TileKey& key, const TileVisitor& tv) override
    {        
        std::string destinationPath = _path;
        std::string name = Stringify() << key.str() << "." << _ext;
        std::string filename = osgDB::concatPaths(destinationPath, name);

        osg::ref_ptr< osg::Node > node;

        if (!osgDB::fileExists(filename) || _overwrite)
        {
             node = _simplePager->createNode(key, nullptr);
            if (node.valid())
            {                
                PrepareForWriting prepare;
                node->accept(prepare);

                std::string path = osgDB::getFilePath(filename);

                // Maybe make this optional
                WriteExternalImages write(path);
                node->accept(write);

                osg::ref_ptr< osgDB::Options > options = new osgDB::Options;
                options->setPluginStringData("Compressor", _compressorName);

                osgDB::makeDirectoryForFile(filename);
                osgDB::writeNodeFile(*node.get(), filename, options);
            }
        }
        return node.valid();
    }

    bool hasData(const TileKey& key) const override
    {
        return true;
    }

    unsigned getEstimatedTileCount(
        const std::vector<GeoExtent>& extents,
        unsigned minLevel,
        unsigned maxLevel) const override
    {
        TileEstimator e;
        e.setMinLevel(minLevel);
        e.setMaxLevel(maxLevel);
        e.setProfile(_simplePager->getProfile());
        for (auto& src_extent : extents)
        {
            auto dest_extent = _simplePager->getProfile()->clampAndTransformExtent(src_extent);
            e.addExtent(dest_extent);
        }
        return e.getNumTiles();
    }

    osg::ref_ptr<SimplePager> _simplePager;
    bool _overwrite;
    std::string _compressorName;
    std::string _path;
    std::string _ext;
};


// Custom progress reporter
struct ProgressReporter : public osgEarth::ProgressCallback
{
    ProgressReporter() : _first(true), _start(0) { }

    bool reportProgress(double             current,
        double             total,
        unsigned           currentStage,
        unsigned           totalStages,
        const std::string& msg)
    {
        ScopedMutexLock lock(_mutex);

        if (_first)
        {
            _first = false;
            _start = osg::Timer::instance()->tick();
        }
        osg::Timer_t now = osg::Timer::instance()->tick();

        if (total > 0.0f)
        {
            float percentage = current / total;

            double timeSoFar = osg::Timer::instance()->delta_s(_start, now);
            double projectedTotalTime = timeSoFar / percentage;
            double timeToGo = projectedTotalTime - timeSoFar;
            double minsToGo = timeToGo / 60.0;
            double secsToGo = fmod(timeToGo, 60.0);
            double minsTotal = projectedTotalTime / 60.0;
            double secsTotal = fmod(projectedTotalTime, 60.0);

            std::cout
                << std::fixed
                << std::setprecision(1) << "\r"
                << (int)current << "/" << (int)total
                << " " << int(100.0f * percentage) << "% complete, "
                << (int)minsTotal << "m" << (int)secsTotal << "s projected, "
                << (int)minsToGo << "m" << (int)secsToGo << "s remaining          "
                << std::flush;

            if (percentage >= 100.0f)
                std::cout << std::endl;
        }
        else
        {

            double timeSoFar = osg::Timer::instance()->delta_s(_start, now);

            std::cout
                << std::fixed
                << std::setprecision(1) << "\r"
                << (int)current << "/" << (int)total
                << " " << timeSoFar << "s elapsed"
                << std::flush;
        }

        return false;
    }

    Threading::Mutex _mutex;
    bool _first;
    osg::Timer_t _start;
};

class TileListVisitor : public osgEarth::MultithreadedTileVisitor
{
public:
    TileListVisitor(const std::vector< TileKey > &keys, TileHandler* handler = nullptr) :
        osgEarth::MultithreadedTileVisitor(handler),
        _keys(keys)
    {
    }

    virtual void run(const Profile* mapProfile)
    {
        // Start up the task service
        OE_INFO << "Starting " << _numThreads << " threads " << std::endl;

        _arena = std::make_shared<JobArena>("oe.mttilevisitor", _numThreads);

        _profile = mapProfile;

        // Reset the progress in case this visitor has been ran before.
        resetProgress();

        _total = _keys.size();

        for (auto &key : _keys)
        {
            this->handleTile(key);
        }

        _group.join();
    }    

    std::vector< TileKey > _keys;
};

std::vector< TileKey > readTilesFromText(const std::string& filename, const osgEarth::Profile* profile, bool invert)
{
    std::vector< TileKey > keys;

    std::ifstream in(filename);
    std::string line;
    while (std::getline(in, line))
    {
        std::stringstream buf(line);
        unsigned int zoom, x, y;
        buf >> zoom >> x >> y;
        if (invert)
        {
            unsigned int numRows, numCols;
            profile->getNumTiles(zoom, numCols, numRows);
            y = numRows - y - 1;
        }
        keys.emplace_back(std::move(TileKey(zoom, x, y, profile)));
    }
    return std::move(keys);
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc, argv);

    if (argc == 1)
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

    osg::ref_ptr< SimplePager > simplePager;

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

        osg::ref_ptr<osg::Node> loaded = osgDB::readRefNodeFile(earthFile, dbo.get());
        mapNode = MapNode::get(loaded.get());
        if (mapNode.valid())
        {
            mapNode->open();
            map = mapNode->getMap();
        }

        osg::ref_ptr< Layer > layer = map->getLayerByName<Layer>(layerName);
        if (!layer.valid())
        {
            OE_WARN << "Layer \"" << layerName << "\" not found in the earth file" << std::endl;
            return -1;
        }

        layer->open();
        osg::Node* layerNode = layer->getNode();
        if (layerNode)
        {
            simplePager = osgEarth::findTopMostNodeOfType<SimplePager>(layerNode);
        }
    }

    if (simplePager.valid())
    {
        OE_NOTICE << "Found simple pager" << std::endl;
    }

    // create the visitor.
    osg::ref_ptr<MultithreadedTileVisitor> visitor;

    unsigned numThreads = 1;
    args.read("--threads", numThreads);

    std::string tileKeyFile;
    args.read("--tile-key-file", tileKeyFile);

    bool invertTileKeys = false;
    if (args.read("--invert-tilekeys"))
    {
        invertTileKeys = true;
    }

    if (!tileKeyFile.empty())
    {
        std::vector< TileKey > keys;
        keys = readTilesFromText(tileKeyFile, simplePager->getProfile(), invertTileKeys);
        visitor = new TileListVisitor(keys);
    }
    else
    {
        visitor = new MultithreadedTileVisitor();
    }
    visitor->setNumThreads(numThreads < 1 ? 1 : numThreads);

    bool overwrite = true;
    if (args.read("--no-overwrite"))
        overwrite = false;

    std::string path = "out";
    args.read("--path", path);

    std::string ext = "osgb";
    args.read("--ext", ext);

    visitor->setTileHandler(new CreateTileHandler(simplePager, overwrite, path, ext));

    // set the manual extents, if specified:
    double minlat, minlon, maxlat, maxlon;
    GeoExtent userExtent;
    while (args.read("--extents", minlat, minlon, maxlat, maxlon))
    {
        userExtent = GeoExtent(SpatialReference::get("wgs84"), minlon, minlat, maxlon, maxlat);
        visitor->addExtentToVisit(userExtent);
    }

    // Set the level limits:
    unsigned minLevel = ~0;
    bool minLevelSet = args.read("--min-level", minLevel);

    unsigned maxLevel = 0;
    bool maxLevelSet = args.read("--max-level", maxLevel);

    if (minLevel < ~0)
    {
        visitor->setMinLevel(minLevel);
    }

    if (maxLevel > 0)
    {        
        visitor->setMaxLevel(maxLevel);     
    }

    // If we've not added any extents to visit just add the entire input extent
    if (visitor->getExtentsToVisit().empty())
    {
        visitor->addExtentToVisit(simplePager->getProfile()->getExtent());
    }

    // Ready!!!
    std::cout << "Working..." << std::endl;

    visitor->setProgressCallback(new ProgressReporter());

    osg::Timer_t t0 = osg::Timer::instance()->tick();

    visitor->run(simplePager->getProfile());

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
