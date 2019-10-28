/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
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

// TODO:  Reconfigure CMake to not require this.....
#define OSGEARTH_HAVE_MVT 1
#define OSGEARTH_HAVE_SQLITE3 1

#include <osgViewer/Viewer>
#include <osgDB/WriteFile>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/Controls>
#include <osgEarth/ViewFitter>
#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/TDTiles>
#include <osgEarth/Random>
#include <osgEarth/TileKey>
#include <osgEarth/CullingUtils>
#include <osgEarth/FeatureNode>
#include <osgEarth/FeatureSource>
#include <osgEarth/FeatureCursor>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/ResampleFilter>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/MVT>
#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/Async>
#include <osgDB/FileUtils>
#include <osgDB/WriteFile>
#include <iostream>

#define LC "[3dtiles test] "

using namespace osgEarth;
using namespace osgEarth::Strings;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;
namespace ui = osgEarth::Util::Controls;

struct App
{
    MapNode* _mapNode;
    ui::HSliderControl* _sse;
    TDTiles::ContentHandler* _handler;
    EarthManipulator* _manip;
    osg::ref_ptr<TDTilesetGroup> _tileset;
    osgViewer::View* _view;
    LODScaleGroup* _sseGroup;
    float _maxSSE;
    bool _randomColors;

    void changeSSE()
    {
        _sseGroup->setLODScaleFactor(_sse->getValue());
    }

    void zoomToData()
    {
        if (_tileset->getBound().valid())
        {
            const osg::BoundingSphere& bs = _tileset->getBound();
            
            const SpatialReference* wgs84 = SpatialReference::get("wgs84");

            std::vector<GeoPoint> points;
            points.push_back(GeoPoint());
            points.back().fromWorld(wgs84, bs.center());

            Viewpoint vp;
            ViewFitter fit(wgs84, _view->getCamera());
            fit.setBuffer(bs.radius()*2.0);
            fit.createViewpoint(points, vp);

            _manip->setViewpoint(vp);
        }
    }

    void apply()
    {
        changeSSE();
    }
};

OE_UI_HANDLER(changeSSE);
OE_UI_HANDLER(zoomToData);

ui::Control* makeUI(App& app)
{
    ui::Grid* container = new ui::Grid();

    int r=0;
    container->setControl(0, r, new ui::LabelControl("Screen-space error (px)"));
    app._sse = container->setControl(1, r, new ui::HSliderControl(app._maxSSE, 1.0f, app._maxSSE, new changeSSE(app)));
    app._sse->setHorizFill(true, 300.0f);
    container->setControl(2, r, new ui::LabelControl(app._sse));

    //++r;
    //container->setControl(0, r, new ui::ButtonControl("Zoom to data", new zoomToData(app)));

    return container;
}

int
usage(const std::string& message)
{
    OE_WARN 
        << "\n\n" << message
        << "\n\nUsage: osgearth_3dtiles"
        << "\n"
        << "\n   --tile                       ; build a tileset"
        << "\n     --in <earthfile>           ; Earth file containing feature source and stylesheet"
        << "\n     --out <tileset>            ; output JSON tileset"
        << "\n     --format <format>          ; output geometry format (default = b3dm)"
        << "\n"
        << "\n   --view <earthfile>           ; view a 3dtiles dataset"
        << "\n     --tileset <filename>       ; 3dtiles tileset JSON file to load"
        << "\n     --maxsse <n>               ; maximum screen space error in pixels for UI"
        << "\n     --features                 ; treat the 3dtiles content as feature data"
        << "\n     --random-colors            ; randomly color feature tiles (instead of one color)"
        << std::endl;

    return -1;
}

/**
 * Custom TDTiles ContentHandler that reads feature data from a URL
 * and renders a simple osg::Node from it.
 */
struct FeatureRenderer : public TDTiles::ContentHandler
{
    App& _app;
    mutable Random _random;

    FeatureRenderer(App& app) : _app(app) { }

    osg::ref_ptr<osg::Node> createNode(TDTiles::Tile* tile, const osgDB::Options* readOptions) const
    {
        osg::ref_ptr<osg::Node> node;

        if (tile->content().isSet() && tile->content()->uri().isSet() && !tile->content()->uri()->empty())
        {
            if (osgEarth::Strings::endsWith(tile->content()->uri()->base(), ".shp"))
            {
                OE_INFO << "Rendering: " << tile->content()->uri()->full() << std::endl;
        
                osg::ref_ptr<OGRFeatureSource> fs = new OGRFeatureSource();
                fs->setURL(tile->content()->uri().get());
                if (fs->open().isOK())
                {
                    Query query;

                    // Gather features whose centroid falls within the query bounds
                    FeatureList features;
                    osg::ref_ptr<FeatureCursor> cursor = fs->createFeatureCursor(query, NULL);
                    if (cursor.valid())
                        cursor->fill(features);
                    fs->close();

                    if (!features.empty())
                    {
                        Style style;
                        osg::Vec4 color;
                        if (_app._randomColors)
                            color.set(_random.next(), _random.next(), _random.next(), 1.0f);
                        else
                            color.set(1.0, 0.6, 0.0, 1.0);

                        style.getOrCreate<PolygonSymbol>()->fill()->color() = color;
                        style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
                        //style.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
                        ExtrusionSymbol* ext = style.getOrCreate<ExtrusionSymbol>();
                        ext->heightExpression() = NumericExpression("[height]");

                        FeatureNode* fn = new FeatureNode(features, style);
                        fn->setMapNode(_app._mapNode);
                        node = fn;
                        osg::BoundingSphere bs = node->getBound();
                    }
                }
            }
            else
            {
                node = osgDB::readRefNodeFile(tile->content()->uri()->full());
            }
        }
        else
        {
            //nop - skip tile with no content
        }

        return node;
    }
};

int
main_view(osg::ArgumentParser& arguments)
{
    App app;

    std::string tilesetLocation;
    if (!arguments.read("--tileset", tilesetLocation))
        return usage("Missing required --tileset");

    bool readFeatures = arguments.read("--features");
    app._randomColors = arguments.read("--random-colors");

    app._maxSSE = 20.0f;
    arguments.read("--maxsse", app._maxSSE);

    // load the tile set:
    URI tilesetURI(tilesetLocation);
    ReadResult rr = tilesetURI.readString();
    if (rr.failed())
        return usage(Stringify()<<"Error loading tileset: " <<rr.errorDetail());

    TDTiles::Tileset* tileset = TDTiles::Tileset::create(rr.getString(), tilesetLocation);
    if (!tileset)
        return usage("Bad tileset");

    // create a viewer:
    osgViewer::Viewer viewer(arguments);
    app._view = &viewer;

    viewer.setCameraManipulator( app._manip = new EarthManipulator(arguments) );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::ref_ptr<osg::Node> node = MapNodeHelper().load(arguments, &viewer);
    if (!node.valid())
        return usage("Failed to load an earth file");

    MapNode* mapNode = MapNode::get(node.get());
    app._mapNode = mapNode;

    if (readFeatures)
        app._tileset = new TDTilesetGroup(new FeatureRenderer(app));
    else
        app._tileset = new TDTilesetGroup();


    app._sseGroup = new LODScaleGroup();
    app._sseGroup->addChild(app._tileset.get());

    ui::ControlCanvas::get(&viewer)->addControl(makeUI(app));

    app._tileset->setTileset(tileset);
    app._tileset->setReadOptions(mapNode->getMap()->getReadOptions());

    mapNode->addChild(app._sseGroup);

    mapNode->addChild(Registry::instance()->getAsyncMemoryManager());

    viewer.setSceneData( node.get() );

    app.apply();

    app.zoomToData();

    return viewer.run();
}

TileKey
parseKey(const std::string& input, const Profile* profile)
{
    std::vector<std::string> tileparts;
    StringTokenizer(input, tileparts, "/", "", false, true);
    if (tileparts.size() != 3)
        return TileKey::INVALID;

    return TileKey(
        atoi(tileparts[0].c_str()),
        atoi(tileparts[1].c_str()),
        atoi(tileparts[2].c_str()),
        profile);
}

/**
* Saves a tileset with the given tile as the root.
*/
void saveTileSet(TDTiles::Tile* tile, const std::string& filename)
{
    osgDB::makeDirectoryForFile(filename);

    osg::ref_ptr<TDTiles::Tileset> tileset = new TDTiles::Tileset();
    tileset->root() = tile;
    tileset->asset()->version() = "1.0";

    std::ofstream fout(filename);
    Util::Json::Value json = tileset->getJSON();
    Util::Json::StyledStreamWriter writer;    
    writer.write(fout, json);
    fout.close();
}

TDTiles::Tile*
createTile(osg::Node* node, const GeoExtent& extent, double error, const std::string& filenamePrefix, TDTiles::RefinePolicy refine)
{
    std::string filename = Stringify() << filenamePrefix << "_" << int(error) << ".b3dm";

    if (!osgDB::writeNodeFile(*node, filename))
    {
        OE_WARN << "Failed to write to output file (" << filename << ")" << std::endl;
        return NULL;
    }

    osg::ref_ptr<TDTiles::Tile> tile = new TDTiles::Tile();
    tile->content()->uri() = filename;

    // Set up a bounding region (radians)
    // TODO: calculate the correct Z min/max instead of hard-coding
    tile->boundingVolume()->region()->set(
        osg::DegreesToRadians(extent.xMin()),
        osg::DegreesToRadians(extent.yMin()),
        -1000.0,
        osg::DegreesToRadians(extent.xMax()),
        osg::DegreesToRadians(extent.yMax()),
        3000.0);

    tile->geometricError() = 0.0; //error;
    tile->refine() = refine;

    return tile.release();
}

int
main_tile(osg::ArgumentParser& args)
{
    // 1. Load an earth file
    // 2. Find the first FeatureSource Layer and StyleSheet Layer
    // 3. Create a 3D-Tiles Tileset object from it
    // 4. Write the tileset to disk.
    std::string infile;
    if (!args.read("--in", infile))
        return usage("Missing required --in <earthfile>");

    std::string outfile;
    if (!args.read("--out", outfile))
        return usage("Missing required --out tileset.json");

    osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFile(infile);
    MapNode* mapnode = MapNode::get(node.get());
    if (!mapnode)
        return usage("Input file is not a valid earth file");

    Map* map = mapnode->getMap();

    OGRFeatureSource* fs = map->getLayer<OGRFeatureSource>();
    if (!fs)
        return usage("No feature source layer found in the map");

    StyleSheet* sheet = map->getLayer<StyleSheet>();
    if (!sheet)
        return usage("No stylesheet found in the map");

    const Style* style = sheet->getDefaultStyle();
    if (!sheet)
        return usage("No default style found in the stylesheet");

    std::string format("b3dm");
    args.read("--format", format);

    if (!map->getProfile())
    {
        const Profile* profile = map->calculateProfile();
        map->setProfile(profile);
    }

    // For best optimization
    Registry::instance()->setMaxNumberOfVertsPerDrawable(UINT_MAX);

    TDTiles::TilesetFactory factory;
    factory.setMap(map);
    factory.setGeometryFormat(format);
    factory.setStyleSheet(sheet);
    factory.setURIContext(URIContext(outfile));

    Query query;
    osg::ref_ptr<TDTiles::Tileset> tileset = factory.create(fs, query, NULL);
    if (!tileset.valid())
        return usage("Failed to create a tileset from the feature source");

    std::ofstream fout(outfile.c_str());
    Util::Json::Value json = tileset->getJSON();
    Util::Json::StyledStreamWriter writer;
    writer.write(fout, json);
    fout.close();
    OE_INFO << "Wrote tileset to " << outfile << std::endl;
    
    return 0;
}

typedef std::map< osgEarth::TileKey, osg::ref_ptr< TDTiles::Tile > > TileMap;

typedef std::map< unsigned int, std::set< TileKey > > LevelToTileKeyMap;

struct MVTContext
{
    MVTContext():
        numComplete(0),
        format("b3dm")
    {
        queue = new osg::OperationQueue;
    }

    osg::ref_ptr< Map > map;
    osg::ref_ptr< StyleSheet> styleSheet;
    const Style* style;
    URIContext uriContext;
    TileMap leafNodes;
    std::string format;
    OpenThreads::Atomic numComplete;
    osg::Timer_t startTime;
    osg::ref_ptr< osg::OperationQueue > queue;
};

class BuildTileOperator : public osg::Operation
{
public:
    BuildTileOperator(const TileKey& key, const FeatureList& features, MVTContext& context) :
        _key(key),
        _features(features),
        _context(context)
    {
    }

    void operator()(osg::Object* object)
    {
        osg::Timer_t startTime = osg::Timer::instance()->tick();

        std::string filename = Stringify() << _key.getLevelOfDetail() << "/" << _key.getTileX() << "/" << _key.getTileY() << "." << _context.format;

        osg::ref_ptr< Session > session = new Session(_context.map.get());
        session->setStyles(_context.styleSheet);

        URI uri(filename, _context.uriContext);

        GeoExtent dataExtent = _key.getExtent();
        FilterContext fc(session, new FeatureProfile(dataExtent), dataExtent);
        GeometryCompiler gc;        
        
        const Style* style = _context.style;
        osg::ref_ptr<osg::Node> node = gc.compile(_features, *style, fc);

        if (node.valid())
        {
            osgDB::makeDirectoryForFile(uri.full());
            osgDB::writeNodeFile(*node.get(), uri.full());
        }
        else
        {
            OE_NOTICE << "Failed to create node for " << filename << std::endl;
        }     

        ++_context.numComplete;

        double totalTime = osg::Timer::instance()->delta_s(_context.startTime, osg::Timer::instance()->tick());
        double tilesPerSecond = (double)_context.numComplete / totalTime;

        osg::Timer_t endTime = osg::Timer::instance()->tick();
        //OE_NOTICE << "Processed " << _key.str() << " with " << _features.size() << " features in " << osg::Timer::instance()->delta_s(startTime, endTime) << "s" << std::endl;
        if (_context.numComplete % 100 == 0)
        {
            OE_NOTICE << "Completed " << _context.numComplete << " tiles.  " << tilesPerSecond << " tiles/s" << std::endl;
        }
    }

    TileKey _key;
    FeatureList _features;
    MVTContext& _context;
};


/**
 * Specialized degress to radians function that makes sure that the radian values stay within a valid range for Cesium.
 */
float Deg2Rad(float deg)
{                            
    //const float PI = 3.141592653589793f;
    const float PI = 3.1415926f;
    float val = osg::clampBetween(deg * PI / 180.0f, -PI, PI);
    return val;
}

/**
 * Sets the extents of a Tile to given extent 
 */
void setTileExtents(TDTiles::Tile* tile, const TileKey& key, double minHeight, double maxHeight)
{
    // Make sure the key's extent is in wgs84
    GeoExtent extent;
    extent = key.getExtent().transform(SpatialReference::create("epsg:4326"));

    // Use bounding spheres until we get to lod 4 to avoid culling issues with bounding regions in Cesium with large bounding regions
    if (key.getLevelOfDetail() < 4)
    {
        osg::BoundingSphered bs = extent.createWorldBoundingSphere(minHeight, maxHeight);
        tile->boundingVolume()->sphere()->set(bs.center(), bs.radius());
    }
    else
    // Use bounding regions instead of spheres to provide tighter bounds to increase culling performance.
    {
        tile->boundingVolume()->region()->set(
            Deg2Rad(extent.xMin()), Deg2Rad(extent.yMin()), minHeight,
            Deg2Rad(extent.xMax()), Deg2Rad(extent.yMax()), maxHeight);
    }
}

// Compute the geometric error of a mercator tile.
float computeGeometricError(const TileKey& key)
{
    return key.getExtent().width() / 500.0;
}


void addChildren(TDTiles::Tile* tile, const TileKey& key, LevelToTileKeyMap& levelKeys, int maxLevel)
{
    SpatialReference* mapSRS = SpatialReference::create("epsg:4326");
    double height = 2000.0;

    // Check to see if the children exist
    for (unsigned int c = 0; c < 4; c++)
    {
        TileKey childKey = key.createChildKey(c);
        std::set< TileKey >::iterator childItr = levelKeys[childKey.getLevelOfDetail()].find(childKey);
        if (childItr != levelKeys[childKey.getLevelOfDetail()].end())
        {
            osg::ref_ptr< TDTiles::Tile > childTile = new TDTiles::Tile;
            childTile->geometricError() = computeGeometricError(key);
            GeoExtent childExtent = childKey.getExtent().transform(mapSRS);
            setTileExtents(childTile, childKey, 0.0, height);            
            tile->children().push_back(childTile);
            if (childKey.getLevelOfDetail() < maxLevel)
            {
                addChildren(childTile, childKey, levelKeys, maxLevel);
            }
            else
            {
                childTile->geometricError() = 0.0;
                // TODO: get the "refine" right
                tile->refine() = TDTiles::REFINE_REPLACE;                
                std::string uri = Stringify() << "../../" << childKey.getLevelOfDetail() << "/" << childKey.getTileX() << "/" << childKey.getTileY() << ".b3dm";
                childTile->content()->uri() = uri;
            }
        }
    }
}

void addChildrenForce(TDTiles::Tile* tile, const TileKey& key, int maxLevel)
{
    SpatialReference* mapSRS = SpatialReference::create("epsg:4326");
    double height = 800.0;

    // Check to see if the children exist
    for (unsigned int c = 0; c < 4; c++)
    {
        TileKey childKey = key.createChildKey(c);
        osg::ref_ptr< TDTiles::Tile > childTile = new TDTiles::Tile;
        childTile->geometricError() = computeGeometricError(key);
        GeoExtent childExtent = childKey.getExtent().transform(mapSRS);
        setTileExtents(childTile, childKey, 0.0, height);
        tile->children().push_back(childTile);
        if (childKey.getLevelOfDetail() < maxLevel)
        {
            addChildrenForce(childTile, childKey, maxLevel);
        }
        else
        {
            childTile->geometricError() = 0.0;
            // TODO: get the "refine" right
            tile->refine() = TDTiles::REFINE_REPLACE;
            std::string uri = Stringify() << "../../" << childKey.getLevelOfDetail() << "/" << childKey.getTileX() << "/" << childKey.getTileY() << ".b3dm";
            childTile->content()->uri() = uri;
        }
    }
}

int build_tilesets(osg::ArgumentParser& args)
{
    osg::Timer_t startTime = osg::Timer::instance()->tick();

    std::string path;
    if (!args.read("--path", path))
        return usage("Missing required --path <tiledirectory>");

    std::string outfile;
    if (!args.read("--out", outfile))
        return usage("Missing required --out tileset.json");

    unsigned int numThreads = 8;
    args.read("--numThreads", numThreads);

    SpatialReference* mapSRS = SpatialReference::create("epsg:4326");
    const Profile* profile = osgEarth::Registry::instance()->getGlobalMercatorProfile();
    LevelToTileKeyMap levelKeys;

    std::string dataFiles;
    args.read("--dataFiles", dataFiles);

    std::ifstream fin(dataFiles);
    std::string line;
    unsigned int numRead = 0;

    unsigned int zoomLevel = 0;
    unsigned int maxWriteLevel = 10;
    unsigned int maxLevel = 14;

    OE_NOTICE << "Collecting leaf nodes" << std::endl;
    while (!fin.eof())
    {
        std::getline(fin, line);
        line = osgDB::convertFileNameToUnixStyle(line);

        std::string keyName = osgDB::getPathRelative(path, line);
        TileKey key = parseKey(keyName, profile);
        if (key.valid())
        {            
            // Assume they are all the same zoom level
            maxLevel = zoomLevel;
            zoomLevel = key.getLevelOfDetail();
            levelKeys[zoomLevel].insert(key);

            double totalTime = osg::Timer::instance()->delta_s(startTime, osg::Timer::instance()->tick());
            double tilesPerSecond = (double)numRead / totalTime;

            if (numRead % 1000 == 0)
            {
                OE_NOTICE << "Read " << numRead << " tiles " << tilesPerSecond << "tiles/s" << std::endl;
            }
        }

        if (line.empty())
            break;
        numRead++;
    }

    unsigned int minLevel = 0;

    // Build the parent hierarchy
    OE_NOTICE << "Building parent hierarchy" << std::endl;
    while (zoomLevel > minLevel)
    {
        unsigned int childZoom = zoomLevel;
        zoomLevel--;
        OE_NOTICE << "Building parents for zoom level " << zoomLevel << std::endl;

        for (std::set<TileKey>::iterator itr = levelKeys[childZoom].begin(); itr != levelKeys[childZoom].end(); ++itr)
        {
            const TileKey parentKey = itr->createParentKey();
            levelKeys[zoomLevel].insert(parentKey);
        }
    }

    OE_NOTICE << "Writing tilesets" << std::endl;

    float height = 800.0;

    osg::ref_ptr< TDTiles::Tile > rootTile = new TDTiles::Tile;

    unsigned int numSaved = 0;
    // Build a full tileset for each tile at the min level
    for (std::set<TileKey>::iterator itr = levelKeys[maxWriteLevel].begin(); itr != levelKeys[maxWriteLevel].end(); ++itr)
    {
        const TileKey key = *itr;

        OE_NOTICE << "Generating tileset for " << key.str() << std::endl;

        osg::ref_ptr< TDTiles::Tile> tile = new TDTiles::Tile;
        tile->geometricError() = computeGeometricError(key);
        GeoExtent dataExtent = key.getExtent().transform(mapSRS);
        setTileExtents(tile, key, 0.0, height);
        addChildren(tile, key, levelKeys, maxLevel);
        std::string tilesetName = Stringify() << path << "/" << key.getLevelOfDetail() << "/" << key.getTileX() << "/" << key.getTileY() << ".json";
        saveTileSet(tile.get(), tilesetName);               
        OE_NOTICE << "Saved tileset to " << tilesetName << std::endl;
        numSaved++;
        OE_NOTICE << "Completed " << numSaved << " of " << levelKeys[maxWriteLevel].size() << std::endl;        
    }

    for (unsigned int z = minLevel; z < maxWriteLevel; z++)
    {
        unsigned int childZoom = z + 1;

        OE_NOTICE << "Writing tilesets for level " << z << " with " << levelKeys[z].size() << " keys" << std::endl;

        // For each key in this zoom level, write out a tileset.
        for (std::set<TileKey>::iterator itr = levelKeys[z].begin(); itr != levelKeys[z].end(); ++itr)
        {
            const TileKey key = *itr;            

            osg::ref_ptr< TDTiles::Tile> tile = new TDTiles::Tile;
            tile->geometricError() = computeGeometricError(key);
            GeoExtent dataExtent = key.getExtent().transform(mapSRS);
            setTileExtents(tile, key, 0.0, height);

            // Check to see if the children exist
            for (unsigned int c = 0; c < 4; c++)
            {
                TileKey childKey = key.createChildKey(c);
                std::set< TileKey >::iterator childItr = levelKeys[childZoom].find(childKey);
                if (childItr != levelKeys[childZoom].end())
                {
                    // don't actually add the child tile, add a reference to a tile that points to the child
                    // Create the parent tile
                    osg::ref_ptr< TDTiles::Tile > childTile = new TDTiles::Tile;
                    childTile->geometricError() = computeGeometricError(childKey);

                    GeoExtent childExtent = childKey.getExtent().transform(mapSRS);
                    setTileExtents(childTile, childKey, 0.0, height);

                    std::string tilesetName;
                    if (z == minLevel)
                    {
                        tilesetName = Stringify() << childKey.getLevelOfDetail() << "/" << childKey.getTileX() << "/" << childKey.getTileY() << ".json";
                    }
                    else
                    {
                        tilesetName = Stringify() << "../../" << childKey.getLevelOfDetail() << "/" << childKey.getTileX() << "/" << childKey.getTileY() << ".json";
                    }
                    childTile->content()->uri() = tilesetName;
                    tile->children().push_back(childTile);
                }
            }

            if (z == minLevel)
            {
                rootTile->children().push_back(tile);
            }
            else
            {
                std::string tilesetName = Stringify() << path << "/" << key.getLevelOfDetail() << "/" << key.getTileX() << "/" << key.getTileY() << ".json";
                saveTileSet(tile.get(), tilesetName);
            }
        }
    }

    rootTile->geometricError() = *rootTile->children()[0]->geometricError() * 2.0;

    // Offset the root b/c Cesium doesn't like having a 0,0,0 center for a Cartesian3.
    rootTile->boundingVolume()->sphere()->set(osg::Vec3(1.0, 1.0, 1.0), 6400000.0f);
    saveTileSet(rootTile.get(), outfile);

    osg::Timer_t endTime = osg::Timer::instance()->tick();
    double time = osg::Timer::instance()->delta_s(startTime, endTime);
    OE_NOTICE << "Completed generating tilesets in " << osgEarth::prettyPrintTime(time) << std::endl;

    return 0;
}

/**
 * Builds a list of test tilesets up to a given max level to test out paging.
 */
int build_test_tilesets(osg::ArgumentParser& args)
{
    osg::Timer_t startTime = osg::Timer::instance()->tick();

    std::string outfile;
    if (!args.read("--out", outfile))
        return usage("Missing required --out tileset.json");

    unsigned int maxLevel = 8;
    args.read("--maxLevel", maxLevel);

    SpatialReference* mapSRS = SpatialReference::create("epsg:4326");
    const Profile* profile = osgEarth::Registry::instance()->getGlobalMercatorProfile();

    osg::ref_ptr< TDTiles::Tile > rootTile = new TDTiles::Tile;
    std::vector< TileKey > rootKeys;
    profile->getRootKeys(rootKeys);

    TileKey rootKey = rootKeys[0];
    rootTile->geometricError() = computeGeometricError(rootKey);
    GeoExtent dataExtent = rootKey.getExtent().transform(mapSRS);
    rootTile->boundingVolume()->sphere()->set(osg::Vec3(1.0, 1.0, 1.0), 6400000.0f);
    addChildrenForce(rootTile, rootKey, 8);
    saveTileSet(rootTile.get(), outfile);

    return 0;
}


// Callback that adds the feature list to a worker queue.
void addTileToQueue(const TileKey& key, const FeatureList& features, void* context)
{
    MVTContext& mvtContext = *(MVTContext*)(context);
    while (mvtContext.queue->getNumOperationsInQueue() > 100)
    {
        OpenThreads::Thread::YieldCurrentThread();
    }
    mvtContext.queue->add(new BuildTileOperator(key, features, mvtContext));
}

/**
 * Builds b3dm files for all the leaf nodes in a mercator mbtiles database.
*/
int
build_leaves(osg::ArgumentParser& args)
{
    osg::Timer_t startTime = osg::Timer::instance()->tick();

    std::string infile;
    if (!args.read("--in", infile))
        return usage("Missing required --in <earthfile>");

    std::string outfile;
    if (!args.read("--out", outfile))
        return usage("Missing required --out tileset.json");

    osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFile(infile);
    MapNode* mapnode = MapNode::get(node.get());
    if (!mapnode)
        return usage("Input file is not a valid earth file");

    Map* map = mapnode->getMap();

    osgEarth::Features::MVTFeatureSource* fs = map->getLayer<MVTFeatureSource>();
    if (!fs)
        return usage("No feature source layer found in the map");

    StyleSheet* sheet = map->getLayer<StyleSheet>();
    if (!sheet)
        return usage("No stylesheet found in the map");

    std::string format("b3dm");
    args.read("--format", format);

    unsigned int zoomLevel = 14;
    args.read("--zoom", zoomLevel);

    unsigned int numThreads = 4;
    args.read("--numThreads", numThreads);

    const Style* style = 0;

    std::string styleName;
    if (args.read("--style", styleName))
    {
        style = sheet->getStyle(styleName);
    }
    else
    {
        style = sheet->getDefaultStyle();
    }

    if (!style)
    {
        return usage("No style specified");
    }
    
    GeoExtent queryExtent;
    double xmin = DBL_MAX, ymin = DBL_MAX, xmax = DBL_MIN, ymax = DBL_MIN;
    while (args.read("--bounds", xmin, ymin, xmax, ymax))
    {
        queryExtent = GeoExtent(osgEarth::SpatialReference::create("epsg:4326"), xmin, ymin, xmax, ymax);
    }

    if (!map->getProfile())
    {
        const Profile* profile = map->calculateProfile();
        map->setProfile(profile);
    }

    // For best optimization
    Registry::instance()->setMaxNumberOfVertsPerDrawable(UINT_MAX);

    URIContext uriContext(outfile);

    MVTContext mvtContext;
    mvtContext.map = map;
    mvtContext.styleSheet = sheet;
    mvtContext.style = style;
    mvtContext.uriContext = uriContext;
    mvtContext.format = format;
    mvtContext.startTime = osg::Timer::instance()->tick();

    std::vector< osg::ref_ptr< osg::OperationsThread > > threads;
    for (unsigned int i = 0; i < numThreads; i++)
    {
        osg::OperationsThread* thread = new osg::OperationsThread();
        thread->setOperationQueue(mvtContext.queue.get());
        thread->start();
        threads.push_back(thread);
    }

    fs->iterateTiles(zoomLevel, 0, 0, queryExtent, addTileToQueue, &mvtContext);
    
    // Wait for all operations to be done.
    while (!mvtContext.queue.get()->empty())
    {
        OpenThreads::Thread::YieldCurrentThread();
    }

    for (unsigned int i = 0; i < numThreads; i++)
    {
        threads[i]->setDone(true);
        threads[i]->join();
    }

    osg::Timer_t endTime = osg::Timer::instance()->tick();
    double time = osg::Timer::instance()->delta_s(startTime, endTime);
    OE_NOTICE << "Completed tiling in " << osgEarth::prettyPrintTime(time) << std::endl;
    return 0;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);

    // Make sure the b3dm plugin is loaded
    std::string libname = osgDB::Registry::instance()->createLibraryNameForExtension("gltf");
    osgDB::Registry::instance()->loadLibrary(libname);

    if (arguments.read("--help"))
        return usage("Help!");

    else if (arguments.read("--tile"))
        return main_tile(arguments);

    else if (arguments.read("--build_leaves"))
        return build_leaves(arguments);

    else if (arguments.read("--build_tilesets"))    
        return build_tilesets(arguments);                

    else if (arguments.read("--build_test_tilesets"))
        return build_test_tilesets(arguments);

    else if (arguments.read("--view"))
        return main_view(arguments);

    else
        return usage("Missing required --build or --view");
}
