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
#include <osgEarth/Registry>
#include <osgEarth/Async>
#include <osgDB/FileUtils>
#include <iostream>

#define LC "[3dtiles test] "

using namespace osgEarth;
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

    ++r;
    container->setControl(0, r, new ui::ButtonControl("Zoom to data", new zoomToData(app)));

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

    // For bets optimization
    Registry::instance()->setMaxNumberOfVertsPerDrawable(UINT_MAX);

    TDTiles::TilesetFactory factory;
    factory.setMap(map);
    factory.setGeometryFormat(format);
    factory.setStyleSheet(sheet);

    Query query;
    osg::ref_ptr<TDTiles::Tileset> tileset = factory.create(fs, query, NULL);
    if (!tileset.valid())
        return usage("Failed to create a tileset from the feature source");

    std::ofstream fout(outfile);
    Support::Json::Value json = tileset->getJSON();
    Support::Json::StyledStreamWriter writer;
    writer.write(fout, json);
    fout.close();
    OE_INFO << "Wrote tileset to " << outfile << std::endl;
    
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

    else if (arguments.read("--view"))
        return main_view(arguments);

    else
        return usage("Missing required --build or --view");
}
