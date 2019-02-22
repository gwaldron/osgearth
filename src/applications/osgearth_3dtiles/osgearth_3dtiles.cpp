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
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ViewFitter>
#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/TDTiles>
#include <osgEarth/Random>
#include <osgEarth/TileKey>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/FeatureCursor>
#include <osgEarthFeatures/FeatureModelLayer>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <iostream>

#define LC "[3dtiles test] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;
namespace ui = osgEarth::Util::Controls;

struct App
{
    ui::HSliderControl* _sse;
    TDTiles::ContentHandler* _handler;
    EarthManipulator* _manip;
    osg::ref_ptr<TDTilesetGroup> _tileset;
    osgViewer::View* _view;
    float _maxSSE;
    bool _randomColors;

    void changeSSE()
    {
        _handler->setMaxScreenSpaceError(_sse->getValue());
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
};

OE_UI_HANDLER(changeSSE);
OE_UI_HANDLER(zoomToData);

ui::Control* makeUI(App& app)
{
    ui::Grid* container = new ui::Grid();

    int r=0;
    container->setControl(0, r, new ui::LabelControl("Screen-space error (px)"));
    app._sse = container->setControl(1, r, new ui::HSliderControl(1.0f, app._maxSSE, 1.0f, new changeSSE(app)));
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
        << "\n   --build [earthfile]         ; build a B3DM file"
        << "\n     --startkey <Z,X,Y>        ; starting key of tiles to build"
        << "\n     --endkey <Z,X,Y>          ; optional ending key"
        << "\n     --out <filename>          ; output file with .b3dm extension"
        << "\n     --error <meters>          ; geometric error (meters) of data (default=100)"
        << "\n"
        << "\n   --view                      ; view a 3dtiles dataset"
        << "\n     --tileset <filename>      ; 3dtiles tileset JSON file to load"
        << "\n     --maxsse <n>              ; maximum screen space error in pixels for UI"
        << "\n     --features                ; treat the 3dtiles content as feature data"
        << "\n     --random-colors           ; randomly color feature tiles (instead of one color)"
        << std::endl;
        //<< MapNodeHelper().usage() << std::endl;

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

        if (tile->content().isSet() && tile->content()->uri().isSet())
        {
            OE_INFO << "Rendering feature tile: " << tile->content()->uri()->base() << std::endl;
        
            OGRFeatureOptions ogr;
            ogr.url() = tile->content()->uri().get();
            osg::ref_ptr<FeatureSource> fs = FeatureSourceFactory::create(ogr);
            if (fs.valid() && fs->open().isOK())
            {
                osg::ref_ptr<FeatureCursor> cursor = fs->createFeatureCursor(0L);
                FeatureList features;
                cursor->fill(features);
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
                    style.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
                    node = new FeatureNode(features, style);
                }
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

    app._maxSSE = 10.0f;
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

    if (readFeatures)
        app._tileset = new TDTilesetGroup(new FeatureRenderer(app));
    else
        app._tileset = new TDTilesetGroup();

    app._handler = app._tileset->getContentHandler();
    ui::ControlCanvas::get(&viewer)->addControl(makeUI(app));

    app._tileset->setTileset(tileset);
    app._tileset->setReadOptions(mapNode->getMap()->getReadOptions());
    mapNode->addChild(app._tileset.get());

    viewer.setSceneData( node.get() );
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

int
main_build(osg::ArgumentParser& arguments)
{
    // Process:
    // 1. Open an earth file and load a Map.
    // 2. Find a feature model layer.
    // 3. Read in one tile from the feature source.
    // 4. Compile it into OSG geometry with a matrix transform
    // 5. Save that geometry to a B3DM file.

    // Establish the tile key to build
    std::string startkey, endkey;
    {
        arguments.read("--startkey", startkey);

        // If we only get startkey, copy it and do one key.
        arguments.read("--endkey", endkey);
        if (!startkey.empty() && endkey.empty())
            endkey = startkey;
    }
    if (startkey.empty() || endkey.empty())
        return usage("Missing required --startkey or --startkey/endkey pair");

    // Output filename prefix
    std::string prefix;
    if (!arguments.read("--out", prefix))
        return usage("Missing required --out <prefix>");
    if (!prefix.empty()) prefix = prefix + "_";

    // Geometric error (only render the data if it's on-screen size
    // is greater than "maxSSE" pixels per "error" meters.
    // For example, if error=100m, and maxSSE=16px, the data will only
    // render once 100m of data takes up at least 16px of screen space.
    double error;
    if (!arguments.read("--error", error))
        error = 100.0;

    // Open an earth file and load a Map.
    osg::ref_ptr<osg::Node> earthFile = osgDB::readNodeFiles(arguments);
    MapNode* mapNode = MapNode::get(earthFile.get());
    if (!mapNode)
        return usage("Unable to load an earth file");

    // Open all the layers:
    mapNode->openMapLayers();

    // Locate the first FeatureModelLayer in the map:
    const Map* map = mapNode->getMap();
    FeatureModelLayer* fml = map->getLayer<FeatureModelLayer>();
    if ( !fml )
        return usage("Unable to find a FeatureModel layer in the Map");
    if ( !fml->getStatus().isOK() )
        return usage(fml->getStatus().message());

    // Extract the feature source from the layer:
    FeatureSource* fs = fml->getFeatureSource();
    if ( !fs )
        return usage("Unable to get feature source from layer");
    if ( !fs->getStatus().isOK() )
        return usage(fs->getStatus().message());
    if ( fs->open(0L).isError() )
        return usage(fs->getStatus().message());

    const FeatureProfile* fp = fs->getFeatureProfile();
    if (!fp)
        return usage("Unable to get a feature profile");
    if (!fp->getProfile())
        return usage("Unagle to find a tiling profile in the feature profile");

    TileKey LL = parseKey(startkey, fp->getProfile());
    TileKey UR = parseKey(endkey, fp->getProfile());
    if (LL.valid() == false || UR.valid() == false)
        return usage("Bad tile keys");

    StyleSheet* sheet = fml->options().styles().get();
    if (!sheet)
        return usage("Missing stylesheet");
    const Style* style = sheet->getDefaultStyle();
    if (!style)
        return usage("No default style in the stylesheet");

    // Make sure the b3dm plugin is loaded
    std::string libname = osgDB::Registry::instance()->createLibraryNameForExtension("gltf");
    osgDB::Registry::instance()->loadLibrary(libname);

    osg::ref_ptr<Session> session = new Session(map, sheet, fs, 0L);
    GeometryCompiler compiler(fml->options());

    // Create the tileset file
    osg::ref_ptr<TDTiles::Tileset> tileset = new TDTiles::Tileset();
    tileset->asset()->version() = "1.0";
    tileset->root() = new TDTiles::Tile();

    // track the largest tile radius - we will use this as the geometric error
    // for the group.
    double maxTileRadius = 0.0;

    // track the total geospatial extent - this will become the bounding volume
    // for the group
    GeoExtent total;

    //TODO: for loop on the feature keys
    for(unsigned x=LL.getTileX(); x<=UR.getTileX(); ++x)
    {
        for(unsigned y=LL.getTileY(); y<=UR.getTileY(); ++y)
        {
            TileKey key(LL.getLOD(), x, y, LL.getProfile());

            // Check the "radius"
            double tileRadius = key.getExtent().computeBoundingGeoCircle().getRadius();
            maxTileRadius = osg::maximum(tileRadius, maxTileRadius);

            // Query the features corresponding to the tile key:
            Query query;
            query.tileKey() = key;

            osg::ref_ptr<FeatureCursor> cursor = fs->createFeatureCursor(query, 0L);
            if (!cursor.valid())
                continue;

            OE_INFO << LC << "Tile " << key.str() << std::endl;

            // Compile into OSG geometry
            FeatureList features;
            cursor->fill(features);
            OE_INFO << LC << "Found " << features.size() << " features" << std::endl;
            if (features.size() == 0)
                continue;

            FilterContext fc(session.get(), fp, key.getExtent());
            osg::ref_ptr<osg::Node> result = compiler.compile(features, *style, fc);
            if (!result)
                return usage("Failed to compile features into OSG geometry");
    
            std::string filename = Stringify() << prefix << key.getLOD() << "_" << key.getTileX() << "_" << key.getTileY() << ".b3dm";

            // Write out a B3DM file
            if (!osgDB::writeNodeFile(*result.get(), filename))
                return usage("Failed to write to output file");

            if (result->getBound().valid())
            {
                GeoExtent e = key.getExtent().transform(SpatialReference::get("wgs84"));
                if (total.isValid())
                    total.expandToInclude(e);
                else
                    total = e;

                osg::ref_ptr<TDTiles::Tile> tile = new TDTiles::Tile();
                tile->content()->uri() = filename;

                // Set up a bounding region (radians)
                // TODO: calculate the correct Z min/max instead of hard-coding
                tile->boundingVolume()->region()->set(
                    osg::DegreesToRadians(e.xMin()),
                    osg::DegreesToRadians(e.yMin()),
                    -1000.0,
                    osg::DegreesToRadians(e.xMax()),
                    osg::DegreesToRadians(e.yMax()),
                    3000.0);

                // This tile is the highest possible refinement (and no chidren),
                // and thus has a geometric error of zero.
                tile->geometricError() = 0;
                tileset->root()->children().push_back(tile.get());
            }
        }
    }

    // TODO: calculate the correct Z min/max instead of hard-coding
    tileset->root()->boundingVolume()->region()->set(
        osg::DegreesToRadians(total.xMin()),
        osg::DegreesToRadians(total.yMin()),
        -1000.0,
        osg::DegreesToRadians(total.xMax()),
        osg::DegreesToRadians(total.yMax()),
        3000.0);

    tileset->root()->geometricError() = error;

    tileset->geometricError() = error;

    // write out the tileset file ("tileset.json")
    std::ofstream out("tileset.json");
    Json::Value tilesetJSON = tileset->getJSON();
    Json::StyledStreamWriter writer;
    writer.write(out, tilesetJSON);
    out.close();

    return 0;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);

    if (arguments.read("--help"))
        return usage("Help!");

    if (arguments.read("--build"))
        return main_build(arguments);
    else if (arguments.read("--view"))
        return main_view(arguments);
    else
        return usage("Missing required --build or --view");
}
