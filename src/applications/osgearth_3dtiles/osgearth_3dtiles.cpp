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
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ViewFitter>
#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/TDTiles>
#include <osgEarth/Random>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/FeatureCursor>
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
    app._sse = container->setControl(1, r, new ui::HSliderControl(1.0f, 100.0f, 1.0f, new changeSSE(app)));
    app._sse->setHorizFill(true, 300.0f);
    container->setControl(2, r, new ui::LabelControl(app._sse));

    ++r;
    container->setControl(0, r, new ui::ButtonControl("Zoom to data", new zoomToData(app)));

    return container;
}

int
usage(const std::string& message)
{
    OE_NOTICE 
        << "\n" << message
        << "\nUsage: osgearth_3dtiles file.earth"
        << "\n     --tileset [filename]"
        << std::endl;
        //<< MapNodeHelper().usage() << std::endl;

    return 0;
}

struct MyFeatureHandler : public TDTiles::ContentHandler
{
    mutable Random _random;

    osg::ref_ptr<osg::Node> createNode(TDTiles::Tile* tile, const osgDB::Options* readOptions) const
    {
        OE_INFO << "SAY HELLO to tile " << tile->content()->uri()->base() << std::endl;
        osg::ref_ptr<osg::Node> node;
        
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
                float r = _random.next(), g = _random.next(), b = _random.next();
                style.getOrCreate<PolygonSymbol>()->fill()->color().set(r,g,b,1);
                style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
                style.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
                node = new FeatureNode(features, style);
            }
        }

        return node;
    }
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage("Help!");

    std::string tilesetLocation;
    if (!arguments.read("--tileset", tilesetLocation))
        return usage("Missing required --tileset");

    bool readFeatures = arguments.read("--features");

    // load the tile set:
    URI tilesetURI(tilesetLocation);
    ReadResult rr = tilesetURI.readString();
    if (rr.failed())
        return usage(Stringify()<<"Error loading tileset: " <<rr.errorDetail());

    TDTiles::Tileset* tileset = TDTiles::Tileset::create(rr.getString(), tilesetLocation);
    if (!tileset)
        return usage("Bad tileset");

    App app;

    // create a viewer:
    osgViewer::Viewer viewer(arguments);
    app._view = &viewer;

    viewer.setCameraManipulator( app._manip = new EarthManipulator(arguments) );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( node )
    {
        MapNode* mapNode = MapNode::get(node);

        if (readFeatures)
            app._tileset = new TDTilesetGroup(new MyFeatureHandler());
        else
            app._tileset = new TDTilesetGroup();

        app._handler = app._tileset->getContentHandler();
        ui::ControlCanvas::get(&viewer)->addControl(makeUI(app));

        app._tileset->setTileset(tileset);
        app._tileset->setReadOptions(mapNode->getMap()->getReadOptions());
        mapNode->addChild(app._tileset.get());

        viewer.setSceneData( node );
        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }

    return 0;
}