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
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Metrics>
#include <osgEarth/DecalLayer>
#include <osgEarth/ImageUtils>
#include <iostream>

#define LC "[decal] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;

int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

struct App
{
    osg::ref_ptr<MapNode> _mapNode;
    osg::ref_ptr<DecalLayer> _layer;
    osg::ref_ptr<osg::Image> _image;

    App()
    {
        //_image = osgDB::readRefImageFile("../data/circle_gradient_2.png");
        _image = osgDB::readRefImageFile("../data/burn.png");
        if ( !_image.valid())
        {
            OE_WARN << "Failed to load decal image!" << std::endl;
        }
    }

    void init(MapNode* mapNode)
    {        
        _mapNode = mapNode;
        _layer = mapNode->getMap()->getLayer<DecalLayer>();
        if (!_layer.valid())
        {
            _layer = new DecalLayer();
            _layer->setMinLevel(11u);
            mapNode->getMap()->addLayer(_layer.get());
        }
    }

    void addDecal(const GeoExtent& extent)
    {
        if (_layer.valid() && _image.valid())
        {
            _layer->addDecal(extent, _image.get());
        }
    }
};


struct ClickToDecal : public osgGA::GUIEventHandler
{
    App _app;
    ClickToDecal(App& app) : _app(app) { }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == 'd')
        {
            osg::Vec3d world;
            _app._mapNode->getTerrain()->getWorldCoordsUnderMouse(aa.asView(), ea.getX(), ea.getY(), world);

            GeoPoint mapPoint;
            mapPoint.fromWorld(_app._mapNode->getMapSRS(), world);

            double t = aa.asView()->getFrameStamp()->getReferenceTime();
            t = 0.1 * (t - (long)t);

            GeoExtent extent(
                mapPoint.getSRS(),
                mapPoint.x()-t,
                mapPoint.y()-t,
                mapPoint.x()+t,
                mapPoint.y()+t);

            _app.addDecal(extent);

            OE_INFO << "ADDING DECAL... " << extent.toString() << std::endl;

            return true;
        }
        return false;
    }
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( true, false );
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    App app;

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( node )
    {
        viewer.setSceneData( node );

        app.init(MapNode::get(node));
        viewer.addEventHandler(new ClickToDecal(app));
        OE_NOTICE << LC << "Press 'd' to drop the bomb." << std::endl;

        Metrics::run(viewer);
    }
    else
    {
        return usage(argv[0]);
    }

    return 0;
}