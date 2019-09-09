/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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
#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Metrics>
#include <osgEarth/Registry>
#include <iostream>

#include <osgEarthAnnotation/PlaceNode>

#include <osgEarthUtil/ClusterNode>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

int
usage(const char* name)
{
    OE_NOTICE
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

void makePlaces(MapNode* mapNode, unsigned int count, const GeoExtent& extent, osg::NodeList& nodes)
{    
    // set up a style to use for placemarks:
    Style placeStyle;    
    placeStyle.getOrCreate<TextSymbol>()->declutter() = false;

    // A lat/long SRS for specifying points.
    const SpatialReference* geoSRS = mapNode->getMapSRS()->getGeographicSRS();

    //--------------------------------------------------------------------

    {
        osg::ref_ptr<osg::Image> pin = osgDB::readRefImageFile("../data/hospital.png");

        for (unsigned int i = 0; i < count; i++)
        {
            double lat = extent.yMin() + extent.height() * (rand() * 1.0) / (RAND_MAX - 1);
            double lon = extent.xMin() + extent.width() * (rand() * 1.0) / (RAND_MAX - 1);
            PlaceNode* place = new PlaceNode("Placemark", placeStyle, pin.get());
            place->setPosition(GeoPoint(geoSRS, lon, lat, 0.0));
            place->setMapNode(mapNode);
            place->setDynamic(true);
            nodes.push_back(place);
        }
    }    
}

void makeModels(MapNode* mapNode, unsigned int count, const GeoExtent& extent, osg::NodeList& nodes)
{
    osg::ref_ptr< osg::Node > cessna = osgDB::readRefNodeFile("cessna.osg.10,10,10.scale");
    osg::ref_ptr< osg::Node > cow = osgDB::readRefNodeFile("cow.osg.100,100,100.scale");
    osgEarth::Registry::shaderGenerator().run(cessna.get());
    osgEarth::Registry::shaderGenerator().run(cow.get());

    // A lat/long SRS for specifying points.
    const SpatialReference* geoSRS = mapNode->getMapSRS()->getGeographicSRS();

    bool useCow = false;

    for (unsigned int i = 0; i < count; i++)
    {
        double lat = extent.yMin() + extent.height() * (rand() * 1.0) / (RAND_MAX - 1);
        double lon = extent.xMin() + extent.width() * (rand() * 1.0) / (RAND_MAX - 1);

        GeoTransform* transform = new GeoTransform();
        transform->setPosition(GeoPoint(geoSRS, lon, lat, 1000));
        if (useCow)
        {
            transform->addChild(cow.get());
            transform->setName("cow");
        }        
        else
        {
            transform->addChild(cessna.get());
            transform->setName("plane");
        }
        nodes.push_back(transform);
        useCow = !useCow;
    }
}

Container*
createControlPanel(osgViewer::View* view)
{
    ControlCanvas* canvas = ControlCanvas::getOrCreate(view);
    VBox* vbox = canvas->addControl(new VBox());
    vbox->setChildSpacing(10);
    return vbox;
}

struct SetRadius : public ControlEventHandler
{
    SetRadius(ClusterNode* clusterNode) :
        _clusterNode( clusterNode )
    { }

    void onValueChanged(Control* control, float value)
    {
        _clusterNode->setRadius(value);
    }

    ClusterNode* _clusterNode;    
};

struct AddIcons : public ControlEventHandler
{
    AddIcons(ClusterNode* clusterNode, MapNode* mapNode) :
        _clusterNode(clusterNode),
        _mapNode(mapNode)
    { }

    void onClick(Control* button)
    {
        osg::NodeList nodes;
        GeoExtent extent(SpatialReference::create("wgs84"), -180, -90, 180, 90);
        makePlaces(_mapNode, 1000, extent, nodes);
        for (unsigned int i = 0; i < nodes.size(); ++i)
        {
            _clusterNode->addNode(nodes[i].get());
        }
    }

    ClusterNode* _clusterNode;
    MapNode* _mapNode;
};

struct ToggleEnabled : public ControlEventHandler
{
    ToggleEnabled(ClusterNode* clusterNode) :
        _clusterNode(clusterNode)
    { }

    virtual void onValueChanged(Control* control, bool value) {
        _clusterNode->setEnabled(value);
    }

    ClusterNode* _clusterNode;
};


void buildControls(Container* container, ClusterNode* clusterNode, MapNode* mapNode)
{
    // the outer container:
    Grid* grid = container->addControl(new Grid());
    grid->setBackColor(0, 0, 0, 0.5);
    grid->setMargin(10);
    grid->setPadding(10);
    grid->setChildSpacing(10);
    grid->setChildVertAlign(Control::ALIGN_CENTER);
    grid->setAbsorbEvents(true);
    grid->setVertAlign(Control::ALIGN_TOP);

    // Radius
    LabelControl* radiusLabel = new LabelControl("Radius");
    radiusLabel->setVertAlign(Control::ALIGN_CENTER);
    grid->setControl(0, 0, radiusLabel);

    HSliderControl* radiusAdjust = new HSliderControl(1, 500, clusterNode->getRadius(), new SetRadius(clusterNode));
    radiusAdjust->setWidth(125);
    radiusAdjust->setHeight(12);
    radiusAdjust->setVertAlign(Control::ALIGN_CENTER);
    grid->setControl(1, 0, radiusAdjust);
    grid->setControl(2, 0, new LabelControl(radiusAdjust));
    
    grid->setControl(0, 1, new LabelControl("Enabled"));
    CheckBoxControl* checkBox = new CheckBoxControl(clusterNode->getEnabled());
    checkBox->setHorizAlign(Control::ALIGN_LEFT);
    checkBox->addEventHandler(new ToggleEnabled(clusterNode));
    grid->setControl(1, 1, checkBox);

    
    grid->setControl(0, 2, new ButtonControl("Add Icons", new AddIcons(clusterNode, mapNode)));
    
}

//! Displays a simplified count for the cluster instead of the exact number.
class SimplifyCountCallback : public ClusterNode::StyleClusterCallback
{
public:
    virtual void operator()(ClusterNode::Cluster& cluster)
    {        
        if (cluster.nodes.size() >= 100)
        {
            cluster.marker->setText("100+");
        }
        else if (cluster.nodes.size() >= 50)
        {
            cluster.marker->setText("50+");
        }
        else if (cluster.nodes.size() >= 25)
        {
            cluster.marker->setText("25+");
        }
        else if (cluster.nodes.size() >= 10)
        {
            cluster.marker->setText("10+");
        }
        else
        {
            cluster.marker->setText("2+");
        } 
    }
};

//! Changes the name of a marker based on the name of the clustered nodes.
class StyleByNameCallback : public ClusterNode::StyleClusterCallback
{
public:

    StyleByNameCallback()
    {
        _planeImage = osgDB::readRefImageFile("../data/airport.png");
        _cowImage = osgDB::readRefImageFile("../data/hospital.png");
    }
    
    virtual void operator()(ClusterNode::Cluster& cluster)
    {    
        std::stringstream buf;
        buf << cluster.nodes[0]->getName() << "(" << cluster.nodes.size() << ")" << std::endl;
        cluster.marker->setText(buf.str());

        if (cluster.nodes[0]->getName() == "plane")
        {
            cluster.marker->setIconImage(_planeImage.get());
        }
        else if (cluster.nodes[0]->getName() == "cow")
        {
            cluster.marker->setIconImage(_cowImage.get());
        } 
    }

    osg::ref_ptr< osg::Image > _planeImage;
    osg::ref_ptr< osg::Image > _cowImage;
};

//! Only allows nodes with the same name to be clustered together.
class ClusterByNameCallback : public ClusterNode::CanClusterCallback
{
public:
    virtual bool operator()(osg::Node* a, osg::Node* b)
    {
        return (a->getName() == b->getName());
    }
};


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);

    // help?
    if (arguments.read("--help"))
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

    //Create the control panel    
    Container* container = createControlPanel(&viewer);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy(true, false);

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator(new EarthManipulator(arguments));

    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.0001);

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (node)
    {
        MapNode* mapNode = MapNode::findMapNode(node);
        osg::NodeList nodes;

        //GeoExtent extent(SpatialReference::create("wgs84"), -180, -90, 180, 90);
        GeoExtent extent(SpatialReference::create("wgs84"), -160.697021484375, 18.208480196039883, -153.951416015625, 22.978623970384913);
        
        makeModels(mapNode, 10000, extent, nodes);

        ClusterNode* clusterNode = new ClusterNode(mapNode, osgDB::readImageFile("../data/placemark32.png"));
        clusterNode->setStyleCallback(new StyleByNameCallback());
        clusterNode->setCanClusterCallback(new ClusterByNameCallback());
        for (unsigned int i = 0; i < nodes.size(); i++)
        {
            clusterNode->addNode(nodes[i].get());
        }              
        mapNode->addChild(clusterNode);

        buildControls(container, clusterNode, mapNode);

        viewer.setSceneData(node);

        while (!viewer.done())
        {
            viewer.frame();
        }
        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }

    return 0;
}
