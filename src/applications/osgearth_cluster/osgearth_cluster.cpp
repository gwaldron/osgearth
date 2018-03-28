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

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Metrics>
#include <iostream>

#include <osgEarthAnnotation/PlaceNode>

#include "ClusterNode"

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

void makePlaces(MapNode* mapNode, unsigned int count, std::vector< osg::ref_ptr< PlaceNode > >& placeNodes)
{    
    // set up a style to use for placemarks:
    Style placeStyle;    
    placeStyle.getOrCreate<TextSymbol>()->declutter() = false;

    // A lat/long SRS for specifying points.
    const SpatialReference* geoSRS = mapNode->getMapSRS()->getGeographicSRS();

    //--------------------------------------------------------------------

    //Create a bunch of placemarks around Mt Rainer so we can actually get some elevation
    {
        osg::ref_ptr<osg::Image> pin = osgDB::readRefImageFile("../data/hospital.png");

        double centerLat = 46.840866;
        double centerLon = -121.769846;
        double height = 180;
        double width = 360;
        double minLat = centerLat - (height / 2.0);
        double minLon = centerLon - (width / 2.0);        

        for (unsigned int i = 0; i < count; i++)
        {
            double lat = minLat + height * (rand() * 1.0) / (RAND_MAX - 1);
            double lon = minLon + width * (rand() * 1.0) / (RAND_MAX - 1);
            PlaceNode* place = new PlaceNode(mapNode, GeoPoint(geoSRS, lon, lat, 0.0), pin.get(), "Placemark", placeStyle);
            place->setDynamic(true);
            placeNodes.push_back(place);
        }
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
        std::vector< osg::ref_ptr< PlaceNode > > placeNodes;
        makePlaces(_mapNode, 1000, placeNodes);
        for (unsigned int i = 0; i < placeNodes.size(); ++i)
        {
            _clusterNode->addNode(placeNodes[i]);
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

class MyClusterStyleCallback : public StyleClusterCallback
{
    virtual void operator()(Cluster& cluster)
    {
        /*
        if (cluster.places.size() >= 100)
        {
            cluster.marker->setText("100+");
        }
        else if (cluster.places.size() >= 50)
        {
            cluster.marker->setText("50+");
        }
        else if (cluster.places.size() >= 25)
        {
            cluster.marker->setText("25+");
        }
        else if (cluster.places.size() >= 10)
        {
            cluster.marker->setText("10+");
        }
        else
        {
            cluster.marker->setText("2+");
        } 
        */
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
        std::vector< osg::ref_ptr< PlaceNode > > placeNodes;

        MapNode* mapNode = MapNode::findMapNode(node);
        makePlaces(mapNode, 10000, placeNodes);

        ClusterNode* clusterNode = new ClusterNode(mapNode);
        clusterNode->setStyleCallback(new MyClusterStyleCallback());
        for (unsigned int i = 0; i < placeNodes.size(); i++)
        {
            clusterNode->addNode(placeNodes[i]);
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
