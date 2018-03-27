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

#include "kdbush.hpp"

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

using TPoint = std::pair<int, int>;
using TIds = std::vector<std::size_t>;

static std::vector<TPoint> points = {
    { 54, 1 },{ 97, 21 },{ 65, 35 },{ 33, 54 },{ 95, 39 },{ 54, 3 },{ 53, 54 },{ 84, 72 },
    { 33, 34 },{ 43, 15 },{ 52, 83 },{ 81, 23 },{ 1, 61 },{ 38, 74 },{ 11, 91 },{ 24, 56 },
    { 90, 31 },{ 25, 57 },{ 46, 61 },{ 29, 69 },{ 49, 60 },{ 4, 98 },{ 71, 15 },{ 60, 25 },
    { 38, 84 },{ 52, 38 },{ 94, 51 },{ 13, 25 },{ 77, 73 },{ 88, 87 },{ 6, 27 },{ 58, 22 },
    { 53, 28 },{ 27, 91 },{ 96, 98 },{ 93, 14 },{ 22, 93 },{ 45, 94 },{ 18, 28 },{ 35, 15 },
    { 19, 81 },{ 20, 81 },{ 67, 53 },{ 43, 3 },{ 47, 66 },{ 48, 34 },{ 46, 12 },{ 32, 38 },
    { 43, 12 },{ 39, 94 },{ 88, 62 },{ 66, 14 },{ 84, 30 },{ 72, 81 },{ 41, 92 },{ 26, 4 },
    { 6, 76 },{ 47, 21 },{ 57, 70 },{ 71, 82 },{ 50, 68 },{ 96, 18 },{ 40, 31 },{ 78, 53 },
    { 71, 90 },{ 32, 14 },{ 55, 6 },{ 32, 88 },{ 62, 32 },{ 21, 67 },{ 73, 81 },{ 44, 64 },
    { 29, 50 },{ 70, 5 },{ 6, 22 },{ 68, 3 },{ 11, 23 },{ 20, 42 },{ 21, 73 },{ 63, 86 },
    { 9, 40 },{ 99, 2 },{ 99, 76 },{ 56, 77 },{ 83, 6 },{ 21, 72 },{ 78, 30 },{ 75, 53 },
    { 41, 11 },{ 95, 20 },{ 30, 38 },{ 96, 82 },{ 65, 48 },{ 33, 18 },{ 87, 28 },{ 10, 10 },
    { 40, 34 },{ 10, 20 },{ 47, 29 },{ 46, 78 }
};

static void testRange() {
    kdbush::KDBush<TPoint> index(points, 10);
    TIds expectedIds = { 3, 90, 77, 72, 62, 96, 47, 8, 17, 15, 69, 71, 44, 19, 18, 45, 60, 20 };
    TIds result;
    index.range(20, 30, 50, 70, [&result](const auto id) { result.push_back(id); });

    assert(std::equal(expectedIds.begin(), expectedIds.end(), result.begin()));
}

static void testRadius() {
    kdbush::KDBush<TPoint> index(points, 10);
    TIds expectedIds = { 3, 96, 71, 44, 18, 45, 60, 6, 25, 92, 42, 20 };
    TIds result;
    index.within(50, 50, 20, [&result](const auto id) { result.push_back(id); });

    assert(std::equal(expectedIds.begin(), expectedIds.end(), result.begin()));
}

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
    // Make a group for 2D items, and activate the decluttering engine. Decluttering
    // will migitate overlap between elements that occupy the same screen real estate.
    osg::Group* labelGroup = new osg::Group();
   
    // set up a style to use for placemarks:
    Style placeStyle;    
    placeStyle.getOrCreate<TextSymbol>()->declutter() = false;

    // A lat/long SRS for specifying points.
    const SpatialReference* geoSRS = mapNode->getMapSRS()->getGeographicSRS();

    //--------------------------------------------------------------------

    //Create a bunch of placemarks around Mt Rainer so we can actually get some elevation
    {
        osg::ref_ptr<osg::Image> pin = osgDB::readRefImageFile("../data/placemark32.png");

        double centerLat = 46.840866;
        double centerLon = -121.769846;
        double height = 5;
        double width = 5;
        double minLat = centerLat - (height / 2.0);
        double minLon = centerLon - (width / 2.0);        

        for (unsigned int i = 0; i < count; i++)
        {
            double lat = minLat + height * (rand() * 1.0) / (RAND_MAX - 1);
            double lon = minLon + width * (rand() * 1.0) / (RAND_MAX - 1);
            PlaceNode* place = new PlaceNode(mapNode, GeoPoint(geoSRS, lon, lat, 0.0), pin.get(), "Placemark", placeStyle);
            place->setDynamic(true);
            placeNodes.push_back(place);
            labelGroup->addChild(place);
        }
    }

    mapNode->addChild(labelGroup);    
}

void updatePlaces(std::vector< osg::ref_ptr< PlaceNode > >& places, osg::Matrixd& mvpw, osg::Viewport* viewport)
{
    if (!viewport)
    {
        return;
    }

    std::vector<TPoint> points;

    std::vector< osg::ref_ptr< PlaceNode > > validPlaces;

    for (unsigned int i = 0; i < places.size(); i++)
    {
        osg::Vec3d world;
        places[i]->getPosition().toWorld(world);
        osg::Vec3d screen = world * mvpw;
        //OE_NOTICE << "Screen " << screen.x() << ", " << screen.y() << std::endl;

        if (screen.x() >= 0 || screen.x() <= viewport->width() ||
            screen.y() >= 0 || screen.y() <= viewport->height())
        {
            validPlaces.push_back(places[i]);
            points.push_back({ screen.x(), screen.y() });
        }
        else
        {
            places[i]->setNodeMask(0);
        }
        
    }    

    kdbush::KDBush<TPoint> index(points);
    std::set< unsigned int > clustered;

    for (unsigned int i = 0; i < validPlaces.size(); i++)
    {
        TPoint &screen = points[i];
        PlaceNode* place = validPlaces[i].get();

        // If this thing is already part of a cluster then just continue.
        if (clustered.find(i) != clustered.end())
        {
            // Hide the placenode
            place->setNodeMask(0);
            continue;
        }

        // Show the place node, this is the master of the cluster
        place->setNodeMask(~0);

        // Get any matching indices that are part of this cluster.
        TIds indices;
        index.within(screen.first, screen.second, 50, [&indices](const auto id) { indices.push_back(id); });
        OE_NOTICE << "Found " << indices.size() << " points in cluster" << std::endl;

        unsigned int actualCount = 0;

        // Add all of the points to the cluster.
        for (unsigned int j = 0; j < indices.size(); j++)
        {
            if (clustered.find(indices[j]) == clustered.end())
            {
                actualCount++;
                clustered.insert(indices[j]);
            }            
        }

        std::stringstream buf;
        buf << actualCount << std::endl;
        place->setText(buf.str());

        clustered.insert(i);
    }
}


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc, argv);

    // help?
    if (arguments.read("--help"))
        return usage(argv[0]);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);

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
        makePlaces(mapNode, 100000, placeNodes);

        viewer.setSceneData(node);

        while (!viewer.done())
        {
            if (viewer.getCamera()->getViewport())
            {
                osg::Matrixd mvpw = viewer.getCamera()->getViewMatrix() *
                    viewer.getCamera()->getProjectionMatrix() *
                    viewer.getCamera()->getViewport()->computeWindowMatrix();
                updatePlaces(placeNodes, mvpw, viewer.getCamera()->getViewport());
            }

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
