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
#include <osgEarth/LineDrawable>

#include <osg/Notify>
#include <osgGA/GUIEventHandler>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgEarth/MapNode>

#include <osgEarth/ExampleResources>
#include <osgEarth/EarthManipulator>
#include <osgEarth/FeatureNode>

#include <osg/io_utils>
#include <s2/s2cell_id.h>
#include <s2/s2latlng_rect.h>
#include <s2/base/commandlineflags.h>
#include <s2/s2earth.h>
#include <s2/s1chord_angle.h>
#include <s2/s2closest_point_query.h>
#include <s2/s2point_index.h>
#include <s2/s2testing.h>

#include <chrono>
#include <thread>
#include <array>


#include <cinttypes>
#include <cstdint>
#include <cstdio>

using namespace std;
using namespace std::chrono;
using namespace osgEarth;
using namespace osgEarth::Util;

static osg::Group* s_root = new osg::Group;
static int s_level = 14;
static float s_height = 500.0f;
static bool s_optimizeRotation = false;

struct AABB
{
    AABB(const osg::Matrixd& localToWorld, const osg::Matrixd& worldToLocal, const osg::BoundingBoxd& boundingBox) :
        _localToWorld(localToWorld),
        _worldToLocal(worldToLocal),
        _boundingBox(boundingBox)
    {
    }

    osg::Matrixd _localToWorld;
    osg::Matrixd _worldToLocal;
    osg::BoundingBoxd _boundingBox;
};

AABB ComputeAABB(const GeoExtent& extent, double zMin, double zMax)
{
    const SpatialReference* srs = SpatialReference::get("epsg:4326");

    GeoPoint centroid;
    extent.getCentroid(centroid);

    osg::Matrixd worldToLocal, localToWorld;
    centroid.createWorldToLocal(worldToLocal);
    centroid.createLocalToWorld(localToWorld);

    osg::Vec3d world;
    osg::BoundingBoxd bb;

    GeoPoint(srs, extent.west(), extent.south(), zMin).toWorld(world);
    bb.expandBy(world * worldToLocal);
    GeoPoint(srs, extent.east(), extent.south(), zMin).toWorld(world);
    bb.expandBy(world * worldToLocal);
    GeoPoint(srs, extent.east(), extent.north(), zMin).toWorld(world);
    bb.expandBy(world * worldToLocal);
    GeoPoint(srs, extent.west(), extent.north(), zMin).toWorld(world);

    GeoPoint(srs, extent.west(), extent.south(), zMax).toWorld(world);
    bb.expandBy(world * worldToLocal);
    GeoPoint(srs, extent.east(), extent.south(), zMax).toWorld(world);
    bb.expandBy(world * worldToLocal);
    GeoPoint(srs, extent.east(), extent.north(), zMax).toWorld(world);
    bb.expandBy(world * worldToLocal);
    GeoPoint(srs, extent.west(), extent.north(), zMax).toWorld(world);

    return AABB(localToWorld, worldToLocal, bb);
}

AABB ComputeAABB(const S2Cell& cell, bool optimizeRotation, double zMin, double zMax)
{
    const SpatialReference* wgs84 = SpatialReference::create("wgs84");
    S2LatLng center(cell.GetCenter());

    GeoPoint centroid(wgs84, center.lng().degrees(), center.lat().degrees());
    osg::Matrixd worldToLocal, localToWorld;
    centroid.createWorldToLocal(worldToLocal);
    centroid.createLocalToWorld(localToWorld);

    osg::Vec3d world;
    osg::BoundingBoxd bb;

    if (optimizeRotation)
    {
        // Try to orient the bounding box so it fits the cell more tightly
        auto urLatLng = S2LatLng(cell.GetVertex(2));
        auto ulLatLng = S2LatLng(cell.GetVertex(3));
        osg::Vec3d ulWorld, urWorld;
        GeoPoint ul(wgs84, ulLatLng.lng().degrees(), ulLatLng.lat().degrees(), 0);
        ul.toWorld(ulWorld);
        osg::Vec3d ulLocal = ulWorld * worldToLocal;
        GeoPoint ur(wgs84, urLatLng.lng().degrees(), urLatLng.lat().degrees(), 0);
        ur.toWorld(urWorld);
        osg::Vec3d urLocal = urWorld * worldToLocal;

        osg::Vec3d sideVector = urLocal - ulLocal;
        sideVector.normalize();
        osg::Quat quat;
        quat.makeRotate(osg::Vec3d(1, 0, 0), sideVector);
        localToWorld = osg::Matrixd(quat) * localToWorld;
        worldToLocal = osg::Matrixd::inverse(localToWorld);
    }

    for (unsigned int i = 0; i < 4; i++)
    {
        auto pt = S2LatLng(cell.GetVertex(i));
        GeoPoint(wgs84, pt.lng().degrees(), pt.lat().degrees(), zMin).toWorld(world);
        bb.expandBy(world * worldToLocal);
        GeoPoint(wgs84, pt.lng().degrees(), pt.lat().degrees(), zMax).toWorld(world);
        bb.expandBy(world * worldToLocal);
    }

    return AABB(localToWorld, worldToLocal, bb);
}

osg::Node* RenderBounds(const AABB& bounds)
{
    const int index[24] = {
                0, 1, 1, 2, 2, 3, 3, 0,
                4, 5, 5, 6, 6, 7, 7, 4,
                0, 4, 1, 5, 2, 6, 3, 7
    };

    std::vector< osg::Vec3 > corners;
    LineDrawable* d = new LineDrawable(GL_LINES);
    d->setUseGPU(false);
    corners.push_back(osg::Vec3(bounds._boundingBox.xMin(), bounds._boundingBox.yMin(), bounds._boundingBox.zMin()));
    corners.push_back(osg::Vec3(bounds._boundingBox.xMax(), bounds._boundingBox.yMin(), bounds._boundingBox.zMin()));
    corners.push_back(osg::Vec3(bounds._boundingBox.xMax(), bounds._boundingBox.yMax(), bounds._boundingBox.zMin()));
    corners.push_back(osg::Vec3(bounds._boundingBox.xMin(), bounds._boundingBox.yMax(), bounds._boundingBox.zMin()));
    corners.push_back(osg::Vec3(bounds._boundingBox.xMin(), bounds._boundingBox.yMin(), bounds._boundingBox.zMax()));
    corners.push_back(osg::Vec3(bounds._boundingBox.xMax(), bounds._boundingBox.yMin(), bounds._boundingBox.zMax()));
    corners.push_back(osg::Vec3(bounds._boundingBox.xMax(), bounds._boundingBox.yMax(), bounds._boundingBox.zMax()));
    corners.push_back(osg::Vec3(bounds._boundingBox.xMin(), bounds._boundingBox.yMax(), bounds._boundingBox.zMax()));

    for (int i = 0; i < 24; ++i)
        d->pushVertex(corners[index[i]]);

    d->setColor(Color::Red);
    d->finish();

    osg::MatrixTransform* transform = new osg::MatrixTransform;
    transform->setMatrix(bounds._localToWorld);
    transform->addChild(d);
    return transform;
}

osg::Node* RenderCell(S2Cell& cell)
{
    const SpatialReference* wgs84 = SpatialReference::create("wgs84");

    Geometry* poly = new osgEarth::Polygon();
    for (unsigned int i = 0; i < 4; i++)
    {
        S2LatLng pt = S2LatLng(cell.GetVertex(i));
        poly->push_back(pt.lng().degrees(), pt.lat().degrees());
    }


    Style style;
    style.getOrCreate<LineSymbol>()->stroke()->color() = Color::Cyan;
    style.getOrCreate<LineSymbol>()->stroke()->width() = 5.0;
    style.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
    style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;

    Feature*     feature = new Feature(poly, wgs84);
    FeatureNode* featureNode = new FeatureNode(feature, style);

    return featureNode;
}

struct ClickGlobeHandler : public osgGA::GUIEventHandler
{
    ClickGlobeHandler(MapNode* mapNode)
        : _mapNode(mapNode)
    {
    }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());

        if (ea.getEventType() == ea.MOVE)
        {
            osg::Vec3d world;
            osgUtil::LineSegmentIntersector::Intersections hits;
            if (view->computeIntersections(ea.getX(), ea.getY(), hits))
            {
                // Get the point under the mouse:
                world = hits.begin()->getWorldIntersectPoint();

                // convert to map coords:
                GeoPoint mapPoint;
                mapPoint.fromWorld(_mapNode->getMapSRS(), world);

                S2CellId cellId(S2LatLng(S1Angle::Degrees(mapPoint.y()), S1Angle::Degrees(mapPoint.x())));
                OSG_NOTICE << "Cell id 0 " << cellId << std::endl;
                OSG_NOTICE << "Cell lat/long" << cellId.ToLatLng() << std::endl;

                S2Cell cell(cellId.parent(s_level));
                AABB aabb = ComputeAABB(cell, s_optimizeRotation, 0.0, s_height);

                if (_boundsDisplay)
                {
                    _mapNode->removeChild(_boundsDisplay);
                }

                osg::Group* group = new osg::Group;
                group->addChild(RenderBounds(aabb));
                group->addChild(RenderCell(cell));

                _boundsDisplay = group;
                _mapNode->addChild(_boundsDisplay);
            }
        }
        return false;
    }

    osg::Node* _boundsDisplay = nullptr;
    osgEarth::MapNode* _mapNode;

};

int
main(int argc, char** argv)
{
    osgEarth::initialize();
    osg::ArgumentParser arguments(&argc, argv);

    arguments.read("--level", s_level);

    osgViewer::Viewer viewer(arguments);
    // install our default manipulator (do this before calling load)
    EarthManipulator* manip = new EarthManipulator(arguments);
    viewer.setCameraManipulator(manip);

    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (node)
    {
        MapNode* mapNode = MapNode::findMapNode(node);
        viewer.getEventHandlers().push_back(new ClickGlobeHandler(mapNode));
        s_root->addChild(node);
    }

    viewer.setSceneData(s_root);

    return viewer.run();
}

