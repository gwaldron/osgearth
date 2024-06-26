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

#include <osg/Notify>
#include <osgViewer/Viewer>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/PlaceNode>
#include <osgEarth/FeatureNode>
#include <osgViewer/CompositeViewer>
#include <osgEarth/XYZ>
#include <osgEarth/GLUtils>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;

/**
 * Makes a simple projected MapNode that contains a basemap of the world
 */
MapNode* makeMiniMapNode( )
{
    Map* map = new Map();
    map->setProfile(Profile::create(Profile::SPHERICAL_MERCATOR));

    // add a semi-transparent XYZ layer:
    XYZImageLayer* osm = new XYZImageLayer();
    osm->setURL("http://[abc].tile.openstreetmap.org/{z}/{x}/{y}.png");
    osm->setProfile(Profile::create(Profile::SPHERICAL_MERCATOR));
    map->addLayer(osm);

    TerrainOptions terrainOptions;
    terrainOptions.lodMethod() = LODMethod::SCREEN_SPACE;

    MapNode::Options mapNodeOptions;
    mapNodeOptions.terrain() = terrainOptions;

    MapNode* mapNode = new MapNode(map, mapNodeOptions);
    mapNode->setEnableLighting(false);

    return mapNode;
}

osg::Node* drawBounds(MapNode* mapNode, osgEarth::GeoExtent& bounds)
{
    if (bounds.crossesAntimeridian())
    {
        GeoExtent first, second;
        bounds.splitAcrossAntimeridian(first, second);
        osg::Group* group = new osg::Group;
        group->addChild( drawBounds( mapNode, first ) );
        group->addChild( drawBounds( mapNode, second) );
        return group;
    }
    else
    {
        osgEarth::LineString* geom = new osgEarth::LineString();
        geom->push_back(osg::Vec3d(bounds.xMin(), bounds.yMin(), 0));
        geom->push_back(osg::Vec3d(bounds.xMax(), bounds.yMin(), 0));
        geom->push_back(osg::Vec3d(bounds.xMax(), bounds.yMax(), 0));
        geom->push_back(osg::Vec3d(bounds.xMin(), bounds.yMax(), 0));
        geom->push_back(osg::Vec3d(bounds.xMin(), bounds.yMin(), 0));
        osgEarth::Feature* feature = new osgEarth::Feature(geom, osgEarth::SpatialReference::create("wgs84"));
        Style style;
        style.getOrCreateSymbol<LineSymbol>()->stroke().mutable_value().color() = Color::Yellow;
        feature->style() = style;
        FeatureNode* featureNode = new FeatureNode(feature);
        featureNode->setMapNode(mapNode);

        featureNode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
        return featureNode;
    }
}

osgEarth::GeoExtent getExtent(osgViewer::View* view)
{
    // Get the corners of all points on the view frustum.  Mostly modified from osgthirdpersonview
    osg::Matrixd proj = view->getCamera()->getProjectionMatrix();
    osg::Matrixd mv = view->getCamera()->getViewMatrix();
    osg::Matrixd invmv = osg::Matrixd::inverse( mv );

    double nearPlane = proj(3,2) / (proj(2,2)-1.0);
    double farPlane = proj(3,2) / (1.0+proj(2,2));

    // Get the sides of the near plane.
    double nLeft = nearPlane * (proj(2,0)-1.0) / proj(0,0);
    double nRight = nearPlane * (1.0+proj(2,0)) / proj(0,0);
    double nTop = nearPlane * (1.0+proj(2,1)) / proj(1,1);
    double nBottom = nearPlane * (proj(2,1)-1.0) / proj(1,1);

    // Get the sides of the far plane.
    double fLeft = farPlane * (proj(2,0)-1.0) / proj(0,0);
    double fRight = farPlane * (1.0+proj(2,0)) / proj(0,0);
    double fTop = farPlane * (1.0+proj(2,1)) / proj(1,1);
    double fBottom = farPlane * (proj(2,1)-1.0) / proj(1,1);

    double dist = farPlane - nearPlane;

    std::vector< osg::Vec3d > verts;
    verts.reserve(9);


    // Include origin?
    //verts.push_back(osg::Vec3d(0., 0., 0. ));
    verts.push_back(osg::Vec3d( nLeft, nBottom, -nearPlane ));
    verts.push_back(osg::Vec3d( nRight, nBottom, -nearPlane ));
    verts.push_back(osg::Vec3d( nRight, nTop, -nearPlane ));
    verts.push_back(osg::Vec3d( nLeft, nTop, -nearPlane ));
    verts.push_back(osg::Vec3d( fLeft, fBottom, -farPlane ));
    verts.push_back(osg::Vec3d( fRight, fBottom, -farPlane ));
    verts.push_back(osg::Vec3d( fRight, fTop, -farPlane ));
    verts.push_back(osg::Vec3d( fLeft, fTop, -farPlane ));

    const osgEarth::SpatialReference* srs = osgEarth::SpatialReference::create("epsg:4326");

    // Compute the bounding sphere of the frustum.
    osg::BoundingSphered bs;
    for (unsigned int i = 0; i < verts.size(); i++)
    {
        osg::Vec3d world = verts[i] * invmv;
        bs.expandBy( world );
    }

    // Get the center of the bounding sphere
    osgEarth::GeoPoint center;
    center.fromWorld(srs, bs.center());

    double radiusDegrees = bs.radius() /= 111000.0;
    double minLon = center.x() - radiusDegrees;
    double minLat = osg::clampAbove(center.y() - radiusDegrees, -90.0);
    double maxLon = center.x() + radiusDegrees;
    double maxLat = osg::clampBelow(center.y() + radiusDegrees, 90.0);

    osgEarth::GeoExtent extent(srs, minLon, minLat, maxLon, maxLat);

    return extent;
}

int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc,argv);

    //Setup a CompositeViewer
    osgViewer::CompositeViewer viewer(arguments);

    //Setup our main view that will show the loaded earth file.
    osgViewer::View* mainView = new osgViewer::View();
    mainView->getCamera()->setNearFarRatio(0.00002);
    mainView->setCameraManipulator( new EarthManipulator() );
    mainView->setUpViewInWindow( 50, 50, 800, 800 );
    viewer.addView( mainView );

    //Setup a MiniMap View that will be embedded in the main view
    int miniMapWidth = 400;
    int miniMapHeight = 400;
    osgViewer::View* miniMapView = new osgViewer::View();
    miniMapView->getCamera()->setNearFarRatio(0.00002);
    miniMapView->getCamera()->setViewport( 0, 0, miniMapWidth, miniMapHeight);
    miniMapView->getCamera()->setClearColor( osg::Vec4(0,0,0,0));
    miniMapView->getCamera()->setProjectionResizePolicy( osg::Camera::FIXED );
    miniMapView->getCamera()->setProjectionMatrixAsOrtho2D(MERC_MINX, MERC_MAXX, MERC_MINY, MERC_MAXY);
    //Share a graphics context with the main view
    miniMapView->getCamera()->setGraphicsContext( mainView->getCamera()->getGraphicsContext());
    GLUtils::setGlobalDefaults(miniMapView->getCamera()->getOrCreateStateSet());
    viewer.addView( miniMapView );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    auto node = MapNodeHelper().load( arguments, &viewer );
    if (node.valid())
    {
        MapNode* mapNode = MapNode::get(node);
        if (!mapNode)
            return -1;

        //Set the main view's scene data to the loaded earth file
        mainView->setSceneData( node );

        //Setup a group to hold the contents of the MiniMap
        osg::Group* miniMapGroup = new osg::Group;

        MapNode* miniMapNode = makeMiniMapNode();
        miniMapGroup->addChild( miniMapNode );

        //Get the main MapNode so we can do transformations between it and our minimap
        MapNode* mainMapNode = MapNode::findMapNode( node );

        //Set the scene data for the minimap
        miniMapView->setSceneData( miniMapGroup );

        //Add a marker we can move around with the main view's eye point
        Style markerStyle;
        markerStyle.getOrCreate<IconSymbol>()->url().mutable_value().setLiteral( "../data/placemark32.png" );
        PlaceNode* eyeMarker = new PlaceNode("", markerStyle);
        eyeMarker->setDynamic(true);
        eyeMarker->setPosition(GeoPoint(miniMapNode->getMapSRS(), 0, 0));
        miniMapNode->addChild( eyeMarker );

        osg::Node* bounds = 0;

        while (!viewer.done())
        {
            //Reset the viewport so that the camera's viewport is static and doesn't resize with window resizes
            miniMapView->getCamera()->setViewport( 0, 0, miniMapWidth, miniMapHeight);

            //Get the eye point of the main view
            osg::Vec3d eye, up, center;
            mainView->getCamera()->getViewMatrixAsLookAt( eye, center, up );

            //Turn the eye into a geopoint
            GeoPoint eyeGeo;
            eyeGeo.fromWorld( mainMapNode->getMapSRS(), eye );

            //We want the marker to be positioned at elevation 0, so zero out any elevation in the eye point
            eyeGeo.z() = 0;

            //Set the position of the marker
            eyeMarker->setPosition( eyeGeo );

            if (bounds)
            {
                miniMapNode->removeChild( bounds );
            }
            GeoExtent extent = getExtent( mainView );
            bounds = drawBounds( miniMapNode, extent );
            miniMapNode->addChild( bounds );

            viewer.frame();
        }
    }
    else
    {
        OE_NOTICE
            << "\nUsage: " << argv[0] << " file.earth" << std::endl
            << MapNodeHelper().usage() << std::endl;
    }
    return 0;
}
