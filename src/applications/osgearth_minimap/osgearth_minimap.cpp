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

    MapNode::Options options;
    options.terrain()->lodMethod() = LODMethod::SCREEN_SPACE;

    return new MapNode(map, options);
}

osgEarth::Feature* getExtent(osgViewer::View* view)
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

    int samples = 24;
    std::vector<osg::Vec3d> verts;
    verts.reserve(samples * 4 - 4);
    for (int i = 0; i < samples - 1; ++i) {
        double j = (double)i / (double)(samples - 1);
        verts.push_back(osg::Vec3d(nLeft + (nRight - nLeft) * j, nBottom, nearPlane));
        verts.push_back(osg::Vec3d(fLeft + (fRight - fLeft) * j, fBottom, farPlane));
    }
    for (int i = 0; i < samples - 1; ++i) {
        double j = (double)i / (double)(samples - 1);
        verts.push_back(osg::Vec3d(nRight, nBottom + (nTop - nBottom) * j, nearPlane));
        verts.push_back(osg::Vec3d(fRight, fBottom + (fTop - fBottom) * j, farPlane));
    }
    for (int i = 0; i < samples - 1; ++i) {
        double j = (double)i / (double)(samples - 1);
        verts.push_back(osg::Vec3d(nRight - (nRight - nLeft) * j, nTop, nearPlane));
        verts.push_back(osg::Vec3d(fRight - (fRight - fLeft) * j, fTop, farPlane));
    }
    for (int i = 0; i < samples - 1; ++i) {
        double j = (double)i / (double)(samples - 1);
        verts.push_back(osg::Vec3d(nLeft, nTop - (nTop - nBottom) * j, nearPlane));
        verts.push_back(osg::Vec3d(fLeft, fTop - (fTop - fBottom) * j, farPlane));
    }

    const auto* wgs84 = osgEarth::SpatialReference::create("epsg:4326");
    auto& ellip = wgs84->getEllipsoid();

    std::vector<osg::Vec3d> points;
    points.reserve(verts.size() / 2);

    auto ecef = wgs84->getGeocentricSRS();

    for (int i = 0; i < verts.size() / 2; ++i)
    {
        osg::Vec3d p1 = verts[i * 2] * invmv;
        osg::Vec3d p2 = verts[i * 2 + 1] * invmv;
        osg::Vec3d out;

        if (ellip.intersectGeocentricLine(p1, p2, out))
        {
            ecef->transform(out, wgs84, out);
            points.push_back(out);
        }
    }

    auto poly = new osgEarth::Polygon(&points);
    return new osgEarth::Feature(poly, wgs84);
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
        miniMapNode->addChild(eyeMarker);

        osg::ref_ptr<FeatureNode> featureNode;
        Style style;
        style.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Orange, 0.5);
        style.getOrCreate<LineSymbol>()->stroke()->color() = Color::Yellow;
        style.getOrCreate<RenderSymbol>()->depthTest() = false;
        style.getOrCreate<RenderSymbol>()->maxTessAngle() = Angle(1.0, Units::DEGREES);

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

            if (featureNode)
            {
                miniMapNode->removeChild(featureNode);
                featureNode = nullptr;
            }

            auto feature = getExtent(mainView);

            if (feature)
            {
                featureNode = new FeatureNode(feature, style);
                miniMapNode->addChild(featureNode);
            }

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
