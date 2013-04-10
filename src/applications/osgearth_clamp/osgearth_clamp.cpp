/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
#include <osgEarth/Terrain>
#include <osgEarth/XmlUtils>
#include <osgEarth/Viewpoint>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/ObjectLocator>

using namespace osgEarth;
using namespace osgEarth::Util;

class ClampObjectLocatorCallback : public osgEarth::TerrainCallback
{
public:
    ClampObjectLocatorCallback(ObjectLocatorNode* locator):
      _locator(locator),
      _maxLevel(-1),
      _minLevel(0)
    {
    }

    virtual void onTileAdded(const osgEarth::TileKey& tileKey, osg::Node* terrain, TerrainCallbackContext&)
    {           
        if ((int)tileKey.getLevelOfDetail() > _minLevel && _maxLevel < (int)tileKey.getLevelOfDetail())
        {
            osg::Vec3d position = _locator->getLocator()->getPosition();

            if (tileKey.getExtent().contains(position.x(), position.y()))
            {
                //Compute our location in geocentric
                const osg::EllipsoidModel* ellipsoid = tileKey.getProfile()->getSRS()->getEllipsoid();
                double x, y, z;            
                ellipsoid->convertLatLongHeightToXYZ(
                    osg::DegreesToRadians(position.y()), osg::DegreesToRadians(position.x()), 0,
                    x, y, z);
                //Compute the up vector
                osg::Vec3d up = ellipsoid->computeLocalUpVector(x, y, z );
                up.normalize();
                osg::Vec3d world(x, y, z);

                double segOffset = 50000;

                osg::Vec3d start = world + (up * segOffset);
                osg::Vec3d end = world - (up * segOffset);

                osgUtil::LineSegmentIntersector* i = new osgUtil::LineSegmentIntersector( start, end );

                osgUtil::IntersectionVisitor iv;            
                iv.setIntersector( i );
                terrain->accept( iv );

                osgUtil::LineSegmentIntersector::Intersections& results = i->getIntersections();
                if ( !results.empty() )
                {
                    const osgUtil::LineSegmentIntersector::Intersection& result = *results.begin();
                    osg::Vec3d hit = result.getWorldIntersectPoint();
                    double lat, lon, height;
                    ellipsoid->convertXYZToLatLongHeight(hit.x(), hit.y(), hit.z(), 
                        lat, lon, height);                
                    position.z() = height;
                    //OE_NOTICE << "Got hit, setting new height to " << height << std::endl;
                    _maxLevel = tileKey.getLevelOfDetail();
                    _locator->getLocator()->setPosition( position );
                }            
            }            
        }            

    }

    osg::ref_ptr< ObjectLocatorNode > _locator;
    int _maxLevel;
    int _minLevel;
};



int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    unsigned int numObjects = 5000;
    while (arguments.read("--count", numObjects)) {}


    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
    {
        OE_NOTICE << "Unable to load earth model" << std::endl;
        return 1;
    }

    osg::Group* root = new osg::Group();

    osgEarth::MapNode * mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if (!mapNode)
    {
        OE_NOTICE << "Could not find MapNode " << std::endl;
        return 1;
    }

    osgEarth::Util::EarthManipulator* manip = new EarthManipulator();
    manip->getSettings()->setArcViewpointTransitions( true );
    viewer.setCameraManipulator( manip );
    
    root->addChild( earthNode );    

    //viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode->getMap()) );

    
    osg::Node* tree = osgDB::readNodeFile("../data/tree.osg");         
    osg::MatrixTransform* mt = new osg::MatrixTransform();
    mt->setMatrix(osg::Matrixd::scale(10,10,10));
    mt->addChild( tree );
    //Create bound around mt rainer
    double centerLat =  46.840866;
    double centerLon = -121.769846;
    double height = 0.2;
    double width = 0.2;
    double minLat = centerLat - (height/2.0);
    double minLon = centerLon - (width/2.0);

    OE_NOTICE << "Placing " << numObjects << " trees" << std::endl;

    for (unsigned int i = 0; i < numObjects; i++)
    {
        osgEarth::Util::ObjectLocatorNode* locator = new osgEarth::Util::ObjectLocatorNode( mapNode->getMap() );        
        double lat = minLat + height * (rand() * 1.0)/(RAND_MAX-1);
        double lon = minLon + width * (rand() * 1.0)/(RAND_MAX-1);        
        //OE_NOTICE << "Placing tree at " << lat << ", " << lon << std::endl;
        locator->getLocator()->setPosition(osg::Vec3d(lon,  lat, 0 ) );        
        locator->addChild( mt );
        root->addChild( locator );
        mapNode->getTerrain()->addTerrainCallback( new ClampObjectLocatorCallback(locator) );        
    }    
    
    manip->setHomeViewpoint(Viewpoint( "Mt Rainier",        osg::Vec3d(    centerLon,   centerLat, 0.0 ), 0.0, -90, 45000 ));

    viewer.setSceneData( root );    

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
