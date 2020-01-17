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

#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
#include <osgEarth/XmlUtils>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/LinearLineOfSight>
#include <osgEarthUtil/RadialLineOfSight>
#include <osg/io_utils>
#include <osg/MatrixTransform>
#include <osg/Depth>
#include <osgEarth/TerrainTileNode>
#include <osgEarth/FileUtils>

using namespace osgEarth;
using namespace osgEarth::Util;


osg::AnimationPath* createAnimationPath(const GeoPoint& pos, const SpatialReference* mapSRS, float radius, double looptime)
{
    // set up the animation path 
    osg::AnimationPath* animationPath = new osg::AnimationPath;
    animationPath->setLoopMode(osg::AnimationPath::LOOP);
    
    int numSamples = 40;

    double delta = osg::PI * 2.0 / (double)numSamples;

    //Get the center point in geocentric
    GeoPoint mapPos = pos.transform(mapSRS);
    osg::Vec3d centerWorld;
    mapPos.toWorld( centerWorld );

    bool isProjected = mapSRS->isProjected();

    osg::Vec3d up = isProjected ? osg::Vec3d(0,0,1) : centerWorld;
    up.normalize();

    //Get the "side" vector
    osg::Vec3d side = isProjected ? osg::Vec3d(1,0,0) : up ^ osg::Vec3d(0,0,1);


    double time=0.0f;
    double time_delta = looptime/(double)numSamples;

    osg::Vec3d firstPosition;
    osg::Quat firstRotation;

    for (unsigned int i = 0; i < (unsigned int)numSamples; i++)
    {
        double angle = delta * (double)i;
        osg::Quat quat(angle, up );
        osg::Vec3d spoke = quat * (side * radius);
        osg::Vec3d end = centerWorld + spoke;                

        osg::Quat makeUp;
        makeUp.makeRotate(osg::Vec3d(0,0,1), up);

        osg::Quat rot = makeUp;
        animationPath->insert(time,osg::AnimationPath::ControlPoint(end,rot));
        if (i == 0)
        {
            firstPosition = end;
            firstRotation = rot;
        }
        time += time_delta;            
    }
   
    animationPath->insert(time, osg::AnimationPath::ControlPoint(firstPosition, firstRotation));

    return animationPath;    
}

osg::Node* createPlane(osg::Node* node, const GeoPoint& pos, const SpatialReference* mapSRS, double radius, double time)
{
    osg::MatrixTransform* positioner = new osg::MatrixTransform;
    positioner->addChild( node );
    osg::AnimationPath* animationPath = createAnimationPath(pos, mapSRS, radius, time);
    positioner->setUpdateCallback( new osg::AnimationPathCallback(animationPath, 0.0, 1.0));
    return positioner;
}

class CacheExtentNodeVisitor : public osg::NodeVisitor
{
public:
    CacheExtentNodeVisitor(GeoExtent& extent):
      osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
          _extent(extent)
      {
      }

      void apply(osg::Node& node)
      {
          TerrainTileNode* tile = dynamic_cast<TerrainTileNode*>(&node);
          if (tile && tile->getKey().valid())
          {              
              if (tile->getKey().getExtent().intersects(_extent) && tile->getKey().getLevelOfDetail() < 11)
              {
                  // Set this tile to not expire.
                  tile->setMinimumExpirationTime(DBL_MAX);
                  OE_NOTICE << "Preloading children for " << tile->getKey().str() << std::endl;
                  tile->loadChildren();
              }
          }          
          traverse(node);
      }

      GeoExtent _extent;
};


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    // load the .earth file from the command line.
    osg::ref_ptr<osg::Node> earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode.valid())
    {
        OE_NOTICE << "Unable to load earth model" << std::endl;
        return 1;
    }

    osg::Group* root = new osg::Group();

    osgEarth::MapNode * mapNode = osgEarth::MapNode::findMapNode( earthNode.get() );
    if (!mapNode)
    {
        OE_NOTICE << "Could not find MapNode " << std::endl;
        return 1;
    }

    osgEarth::Util::EarthManipulator* manip = new EarthManipulator();
    viewer.setCameraManipulator( manip );
    
    root->addChild( earthNode );    
    //viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode));

    osg::Group* losGroup = new osg::Group();
    losGroup->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    losGroup->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::ALWAYS, 0, 1, false));
    root->addChild(losGroup);

    // so we can speak lat/long:
    const SpatialReference* mapSRS = mapNode->getMapSRS();
    const SpatialReference* geoSRS = mapSRS->getGeographicSRS();

    //Create a point to point LineOfSightNode.
    LinearLineOfSightNode* los = new LinearLineOfSightNode(
        mapNode, 
        GeoPoint(geoSRS, -121.665, 46.0878, 1258.00, ALTMODE_ABSOLUTE),
        GeoPoint(geoSRS, -121.488, 46.2054, 3620.11, ALTMODE_ABSOLUTE) );

    losGroup->addChild( los );
    
    //Create an editor for the point to point line of sight that allows you to drag the beginning and end points around.
    //This is just one way that you could manipulator the LineOfSightNode.
    LinearLineOfSightEditor* p2peditor = new LinearLineOfSightEditor( los );
    root->addChild( p2peditor );

    //Create a relative point to point LineOfSightNode.
    LinearLineOfSightNode* relativeLOS = new LinearLineOfSightNode( 
        mapNode, 
        GeoPoint(geoSRS, -121.2, 46.1, 10, ALTMODE_RELATIVE),
        GeoPoint(geoSRS, -121.488, 46.2054, 10, ALTMODE_RELATIVE) );

    losGroup->addChild( relativeLOS );

    LinearLineOfSightEditor* relEditor = new LinearLineOfSightEditor( relativeLOS );
    root->addChild( relEditor );

    //Create a RadialLineOfSightNode that allows you to do a 360 degree line of sight analysis.
    RadialLineOfSightNode* radial = new RadialLineOfSightNode( mapNode );
    radial->setCenter( GeoPoint(geoSRS, -121.515, 46.054, 847.604, ALTMODE_ABSOLUTE) );
    radial->setRadius( 2000 );
    radial->setNumSpokes( 100 );    
    losGroup->addChild( radial );
    RadialLineOfSightEditor* radialEditor = new RadialLineOfSightEditor( radial );
    losGroup->addChild( radialEditor );

    //Create a relative RadialLineOfSightNode that allows you to do a 360 degree line of sight analysis.
    RadialLineOfSightNode* radialRelative = new RadialLineOfSightNode( mapNode );
    radialRelative->setCenter( GeoPoint(geoSRS, -121.2, 46.054, 10, ALTMODE_RELATIVE) );
    radialRelative->setRadius( 3000 );
    radialRelative->setNumSpokes(60);    
    losGroup->addChild( radialRelative );
    RadialLineOfSightEditor* radialRelEditor = new RadialLineOfSightEditor( radialRelative );
    losGroup->addChild( radialRelEditor );

    //Load a plane model.  
    osg::ref_ptr< osg::Node >  plane = osgDB::readRefNodeFile("../data/cessna.osgb.5,5,5.scale");

    //Create 2 moving planes
    osg::Node* plane1 = createPlane(plane.get(), GeoPoint(geoSRS, -121.656, 46.0935, 4133.06, ALTMODE_ABSOLUTE), mapSRS, 5000, 20);
    osg::Node* plane2 = createPlane(plane.get(), GeoPoint(geoSRS, -121.321, 46.2589, 1390.09, ALTMODE_ABSOLUTE), mapSRS, 3000, 5);
    root->addChild( plane1 );
    root->addChild( plane2 );

    //Create a LineOfSightNode that will use a LineOfSightTether callback to monitor
    //the two plane's positions and recompute the LOS when they move
    LinearLineOfSightNode* tetheredLOS = new LinearLineOfSightNode( mapNode);
    losGroup->addChild( tetheredLOS );
    tetheredLOS->setUpdateCallback( new LineOfSightTether( plane1, plane2 ) );

    //Create another plane and attach a RadialLineOfSightNode to it using the RadialLineOfSightTether
    osg::Node* plane3 = createPlane(plane.get(), GeoPoint(geoSRS, -121.463, 46.3548, 1348.71, ALTMODE_ABSOLUTE), mapSRS, 10000, 5);
    losGroup->addChild( plane3 );
    RadialLineOfSightNode* tetheredRadial = new RadialLineOfSightNode( mapNode );
    tetheredRadial->setRadius( 5000 );

    //This RadialLineOfSightNode is going to be filled, so set some alpha values for the colors so it's partially transparent
    tetheredRadial->setFill( true );
    tetheredRadial->setGoodColor( osg::Vec4(0,1,0,0.3) );
    tetheredRadial->setBadColor( osg::Vec4(1,0,0,0.3) );
    tetheredRadial->setNumSpokes( 100 );
    losGroup->addChild( tetheredRadial );
    tetheredRadial->setUpdateCallback( new RadialLineOfSightTether( plane3 ) );

    osgEarth::Viewpoint vp;
    vp.name() = "Mt Ranier";
    vp.focalPoint()->set(geoSRS, -121.488, 46.2054, 0, ALTMODE_ABSOLUTE);
    vp.pitch() = -50.0;
    vp.range() = 100000;

    manip->setHomeViewpoint( vp );

    viewer.setSceneData( root );    

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    /*
    TerrainTileNodeVisitor v;
    root->accept(v);
    */

    GeoPoint center(geoSRS, -121.656, 46.0935, 4133.06, ALTMODE_ABSOLUTE);

    GeoExtent extent(geoSRS, center.x() - 0.5, center.y() - 0.5, center.x() + 0.5, center.y() + 0.5);
    CacheExtentNodeVisitor v(extent);

    root->accept(v);

    while (!viewer.done())
    {        
        viewer.frame();
    }
    return 0;
}
