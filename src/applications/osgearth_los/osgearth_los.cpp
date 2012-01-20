/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/XmlUtils>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/LineOfSight>
#include <osg/io_utils>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;


osg::AnimationPath* createAnimationPath( MapNode* mapNode, const osg::Vec3& center, float radius,double looptime)
{
    // set up the animation path 
    osg::AnimationPath* animationPath = new osg::AnimationPath;
    animationPath->setLoopMode(osg::AnimationPath::LOOP);
    
    int numSamples = 40;

    double delta = osg::PI * 2.0 / (double)numSamples;

    //Get the center point in geocentric
    osg::Vec3d centerWorld;
    mapNode->getMap()->mapPointToWorldPoint( center, centerWorld );

    osg::Vec3d up = centerWorld;
    up.normalize();

    //Get the "side" vector
    osg::Vec3d side = up ^ osg::Vec3d(0,0,1);


    double time=0.0f;
    double time_delta = looptime/(double)numSamples;

    osg::Vec3d firstPosition;
    osg::Quat firstRotation;

    for (unsigned int i = 0; i < numSamples; i++)
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

osg::Node* createPlane(osg::Node* node, MapNode* mapNode, const osg::Vec3d& center, double radius, double time)
{
    osg::MatrixTransform* positioner = new osg::MatrixTransform;
    positioner->addChild( node);
    osg::AnimationPath* animationPath = createAnimationPath(mapNode, center, radius, time);
    positioner->setUpdateCallback( new osg::AnimationPathCallback(animationPath, 0.0, 1.0));
    return positioner;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

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
    viewer.setCameraManipulator( manip );
    
    root->addChild( earthNode );    
    viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode->getMap()) );

    //Create a point to point LineOfSightNode.
    LineOfSightNode* los = new LineOfSightNode( mapNode, osg::Vec3d(-121.665, 46.0878, 1258.00), osg::Vec3d(-121.488, 46.2054, 3620.11));
    root->addChild( los );
    los->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    
    //Create an editor for the point to point line of sight that allows you to drag the beginning and end points around.
    //This is just one way that you could manipulator the LineOfSightNode.
    LineOfSightEditor* p2peditor = new LineOfSightEditor( los );
    root->addChild( p2peditor );

    //Create a relative point to point LineOfSightNode.
    LineOfSightNode* relativeLOS = new LineOfSightNode( mapNode, osg::Vec3d(-121.2, 46.1, 10), osg::Vec3d(-121.488, 46.2054, 10));
    relativeLOS->setAltitudeMode( ALTITUDE_RELATIVE );
    root->addChild( relativeLOS );
    relativeLOS->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    LineOfSightEditor* relEditor = new LineOfSightEditor( relativeLOS );
    root->addChild( relEditor );
    
    //Create a RadialLineOfSightNode that allows you to do a 360 degree line of sight analysis.
    RadialLineOfSightNode* radial = new RadialLineOfSightNode( mapNode );
    radial->setCenter( osg::Vec3d(-121.515, 46.054, 847.604) );
    radial->setRadius( 2000 );
    radial->setNumSpokes(100);    
    radial->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    root->addChild( radial );
    RadialLineOfSightEditor* radialEditor = new RadialLineOfSightEditor( radial );
    root->addChild( radialEditor );

    //Create a relative RadialLineOfSightNode that allows you to do a 360 degree line of sight analysis.
    RadialLineOfSightNode* radialRelative = new RadialLineOfSightNode( mapNode );
    radialRelative->setCenter( osg::Vec3d(-121.2, 46.054, 10) );
    radialRelative->setAltitudeMode( ALTITUDE_RELATIVE );
    radialRelative->setRadius( 3000 );
    radialRelative->setNumSpokes(60);    
    radialRelative->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    root->addChild( radialRelative );
    RadialLineOfSightEditor* radialRelEditor = new RadialLineOfSightEditor( radialRelative );
    root->addChild( radialRelEditor );


    //Create an editor for the radial line of sight that allows you to drag it around.


    //Load a plane model.  
    osg::ref_ptr< osg::Node >  plane = osgDB::readNodeFile("cessna.osg.5,5,5.scale");

    //Create 2 moving planes
    osg::Node* plane1 = createPlane(plane, mapNode, osg::Vec3d(-121.656, 46.0935, 4133.06), 5000, 20);
    osg::Node* plane2 = createPlane(plane, mapNode, osg::Vec3d(-121.321, 46.2589, 1390.09), 3000, 5);
    root->addChild( plane1  );
    root->addChild( plane2 );

    //Create a LineOfSightNode that will use a LineOfSightTether callback to monitor
    //the two plane's positions and recompute the LOS when they move
    LineOfSightNode* tetheredLOS = new LineOfSightNode( mapNode);
    tetheredLOS->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    root->addChild( tetheredLOS );
    tetheredLOS->setUpdateCallback( new LineOfSightTether( plane1, plane2 ) );

    //Create another plane and attach a RadialLineOfSightNode to it using the RadialLineOfSightTether
    osg::Node* plane3 = createPlane(plane, mapNode, osg::Vec3d( -121.463, 46.3548, 1348.71), 10000, 5);    
    root->addChild( plane3 );
    RadialLineOfSightNode* tetheredRadial = new RadialLineOfSightNode( mapNode );    
    tetheredRadial->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);    
    tetheredRadial->setRadius( 5000 );
    tetheredRadial->setNumSpokes( 100 );
    root->addChild( tetheredRadial );
    tetheredRadial->setUpdateCallback( new RadialLineOfSightTether( plane3 ) );      
      

    // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
    // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
    viewer.getDatabasePager()->setDoPreCompile( true );

    manip->setHomeViewpoint(Viewpoint( "Mt Rainier",        osg::Vec3d( -121.488, 46.2054, 0 ), 0.0, -50, 100000 ));

    viewer.setSceneData( root );    

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
