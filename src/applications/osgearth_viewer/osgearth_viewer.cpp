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
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Graticule>
#include <osgEarthUtil/SkyNode>

using namespace osgEarth::Util;

int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl;
    OE_NOTICE << "USAGE: osgearth_viewer [--graticule] [--autoclip] file.earth" << std::endl;
    OE_NOTICE << "   --graticule     : displays a lat/long grid in geocentric mode" << std::endl;
    OE_NOTICE << "   --sky           : activates the atmospheric model" << std::endl;
    OE_NOTICE << "   --animateSky    : animates the sun across the sky" << std::endl;
    OE_NOTICE << "   --autoclip      : activates the auto clip-plane handler" << std::endl;
        
    return -1;
}

struct AnimateSunCallback : public osg::NodeCallback
{
    AnimateSunCallback(): _longitude(-180), _rate(10) { }

    void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        SkyNode* skyNode = static_cast<SkyNode*>(node);
        osg::Timer_t curr_time = osg::Timer::instance()->tick();

        double elapsedTime = osg::Timer::instance()->delta_s(_lastUpdate, curr_time);
        _longitude += _rate * elapsedTime;
        skyNode->setSunPosition(0, osg::DegreesToRadians(_longitude));
        _lastUpdate = curr_time;
    }

    double _longitude;
    osg::Timer_t _lastUpdate;
    double _rate; //Rate in degrees per second
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    bool useGraticule = arguments.read( "--graticule" );
    bool useAutoClip  = arguments.read( "--autoclip" );
    bool useSky       = arguments.read( "--sky" );
    bool animateSky   = arguments.read( "--animateSky");

    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
        return usage( "Unable to load earth model." );

    osgViewer::Viewer viewer(arguments);

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );

    // create a graticule and clip plane handler.
    Graticule* graticule = 0L;
    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if ( mapNode )
    {
        if ( mapNode->getMap()->isGeocentric() )
        {
            // the AutoClipPlaneHandler will automatically adjust the near/far clipping
            // planes based on your view of the horizon. This prevents near clipping issues
            // when you are very close to the ground. If your app never brings a user very
            // close to the ground, you may not need this.
            if ( useAutoClip )
                viewer.addEventHandler( new AutoClipPlaneHandler );

            // the Graticule is a lat/long grid that overlays the terrain. It only works
            // in a round-earth geocentric terrain.
            if ( useGraticule )
            {
                graticule = new Graticule( mapNode->getMap() );
                root->addChild( graticule );
            }

            if ( useSky )
            {
                SkyNode* sky = new SkyNode( mapNode->getMap() );
                sky->setSunPosition( osg::Vec3(0,-1,0) );
                sky->attach( &viewer );
                root->addChild( sky );
                if (animateSky)
                {
                    sky->setUpdateCallback( new AnimateSunCallback());
                }
            }
        }
    }

    // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
    // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
    viewer.getDatabasePager()->setDoPreCompile( true );

    viewer.setSceneData( root );

    EarthManipulator* manip = new EarthManipulator();
    manip->getSettings()->setMinMaxDistance( 250.0, DBL_MAX );
    viewer.setCameraManipulator( manip );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    return viewer.run();
}
