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
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/Graticule>
#include <osgEarthUtil/SkyNode>
#include <osgEarthUtil/Viewpoint>

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl;
    OE_NOTICE << "USAGE: osgearth_viewer [--graticule] [--autoclip] file.earth" << std::endl;
    OE_NOTICE << "   --graticule     : displays a lat/long grid in geocentric mode" << std::endl;
    OE_NOTICE << "   --sky           : activates the atmospheric model" << std::endl;
    OE_NOTICE << "   --animateSky    : animates the sun across the sky" << std::endl;
    OE_NOTICE << "   --autoclip      : activates the auto clip-plane handler" << std::endl;
    OE_NOTICE << "   --jump          : automatically jumps to first viewpoint" << std::endl;
        
    return -1;
}
 
osg::Node*
createControlPanel( osgViewer::View* view, const std::vector<Viewpoint>& vps )
{
    ControlCanvas* canvas = new ControlCanvas( view );

    // the outer container:
    Grid* g = new Grid();
    g->setBackColor(0,0,0,0.5);
    g->setMargin( 10 );
    g->setPadding( 10 );
    g->setSpacing( 10 );
    g->setChildVertAlign( Control::ALIGN_CENTER );
    g->setAbsorbEvents( true );
    g->setVertAlign( Control::ALIGN_BOTTOM );

    for( unsigned i=0; i<vps.size(); ++i )
    {
        const Viewpoint& vp = vps[i];
        std::stringstream buf;
        buf << (i+1);
        g->setControl( 0, i, new LabelControl(buf.str(), osg::Vec4f(1,1,0,1)) );
        g->setControl( 1, i, new LabelControl(vp.getName().empty() ? "<no name>" : vp.getName()) );
    }

    canvas->addControl( g );
    return canvas;
}

struct AnimateSunCallback : public osg::NodeCallback
{
    void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        SkyNode* skyNode = static_cast<SkyNode*>(node);
        double hours = fmod( osg::Timer::instance()->time_s()/4.0, 24.0 );
        skyNode->setDateTime( 2011, 6, 6, hours );
        OE_INFO << "TIME: " << hours << std::endl;
    }
};

struct ViewpointHandler : public osgGA::GUIEventHandler
{
    ViewpointHandler( const std::vector<Viewpoint>& viewpoints, EarthManipulator* manip )
        : _viewpoints( viewpoints ), _manip( manip ){ }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == ea.KEYDOWN )
        {
            int index = (int)ea.getKey() - (int)'1';
            if ( index >= 0 && index < (int)_viewpoints.size() )
            {
                _manip->setViewpoint( _viewpoints[index], 3.0 );
            }
            else if ( ea.getKey() == 'v' )
            {
                Viewpoint vp = _manip->getViewpoint();
                OE_NOTICE << vp.getConfig().toString() << std::endl;
            }
        }
        return false;
    }

    std::vector<Viewpoint> _viewpoints;
    EarthManipulator* _manip;
};

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    bool useGraticule = arguments.read( "--graticule" );
    bool useAutoClip  = arguments.read( "--autoclip" );
    bool animateSky   = arguments.read( "--animateSky");
    bool useSky       = arguments.read( "--sky" ) || animateSky;
    bool jump         = arguments.read( "--jump" );

    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
        return usage( "Unable to load earth model." );

    osgViewer::Viewer viewer(arguments);
    
    EarthManipulator* manip = new EarthManipulator();
    viewer.setCameraManipulator( manip );

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
                viewer.getCamera()->addEventCallback( new AutoClipPlaneCallback() );

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
                sky->setDateTime( 2011, 3, 6, 12.0 );
                sky->attach( &viewer );
                root->addChild( sky );
                if (animateSky)
                {
                    sky->setUpdateCallback( new AnimateSunCallback());
                }
            }
        }

        // read in viewpoints, if any
        std::vector<Viewpoint> viewpoints;
        const Config& conf = mapNode->externalConfig();
        const ConfigSet children = conf.children("viewpoint");
        for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
            viewpoints.push_back( Viewpoint(*i) );

        viewer.addEventHandler( new ViewpointHandler(viewpoints, manip) );
        if ( viewpoints.size() > 0 && jump )
            manip->setViewpoint(viewpoints[0]);

        if ( viewpoints.size() > 0 )
            root->addChild( createControlPanel(&viewer, viewpoints) );
    }

    // osgEarth benefits from pre-compilation of GL objects in the pager. In newer versions of
    // OSG, this activates OSG's IncrementalCompileOpeartion in order to avoid frame breaks.
    viewer.getDatabasePager()->setDoPreCompile( true );

    viewer.setSceneData( root );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    return viewer.run();
}
