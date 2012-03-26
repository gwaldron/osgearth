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
#include <osgEarth/Viewpoint>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/SkyNode>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarthUtil/MouseCoordsTool>
#include <osgEarthUtil/FeatureManipTool>

#define LC "[featuremanip] "

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Symbology;

int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl;
    OE_NOTICE << std::endl;
    OE_NOTICE << "USAGE: osgearth_featuremanip [options] file.earth" << std::endl;
    
    return -1;
}

static EarthManipulator* s_manip         =0L;
static Control*          s_controlPanel  =0L;
static Container*        s_readout       =0L;
static SkyNode*          s_sky           =0L;


struct ClickViewpointHandler : public ControlEventHandler
{
    ClickViewpointHandler( const Viewpoint& vp ) : _vp(vp) { }
    Viewpoint _vp;
    virtual void onClick( class Control* control ) {
        s_manip->setViewpoint( _vp, 4.5 );
    }
};


void
createControls( osgViewer::View* view, std::vector<Viewpoint>& viewpoints )
{
    ControlCanvas* canvas = ControlCanvas::get(view, false);

    // instructions.
    VBox* help = canvas->addControl(new VBox());
    help->setBackColor(0,0,0,0.8);
    help->setChildSpacing( 8 );
    help->setPadding( 10 );
    help->setVertAlign( Control::ALIGN_TOP );
    help->addControl( new LabelControl("Feature Manipulation Demo", 16.0f, Color::Yellow) );
    help->addControl( new LabelControl("- Shift-click a feature to manipulate;") );
    help->addControl( new LabelControl("- Drag or rotate using the handles.") );

    // list of viewpoints.
    if ( viewpoints.size() > 0 )
    {
        Grid* grid = canvas->addControl(new Grid());
        grid->setVertAlign( Control::ALIGN_BOTTOM );
        grid->setBackColor(0,0,0,0.8);
        grid->setChildSpacing( 0 );
        grid->setPadding( 10 );
        grid->setAbsorbEvents( true );
        grid->setChildVertAlign( Control::ALIGN_CENTER );

        for( unsigned i=0; i<viewpoints.size(); ++i )
        {
            const Viewpoint& vp = viewpoints[i];
            Control* num = new LabelControl( Stringify() << (i+1), 16.0f, osg::Vec4f(1,1,0,1));
            num->setPadding( 4 );
            grid->setControl( 0, i, num );

            Control* vpc = new LabelControl(vp.getName().empty() ? "<no name>" : vp.getName(), 16.0f);
            vpc->setPadding( 4 );
            vpc->setHorizFill( true );
            vpc->setActiveColor( Color::Blue );
            vpc->addEventHandler( new ClickViewpointHandler(vp) );
            grid->setControl( 1, i, vpc );
        }
    }

    // feature attributes readout.
    s_readout = canvas->addControl(new VBox());
    s_readout->setHorizAlign( Control::ALIGN_RIGHT );
}


/**
 * Handler that dumps the current viewpoint out to the console.
 */
struct ViewpointHandler : public osgGA::GUIEventHandler
{
    ViewpointHandler( const std::vector<Viewpoint>& viewpoints )
        : _viewpoints( viewpoints ) { }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if ( ea.getEventType() == ea.KEYDOWN )
        {
            int index = (int)ea.getKey() - (int)'1';
            if ( index >= 0 && index < (int)_viewpoints.size() )
            {
                s_manip->setViewpoint( _viewpoints[index], 4.5 );
            }
            aa.requestRedraw();
        }
        return false;
    }

    std::vector<Viewpoint> _viewpoints;
};

//------------------------------------------------------------------------

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );
    osgViewer::Viewer viewer(arguments);

    bool dontUseAutoClip = arguments.read( "--noautoclip" );
    bool useSky          = arguments.read( "--sky" );
    bool useOcean        = arguments.read( "--ocean" );

    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
        return usage( "Unable to load earth model." );
    
    // install our  manipulator:
    s_manip = new EarthManipulator();
    viewer.setCameraManipulator( s_manip );

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );

    // a root canvas for any control we create.
    root->addChild( ControlCanvas::get( &viewer ) );


    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if ( mapNode )
    {
        // look for external data:
        const Config& externals = mapNode->externalConfig();

        // read in viewpoints, if any
        std::vector<Viewpoint> viewpoints;
        const ConfigSet children = externals.children("viewpoint");
        if ( children.size() > 0 )
        {
            for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
                viewpoints.push_back( Viewpoint(*i) );
        }
        viewer.addEventHandler( new ViewpointHandler(viewpoints) );

        // Add a control panel to the scene
        createControls(&viewer, viewpoints);
    }
    
    // readout for coordinates under the mouse   
    LabelControl* mouseCoords = new LabelControl();
    mouseCoords->setHorizAlign( Control::ALIGN_RIGHT );
    mouseCoords->setVertAlign( Control::ALIGN_BOTTOM );
    ControlCanvas::get(&viewer, false)->addControl(mouseCoords);

    MouseCoordsTool* mcTool = new MouseCoordsTool( mapNode );
    mcTool->addCallback( new MouseCoordsLabelCallback(mouseCoords) );
    viewer.addEventHandler( mcTool );

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

    // Feature manipulator
    FeatureManipTool* manipTool = new FeatureManipTool(mapNode);
    viewer.addEventHandler( manipTool );

    // Feature readout as well
    manipTool->addCallback( new FeatureReadoutCallback(s_readout) );

    return viewer.run();
}
