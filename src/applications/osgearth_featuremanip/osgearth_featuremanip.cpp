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
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/Controls>

#include <osgEarthUtil/FeatureManipTool>

#define LC "[feature_manip] "

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

//------------------------------------------------------------------------

static FeatureManipTool* s_manipTool;

static VBox* s_state_normal;
static VBox* s_state_active;

// Callback to toggle the visibility of the save/cancel buttons based on tool state
struct ToggleUIStateCallback : public FeatureQueryTool::Callback
{
    // called when a valid feature is found under the mouse coords
    virtual void onHit( FeatureSourceIndexNode* index, FeatureID fid, const EventArgs& args )
    {
        s_state_active->setVisible( true );
    }

    // called when no feature is found under the mouse coords
    virtual void onMiss( const EventArgs& args )
    {
        s_state_active->setVisible( false );
    }
};


// Cancels the manipulation when user clicks "cancel"
struct OnCancel : public ControlEventHandler
{
    void onClick( Control* control )
    {
        s_manipTool->cancel();
        s_state_active->setVisible( false );
    }
};


// Commits the manipulation when user clicks "save"
struct OnSave : public ControlEventHandler
{
    void onClick( Control* saveButton )
    {
        s_manipTool->commit();
        s_state_active->setVisible( false );
    }
};


// creaes a simple user interface for the manip demo
Control*
createUI()
{
    VBox* vbox = new VBox();
    vbox->addControl( new LabelControl("Feature Manipulator Demo", Color::Yellow) );

    s_state_normal = vbox->addControl(new VBox());
    s_state_normal->addControl( new LabelControl("Shift-click on a feature to enter edit mode.") );
    
    s_state_active = vbox->addControl(new VBox());
    s_state_active->setVisible( false );
    s_state_active->addControl( new LabelControl("Drag the handles to position or rotation the feature.") );
    
    HBox* buttons = s_state_active->addControl(new HBox());
    
    LabelControl* cancel = buttons->addControl(new LabelControl("cancel"));
    cancel->setBackColor(Color(Color::White,0.5));
    cancel->setActiveColor(Color::Blue);
    cancel->addEventHandler(new OnCancel());
    cancel->setPadding( 5.0f );
    cancel->setVertFill( true );

    LabelControl* save = buttons->addControl(new LabelControl("save"));
    save->setBackColor(Color(Color::White,0.5));
    save->setActiveColor(Color::Blue);
    save->addEventHandler(new OnSave());
    save->setPadding( 5.0f );
    save->setMargin(Control::SIDE_LEFT, 20.0f);
    save->setVertFill( true );

    vbox->setMargin( Control::SIDE_BOTTOM, 15.0f );
    return vbox;
} 

//------------------------------------------------------------------------

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--stencil") )
        osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    // a basic OSG viewer
    osgViewer::Viewer viewer(arguments);

    // install our default manipulator (do this before using MapNodeHelper)
    viewer.setCameraManipulator( new EarthManipulator() );

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags
    osg::Group* root = MapNodeHelper().load( arguments, &viewer, createUI() );
    if ( root )
    {
        viewer.setSceneData( root );

        // configure the near/far so we don't clip things that are up close
        viewer.getCamera()->setNearFarRatio(0.00002);

        // add some stock OSG handlers:
        viewer.addEventHandler(new osgViewer::StatsHandler());
        viewer.addEventHandler(new osgViewer::WindowSizeHandler());
        viewer.addEventHandler(new osgViewer::ThreadingHandler());
        viewer.addEventHandler(new osgViewer::LODScaleHandler());
        viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

        MapNode* mapNode = MapNode::findMapNode( root );
        if ( mapNode )
        {
            // install the Feature Manipulation tool.
            s_manipTool = new FeatureManipTool( mapNode, true );
            viewer.addEventHandler( s_manipTool );

            s_manipTool->addCallback( new ToggleUIStateCallback() );
        }

        return viewer.run();
    }
    else
    {
        OE_NOTICE 
            << "\nUsage: " << argv[0] << " file.earth" << std::endl
            << MapNodeHelper().usage() << std::endl;
    }
}
