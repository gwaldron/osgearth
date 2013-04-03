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
#include <osgEarthUtil/FeatureQueryTool>

#define LC "[feature_query] "

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

//------------------------------------------------------------------------
// creaes a simple user interface for the manip demo
Control*
createUI()
{
    VBox* vbox = new VBox();
    vbox->setVertAlign( Control::ALIGN_TOP );
    vbox->setHorizAlign( Control::ALIGN_LEFT );
    vbox->addControl( new LabelControl("Feature Query Demo", Color::Yellow) );
    vbox->addControl( new LabelControl("Click on a feature to see its attributes.") );
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
            FeatureQueryTool* tool = new FeatureQueryTool( mapNode );
            viewer.addEventHandler( tool );

            VBox* readout = ControlCanvas::get(&viewer)->addControl( new VBox() );
            readout->setHorizAlign( Control::ALIGN_RIGHT );
            readout->setBackColor( Color(Color::Black,0.8) );
            tool->addCallback( new FeatureReadoutCallback(readout) );
            tool->addCallback( new FeatureHighlightCallback() );
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
