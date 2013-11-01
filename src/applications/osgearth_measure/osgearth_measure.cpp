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
#include <osgEarth/XmlUtils>
#include <osgEarth/Registry>
#include <osgEarth/Viewpoint>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/SkyNode>
#include <osgEarthUtil/MouseCoordsTool>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSymbology/Color>

#include <osgEarthUtil/MeasureTool>

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Symbology;

class MyMeasureToolCallback : public MeasureToolHandler::MeasureToolEventHandler
{
public:
    MyMeasureToolCallback(LabelControl* label):
      _label(label)
    {
    }

    virtual void onDistanceChanged(MeasureToolHandler* sender, double distance)
    {
        std::stringstream ss;
        ss << "Distance = " << std::setprecision(10) << distance << "m" << std::endl; 
        std::string str;
        str = ss.str();
        _label->setText( str );
    }
    LabelControl* _label;
};

struct TogglePathHandler : public ControlEventHandler
{
    TogglePathHandler( MeasureToolHandler* tool) :
    _tool( tool )
    { }

    virtual void onValueChanged(Control* control, bool value) {
        _tool->setIsPath( value );
    }

    osg::ref_ptr<MeasureToolHandler> _tool;
};

struct ToggleModeHandler : public ControlEventHandler
{
    ToggleModeHandler( MeasureToolHandler* tool) :
    _tool( tool )
    { }

    virtual void onValueChanged(Control* control, bool value) {
        if (_tool->getGeoInterpolation() == GEOINTERP_GREAT_CIRCLE)
        {
            _tool->setGeoInterpolation( GEOINTERP_RHUMB_LINE);
        }
        else
        {
            _tool->setGeoInterpolation( GEOINTERP_GREAT_CIRCLE);
        }        
    }

    osg::ref_ptr<MeasureToolHandler> _tool;
};



int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    osgViewer::Viewer viewer(arguments);

    // load the .earth file from the command line.
    osg::Node* earthNode = MapNodeHelper().load( arguments, &viewer );
    if (!earthNode)
    {
        OE_NOTICE << "Unable to load earth model." << std::endl;
        return 1;
    }

    MapNode* mapNode = MapNode::findMapNode( earthNode );
    if ( !mapNode )
    {
        OE_NOTICE << "Input file was not a .earth file" << std::endl;
        return 1;
    }

    earthNode->setNodeMask( 0x1 );
    
    osgEarth::Util::EarthManipulator* earthManip = new EarthManipulator();
    viewer.setCameraManipulator( earthManip );

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );

    //Create the MeasureToolHandler
    MeasureToolHandler* measureTool = new MeasureToolHandler(root, mapNode);    
    measureTool->setIntersectionMask( 0x1 );
    viewer.addEventHandler( measureTool );

    //Create some controls to interact with the measuretool
    ControlCanvas* canvas = new ControlCanvas( &viewer );
    root->addChild( canvas );
    canvas->setNodeMask( 0x1 << 1 );

    Grid* grid = new Grid();
    grid->setBackColor(0,0,0,0.5);
    grid->setMargin( 10 );
    grid->setPadding( 10 );
    grid->setChildSpacing( 10 );
    grid->setChildVertAlign( Control::ALIGN_CENTER );
    grid->setAbsorbEvents( true );
    grid->setVertAlign( Control::ALIGN_BOTTOM );    

    canvas->addControl( grid );

    //Add a label to display the distance
    // Add a text label:
    grid->setControl( 0, 0, new LabelControl("Distance:") );
    LabelControl* label = new LabelControl();
    label->setFont( osgEarth::Registry::instance()->getDefaultFont() );
    label->setFontSize( 24.0f );
    label->setHorizAlign( Control::ALIGN_LEFT );    
    label->setText("click to measure");
    grid->setControl( 1, 0, label );

    //Add a callback to update the label when the distance changes
    measureTool->addEventHandler( new MyMeasureToolCallback(label) );
    
    Style style = measureTool->getLineStyle();
    style.getOrCreate<LineSymbol>()->stroke()->color() = Color::Red;
    style.getOrCreate<LineSymbol>()->stroke()->width() = 4.0f;
    measureTool->setLineStyle(style);

    //Add a checkbox to control if we are doing path based measurement or just point to point
    grid->setControl( 0, 1, new LabelControl("Path"));
    CheckBoxControl* checkBox = new CheckBoxControl(false);
    checkBox->setHorizAlign(Control::ALIGN_LEFT);
    checkBox->addEventHandler( new TogglePathHandler(measureTool));
    grid->setControl( 1, 1, checkBox);

    //Add a toggle to set the mode of the measuring tool
    grid->setControl( 0, 2, new LabelControl("Great Circle"));
    CheckBoxControl* mode = new CheckBoxControl(true);
    mode->setHorizAlign(Control::ALIGN_LEFT);
    mode->addEventHandler( new ToggleModeHandler(measureTool));
    grid->setControl( 1, 2, mode);

    //Add a mouse coords readout:
    LabelControl* mouseLabel = new LabelControl();
    grid->setControl( 0, 3, new LabelControl("Mouse:"));
    grid->setControl( 1, 3, mouseLabel );
    viewer.addEventHandler(new MouseCoordsTool(mapNode, mouseLabel) );

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
