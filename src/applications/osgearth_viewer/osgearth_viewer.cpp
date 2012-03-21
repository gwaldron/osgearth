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
#include <osgEarth/Viewpoint>

#include <osgEarthSymbology/Color>

#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/AnnotationData>
#include <osgEarthAnnotation/Decluttering>

#include <osgEarthDrivers/kml/KML>
#include <osgEarthDrivers/ocean_surface/OceanSurface>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/SkyNode>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarthUtil/MouseCoordsTool>
#include <osgEarthUtil/FeatureQueryTool>

#define LC "[viewer] "

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;
using namespace osgEarth::Annotation;

int
usage( const std::string& msg )
{
    OE_NOTICE << msg << std::endl;
    OE_NOTICE << std::endl;
    OE_NOTICE << "USAGE: osgearth_viewer [options] file.earth" << std::endl;
    OE_NOTICE << "   --sky           : activates the atmospheric model" << std::endl;
    OE_NOTICE << "   --ocean         : activates the ocean surface model" << std::endl;
    OE_NOTICE << "   --noautoclip    : deactivates the auto clip-plane handler" << std::endl;
    OE_NOTICE << "   --dms           : format coordinates as degrees/minutes/seconds" << std::endl;
    OE_NOTICE << "   --mgrs          : format coordinates as MGRS" << std::endl;
    
    return -1;
}

static EarthManipulator* s_manip         =0L;
static Control*          s_controlPanel  =0L;
static SkyNode*          s_sky           =0L;
static OceanSurfaceNode* s_ocean         =0L;

struct SkySliderHandler : public ControlEventHandler
{
    virtual void onValueChanged( class Control* control, float value )
    {
        s_sky->setDateTime( 2011, 3, 6, value );
    }
};

struct ChangeSeaLevel : public ControlEventHandler
{
    virtual void onValueChanged( class Control* control, float value )
    {
        s_ocean->options().seaLevel() = value;
        s_ocean->dirty();
    }
};

struct ChangeLowFeather : public ControlEventHandler
{
    virtual void onValueChanged( class Control* control, float value )
    {
        s_ocean->options().lowFeatherOffset() = value;
        s_ocean->dirty();
    }
};

struct ChangeHighFeather : public ControlEventHandler
{
    virtual void onValueChanged( class Control* control, float value )
    {
        s_ocean->options().highFeatherOffset() = value;
        s_ocean->dirty();
    }
};

struct ToggleNodeHandler : public ControlEventHandler
{
    ToggleNodeHandler( osg::Node* node ) : _node(node) { }

    virtual void onValueChanged( class Control* control, bool value )
    {
        osg::ref_ptr<osg::Node> safeNode = _node.get();
        if ( safeNode.valid() )
            safeNode->setNodeMask( value ? ~0 : 0 );
    }

    osg::observer_ptr<osg::Node> _node;
};

struct ClickViewpointHandler : public ControlEventHandler
{
    ClickViewpointHandler( const Viewpoint& vp ) : _vp(vp) { }
    Viewpoint _vp;

    virtual void onClick( class Control* control )
    {
        s_manip->setViewpoint( _vp, 4.5 );
    }
};

void
createControlPanel( osgViewer::View* view, std::vector<Viewpoint>& vps )
{
    ControlCanvas* canvas = ControlCanvas::get( view );

    VBox* main = new VBox();
    main->setBackColor(0,0,0,0.5);
    main->setMargin( 10 );
    main->setPadding( 10 );
    main->setChildSpacing( 10 );
    main->setAbsorbEvents( true );
    main->setVertAlign( Control::ALIGN_BOTTOM );

    if ( vps.size() > 0 )
    {
        // the viewpoint container:
        Grid* g = new Grid();
        g->setChildSpacing( 0 );
        g->setChildVertAlign( Control::ALIGN_CENTER );

        unsigned i;
        for( i=0; i<vps.size(); ++i )
        {
            const Viewpoint& vp = vps[i];
            std::stringstream buf;
            buf << (i+1);
            std::string str;
            str = buf.str();
            Control* num = new LabelControl(str, 16.0f, osg::Vec4f(1,1,0,1));
            num->setPadding( 4 );
            g->setControl( 0, i, num );

            Control* vpc = new LabelControl(vp.getName().empty() ? "<no name>" : vp.getName(), 16.0f);
            vpc->setPadding( 4 );
            vpc->setHorizFill( true );
            vpc->setActiveColor( Color::Blue );
            vpc->addEventHandler( new ClickViewpointHandler(vp) );
            g->setControl( 1, i, vpc );
        }
        main->addControl( g );
    }

    // sky time slider:
    if ( s_sky )
    {
        HBox* skyBox = main->addControl(new HBox());
        skyBox->setChildVertAlign( Control::ALIGN_CENTER );
        skyBox->setChildSpacing( 10 );
        skyBox->setHorizFill( true );

        skyBox->addControl( new LabelControl("Time: ", 16) );

        HSliderControl* skySlider = skyBox->addControl(new HSliderControl( 0.0f, 24.0f, 18.0f ));
        skySlider->setBackColor( Color::Gray );
        skySlider->setHeight( 12 );
        skySlider->setHorizFill( true, 200 );
        skySlider->addEventHandler( new SkySliderHandler );
    }

    // ocean sliders:
    if ( s_ocean )
    {
        HBox* oceanBox1 = main->addControl( new HBox() );
        oceanBox1->setChildVertAlign( Control::ALIGN_CENTER );
        oceanBox1->setChildSpacing( 10 );
        oceanBox1->setHorizFill( true );

        oceanBox1->addControl( new LabelControl("Sea Level: ", 16) );

        HSliderControl* mslSlider = oceanBox1->addControl(new HSliderControl( -250.0f, 250.0f, 0.0f ));
        mslSlider->setBackColor( Color::Gray );
        mslSlider->setHeight( 12 );
        mslSlider->setHorizFill( true, 200 );
        mslSlider->addEventHandler( new ChangeSeaLevel() );

        HBox* oceanBox2 = main->addControl(new HBox());
        oceanBox2->setChildVertAlign( Control::ALIGN_CENTER );
        oceanBox2->setChildSpacing( 10 );
        oceanBox2->setHorizFill( true );

        oceanBox2->addControl( new LabelControl("Low Feather: ", 16) );

        HSliderControl* lfSlider = oceanBox2->addControl(new HSliderControl( -1000.0, 250.0f, -100.0f ));
        lfSlider->setBackColor( Color::Gray );
        lfSlider->setHeight( 12 );
        lfSlider->setHorizFill( true, 200 );
        lfSlider->addEventHandler( new ChangeLowFeather() );

        HBox* oceanBox3 = main->addControl(new HBox());
        oceanBox3->setChildVertAlign( Control::ALIGN_CENTER );
        oceanBox3->setChildSpacing( 10 );
        oceanBox3->setHorizFill( true );

        oceanBox3->addControl( new LabelControl("High Feather: ", 16) );

        HSliderControl* hfSlider = oceanBox3->addControl(new HSliderControl( -500.0f, 500.0f, -10.0f ));
        hfSlider->setBackColor( Color::Gray );
        hfSlider->setHeight( 12 );
        hfSlider->setHorizFill( true, 200 );
        hfSlider->addEventHandler( new ChangeHighFeather() );
    }
    
    canvas->addControl( main );

    s_controlPanel = main;
}

/**
 * Visitor that builds a UI control for a loaded KML file.
 */
struct KMLUIBuilder : public osg::NodeVisitor
{
    KMLUIBuilder( ControlCanvas* canvas ) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), _canvas(canvas)
    {
        _grid = new Grid();
        _grid->setAbsorbEvents( true );
        _grid->setPadding( 5 );
        _grid->setVertAlign( Control::ALIGN_TOP );
        _grid->setHorizAlign( Control::ALIGN_LEFT );
        _grid->setBackColor( Color(Color::Black,0.5) );
        _canvas->addControl( _grid );
    }

    void apply( osg::Node& node )
    {
        AnnotationData* data = dynamic_cast<AnnotationData*>( node.getUserData() );
        if ( data )
        {
            ControlVector row;
            CheckBoxControl* cb = new CheckBoxControl( node.getNodeMask() != 0, new ToggleNodeHandler( &node ) );
            cb->setSize( 12, 12 );
            row.push_back( cb );
            std::string name = data->getName().empty() ? "<unnamed>" : data->getName();
            LabelControl* label = new LabelControl( name, 14.0f );
            label->setMargin(Gutter(0,0,0,(this->getNodePath().size()-3)*20));
            if ( data->getViewpoint() )
            {
                label->addEventHandler( new ClickViewpointHandler(*data->getViewpoint()) );
                label->setActiveColor( Color::Blue );
            }
            row.push_back( label );
            _grid->addControls( row );
        }
        traverse(node);
    }

    ControlCanvas* _canvas;
    Grid*          _grid;
};

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
            else if ( ea.getKey() == 'v' )
            {
                XmlDocument xml( s_manip->getViewpoint().getConfig() );
                xml.store( std::cout );
                std::cout << std::endl;
            }
            else if ( ea.getKey() == '?' )
            {
                s_controlPanel->setVisible( !s_controlPanel->visible() );
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

    // reads in a KML file:
    std::string kmlFile;
    arguments.read( "--kml", kmlFile );

    // load the .earth file from the command line.
    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
        return usage( "Unable to load earth model." );
    
    // install our default manipulator:
    s_manip = new EarthManipulator();
    s_manip->getSettings()->setArcViewpointTransitions( true );
    viewer.setCameraManipulator( s_manip );

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );


    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );
    if ( mapNode )
    {
        // look for external data:
        const Config& externals = mapNode->externalConfig();

        if ( mapNode->getMap()->isGeocentric() )
        {
            // Sky model.
            Config skyConf = externals.child( "sky" );
            if ( !skyConf.empty() )
                useSky = true;

            if ( useSky )
            {
                double hours = skyConf.value( "hours", 12.0 );
                s_sky = new SkyNode( mapNode->getMap() );
                s_sky->setDateTime( 2011, 3, 6, hours );
                s_sky->attach( &viewer );
                root->addChild( s_sky );
            }

            // Ocean surface.
            if ( externals.hasChild( "ocean" ) )
                useOcean = true;

            if ( useOcean )
            {
                s_ocean = new OceanSurfaceNode( mapNode, externals.child("ocean") );
                if ( s_ocean )
                    root->addChild( s_ocean );
            }

            // The automatic clip plane generator will adjust the near/far clipping
            // planes based on your view of the horizon. This prevents near clipping issues
            // when you are very close to the ground. If your app never brings a user very
            // close to the ground, you may not need this.
            if ( externals.hasChild("noautoclip") )
            {
                dontUseAutoClip = externals.child("noautoclip").boolValue( dontUseAutoClip );
            }

            if ( useSky || useOcean || !dontUseAutoClip )
            {
                viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode) );
                OE_INFO << LC << "Activated auto-clip callback" << std::endl;
            }
        }

        // read in viewpoints, if any
        std::vector<Viewpoint> viewpoints;
        const ConfigSet children = externals.children("viewpoint");
        if ( children.size() > 0 )
        {
            for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
                viewpoints.push_back( Viewpoint(*i) );
        }
        viewer.addEventHandler( new ViewpointHandler(viewpoints) );

        // read in annotations, if any
        Config annoConf = externals.child("annotations");
        if ( !annoConf.empty() )
        {
            osg::Group* annotations = 0L;
            AnnotationRegistry::instance()->create( mapNode, annoConf, annotations );
            if ( annotations )
            {
                //annotations->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );
                root->addChild( annotations );
            }
        }

        // Configure the de-cluttering engine for labels and annotations:
        const Config& declutterConf = externals.child("decluttering");
        if ( !declutterConf.empty() )
        {
            Decluttering::setOptions( DeclutteringOptions(declutterConf) );
        }

        // Add a control panel to the scene
        root->addChild( ControlCanvas::get( &viewer ) );
        if ( viewpoints.size() > 0 || s_sky || s_ocean )
        {
            createControlPanel(&viewer, viewpoints);
        }

        // Load a KML file if specified
        if ( !kmlFile.empty() )
        {
            KMLOptions kmlo;
            kmlo.declutter() = true;
            kmlo.defaultIconImage() = URI("http://demo.pelicanmapping.com/icons/pushpin_yellow.png").getImage();

            osg::Node* kml = KML::load( URI(kmlFile), mapNode, kmlo );
            if ( kml )
            {
                root->addChild( kml );

                KMLUIBuilder uibuilder( ControlCanvas::get(&viewer) );
                root->accept( uibuilder );                
            }
        }
    }
    
    // readout for coordinates under the mouse   
    LabelControl* mouseCoords = new LabelControl();
    mouseCoords->setHorizAlign( Control::ALIGN_RIGHT );
    mouseCoords->setVertAlign( Control::ALIGN_BOTTOM );
    ControlCanvas::get(&viewer, false)->addControl(mouseCoords);

    Formatter* formatter = new LatLongFormatter(LatLongFormatter::FORMAT_DECIMAL_DEGREES);
    MouseCoordsTool* mcTool = new MouseCoordsTool( mapNode );
    mcTool->addCallback( new MouseCoordsLabelCallback(mouseCoords, formatter) );
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

    // Feature query tool setup:
    VBox* featureQueryContainer = ControlCanvas::get(&viewer, false)->addControl( new VBox() );
    featureQueryContainer->setHorizAlign( Control::ALIGN_RIGHT );
    FeatureQueryTool* queryTool = new FeatureQueryTool(mapNode);
    queryTool->addCallback( new FeatureHighlightCallback() );
    queryTool->addCallback( new FeatureReadoutCallback(featureQueryContainer) );
    viewer.addEventHandler( queryTool );

    return viewer.run();
}
