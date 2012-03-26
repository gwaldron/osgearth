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

#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarthUtil/MGRSFormatter>
#include <osgEarthUtil/MouseCoordsTool>

#include <osgEarthAnnotation/AnnotationData>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/Decluttering>

#include <osgEarth/XmlUtils>
#include <osgEarth/StringUtils>

#include <osgEarthDrivers/kml/KML>

#define KML_PUSHPIN_URL "http://demo.pelicanmapping.com/icons/pushpin_yellow.png"

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Symbology;
using namespace osgEarth::Annotation;

//------------------------------------------------------------------------

/** Shared event handlers. */
namespace
{
    // flies to a viewpoint in response to control event (click)
    struct ClickViewpointHandler : public ControlEventHandler
    {
        ClickViewpointHandler( const Viewpoint& vp, osgGA::CameraManipulator* manip ) 
            : _vp(vp), _manip( dynamic_cast<EarthManipulator*>(manip) ) { }

        Viewpoint         _vp;
        EarthManipulator* _manip;

        virtual void onClick( class Control* control )
        {
            if ( _manip )
                _manip->setViewpoint( _vp, 4.5 );
        }
    };


    // toggles a node in response to a control event (checkbox)
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
}

//------------------------------------------------------------------------

namespace
{
    struct ViewpointHandler : public osgGA::GUIEventHandler
    {
        ViewpointHandler( const std::vector<Viewpoint>& viewpoints, osgViewer::View* view )
            : _viewpoints( viewpoints ),
              _manip( dynamic_cast<EarthManipulator*>(view->getCameraManipulator()) ) { }

        bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
        {
            if ( ea.getEventType() == ea.KEYDOWN )
            {
                int index = (int)ea.getKey() - (int)'1';
                if ( index >= 0 && index < (int)_viewpoints.size() )
                {
                    _manip->setViewpoint( _viewpoints[index], 4.5 );
                }
                else if ( ea.getKey() == 'v' )
                {
                    XmlDocument xml( _manip->getViewpoint().getConfig() );
                    xml.store( std::cout );
                    std::cout << std::endl;
                }
                aa.requestRedraw();
            }
            return false;
        }

        std::vector<Viewpoint> _viewpoints;
        EarthManipulator*      _manip;
    };
}


Control*
ViewpointControlFactory::create(const std::vector<Viewpoint>& viewpoints,
                                osgViewer::View*              view) const
{
    Grid* grid = 0L;

    if ( viewpoints.size() > 0 )
    {
        // the viewpoint container:
        grid = new Grid();
        grid->setChildSpacing( 0 );
        grid->setChildVertAlign( Control::ALIGN_CENTER );

        for( unsigned i=0; i<viewpoints.size(); ++i )
        {
            const Viewpoint& vp = viewpoints[i];
            Control* num = new LabelControl(Stringify() << (i+1), 16.0f, osg::Vec4f(1,1,0,1));
            num->setPadding( 4 );
            grid->setControl( 0, i, num );

            Control* vpc = new LabelControl(vp.getName().empty() ? "<no name>" : vp.getName(), 16.0f);
            vpc->setPadding( 4 );
            vpc->setHorizFill( true );
            vpc->setActiveColor( Color::Blue );
            vpc->addEventHandler( new ClickViewpointHandler(vp, view->getCameraManipulator()) );
            grid->setControl( 1, i, vpc );
        }

        view->addEventHandler( new ViewpointHandler(viewpoints, view) );
    }

    return grid;
}

//------------------------------------------------------------------------

Control*
MouseCoordsControlFactory::create(MapNode*         mapNode,
                                  osgViewer::View* view     ) const
{
    // readout for coordinates under the mouse   
    LabelControl* readout = new LabelControl();
    readout->setHorizAlign( Control::ALIGN_RIGHT );
    readout->setVertAlign( Control::ALIGN_BOTTOM );

    Formatter* formatter = new LatLongFormatter(LatLongFormatter::FORMAT_DECIMAL_DEGREES);
    MouseCoordsTool* mcTool = new MouseCoordsTool( mapNode );
    mcTool->addCallback( new MouseCoordsLabelCallback(readout, formatter) );
    view->addEventHandler( mcTool );

    return readout;
}

//------------------------------------------------------------------------

namespace
{
    struct SkySliderHandler : public ControlEventHandler
    {
        SkySliderHandler(SkyNode* sky) : _sky(sky) { }

        SkyNode* _sky;

        virtual void onValueChanged( class Control* control, float value )
        {
            _sky->setDateTime( 2011, 3, 6, value );
        }
    };
}

Control*
SkyControlFactory::create(SkyNode*         sky,
                          osgViewer::View* view) const
{
    HBox* skyBox = new HBox();
    skyBox->setChildVertAlign( Control::ALIGN_CENTER );
    skyBox->setChildSpacing( 10 );
    skyBox->setHorizFill( true );

    skyBox->addControl( new LabelControl("Time: ", 16) );

    HSliderControl* skySlider = skyBox->addControl(new HSliderControl( 0.0f, 24.0f, 18.0f ));
    skySlider->setBackColor( Color::Gray );
    skySlider->setHeight( 12 );
    skySlider->setHorizFill( true, 200 );
    skySlider->addEventHandler( new SkySliderHandler(sky) );

    return skyBox;
}

//------------------------------------------------------------------------

namespace
{
    struct ChangeSeaLevel : public ControlEventHandler
    {
        ChangeSeaLevel( OceanSurfaceNode* ocean ) : _ocean(ocean) { }

        OceanSurfaceNode* _ocean;

        virtual void onValueChanged( class Control* control, float value )
        {
            _ocean->options().seaLevel() = value;
            _ocean->dirty();
        }
    };

    struct ChangeLowFeather : public ControlEventHandler
    {
        ChangeLowFeather( OceanSurfaceNode* ocean ) : _ocean(ocean) { }

        OceanSurfaceNode* _ocean;

        virtual void onValueChanged( class Control* control, float value )
        {
            _ocean->options().lowFeatherOffset() = value;
            _ocean->dirty();
        }
    };

    struct ChangeHighFeather : public ControlEventHandler
    {
        ChangeHighFeather( OceanSurfaceNode* ocean ) : _ocean(ocean) { }

        OceanSurfaceNode* _ocean;

        virtual void onValueChanged( class Control* control, float value )
        {
            _ocean->options().highFeatherOffset() = value;
            _ocean->dirty();
        }
    };
}

Control*
OceanControlFactory::create(OceanSurfaceNode* ocean,
                            osgViewer::View*  view   ) const
{
    VBox* main = new VBox();

    HBox* oceanBox1 = main->addControl(new HBox());
    oceanBox1->setChildVertAlign( Control::ALIGN_CENTER );
    oceanBox1->setChildSpacing( 10 );
    oceanBox1->setHorizFill( true );

    oceanBox1->addControl( new LabelControl("Sea Level: ", 16) );

    HSliderControl* mslSlider = oceanBox1->addControl(new HSliderControl( -250.0f, 250.0f, 0.0f ));
    mslSlider->setBackColor( Color::Gray );
    mslSlider->setHeight( 12 );
    mslSlider->setHorizFill( true, 200 );
    mslSlider->addEventHandler( new ChangeSeaLevel(ocean) );

    HBox* oceanBox2 = main->addControl(new HBox());
    oceanBox2->setChildVertAlign( Control::ALIGN_CENTER );
    oceanBox2->setChildSpacing( 10 );
    oceanBox2->setHorizFill( true );

    oceanBox2->addControl( new LabelControl("Low Feather: ", 16) );

    HSliderControl* lfSlider = oceanBox2->addControl(new HSliderControl( -1000.0, 250.0f, -100.0f ));
    lfSlider->setBackColor( Color::Gray );
    lfSlider->setHeight( 12 );
    lfSlider->setHorizFill( true, 200 );
    lfSlider->addEventHandler( new ChangeLowFeather(ocean) );

    HBox* oceanBox3 = main->addControl(new HBox());
    oceanBox3->setChildVertAlign( Control::ALIGN_CENTER );
    oceanBox3->setChildSpacing( 10 );
    oceanBox3->setHorizFill( true );

    oceanBox3->addControl( new LabelControl("High Feather: ", 16) );

    HSliderControl* hfSlider = oceanBox3->addControl(new HSliderControl( -500.0f, 500.0f, -10.0f ));
    hfSlider->setBackColor( Color::Gray );
    hfSlider->setHeight( 12 );
    hfSlider->setHorizFill( true, 200 );
    hfSlider->addEventHandler( new ChangeHighFeather(ocean) );

    return main;
}

//------------------------------------------------------------------------

namespace
{
    struct AnnoControlBuilder : public osg::NodeVisitor
    {
        AnnoControlBuilder(osgViewer::View* view)
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
        {
            _grid = new Grid();
            _grid->setHorizFill( true );
            _grid->setAbsorbEvents( true );
            _grid->setPadding( 5 );
            _grid->setBackColor( Color(Color::Black,0.5) );

            _manip = dynamic_cast<EarthManipulator*>(view->getCameraManipulator());
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
                std::string name = trim(data->getName());
                if ( name.empty() ) name = "<unnamed>";
                LabelControl* label = new LabelControl( name, 14.0f );
                unsigned relDepth = osg::clampAbove(3u, this->getNodePath().size());
                label->setMargin(Gutter(0,0,0,(relDepth-3)*20));
                if ( data->getViewpoint() )
                {
                    label->addEventHandler( new ClickViewpointHandler(*data->getViewpoint(), _manip) );
                    label->setActiveColor( Color::Blue );
                }
                row.push_back( label );
                _grid->addControls( row );
            }
            traverse(node);
        }

        Grid*             _grid;
        EarthManipulator* _manip;
    };
}

Control*
AnnotationGraphControlFactory::create(osg::Node*       graph,
                                      osgViewer::View* view) const
{
    AnnoControlBuilder builder( view );
    if ( graph )
        graph->accept( builder );

    return builder._grid;
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[ExampleMapNodeHelper] "

osg::Node*
ExampleMapNodeHelper::load(osg::ArgumentParser& args,
                           osgViewer::View*     view) const
{
    // read in the Earth file:
    osg::Node* node = osgDB::readNodeFiles( args );
    if ( !node )
    {
        OE_WARN << LC << "Unable to load an earth file from the command line." << std::endl;
        return 0L;
    }

    osg::ref_ptr<MapNode> mapNode = MapNode::findMapNode(node);
    if ( !mapNode.valid() )
    {
        OE_WARN << LC << "Loaded scene graph does not contain a MapNode - aborting" << std::endl;
        return 0L;
    }

    // a root node to hold everything:
    osg::Group* root = new osg::Group();

    root->addChild( mapNode.get() );

    parse( mapNode.get(), args, view, root );
    return root;
}



void
ExampleMapNodeHelper::parse(MapNode*             mapNode,
                            osg::ArgumentParser& args,
                            osgViewer::View*     view,
                            osg::Group*          root ) const
{
    if ( !root )
        root = mapNode;

    // parse out custom example arguments first:

    bool useSky        = args.read("--sky");
    bool useOcean      = args.read("--ocean");

    bool useMGRS       = args.read("--mgrs");
    bool useDMS        = args.read("--dms");
    bool useDD         = args.read("--dd");
    bool useCoords     = args.read("--coords") || useMGRS || useDMS || useDD;

    std::string kmlFile;
    args.read( "--kml", kmlFile );

    // install a canvas for any UI controls we plan to create:
    ControlCanvas* canvas = ControlCanvas::get(view, false);

    Container* llContainer = canvas->addControl( new VBox() );
    llContainer->setBackColor( Color(Color::Black, 0.8) );
    llContainer->setHorizAlign( Control::ALIGN_LEFT );
    llContainer->setVertAlign( Control::ALIGN_BOTTOM );

    // look for external data in the map node:
    const Config& externals = mapNode->externalConfig();
    const Config& skyConf         = externals.child("sky");
    const Config& oceanConf       = externals.child("ocean");
    const Config& annoConf        = externals.child("annotations");
    const Config& declutterConf   = externals.child("decluttering");
    Config        viewpointsConf  = externals.child("viewpoints");

    // backwards-compatibility: read viewpoints at the top level:
    const ConfigSet& old_viewpoints = externals.children("viewpoint");
    for( ConfigSet::const_iterator i = old_viewpoints.begin(); i != old_viewpoints.end(); ++i )
        viewpointsConf.add( *i );

    // Loading a viewpoint list from the earth file:
    if ( !viewpointsConf.empty() )
    {
        std::vector<Viewpoint> viewpoints;

        const ConfigSet& children = viewpointsConf.children();
        if ( children.size() > 0 )
        {
            for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
            {
                viewpoints.push_back( Viewpoint(*i) );
            }
        }

        if ( viewpoints.size() > 0 )
        {
            Control* c = ViewpointControlFactory().create(viewpoints, view);
            if ( c )
                llContainer->addControl( c );
        }
    }

    // Adding a sky model:
    if ( useSky || !skyConf.empty() )
    {
        double hours = skyConf.value( "hours", 12.0 );
        SkyNode* sky = new SkyNode( mapNode->getMap() );
        sky->setDateTime( 2011, 3, 6, hours );
        sky->attach( view );
        root->addChild( sky );
        Control* c = SkyControlFactory().create(sky, view);
        if ( c )
            llContainer->addControl( c );
    }

    // Adding an ocean model:
    if ( useOcean || !oceanConf.empty() )
    {
        OceanSurfaceNode* ocean = new OceanSurfaceNode( mapNode, oceanConf );
        if ( ocean )
        {
            root->addChild( ocean );
            Control* c = OceanControlFactory().create(ocean, view);
            if ( c )
                llContainer->addControl(c);
        }
    }

    // Loading KML from the command line:
    if ( !kmlFile.empty() )
    {
        KMLOptions kml_options;
        kml_options.declutter() = true;
        kml_options.defaultIconImage() = URI( KML_PUSHPIN_URL ).getImage();

        osg::Node* kml = KML::load( URI(kmlFile), mapNode, kml_options );
        if ( kml )
        {
            Control* c = AnnotationGraphControlFactory().create(kml, view);
            if ( c )
            {
                c->setVertAlign( Control::ALIGN_TOP );
                canvas->addControl( c );
            }
            root->addChild( kml );
        }
    }

    // Annotations in the map node externals:
    if ( !annoConf.empty() )
    {
        osg::Group* annotations = 0L;
        AnnotationRegistry::instance()->create( mapNode, annoConf, annotations );
        if ( annotations )
        {
            root->addChild( annotations );
        }
    }

    // Configure the de-cluttering engine for labels and annotations:
    if ( !declutterConf.empty() )
    {
        Decluttering::setOptions( DeclutteringOptions(declutterConf) );
    }

    // Configure the mouse coordinate readout:
    if ( useCoords )
    { 
        LabelControl* readout = new LabelControl();
        readout->setBackColor( Color(Color::Black, 0.8) );
        readout->setHorizAlign( Control::ALIGN_RIGHT );
        readout->setVertAlign( Control::ALIGN_BOTTOM );

        Formatter* formatter = 
            useMGRS ? (Formatter*)new MGRSFormatter(MGRSFormatter::PRECISION_1M, 0L, MGRSFormatter::USE_SPACES) :
            useDMS  ? (Formatter*)new LatLongFormatter(LatLongFormatter::FORMAT_DEGREES_MINUTES_SECONDS) :
            useDD   ? (Formatter*)new LatLongFormatter(LatLongFormatter::FORMAT_DECIMAL_DEGREES) :
            0L;

        MouseCoordsTool* mcTool = new MouseCoordsTool( mapNode );
        mcTool->addCallback( new MouseCoordsLabelCallback(readout, formatter) );
        view->addEventHandler( mcTool );

        canvas->addControl( readout );
    }

    root->addChild( canvas );
}


std::string
ExampleMapNodeHelper::usage() const
{
    return Stringify()
        << "    --sky                : add a sky model\n"
        << "    --ocean              : add an ocean model\n"
        << "    --kml <file.kml>     : load a KML or KMZ file\n"
        << "    --coords             : display map coords under mouse\n"
        << "    --dms                : dispay deg/min/sec coords under mouse\n"
        << "    --dd                 : display decimal degrees coords under mouse\n"
        << "    --mgrs               : show MGRS coords under mouse\n";
}
