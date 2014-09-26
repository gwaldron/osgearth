/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/DataScanner>
#include <osgEarthUtil/Sky>
#include <osgEarthUtil/Ocean>
#include <osgEarthUtil/Shadowing>
#include <osgEarthUtil/ActivityMonitorTool>
#include <osgEarthUtil/LogarithmicDepthBuffer>

#include <osgEarthUtil/NormalMap>
#include <osgEarthUtil/DetailTexture>
#include <osgEarthUtil/LODBlending>
#include <osgEarthUtil/VerticalScale>
#include <osgEarthUtil/ContourMap>
#include <osgEarthUtil/TextureSplatter>

#include <osgEarthAnnotation/AnnotationData>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarth/Decluttering>

#include <osgEarth/XmlUtils>
#include <osgEarth/StringUtils>

#include <osgEarthDrivers/kml/KML>

#include <osgDB/FileNameUtils>
#include <osgDB/WriteFile>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>

#define KML_PUSHPIN_URL "http://demo.pelicanmapping.com/icons/pushpin_yellow.png"

#define VP_MIN_DURATION      2.0     // minimum fly time.
#define VP_METERS_PER_SECOND 2500.0  // fly speed
#define VP_MAX_DURATION      2.0 //8.0     // maximum fly time.

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Symbology;
using namespace osgEarth::Annotation;
using namespace osgEarth::Drivers;

//------------------------------------------------------------------------

/** Shared event handlers. */
namespace
{
    void flyToViewpoint(EarthManipulator* manip, const Viewpoint& vp)
    {
        Viewpoint currentVP = manip->getViewpoint();
        GeoPoint vp0(currentVP.getSRS(), currentVP.getFocalPoint(), ALTMODE_ABSOLUTE);
        GeoPoint vp1(vp.getSRS(), vp.getFocalPoint(), ALTMODE_ABSOLUTE);
        double distance = vp0.distanceTo(vp1);
        double duration = osg::clampBetween(distance / VP_METERS_PER_SECOND, VP_MIN_DURATION, VP_MAX_DURATION);
        manip->setViewpoint( vp, duration );
    }


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
                flyToViewpoint(_manip, _vp);
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

    // sets a user-specified uniform.
    struct ApplyValueUniform : public ControlEventHandler
    {
        osg::ref_ptr<osg::Uniform> _u;
        ApplyValueUniform(osg::Uniform* u) :_u(u) { }
        void onValueChanged(Control* c, double value) {
            _u->set( float(value) );
        }
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
                if ( !_viewpoints.empty() )
                {
                    int index = (int)ea.getKey() - (int)'1';
                    if ( index >= 0 && index < (int)_viewpoints.size() )
                    {
                        flyToViewpoint( _manip, _viewpoints[index] );
                    }
                }
                if ( ea.getKey() == 'v' )
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
    }

    view->addEventHandler( new ViewpointHandler(viewpoints, view) );

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
    struct SkyTimeSliderHandler : public ControlEventHandler
    {
        SkyTimeSliderHandler(SkyNode* sky) : _sky(sky)  { }

        SkyNode* _sky;

        virtual void onValueChanged( class Control* control, float value )
        {
            DateTime d = _sky->getDateTime();
            _sky->setDateTime(DateTime(d.year(), d.month(), d.day(), value));
        }
    };

//#undef USE_AMBIENT_SLIDER
#define USE_AMBIENT_SLIDER 1

#ifdef USE_AMBIENT_SLIDER
    struct AmbientBrightnessHandler : public ControlEventHandler
    {
        AmbientBrightnessHandler(SkyNode* sky) : _sky(sky) { }

        SkyNode* _sky;

        virtual void onValueChanged( class Control* control, float value )
        {
            _sky->getSunLight()->setAmbient(osg::Vec4(value,value,value,1));
        }
    };
#endif

    struct AnimateSkyUpdateCallback : public osg::NodeCallback
    {    
        /**
        * Creates an AnimateSkyCallback.  
        * @param rate    The time multipler from real time.  Default of 1440 means 1 minute real time will equal 1 day simulation time.
        */
        AnimateSkyUpdateCallback( double rate = 1440 ):
    _rate( rate ),
        _prevTime( -1 ),
        _accumTime( 0.0 )
    {
    }

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {             
        SkyNode* sky = dynamic_cast< SkyNode* >( node );
        if (sky)
        {            
            double time = nv->getFrameStamp()->getSimulationTime();            
            if (_prevTime > 0)
            {                
                TimeStamp t = sky->getDateTime().asTimeStamp();                  
                double delta = ceil((time - _prevTime) * _rate);
                _accumTime += delta;
                // The time stamp only works in seconds so we wait until we've accumulated at least 1 second to change the date.
                if (_accumTime > 1.0)
                {
                    double deltaS = floor(_accumTime );                    
                    _accumTime -= deltaS;
                    t += deltaS;
                    sky->setDateTime( t );                        
                }                
            }            
            _prevTime = time;
        }
        traverse( node, nv );
    }

    double _accumTime;
    double _prevTime;    
    double _rate;
    };

}

Control*
SkyControlFactory::create(SkyNode*         sky,
                          osgViewer::View* view) const
{
    Grid* grid = new Grid();
    grid->setChildVertAlign( Control::ALIGN_CENTER );
    grid->setChildSpacing( 10 );
    grid->setHorizFill( true );

    grid->setControl( 0, 0, new LabelControl("Time (Hours UTC): ", 16) );

    DateTime dt = sky->getDateTime();

    HSliderControl* skySlider = grid->setControl(1, 0, new HSliderControl( 0.0f, 24.0f, dt.hours() ));
    skySlider->setHorizFill( true, 300 );
    skySlider->addEventHandler( new SkyTimeSliderHandler(sky) );

    grid->setControl(2, 0, new LabelControl(skySlider) );

#ifdef USE_AMBIENT_SLIDER
    grid->setControl(0, 1, new LabelControl("Min.Ambient: ", 16) );
    HSliderControl* ambient = grid->setControl(1, 1, new HSliderControl(0.0f, 1.0f, sky->getSunLight()->getAmbient().r()));
    ambient->addEventHandler( new AmbientBrightnessHandler(sky) );
    grid->setControl(2, 1, new LabelControl(ambient) );
#endif

    return grid;
}

//------------------------------------------------------------------------

namespace
{
    struct ChangeSeaLevel : public ControlEventHandler
    {
        ChangeSeaLevel( OceanNode* ocean ) : _ocean(ocean) { }

        OceanNode* _ocean;

        virtual void onValueChanged( class Control* control, float value )
        {
            _ocean->setSeaLevel( value );
        }
    };
}

Control*
OceanControlFactory::create(OceanNode* ocean) const
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

    return main;
}

//------------------------------------------------------------------------

namespace
{
    struct AnnoControlBuilder : public osg::NodeVisitor
    {
        AnnoControlBuilder(osgViewer::View* view)
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
              _mindepth(-1)
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
                int depth = (int)this->getNodePath().size();
                if ( _mindepth < 0 )
                    _mindepth = depth;
                label->setMargin(Gutter(0,0,0,(depth-_mindepth)*20));
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
        int               _mindepth;
    };
}

Control*
AnnotationGraphControlFactory::create(osg::Node*       graph,
                                      osgViewer::View* view) const
{
    AnnoControlBuilder builder( view );
    builder.setNodeMaskOverride(~0);
    if ( graph )
        graph->accept( builder );

    return builder._grid;
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[MapNodeHelper] "

osg::Group*
MapNodeHelper::load(osg::ArgumentParser& args,
                    osgViewer::View*     view,
                    Control*             userControl ) const
{
    // do this first before scanning for an earth file
    std::string outEarth;
    args.read( "--out-earth", outEarth );

    // read in the Earth file:
    osg::Node* node = 0L;
    for( int i=0; i<args.argc(); ++i )
    {
        if ( osgDB::getLowerCaseFileExtension(args[i]) == "earth" )
        {
            node = osgDB::readNodeFile( args[i] );
            args.remove(i);
            break;
        }
    }

    osg::ref_ptr<MapNode> mapNode;
    if ( !node )
    {
        if ( !args.find("--images") )
        {
            OE_WARN << LC << "No earth file." << std::endl;
            return 0L;
        }
        else
        {
            mapNode = new MapNode();
        }
    }
    else
    {
        mapNode = MapNode::get(node);
        if ( !mapNode.valid() )
        {
            OE_WARN << LC << "Loaded scene graph does not contain a MapNode - aborting" << std::endl;
            return 0L;
        }
    }

    // warn about not having an earth manip
    if ( view )
    {
        EarthManipulator* manip = dynamic_cast<EarthManipulator*>(view->getCameraManipulator());
        if ( manip == 0L )
        {
            OE_WARN << LC << "Helper used before installing an EarthManipulator" << std::endl;
        }
    }

    // a root node to hold everything:
    osg::Group* root = new osg::Group();
    
    root->addChild( mapNode.get() );

    // parses common cmdline arguments.
    if ( view )
    {
        parse( mapNode.get(), args, view, root, userControl );
    }

    // Dump out an earth file if so directed.
    if ( !outEarth.empty() )
    {
        OE_NOTICE << LC << "Writing earth file: " << outEarth << std::endl;
        osgDB::writeNodeFile( *mapNode, outEarth );
    }

    // configures the viewer with some stock goodies
    if ( view )
    {
        configureView( view );
    }

    return root;
}


void
MapNodeHelper::parse(MapNode*             mapNode,
                     osg::ArgumentParser& args,
                     osgViewer::View*     view,
                     osg::Group*          root,
                     Control*             userControl ) const
{
    if ( !root )
        root = mapNode;

    // options to use for the load
    osg::ref_ptr<osgDB::Options> dbOptions = Registry::instance()->cloneOrCreateOptions();

    // parse out custom example arguments first:
    bool useSky        = args.read("--sky");
    bool useOcean      = args.read("--ocean");
    bool useMGRS       = args.read("--mgrs");
    bool useDMS        = args.read("--dms");
    bool useDD         = args.read("--dd");
    bool useCoords     = args.read("--coords") || useMGRS || useDMS || useDD;
    bool useOrtho      = args.read("--ortho");
    bool useAutoClip   = args.read("--autoclip");
    bool useShadows    = args.read("--shadows");
    bool animateSky    = args.read("--animate-sky");
    bool showActivity  = args.read("--activity");
    bool useLogDepth   = args.read("--logdepth");

    if (args.read("--verbose"))
        osgEarth::setNotifyLevel(osg::INFO);
    
    if (args.read("--quiet"))
        osgEarth::setNotifyLevel(osg::FATAL);

    float ambientBrightness = 0.2f;
    args.read("--ambientBrightness", ambientBrightness);

    std::string kmlFile;
    args.read( "--kml", kmlFile );

    std::string imageFolder;
    args.read( "--images", imageFolder );

    std::string imageExtensions;
    args.read("--image-extensions", imageExtensions);
    
    // animation path:
    std::string animpath;
    if ( args.read("--path", animpath) )
    {
        view->setCameraManipulator( new osgGA::AnimationPathManipulator(animpath) );
    }

    // Install a new Canvas for our UI controls, or use one that already exists.
    ControlCanvas* canvas = ControlCanvas::getOrCreate( view );

    Container* mainContainer = canvas->addControl( new VBox() );
    mainContainer->setAbsorbEvents( true );
    mainContainer->setBackColor( Color(Color::Black, 0.8) );
    mainContainer->setHorizAlign( Control::ALIGN_LEFT );
    mainContainer->setVertAlign( Control::ALIGN_BOTTOM );

    // install the user control:
    if ( userControl )
        mainContainer->addControl( userControl );

    // look for external data in the map node:
    const Config& externals = mapNode->externalConfig();

    const Config& skyConf         = externals.child("sky");
    const Config& oceanConf       = externals.child("ocean");
    const Config& annoConf        = externals.child("annotations");
    const Config& declutterConf   = externals.child("decluttering");
    Config        viewpointsConf  = externals.child("viewpoints");

    // some terrain effects.
    const Config& normalMapConf   = externals.child("normal_map");
    const Config& detailTexConf   = externals.child("detail_texture");
    const Config& lodBlendingConf = externals.child("lod_blending");
    const Config& vertScaleConf   = externals.child("vertical_scale");
    const Config& contourMapConf  = externals.child("contour_map");
    const Config& texSplatConf    = externals.child("texture_splatter");

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
                mainContainer->addControl( c );
        }
    }

    // Adding a sky model:
    if ( useSky || !skyConf.empty() )
    {
        SkyOptions options(skyConf);
        if ( options.getDriver().empty() )
        {
            if ( mapNode->getMapSRS()->isGeographic() )
                options.setDriver("simple");
            else
                options.setDriver("gl");
        }

        SkyNode* sky = SkyNode::create(options, mapNode);
        if ( sky )
        {
            sky->attach( view, 0 );
            if ( mapNode->getNumParents() > 0 )
            {
                osgEarth::insertGroup(sky, mapNode->getParent(0));
            }
            else
            {
                sky->addChild( mapNode );
                root = sky;
            }
                
            Control* c = SkyControlFactory().create(sky, view);
            if ( c )
                mainContainer->addControl( c );

            if (animateSky)
            {
                sky->setUpdateCallback( new AnimateSkyUpdateCallback() );
            }

        }
    }

    // Adding an ocean model:
    if ( useOcean || !oceanConf.empty() )
    {
        OceanNode* ocean = OceanNode::create(OceanOptions(oceanConf), mapNode);
        if ( ocean )
        {
            // if there's a sky, we want to ocean under it
            osg::Group* parent = osgEarth::findTopMostNodeOfType<SkyNode>(root);
            if ( !parent ) parent = root;
            parent->addChild( ocean );

            Control* c = OceanControlFactory().create(ocean);
            if ( c )
                mainContainer->addControl(c);
        }
    }

    // Shadowing.
    if ( useShadows )
    {
        ShadowCaster* caster = new ShadowCaster();
        caster->setLight( view->getLight() );
        caster->getShadowCastingGroup()->addChild( mapNode->getModelLayerGroup() );
        if ( mapNode->getNumParents() > 0 )
        {
            insertGroup(caster, mapNode->getParent(0));
        }
        else
        {
            caster->addChild(mapNode);
            root = caster;
        }
    }

    // Loading KML from the command line:
    if ( !kmlFile.empty() )
    {
        KMLOptions kml_options;
        kml_options.declutter() = true;

        // set up a default icon for point placemarks:
        IconSymbol* defaultIcon = new IconSymbol();
        defaultIcon->url()->setLiteral(KML_PUSHPIN_URL);
        kml_options.defaultIconSymbol() = defaultIcon;

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
        AnnotationRegistry::instance()->create( mapNode, annoConf, dbOptions.get(), annotations );
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

    // Configure for an ortho camera:
    if ( useOrtho )
    {
        EarthManipulator* manip = dynamic_cast<EarthManipulator*>(view->getCameraManipulator());
        if ( manip )
        {
            manip->getSettings()->setCameraProjection( EarthManipulator::PROJ_ORTHOGRAPHIC );
        }
    }

    // activity monitor (debugging)
    if ( showActivity )
    {
        VBox* vbox = new VBox();
        vbox->setBackColor( Color(Color::Black, 0.8) );
        vbox->setHorizAlign( Control::ALIGN_RIGHT );
        vbox->setVertAlign( Control::ALIGN_BOTTOM );
        view->addEventHandler( new ActivityMonitorTool(vbox) );
        canvas->addControl( vbox );
    }

    // Install an auto clip plane clamper
    if ( useAutoClip )
    {
        mapNode->addCullCallback( new AutoClipPlaneCullCallback(mapNode) );
    }

    // Install logarithmic depth buffer on main camera
    if ( useLogDepth )
    {
        OE_INFO << LC << "Activating logarithmic depth buffer on main camera" << std::endl;
        osgEarth::Util::LogarithmicDepthBuffer logDepth;
        logDepth.install( view->getCamera() );
    }

    // Scan for images if necessary.
    if ( !imageFolder.empty() )
    {
        std::vector<std::string> extensions;
        if ( !imageExtensions.empty() )
            StringTokenizer( imageExtensions, extensions, ",;", "", false, true );
        if ( extensions.empty() )
            extensions.push_back( "tif" );

        OE_INFO << LC << "Loading images from " << imageFolder << "..." << std::endl;
        ImageLayerVector imageLayers;
        DataScanner scanner;
        scanner.findImageLayers( imageFolder, extensions, imageLayers );

        if ( imageLayers.size() > 0 )
        {
            mapNode->getMap()->beginUpdate();
            for( ImageLayerVector::iterator i = imageLayers.begin(); i != imageLayers.end(); ++i )
            {
                mapNode->getMap()->addImageLayer( i->get() );
            }
            mapNode->getMap()->endUpdate();
        }
        OE_INFO << LC << "...found " << imageLayers.size() << " image layers." << std::endl;
    }

    // Install a normal map layer.
    if ( !normalMapConf.empty() )
    {
        osg::ref_ptr<NormalMap> effect = new NormalMap(normalMapConf, mapNode->getMap());
        if ( effect->getNormalMapLayer() )
        {
            mapNode->getTerrainEngine()->addEffect( effect.get() );
        }
    }

    // Install a detail texturer
    if ( !detailTexConf.empty() )
    {
        osg::ref_ptr<DetailTexture> effect = new DetailTexture(detailTexConf, mapNode->getMap());
        if ( true ) //effect->getImage() )
        {
            mapNode->getTerrainEngine()->addEffect( effect.get() );
        }
    }

    // Install a texture splatter
    if ( !texSplatConf.empty() )
    {
        osg::ref_ptr<TextureSplatter> effect = new TextureSplatter(texSplatConf, mapNode->getMap());
        mapNode->getTerrainEngine()->addEffect( effect.get() );
    }

    // Install elevation morphing
    if ( !lodBlendingConf.empty() )
    {
        mapNode->getTerrainEngine()->addEffect( new LODBlending(lodBlendingConf) );
    }

    // Install vertical scaler
    if ( !vertScaleConf.empty() )
    {
        mapNode->getTerrainEngine()->addEffect( new VerticalScale(vertScaleConf) );
    }

    // Install a contour map effect.
    if ( !contourMapConf.empty() )
    {
        mapNode->getTerrainEngine()->addEffect( new ContourMap(contourMapConf) );
    }

    // Generic named value uniform with min/max.
    VBox* uniformBox = 0L;
    while( args.find( "--uniform" ) >= 0 )
    {
        std::string name;
        float minval, maxval;
        if ( args.read( "--uniform", name, minval, maxval ) )
        {
            if ( uniformBox == 0L )
            {
                uniformBox = new VBox();
                uniformBox->setBackColor(0,0,0,0.5);
                uniformBox->setAbsorbEvents( true );
                canvas->addControl( uniformBox );
            }
            osg::Uniform* uniform = new osg::Uniform(osg::Uniform::FLOAT, name);
            uniform->set( minval );
            root->getOrCreateStateSet()->addUniform( uniform, osg::StateAttribute::OVERRIDE );
            HBox* box = new HBox();
            box->addControl( new LabelControl(name) );
            HSliderControl* hs = box->addControl( new HSliderControl(minval, maxval, minval, new ApplyValueUniform(uniform)));
            hs->setHorizFill(true, 200);
            box->addControl( new LabelControl(hs) );
            uniformBox->addControl( box );
            OE_INFO << LC << "Installed uniform controller for " << name << std::endl;
        }
    }

    root->addChild( canvas );
}


void
MapNodeHelper::configureView( osgViewer::View* view ) const
{
    // add some stock OSG handlers:
    view->addEventHandler(new osgViewer::StatsHandler());
    view->addEventHandler(new osgViewer::WindowSizeHandler());
    view->addEventHandler(new osgViewer::ThreadingHandler());
    view->addEventHandler(new osgViewer::LODScaleHandler());
    view->addEventHandler(new osgGA::StateSetManipulator(view->getCamera()->getOrCreateStateSet()));
    view->addEventHandler(new osgViewer::RecordCameraPathHandler());
}


std::string
MapNodeHelper::usage() const
{
    return Stringify()
        << "  --sky                         : add a sky model\n"
        << "  --ocean                       : add an ocean model\n"
        << "  --kml <file.kml>              : load a KML or KMZ file\n"
        << "  --coords                      : display map coords under mouse\n"
        << "  --dms                         : dispay deg/min/sec coords under mouse\n"
        << "  --dd                          : display decimal degrees coords under mouse\n"
        << "  --mgrs                        : show MGRS coords under mouse\n"
        << "  --ortho                       : use an orthographic camera\n"
        << "  --logdepth                    : activates the logarithmic depth buffer\n"
        << "  --autoclip                    : installs an auto-clip plane callback\n"
        << "  --images [path]               : finds and loads image layers from folder [path]\n"
        << "  --image-extensions [ext,...]  : with --images, extensions to use\n"
        << "  --out-earth [file]            : write the loaded map to an earth file\n"
        << "  --uniform [name] [min] [max]  : create a uniform controller with min/max values\n"
        << "  --path [file]                 : load and playback an animation path\n";
}
