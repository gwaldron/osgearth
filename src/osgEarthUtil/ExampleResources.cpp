/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
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
#include <osgEarthUtil/ContourMap>

#include <osgEarthUtil/LODBlending>
#include <osgEarthUtil/VerticalScale>

#include <osgEarthAnnotation/AnnotationData>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarth/ScreenSpaceLayout>
#include <osgEarth/TerrainEngineNode>

#include <osgEarth/XmlUtils>
#include <osgEarth/StringUtils>

#include <osgEarthDrivers/kml/KML>

#include <osgDB/FileNameUtils>
#include <osgDB/WriteFile>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>

#define KML_PUSHPIN_URL "../data/placemark32.png"

#define VP_MIN_DURATION      2.0     // minimum fly time.
#define VP_METERS_PER_SECOND 2500.0  // fly speed
#define VP_MAX_DURATION      2.0 //8.0     // maximum fly time.

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Symbology;
using namespace osgEarth::Annotation;
using namespace osgEarth::Drivers;

namespace ui = osgEarth::Util::Controls;

//------------------------------------------------------------------------

/** Shared event handlers. */
namespace
{
    void flyToViewpoint(EarthManipulator* manip, const Viewpoint& vp)
    {
        Viewpoint currentVP = manip->getViewpoint();
        double distance = currentVP.focalPoint()->distanceTo(currentVP.focalPoint().get());
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

    /**
     * Toggles the main control canvas on and off.
     */
    struct ToggleCanvasEventHandler : public osgGA::GUIEventHandler
    {
        ToggleCanvasEventHandler(osg::Node* canvas, char key) :
            _canvas(canvas), _key(key)
        {
        }

        bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
        {
            if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
            {
                if (ea.getKey() == _key)
                {
                    osg::ref_ptr< osg::Node > safeNode = _canvas.get();
                    if (safeNode.valid())
                    {
                        safeNode->setNodeMask( safeNode->getNodeMask() ? 0 : ~0 );
                    }
                    return true;
                }
            }
            return false;
        }

        osg::observer_ptr<osg::Node> _canvas;
        char _key;
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
MapNodeHelper::load(osg::ArgumentParser&  args,
                    osgViewer::View*      view,
                    Container*            userContainer,
                    const osgDB::Options* readOptions) const
{
    // do this first before scanning for an earth file
    std::string outEarth;
    args.read( "--out-earth", outEarth );

    osg::ref_ptr<osgDB::Options> myReadOptions = Registry::cloneOrCreateOptions(readOptions);
    
    Config c;
    c.add("elevation_smoothing", false);
    TerrainOptions to(c);

    MapNodeOptions defMNO;
    defMNO.setTerrainOptions( to );

    myReadOptions->setPluginStringData("osgEarth.defaultOptions", defMNO.getConfig().toJSON());

    // read in the Earth file:
    osg::Node* node = osgDB::readNodeFiles(args, myReadOptions.get());

    osg::ref_ptr<MapNode> mapNode;
    if ( !node )
    {
        if ( args.find("--images") < 0 )
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
    
    root->addChild( node );

    // parses common cmdline arguments.
    if ( view )
    {
        parse( mapNode.get(), args, view, root, userContainer );
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
                     LabelControl*        userLabel ) const
{
    VBox* vbox = new VBox();
    vbox->setAbsorbEvents( true );
    vbox->setBackColor( Color(Color::Black, 0.8) );
    vbox->setHorizAlign( Control::ALIGN_LEFT );
    vbox->setVertAlign( Control::ALIGN_BOTTOM );
    vbox->addControl( userLabel );

    parse(mapNode, args, view, root, vbox);
}

void
MapNodeHelper::parse(MapNode*             mapNode,
                     osg::ArgumentParser& args,
                     osgViewer::View*     view,
                     osg::Group*          root,
                     Container*           userContainer ) const
{
    if ( !root )
        root = mapNode;

    // options to use for the load
    osg::ref_ptr<osgDB::Options> dbOptions = Registry::instance()->cloneOrCreateOptions();

    // parse out custom example arguments first:
    bool useMGRS       = args.read("--mgrs");
    bool useDMS        = args.read("--dms");
    bool useDD         = args.read("--dd");
    bool useCoords     = args.read("--coords") || useMGRS || useDMS || useDD;

    bool useAutoClip   = args.read("--autoclip");
    bool showActivity  = args.read("--activity");
    bool useLogDepth   = args.read("--logdepth");
    bool useLogDepth2  = args.read("--logdepth2");
    bool kmlUI         = args.read("--kmlui");

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

    Container* mainContainer;
    if ( userContainer )
    {
        mainContainer = userContainer;
    }
    else
    {
        mainContainer = new VBox();
        mainContainer->setAbsorbEvents( true );
        mainContainer->setBackColor( Color(Color::Black, 0.8) );
        mainContainer->setHorizAlign( Control::ALIGN_LEFT );
        mainContainer->setVertAlign( Control::ALIGN_BOTTOM );
    }
    canvas->addControl( mainContainer );

    // Add an event handler to toggle the canvas with a key press;
    view->addEventHandler(new ToggleCanvasEventHandler(canvas, 'y'));

    // look for external data in the map node:
    const Config& externals = mapNode->externalConfig();

    // some terrain effects.
    // TODO: Most of these are likely to move into extensions.
    const Config& vertScaleConf   = externals.child("vertical_scale");

    // Loading KML from the command line:
    if ( !kmlFile.empty() )
    {
        KMLOptions kml_options;
        kml_options.declutter() = true;

        // set up a default icon for point placemarks:
        IconSymbol* defaultIcon = new IconSymbol();
        defaultIcon->url()->setLiteral(KML_PUSHPIN_URL);
        kml_options.defaultIconSymbol() = defaultIcon;

        TextSymbol* defaultText = new TextSymbol();
        defaultText->halo() = Stroke(0.3,0.3,0.3,1.0);
        kml_options.defaultTextSymbol() = defaultText;

        osg::Node* kml = KML::load( URI(kmlFile), mapNode, kml_options );
        if ( kml )
        {
            if (kmlUI)
            {
                Control* c = AnnotationGraphControlFactory().create(kml, view);
                if ( c )
                {
                    c->setVertAlign( Control::ALIGN_TOP );
                    canvas->addControl( c );
                }
            }
            root->addChild( kml );
        }
        else
        {
            OE_NOTICE << "Failed to load " << kmlFile << std::endl;
        }
    }

    //// Configure the de-cluttering engine for labels and annotations:
    //if ( !screenSpaceLayoutConf.empty() )
    //{
    //    ScreenSpaceLayout::setOptions( ScreenSpaceLayoutOptions(screenSpaceLayoutConf) );
    //}

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
    if ( args.read("--ortho") )
    {
        view->getCamera()->setProjectionMatrixAsOrtho(-1, 1, -1, 1, 0, 1);
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
        OE_INFO << LC << "Activating logarithmic depth buffer (vertex-only) on main camera" << std::endl;
        osgEarth::Util::LogarithmicDepthBuffer logDepth;
        logDepth.setUseFragDepth( false );
        logDepth.install( view->getCamera() );
    }

    else if ( useLogDepth2 )
    {
        OE_INFO << LC << "Activating logarithmic depth buffer (precise) on main camera" << std::endl;
        osgEarth::Util::LogarithmicDepthBuffer logDepth;
        logDepth.setUseFragDepth( true );
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

    // Install vertical scaler.
    // TODO: deprecate this, or move it to an extension.
    if ( !vertScaleConf.empty() )
    {
        mapNode->getTerrainEngine()->addEffect( new VerticalScale(vertScaleConf) );
    }

    // Install a contour map effect.
    if (args.read("--contourmap"))
    {
        mapNode->addExtension(Extension::create("contourmap", ConfigOptions()));

        // with the cmdline switch, hids all the image layer so we can see the contour map.
        for (unsigned i = 0; i < mapNode->getMap()->getNumImageLayers(); ++i) {
            mapNode->getMap()->getImageLayerAt(i)->setVisible(false);
        }
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

    // Map inspector:
    if (args.read("--inspect"))
    {
        mapNode->addExtension( Extension::create("mapinspector", ConfigOptions()) );
    }

    // Memory monitor:
    if (args.read("--monitor"))
    {
        mapNode->addExtension(Extension::create("monitor", ConfigOptions()) );
    }

    // Simple sky model:
    if (args.read("--sky"))
    {
        mapNode->addExtension(Extension::create("sky_simple", ConfigOptions()) );
    }

    // Simple ocean model:
    if (args.read("--ocean"))
    {
        mapNode->addExtension(Extension::create("ocean_simple", ConfigOptions()));
    }

    // Arbitrary extension:
    std::string extname;
    if (args.read("--extension", extname))
    {
        Extension* ext = Extension::create(extname, ConfigOptions());
        if (ext)
            mapNode->addExtension(ext);
    }
    

    // Hook up the extensions!
    for(std::vector<osg::ref_ptr<Extension> >::const_iterator eiter = mapNode->getExtensions().begin();
        eiter != mapNode->getExtensions().end();
        ++eiter)
    {
        Extension* e = eiter->get();

        // Check for a View interface:
        ExtensionInterface<osg::View>* viewIF = ExtensionInterface<osg::View>::get( e );
        if ( viewIF )
            viewIF->connect( view );

        // Check for a Control interface:
        ExtensionInterface<Control>* controlIF = ExtensionInterface<Control>::get( e );
        if ( controlIF )
            controlIF->connect( mainContainer );
    }


    // Shadowing. This is last because it needs access to a light which may be provided
    // by one of the Sky extensions.
    if (args.read("--shadows"))
    {
        int unit;
        if ( mapNode->getTerrainEngine()->getResources()->reserveTextureImageUnit(unit, "ShadowCaster") )
        {
            ShadowCaster* caster = new ShadowCaster();
            caster->setTextureImageUnit( unit );
            caster->setLight( view->getLight() );
            caster->getShadowCastingGroup()->addChild( mapNode->getModelLayerGroup() );
            //insertParent(caster, mapNode);
            //root = findTopOfGraph(caster)->asGroup();
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
    view->addEventHandler(new osgViewer::ScreenCaptureHandler());
}


std::string
MapNodeHelper::usage() const
{
    return Stringify()
        << "  --sky                         : add a sky model\n"
        << "  --kml <file.kml>              : load a KML or KMZ file\n"
        << "  --kmlui                       : display a UI for toggling nodes loaded with --kml\n"
        << "  --coords                      : display map coords under mouse\n"
        << "  --dms                         : dispay deg/min/sec coords under mouse\n"
        << "  --dd                          : display decimal degrees coords under mouse\n"
        << "  --mgrs                        : show MGRS coords under mouse\n"
        << "  --ortho                       : use an orthographic camera\n"
        << "  --logdepth                    : activates the logarithmic depth buffer\n"
        << "  --logdepth2                   : activates logarithmic depth buffer with per-fragment interpolation\n"
        << "  --shadows                     : activates model layer shadows\n"
        << "  --images [path]               : finds and loads image layers from folder [path]\n"
        << "  --image-extensions [ext,...]  : with --images, extensions to use\n"
        << "  --out-earth [file]            : write the loaded map to an earth file\n"
        << "  --uniform [name] [min] [max]  : create a uniform controller with min/max values\n"
        << "  --path [file]                 : load and playback an animation path\n"
        << "  --extension [name]            : loads a named extension\n";
}


//........................................................................


namespace
{
    struct SkyHoursSlider : public ui::ControlEventHandler
    {
        SkyHoursSlider(SkyNode* sky) : _sky(sky)  { }
        SkyNode* _sky;
        void onValueChanged(ui::Control* control, float value )
        {
            DateTime d = _sky->getDateTime();
            _sky->setDateTime(DateTime(d.year(), d.month(), d.day(), value));
        }
    };
    
    static std::string s_month[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

    struct SkyMonthSlider : public ui::ControlEventHandler
    {
        SkyMonthSlider(SkyNode* sky, ui::LabelControl* label) : _sky(sky), _label(label) { }
        SkyNode* _sky;
        ui::LabelControl* _label;
        void onValueChanged(ui::Control* control, float value )
        {
            int m = std::min((int)value, 11);
            DateTime d = _sky->getDateTime();            
            _sky->setDateTime(DateTime(d.year(), m, d.day(), d.hours()));
            _label->setText(s_month[m]);
        }
    };

    struct SkyYearSlider : public ui::ControlEventHandler
    {
        SkyYearSlider(SkyNode* sky, ui::LabelControl* label) : _sky(sky), _label(label) { }
        SkyNode* _sky;
        ui::LabelControl* _label;
        void onValueChanged(ui::Control* control, float value )
        {
            DateTime d = _sky->getDateTime();            
            _sky->setDateTime(DateTime((int)value, d.month(), d.day(), d.hours()));
            _label->setText(Stringify() << (int)value);
        }
    };

    struct AmbientBrightnessHandler : public ui::ControlEventHandler
    {
        AmbientBrightnessHandler(SkyNode* sky) : _sky(sky) { }

        SkyNode* _sky;

        void onValueChanged(ui::Control* control, float value )
        {
            _sky->setMinimumAmbient(osg::Vec4(value,value,value,1));
        }
    };
}

ui::Control* SkyControlFactory::create(SkyNode* sky)
{
    ui::Grid* grid = new ui::Grid();
    grid->setChildVertAlign( ui::Control::ALIGN_CENTER );
    grid->setChildSpacing( 10 );
    grid->setHorizFill( true );

    if (sky)
    {
        DateTime dt = sky->getDateTime();

        int r=0;
        grid->setControl( 0, r, new ui::LabelControl("Hours UTC: ", 16) );
        ui::HSliderControl* skyHoursSlider = grid->setControl(1, r, new ui::HSliderControl( 0.0f, 24.0f, dt.hours() ));
        skyHoursSlider->setHorizFill( true, 250 );
        skyHoursSlider->addEventHandler( new SkyHoursSlider(sky) );
        grid->setControl(2, r, new ui::LabelControl(skyHoursSlider) );
    
        ++r;
        grid->setControl( 0, r, new ui::LabelControl("Month: ", 16) );
        ui::HSliderControl* skyMonthSlider = grid->setControl(1, r, new ui::HSliderControl( 0.0f, 12.0f, dt.month() ));
        skyMonthSlider->setHorizFill( true, 250 );
        ui::LabelControl* monthLabel = grid->setControl(2, r, new ui::LabelControl(s_month[dt.month()]));
        skyMonthSlider->addEventHandler( new SkyMonthSlider(sky, monthLabel) );
    
        ++r;
        grid->setControl( 0, r, new ui::LabelControl("Year: ", 16) );
        ui::HSliderControl* skyYearSlider = grid->setControl(1, r, new ui::HSliderControl( 1970.0f, 2061.0f, dt.year() ));
        skyYearSlider->setHorizFill( true, 250 );
        ui::LabelControl* yearLabel = grid->setControl(2, r, new ui::LabelControl(Stringify()<<dt.year()));
        skyYearSlider->addEventHandler( new SkyYearSlider(sky, yearLabel) );

        ++r;
        grid->setControl(0, r, new ui::LabelControl("Min.Ambient: ", 16) );
        ui::HSliderControl* ambient = grid->setControl(1, r, new ui::HSliderControl(0.0f, 1.0f, sky->getSunLight()->getAmbient().r()));
        ambient->addEventHandler( new AmbientBrightnessHandler(sky) );
        grid->setControl(2, r, new ui::LabelControl(ambient) );
    }

    return grid;
}
