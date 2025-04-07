/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarth/MapNode>
#include <osgEarth/Random>
#include <osgEarth/StringUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/GeoMath>
#include <osgEarth/Units>
#include <osgEarth/StringUtils>
#include <osgEarth/ScreenSpaceLayout>
#include <osgEarth/ExampleResources>
#include <osgEarth/EarthManipulator>
#include <osgEarth/MGRSFormatter>
#include <osgEarth/Controls>
#include <osgEarth/TrackNode>
#include <osgEarth/Color>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

#define LC "[osgearth_tracks] "

/**
 * Demonstrates use of the TrackNode to display entity track symbols.
 */

// field names for the track labelsL
#define FIELD_NAME     "name"
#define FIELD_POSITION "position"
#define FIELD_NUMBER   "number"

// icon to use, and size in pixels
#define ICON_URL       "../data/m2525_air.png"
#define ICON_SIZE      40

// format coordinates as MGRS
static MGRSFormatter s_format(MGRSFormatter::PRECISION_10000M);

// globals for this demo
bool                g_showCoords        = true;
optional<float>     g_duration          = 60.0;
unsigned            g_numTracks         = 500;
ScreenSpaceLayoutOptions g_dcOptions;


/** Prints an error message */
int
usage( const std::string& message )
{
    OE_WARN << LC << message << std::endl;
    return -1;
}


/** A little track simulator that goes a simple great circle interpolation */
struct TrackSim : public osg::Referenced
{
    TrackNode* _track;
    GeoPoint _start, _end;

    void update( double t )
    {
        osg::Vec3d pos;

        GeoPoint geo = _start.interpolate(_end, t);
        geo.alt() = 10000.0;

        // update the position label.
        _track->setPosition(geo);

        if ( g_showCoords )
        {
            _track->setFieldValue( FIELD_POSITION, s_format(geo) );
        }
        else
            _track->setFieldValue( FIELD_POSITION, "" );
    }
};
typedef std::list< osg::ref_ptr<TrackSim> > TrackSims;


/** Update operation that runs the simulators. */
struct TrackSimUpdate : public osg::Operation
{
    TrackSimUpdate(TrackSims& sims) : osg::Operation( "tasksim", true ), _sims(sims) { }

    void operator()( osg::Object* obj ) {
        osg::View* view = dynamic_cast<osg::View*>(obj);
        double t = fmod(view->getFrameStamp()->getSimulationTime(), (double)g_duration.get()) / (double)g_duration.get();
        for( TrackSims::iterator i = _sims.begin(); i != _sims.end(); ++i )
            i->get()->update( t );
    }

    TrackSims& _sims;
};


/**
 * Creates a field schema that we'll later use as a labeling template for
 * TrackNode instances.
 */
void
createFieldSchema( TrackNodeFieldSchema& schema )
{
    const float R = 2.0f;

    // draw the track name above the icon:
    TextSymbol* nameSymbol = new TextSymbol();
    nameSymbol->pixelOffset() = osg::Vec2s(0, R+ICON_SIZE/2 );
    nameSymbol->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM;
    nameSymbol->halo().mutable_value().color() = Color::Black;
    nameSymbol->size() = nameSymbol->size()->eval() + 2.0f;
    schema[FIELD_NAME] = TrackNodeField(nameSymbol, false); // false => static label (won't change after set)

    // draw the track coordinates below the icon:
    TextSymbol* posSymbol = new TextSymbol();
    posSymbol->pixelOffset() = osg::Vec2s(0, -R-ICON_SIZE/2 );
    posSymbol->alignment() = TextSymbol::ALIGN_CENTER_TOP;
    posSymbol->fill().mutable_value().color() = Color::Yellow;
    posSymbol->size() = posSymbol->size()->eval() - 2.0f;
    schema[FIELD_POSITION] = TrackNodeField(posSymbol, true); // true => may change at runtime

    // draw some other field to the left:
    TextSymbol* numberSymbol = new TextSymbol();
    numberSymbol->pixelOffset() = osg::Vec2s(-R-ICON_SIZE/2, 0 );
    numberSymbol->alignment() = TextSymbol::ALIGN_RIGHT_CENTER;
    schema[FIELD_NUMBER] = TrackNodeField(numberSymbol, false);
}


/** Builds a bunch of tracks. */
void
createTrackNodes(const SpatialReference* mapSRS, osg::Group* parent, const TrackNodeFieldSchema& schema, TrackSims& sims )
{
    // load an icon to use:
    osg::ref_ptr<osg::Image> srcImage = osgDB::readRefImageFile( ICON_URL );
    osg::ref_ptr<osg::Image> image;
    ImageUtils::resizeImage( srcImage.get(), ICON_SIZE, ICON_SIZE, image );

    // make some tracks, choosing a random simulation for each.
    Random prng;
    const SpatialReference* geoSRS = mapSRS->getGeographicSRS();

    for( unsigned i=0; i<g_numTracks; ++i )
    {
        double lon0 = -180.0 + prng.next() * 360.0;
        double lat0 = -80.0 + prng.next() * 160.0;

        GeoPoint pos(geoSRS, lon0, lat0);

        TrackNode* track = new TrackNode(pos, image.get(), schema);

        track->setFieldValue( FIELD_NAME,     Stringify() << "Track:" << i );
        track->setFieldValue( FIELD_POSITION, Stringify() << s_format(pos) );
        track->setFieldValue( FIELD_NUMBER,   Stringify() << (1 + prng.next(9)) );

        // add a priority
        track->setPriority( float(i) );

        parent->addChild( track );

        // add a simulator for this guy
        double lon1 = -180.0 + prng.next() * 360.0;
        double lat1 = -80.0 + prng.next() * 160.0;
        TrackSim* sim = new TrackSim();
        sim->_track = track;
        sim->_start.set(mapSRS, lon0, lat0, 0.0, ALTMODE_ABSOLUTE);
        sim->_end.set(mapSRS, lon1, lat1, 0.0, ALTMODE_ABSOLUTE);
        sims.push_back( sim );
    }
}


/** creates some UI controls for adjusting the decluttering parameters. */
Container*
createControls( osgViewer::View* view )
{
    //ControlCanvas* canvas = ControlCanvas::getOrCreate(view);

    // title bar
    VBox* vbox = new VBox(Control::ALIGN_NONE, Control::ALIGN_BOTTOM, 2, 1 );
    vbox->setBackColor( Color(Color::Black, 0.5) );
    vbox->addControl( new LabelControl("osgEarth Tracks Demo", Color::Yellow) );

    // checkbox that toggles decluttering of tracks
    struct ToggleDecluttering : public ControlEventHandler {
        void onValueChanged( Control* c, bool on ) {
            ScreenSpaceLayout::setDeclutteringEnabled( on );
        }
    };
    HBox* dcToggle = vbox->addControl( new HBox() );
    dcToggle->addControl( new CheckBoxControl(true, new ToggleDecluttering()) );
    dcToggle->addControl( new LabelControl("Declutter") );

    // checkbox that toggles the coordinate display
    struct ToggleCoords : public ControlEventHandler {
        void onValueChanged( Control* c, bool on ) {
            g_showCoords = on;
        }
    };
    HBox* coordsToggle = vbox->addControl( new HBox() );
    coordsToggle->addControl( new CheckBoxControl(true, new ToggleCoords()) );
    coordsToggle->addControl( new LabelControl("Show locations") );

    // grid for the slider controls so they look nice
    Grid* grid = vbox->addControl( new Grid() );
    grid->setHorizFill( true );
    grid->setChildHorizAlign( Control::ALIGN_LEFT );
    grid->setChildSpacing( 6 );

    unsigned r=0;

    // event handler for changing decluttering options
    struct ChangeFloatOption : public ControlEventHandler {
        optional<float>& _param;
        LabelControl* _label;
        ChangeFloatOption( optional<float>& param, LabelControl* label ) : _param(param), _label(label) { }
        void onValueChanged( Control* c, float value ) {
            _param = value;
            _label->setText( Stringify() << std::fixed << std::setprecision(1) << value );
            ScreenSpaceLayout::setOptions( g_dcOptions );
        }
    };

    grid->setControl( 0, r, new LabelControl("Sim loop duration:") );
    LabelControl* speedLabel = grid->setControl( 2, r, new LabelControl(Stringify() << std::fixed << std::setprecision(1) << *g_duration) );
    HSliderControl* speedSlider = grid->setControl( 1, r, new HSliderControl(
        600.0, 30.0, *g_duration, new ChangeFloatOption(g_duration, speedLabel) ) );
    speedSlider->setHorizFill( true, 200 );

    grid->setControl( 0, ++r, new LabelControl("Min scale:") );
    LabelControl* minAnimationScaleLabel = grid->setControl( 2, r, new LabelControl(Stringify() << std::fixed << std::setprecision(1) << *g_dcOptions.minAnimationScale()) );
    grid->setControl( 1, r, new HSliderControl(
        0.0, 1.0, *g_dcOptions.minAnimationScale(), new ChangeFloatOption(g_dcOptions.minAnimationScale(), minAnimationScaleLabel) ) );

    grid->setControl( 0, ++r, new LabelControl("Min alpha:") );
    LabelControl* alphaLabel = grid->setControl( 2, r, new LabelControl(Stringify() << std::fixed << std::setprecision(1) << *g_dcOptions.minAnimationAlpha()) );
    grid->setControl( 1, r, new HSliderControl(
        0.0, 1.0, *g_dcOptions.minAnimationAlpha(), new ChangeFloatOption(g_dcOptions.minAnimationAlpha(), alphaLabel) ) );

    grid->setControl( 0, ++r, new LabelControl("Activate time (s):") );
    LabelControl* actLabel = grid->setControl( 2, r, new LabelControl(Stringify() << std::fixed << std::setprecision(1) << *g_dcOptions.inAnimationTime()) );
    grid->setControl( 1, r, new HSliderControl(
        0.0, 2.0, *g_dcOptions.inAnimationTime(), new ChangeFloatOption(g_dcOptions.inAnimationTime(), actLabel) ) );

    grid->setControl( 0, ++r, new LabelControl("Deactivate time (s):") );
    LabelControl* deactLabel = grid->setControl( 2, r, new LabelControl(Stringify() << std::fixed << std::setprecision(1) << *g_dcOptions.outAnimationTime()) );
    grid->setControl( 1, r, new HSliderControl(
        0.0, 2.0, *g_dcOptions.outAnimationTime(), new ChangeFloatOption(g_dcOptions.outAnimationTime(), deactLabel) ) );

    return vbox;
}


/**
 * Main application.
 * Creates some simulated track data and runs the simulation.
 */
int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser arguments(&argc,argv);

    // initialize a viewer.
    osgViewer::Viewer viewer( arguments );
    viewer.setCameraManipulator( new EarthManipulator );

    // load a map from an earth file.
    auto earth = MapNodeHelper().load(arguments, &viewer);

    MapNode* mapNode = MapNode::get(earth);
    if ( !mapNode )
        return usage("Missing required .earth file" );

    // count on the cmd line?
    arguments.read("--count", g_numTracks);

    auto canvas = new ControlCanvas();
    canvas->addChild(createControls(&viewer));

    auto group = new osg::Group();
    group->addChild(earth);
    group->addChild(canvas);

    viewer.setSceneData(group);

    // build a track field schema.
    TrackNodeFieldSchema schema;
    createFieldSchema( schema );

    // create some track nodes.
    TrackSims trackSims;
    osg::Group* tracks = new osg::Group();
    createTrackNodes( mapNode->getMapSRS(), tracks, schema, trackSims );
    mapNode->addChild( tracks );

    // Set up the automatic decluttering. setEnabled() activates decluttering for
    // all drawables under that state set. We are also activating priority-based
    // sorting, which looks at the AnnotationData::priority field for each drawable.
    // (By default, objects are sorted by disatnce-to-camera.) Finally, we customize
    // a couple of the decluttering options to get the animation effects we want.
    g_dcOptions = ScreenSpaceLayout::getOptions();
    g_dcOptions.inAnimationTime()  = 1.0f;
    g_dcOptions.outAnimationTime() = 1.0f;
    g_dcOptions.sortByPriority()   = true;
    ScreenSpaceLayout::setOptions( g_dcOptions );

    // attach the simulator to the viewer.
    viewer.addUpdateOperation( new TrackSimUpdate(trackSims) );
    viewer.setRunFrameScheme( viewer.CONTINUOUS );

    viewer.run();
}
