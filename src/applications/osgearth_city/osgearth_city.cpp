/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
* MIT License
*/
#include <osg/Notify>
#include <osgGA/GUIEventHandler>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>

#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarth/TMS>

#include <osgEarth/ExampleResources>
#include <osgEarth/EarthManipulator>
#include <osgEarth/LogarithmicDepthBuffer>

#include <osgEarth/FeatureModelLayer>
#include <osgEarth/OGRFeatureSource>

using namespace osgEarth;
using namespace osgEarth::Util;

#define IMAGERY_URL      "http://readymap.org/readymap/tiles/1.0.0/22/"
#define ELEVATION_URL    "http://readymap.org/readymap/tiles/1.0.0/116/"
#define BUILDINGS_URL    "../data/boston_buildings_utm19.shp"
#define RESOURCE_LIB_URL "../data/resources/textures_us/catalog.xml"
#define STREETS_URL      "../data/boston-scl-utm19n-meters.shp"
#define PARKS_URL        "../data/boston-parks.shp"
#define TREE_MODEL_URL   "../data/tree.osg"

// forward declarations.
void addImagery  (Map* map);
void addElevation(Map* map);
void addBuildings(Map* map);
void addStreets  (Map* map);
void addParks    (Map* map);


/**
 * This code example effectively duplicates the "boston.earth" sample,
 * demonstrating how to create a 3D city model in osgEarth.
 *
 * Run this from the tests folder.
 */
int
main(int argc, char** argv)
{
    osgEarth::initialize();
    osg::ArgumentParser arguments(&argc,argv);

    // create the map.
    osg::ref_ptr<Map> map = new Map();

    addImagery( map.get() );
    addElevation( map.get() );
    addBuildings( map.get() );
    addStreets( map.get() );
    addParks( map.get() );

    // initialize a viewer:
    osgViewer::Viewer viewer(arguments);

    EarthManipulator* manip = new EarthManipulator();
    viewer.setCameraManipulator( manip );

    osg::Group* root = new osg::Group();
    viewer.setSceneData( root );

    // make the map scene graph:
    MapNode* mapNode = new MapNode(map.get());
    root->addChild( mapNode );

    // zoom to a good startup position
    manip->setViewpoint( Viewpoint(
        "Home",
        -71.0763, 42.34425, 0,   // longitude, latitude, altitude
         24.261, -21.6, 3450.0), // heading, pitch, range
         5.0 );                    // duration

    // This will mitigate near clip plane issues if you zoom in close to the ground:
    LogarithmicDepthBuffer buf;
    buf.install( viewer.getCamera() );

    return viewer.run();
}

void addImagery(Map* map)
{
    // add a TMS imagery layer:
    TMSImageLayer* layer = new TMSImageLayer();
    layer->setURL(IMAGERY_URL);
    map->addLayer(layer);
}


void addElevation(Map* map)
{
    // add a TMS elevation layer:
    TMSElevationLayer* layer = new TMSElevationLayer();
    layer->setURL(ELEVATION_URL);
    map->addLayer(layer);
}


void addBuildings(Map* map)
{
    // create a feature source to load the building footprint shapefile.
    OGRFeatureSource* data = new OGRFeatureSource();
    data->setName("buildings-data");
    data->setURL(BUILDINGS_URL);

    // a style for the building data:
    Style buildingStyle;
    buildingStyle.setName( "default" );

    // Extrude the shapes into 3D buildings.
    ExtrusionSymbol* extrusion = buildingStyle.getOrCreate<ExtrusionSymbol>();
    extrusion->heightExpression() = NumericExpression( "3.5 * max( [story_ht_], 1 )" );
    extrusion->flatten() = true;
    extrusion->wallStyleName() = "building-wall";
    extrusion->roofStyleName() = "building-roof";

    PolygonSymbol* poly = buildingStyle.getOrCreate<PolygonSymbol>();
    poly->fill()->color() = Color::White;

    // Clamp the buildings to the terrain.
    AltitudeSymbol* alt = buildingStyle.getOrCreate<AltitudeSymbol>();
    alt->clamping() = alt->CLAMP_TO_TERRAIN;
    alt->binding() = alt->BINDING_VERTEX;

    // a style for the wall textures:
    Style wallStyle;
    wallStyle.setName( "building-wall" );
    SkinSymbol* wallSkin = wallStyle.getOrCreate<SkinSymbol>();
    wallSkin->library() = "us_resources";
    wallSkin->addTag( "building" );
    wallSkin->randomSeed() = 1;

    // a style for the rooftop textures:
    Style roofStyle;
    roofStyle.setName( "building-roof" );
    SkinSymbol* roofSkin = roofStyle.getOrCreate<SkinSymbol>();
    roofSkin->library() = "us_resources";
    roofSkin->addTag( "rooftop" );
    roofSkin->randomSeed() = 1;
    roofSkin->isTiled() = true;

    // assemble a stylesheet and add our styles to it:
    StyleSheet* styleSheet = new StyleSheet();
    styleSheet->addStyle( buildingStyle );
    styleSheet->addStyle( wallStyle );
    styleSheet->addStyle( roofStyle );

    // load a resource library that contains the building textures.
    ResourceLibrary* reslib = new ResourceLibrary( "us_resources", RESOURCE_LIB_URL );
    styleSheet->addResourceLibrary( reslib );

    // set up a paging layout for incremental loading. The tile size factor and
    // the visibility range combine to determine the tile size, such that
    // tile radius = max range / tile size factor.
    FeatureDisplayLayout layout;
    layout.tileSize() = 500;

    FeatureModelLayer* layer = new FeatureModelLayer();
    layer->setName("Buildings");
    layer->setFeatureSource(data);
    layer->setStyleSheet(styleSheet);
    layer->setLayout(layout);
    layer->setMaxVisibleRange(20000.0);

    map->addLayer(layer);
}


void addStreets(Map* map)
{
    // create a feature source to load the street shapefile.
    OGRFeatureSource* data = new OGRFeatureSource();
    data->setURL(STREETS_URL);
    data->options().buildSpatialIndex() = true;

    // a resampling filter will ensure that the length of each segment falls
    // within the specified range. That can be helpful to avoid cropping
    // very long lines segments.
    ResampleFilterOptions resample;
    resample.minLength() = 0.0f;
    resample.maxLength() = 25.0f;
    data->options().filters().push_back( resample );

    // a style:
    Style style;
    style.setName( "streets" );

    // Render the data as translucent yellow lines that are 7.5m wide.
    LineSymbol* line = style.getOrCreate<LineSymbol>();
    line->stroke()->color() = Color(Color::Yellow, 0.5f);
    line->stroke()->width() = 7.5f;
    line->stroke()->widthUnits() = Units::METERS;

    // Clamp the lines to the terrain.
    AltitudeSymbol* alt = style.getOrCreate<AltitudeSymbol>();
    alt->clamping() = alt->CLAMP_TO_TERRAIN;

    // Apply a depth offset to avoid z-fighting. The "min bias" is the minimum
    // apparent offset (towards the camera) of the geometry from its actual position.
    // The value here was chosen empirically by tweaking the "oe_doff_min_bias" uniform.
    RenderSymbol* render = style.getOrCreate<RenderSymbol>();
    render->depthOffset()->minBias()->set(6.6, Units::METERS);

    // Set up a paging layout. The tile size factor and the visibility range combine
    // to determine the tile size, such that tile radius = max range / tile size factor.
    FeatureDisplayLayout layout;
    layout.tileSize() = 500;

    // create a model layer that will render the buildings according to our style sheet.
    FeatureModelLayer* layer = new FeatureModelLayer();
    layer->setName("Streets");
    layer->setFeatureSource(data);
    layer->options().layout() = layout;
    layer->setStyleSheet(new StyleSheet());
    layer->getStyleSheet()->addStyle(style);
    layer->setMaxVisibleRange(5000.0f);

    map->addLayer(layer);
}


void addParks(Map* map)
{
    // create a feature source to load the shapefile.
    OGRFeatureSource* data = new OGRFeatureSource();
    data->setURL(PARKS_URL);
    data->options().buildSpatialIndex() = true;

    // a style:
    Style style;
    style.setName( "parks" );

    // Render the data using point-model substitution, which replaces each point
    // in the feature geometry with an instance of a 3D model. Since the input
    // data are polygons, the PLACEMENT_RANDOM directive below will scatter
    // points within the polygon boundary at the specified density.
    ModelSymbol* model = style.getOrCreate<ModelSymbol>();
    model->url()->setLiteral(TREE_MODEL_URL);
    model->placement() = model->PLACEMENT_RANDOM;
    model->density() = 6000.0f; // instances per sqkm

    // Clamp to the terrain:
    AltitudeSymbol* alt = style.getOrCreate<AltitudeSymbol>();
    alt->clamping() = alt->CLAMP_TO_TERRAIN;

    // Since the tree model contains alpha components, we will discard any data
    // that's sufficiently transparent; this will prevent depth-sorting anomalies
    // common when rendering lots of semi-transparent objects.
    RenderSymbol* render = style.getOrCreate<RenderSymbol>();
    render->transparent() = true;
    render->minAlpha() = 0.15f;

    // Set up a paging layout. The tile size factor and the visibility range combine
    // to determine the tile size, such that tile radius = max range / tile size factor.
    FeatureDisplayLayout layout;
    layout.tileSize() = 650;
    layout.addLevel(FeatureLevel(0.0f, 2000.0f, "parks"));

    // create a model layer that will render the buildings according to our style sheet.
    FeatureModelLayer* layer = new FeatureModelLayer();
    layer->setFeatureSource(data);
    layer->options().layout() = layout;
    layer->setStyleSheet(new StyleSheet());
    layer->getStyleSheet()->addStyle(style);

    map->addLayer(layer);

    if (layer->getStatus().isError())
    {
        OE_WARN << layer->getStatus().message() << std::endl;
    }
}
