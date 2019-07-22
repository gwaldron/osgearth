/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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
#include <osg/Notify>
#include <osgGA/GUIEventHandler>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>

#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>

#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/LogarithmicDepthBuffer>

#include <osgEarthFeatures/FeatureModelLayer>

#include <osgEarthDrivers/tms/TMSOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>
#include <osgEarthDrivers/engine_rex/RexTerrainEngineOptions>

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
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
    osg::ArgumentParser arguments(&argc,argv);

    // create the map.
    Map* map = new Map();

    addImagery( map );
    addElevation( map );
    addBuildings( map );
    addStreets( map );
    addParks( map );

    // initialize a viewer:
    osgViewer::Viewer viewer(arguments);
    
    EarthManipulator* manip = new EarthManipulator();
    viewer.setCameraManipulator( manip );

    osg::Group* root = new osg::Group();
    viewer.setSceneData( root );

    // make the map scene graph:
    MapNode* mapNode = new MapNode(map);
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
    TMSOptions imagery;
    imagery.url() = IMAGERY_URL;
    map->addLayer( new ImageLayer("ReadyMap imagery", imagery) );
}


void addElevation(Map* map)
{
    // add a TMS elevation layer:
    TMSOptions elevation;
    elevation.url() = ELEVATION_URL;
    map->addLayer( new ElevationLayer("ReadyMap elevation", elevation) );
}


void addBuildings(Map* map)
{
    // create a feature source to load the building footprint shapefile.
    OGRFeatureOptions buildingData;
    buildingData.name() = "buildings";
    buildingData.url() = BUILDINGS_URL;
    buildingData.buildSpatialIndex() = true;
    
    // a style for the building data:
    Style buildingStyle;
    buildingStyle.setName( "buildings" );

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
    layout.addLevel( FeatureLevel(0.0f, 20000.0f, "buildings") );

    FeatureModelLayer* layer = new FeatureModelLayer();
    layer->setName("Buildings");
    layer->options().featureSource() = buildingData;
    layer->options().styles() = styleSheet;
    layer->options().layout() = layout;

    map->addLayer(layer);
}


void addStreets(Map* map)
{
    // create a feature source to load the street shapefile.
    OGRFeatureOptions feature_opt;
    feature_opt.name() = "streets";
    feature_opt.url() = STREETS_URL;
    feature_opt.buildSpatialIndex() = true;

    // a resampling filter will ensure that the length of each segment falls
    // within the specified range. That can be helpful to avoid cropping 
    // very long lines segments.
    ResampleFilterOptions resample;
    resample.minLength() = 0.0f;
    resample.maxLength() = 25.0f;
    feature_opt.filters().push_back( resample );

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
    render->depthOffset()->minBias() = 6.6f;

    // Set up a paging layout. The tile size factor and the visibility range combine
    // to determine the tile size, such that tile radius = max range / tile size factor.
    FeatureDisplayLayout layout;
    layout.tileSize() = 500;
    layout.maxRange() = 5000.0f;

    // create a model layer that will render the buildings according to our style sheet.
    FeatureModelLayerOptions streets;
    streets.name() = "streets";
    streets.featureSource() = feature_opt;
    streets.layout() = layout;
    streets.styles() = new StyleSheet();
    streets.styles()->addStyle( style );

    map->addLayer(new FeatureModelLayer(streets));
}


void addParks(Map* map)
{
    // create a feature source to load the shapefile.
    OGRFeatureOptions parksData;
    parksData.name() = "parks";
    parksData.url() = PARKS_URL;
    parksData.buildSpatialIndex() = true;

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
    FeatureModelLayerOptions parks;
    parks.name() = "parks";
    parks.featureSource() = parksData;
    parks.layout() = layout;
    parks.styles() = new StyleSheet();
    parks.styles()->addStyle( style );

    Layer* parksLayer = new FeatureModelLayer(parks);
    map->addLayer(parksLayer);

    if (parksLayer->getStatus().isError())
    {
        OE_WARN << parksLayer->getStatus().message() << std::endl;
    }
}
