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
#include <osg/Notify>
#include <osgGA/GUIEventHandler>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>

#include <osgEarth/MapNode>

#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/LogarithmicDepthBuffer>

#include <osgEarthDrivers/tms/TMSOptions>
#include <osgEarthDrivers/xyz/XYZOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;

#define IMAGERY_URL      "http://readymap.org/readymap/tiles/1.0.0/22/"
#define ELEVATION_URL    "http://readymap.org/readymap/tiles/1.0.0/9/"
#define BUILDINGS_URL    "../data/boston_buildings_utm19.shp"
#define RESOURCE_LIB_URL "../data/resources/textures_us/catalog.xml"
#define STREETS_URL      "../data/boston-scl-utm19n-meters.shp"
#define PARKS_URL        "../data/boston-parks.shp"
#define TREE_MODEL_URL   "../data/loopix/tree4.osgb"

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
    MapNode* mapNode = new MapNode( map );
    root->addChild( mapNode );
    
    // Process cmdline args.
    MapNodeHelper helper;
    helper.configureView( &viewer );
    helper.parse(mapNode, arguments, &viewer, root, new LabelControl("City Demo"));

    // zoom to a good startup position
    manip->setViewpoint( Viewpoint(
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
    map->addImageLayer( new ImageLayer("ReadyMap imagery", imagery) );
}


void addElevation(Map* map)
{
    // add a TMS elevation layer:
    TMSOptions elevation;
    elevation.url() = ELEVATION_URL;
    map->addElevationLayer( new ElevationLayer("ReadyMap elevation", elevation) );
}


void addBuildings(Map* map)
{
    // create a feature source to load the building footprint shapefile.
    OGRFeatureOptions feature_opt;
    feature_opt.name() = "buildings";
    feature_opt.url() = BUILDINGS_URL;
    feature_opt.buildSpatialIndex() = true;
    
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
    wallSkin->libraryName() = "us_resources";
    wallSkin->addTag( "building" );
    wallSkin->randomSeed() = 1;

    // a style for the rooftop textures:
    Style roofStyle;
    roofStyle.setName( "building-roof" );
    SkinSymbol* roofSkin = roofStyle.getOrCreate<SkinSymbol>();
    roofSkin->libraryName() = "us_resources";
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
    layout.tileSizeFactor() = 52.0;
    layout.addLevel( FeatureLevel(0.0f, 20000.0f, "buildings") );

    // create a model layer that will render the buildings according to our style sheet.
    FeatureGeomModelOptions fgm_opt;
    fgm_opt.featureOptions() = feature_opt;
    fgm_opt.styles() = styleSheet;
    fgm_opt.layout() = layout;

    map->addModelLayer( new ModelLayer( "buildings", fgm_opt ) );
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
    feature_opt.filters().push_back( new ResampleFilter(0.0, 25.0) );

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
    layout.tileSizeFactor() = 7.5f;
    layout.maxRange()       = 5000.0f;

    // create a model layer that will render the buildings according to our style sheet.
    FeatureGeomModelOptions fgm_opt;
    fgm_opt.featureOptions() = feature_opt;
    fgm_opt.layout() = layout;
    fgm_opt.styles() = new StyleSheet();
    fgm_opt.styles()->addStyle( style );

    map->addModelLayer( new ModelLayer("streets", fgm_opt) );
}


void addParks(Map* map)
{
    // create a feature source to load the shapefile.
    OGRFeatureOptions feature_opt;
    feature_opt.name() = "parks";
    feature_opt.url() = PARKS_URL;
    feature_opt.buildSpatialIndex() = true;

    // a style:
    Style style;
    style.setName( "parks" );

    // Render the data using point-model substitution, which replaces each point
    // in the feature geometry with an instance of a 3D model. Since the input
    // data are polygons, the PLACEMENT_RANDOM directive below will scatter
    // points within the polygon boundary at the specified density.
    ModelSymbol* model = style.getOrCreate<ModelSymbol>();
    model->url()->setLiteral(TREE_MODEL_URL);
    model->scale()->setLiteral( 0.2 );
    model->placement() = model->PLACEMENT_RANDOM;
    model->density() = 3000.0f; // instances per sqkm
    
    // Clamp to the terrain:
    AltitudeSymbol* alt = style.getOrCreate<AltitudeSymbol>();
    alt->clamping() = alt->CLAMP_TO_TERRAIN;

    // Since the tree model contains alpha components, we will discard any data
    // that's sufficiently transparent; this will prevent depth-sorting anomolies
    // common when rendering lots of semi-transparent objects.
    RenderSymbol* render = style.getOrCreate<RenderSymbol>();
    render->minAlpha() = 0.15f;

    // Set up a paging layout. The tile size factor and the visibility range combine
    // to determine the tile size, such that tile radius = max range / tile size factor.
    FeatureDisplayLayout layout;
    layout.tileSizeFactor() = 3.0f;
    layout.maxRange()       = 2000.0f;

    // create a model layer that will render the buildings according to our style sheet.
    FeatureGeomModelOptions fgm_opt;
    fgm_opt.featureOptions() = feature_opt;
    fgm_opt.layout() = layout;
    fgm_opt.styles() = new StyleSheet();
    fgm_opt.styles()->addStyle( style );
    fgm_opt.compilerOptions().instancing() = true;

    map->addModelLayer( new ModelLayer("parks", fgm_opt) );
}
