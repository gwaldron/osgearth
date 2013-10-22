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
#include <osgGA/GUIEventHandler>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgEarth/MapNode>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/SkyNode>
#include <osgEarthDrivers/tms/TMSOptions>
#include <osgEarthDrivers/xyz/XYZOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;

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

    // add a TMS imagery layer:
    TMSOptions imagery;
    imagery.url() = "http://readymap.org/readymap/tiles/1.0.0/22/";
    map->addImageLayer( new ImageLayer("ReadyMap imagery", imagery) );

    // create a feature source to load the building footprint shapefile.
    OGRFeatureOptions feature_opt;
    feature_opt.name() = "buildings";
    feature_opt.url() = "../data/boston_buildings_utm19.shp";
    feature_opt.buildSpatialIndex() = true;
    
    // a style for the building data:
    Style buildingStyle;
    buildingStyle.setName( "buildings" );

    ExtrusionSymbol* extrusion = buildingStyle.getOrCreate<ExtrusionSymbol>();
    extrusion->heightExpression() = NumericExpression( "3.5 * max( [story_ht_], 1 )" );
    extrusion->flatten() = true;
    extrusion->wallStyleName() = "building-wall";
    extrusion->roofStyleName() = "building-roof";
    extrusion->wallGradientPercentage() = 0.8;

    PolygonSymbol* poly = buildingStyle.getOrCreate<PolygonSymbol>();
    poly->fill()->color() = Color::White;

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
    ResourceLibrary* reslib = new ResourceLibrary( "us_resources", "../data/resources/textures_us/catalog.xml" );
    styleSheet->addResourceLibrary( reslib );

    // set up a paging layout for incremental loading.
    FeatureDisplayLayout layout;
    layout.tileSizeFactor() = 52.0;
    layout.addLevel( FeatureLevel(0.0f, 20000.0f, "buildings") );

    // create a model layer that will render the buildings according to our style sheet.
    FeatureGeomModelOptions fgm_opt;
    fgm_opt.featureOptions() = feature_opt;
    fgm_opt.styles() = styleSheet;
    fgm_opt.layout() = layout;

    map->addModelLayer( new ModelLayer( "buildings", fgm_opt ) );


    // initialize a viewer:
    osgViewer::Viewer viewer(arguments);
    EarthManipulator* manip = new EarthManipulator();
    viewer.setCameraManipulator( manip );

    // make the map scene graph:
    osg::Group* root = new osg::Group();
    viewer.setSceneData( root );

    MapNode* mapNode = new MapNode( map );
    root->addChild( mapNode );
    
    // Process cmdline args
    MapNodeHelper helper;
    helper.configureView( &viewer );
    helper.parse(mapNode, arguments, &viewer, root, new LabelControl("City Demo"));

    // zoom to a good startup position
    manip->setViewpoint( Viewpoint(-71.0763, 42.34425, 0, 24.261, -21.6, 3450.0), 5.0 );

    viewer.getCamera()->addCullCallback( new AutoClipPlaneCullCallback(mapNode) );

    return viewer.run();
}
