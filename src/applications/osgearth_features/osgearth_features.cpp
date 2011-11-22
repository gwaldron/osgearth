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
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>

#include <osgEarthSymbology/Style>
#include <osgEarthFeatures/ConvertTypeFilter>

#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/agglite/AGGLiteOptions>
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>
#include <osgEarthDrivers/model_feature_stencil/FeatureStencilModelOptions>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;

//
// NOTE: run this sample from the repo/tests directory.
//
int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osg::DisplaySettings::instance()->setMinimumNumStencilBits( 8 );

    bool useRaster  = arguments.read("--rasterize");
    bool useOverlay = arguments.read("--overlay");
    bool useStencil = arguments.read("--stencil");
    bool useMem     = arguments.read("--mem");
    bool useLabels  = arguments.read("--labels");

    osgViewer::Viewer viewer(arguments);

    // Start by creating the map:
    Map* map = new Map();

    // Start with a basemap imagery layer; we'll be using the GDAL driver
    // to load a local GeoTIFF file:
    GDALOptions basemapOpt;
    basemapOpt.url() = "../data/world.tif";
    map->addImageLayer( new ImageLayer( ImageLayerOptions("basemap", basemapOpt) ) );

    // Next we add a feature layer. 
    OGRFeatureOptions featureOptions;
    if ( !useMem )
    {
        // Configures the feature driver to load the vectors from a shapefile:
        featureOptions.url() = "../data/usa.shp";

        // installs an inline filter to convert geometry to lines
        ConvertTypeFilter* filter = new ConvertTypeFilter();
        filter->toType() = Geometry::TYPE_LINESTRING;
        featureOptions.filters().push_back( filter );
    }
    else
    {
        // the --mem options tells us to just make an in-memory geometry:
        Ring* line = new Ring();
        line->push_back( osg::Vec3d(-60, 20, 0) );
        line->push_back( osg::Vec3d(-120, 20, 0) );
        line->push_back( osg::Vec3d(-120, 60, 0) );
        line->push_back( osg::Vec3d(-60, 60, 0) );
        featureOptions.geometry() = line;
    }

    // Define a style for the feature data. Since we are going to render the
    // vectors as lines, configure the line symbolizer:
    Style style;

    LineSymbol* ls = style.getOrCreateSymbol<LineSymbol>();
    ls->stroke()->color() = Color::Yellow;
    ls->stroke()->width() = 2.0f;

    // That's it, the map is ready; now create a MapNode to render the Map:
    MapNodeOptions mapNodeOptions;
    mapNodeOptions.enableLighting() = false;
    MapNode* mapNode = new MapNode( map, mapNodeOptions );
   
    if (useStencil)
    {
        FeatureStencilModelOptions stencilOptions;
        stencilOptions.featureOptions() = featureOptions;
        stencilOptions.styles() = new StyleSheet();
        stencilOptions.styles()->addStyle( style );
        stencilOptions.enableLighting() = false;
        stencilOptions.depthTestEnabled() = false;
        map->addModelLayer( new ModelLayer("my features", stencilOptions) );
    }
    else if (useRaster)
    {
        AGGLiteOptions rasterOptions;
        rasterOptions.featureOptions() = featureOptions;
        rasterOptions.styles() = new StyleSheet();
        rasterOptions.styles()->addStyle( style );
        map->addImageLayer( new ImageLayer("my features", rasterOptions) );
    }
    else //if (useGeom || useOverlay)
    {
        FeatureGeomModelOptions geomOptions;
        geomOptions.featureOptions() = featureOptions;
        geomOptions.styles() = new StyleSheet();
        geomOptions.styles()->addStyle( style );
        geomOptions.enableLighting() = false;

        ModelLayerOptions layerOptions( "my features", geomOptions );
        layerOptions.overlay() = useOverlay;
        map->addModelLayer( new ModelLayer(layerOptions) );
    }

    if ( useLabels )
    {
        // set up symbology for drawing labels. We're pulling the label
        // text from the "name" attribute, and its draw priority from the
        // "area" attribute (meaning bigger features get higher-priotity
        // labels.)
        Style labelStyle;

        TextSymbol* text = labelStyle.getOrCreateSymbol<TextSymbol>();
        text->content() = StringExpression( "[name]" );
        text->priority() = NumericExpression( "[area]" );
        text->removeDuplicateLabels() = true;
        text->size() = 16.0f;
        text->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
        text->fill()->color() = Color::White;
        text->halo()->color() = Color::DarkGray;

        // and configure a model layer:
        FeatureGeomModelOptions geomOptions;
        geomOptions.featureOptions() = featureOptions;
        geomOptions.styles() = new StyleSheet();
        geomOptions.styles()->addStyle( labelStyle );

        map->addModelLayer( new ModelLayer("labels", geomOptions) );
    }

    viewer.setSceneData( mapNode );
    viewer.setCameraManipulator( new EarthManipulator() );

    if ( !useStencil )
        viewer.getCamera()->addCullCallback( new osgEarth::Util::AutoClipPlaneCullCallback(map) );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
