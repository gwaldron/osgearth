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
#include <osgEarthSymbology/GeometrySymbol>

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

    // Next we add a feature layer. First configure a feature driver to 
    // load the vectors from a shapefile:
    OGRFeatureOptions featureOpt;
    if ( !useMem )
    {
        featureOpt.url() = "../data/usa.shp";
    }
    else
    {
        Ring* line = new Ring();
        line->push_back( osg::Vec3d(-60, 20, 0) );
        line->push_back( osg::Vec3d(-120, 20, 0) );
        line->push_back( osg::Vec3d(-120, 60, 0) );
        line->push_back( osg::Vec3d(-60, 60, 0) );
        featureOpt.geometry() = line;
    }

    // Define a style for the feature data. Since we are going to render the
    // vectors as lines, configure the line symbolizer:
    Style style;

    LineSymbol* ls = style.getOrCreateSymbol<LineSymbol>();
    ls->stroke()->color() = osg::Vec4f( 1,1,0,1 ); // yellow
    ls->stroke()->width() = 2.0f;

    // Add some text labels.
    if ( useLabels )
    {
        TextSymbol* text = style.getOrCreateSymbol<TextSymbol>();
        text->provider() = "overlay";
        text->content() = StringExpression( "[name]" );
        text->priority() = NumericExpression( "[area]" );
        text->removeDuplicateLabels() = true;
        text->size() = 16.0f;
        text->fill()->color() = Color::White;
        text->halo()->color() = Color::DarkGray;
    }

    // That's it, the map is ready; now create a MapNode to render the Map:
    MapNodeOptions mapNodeOptions;
    mapNodeOptions.enableLighting() = false;

    MapNode* mapNode = new MapNode( map, mapNodeOptions );

    // Now we'll choose the AGG-Lite driver to render the features. By the way, the
    // feature data is actually polygons, so we override that to treat it as lines.
    // We apply the feature driver and set the style as well.
    if (useStencil)
    {
        FeatureStencilModelOptions worldOpt;
        worldOpt.featureOptions() = featureOpt;
        worldOpt.geometryTypeOverride() = Geometry::TYPE_LINESTRING;
        worldOpt.styles()->addStyle( style );
        worldOpt.enableLighting() = false;
        worldOpt.depthTestEnabled() = false;
        map->addModelLayer( new ModelLayer( "my features", worldOpt ) );
    }
    else if (useRaster)
    {
        AGGLiteOptions worldOpt;
        worldOpt.featureOptions() = featureOpt;
        worldOpt.geometryTypeOverride() = Geometry::TYPE_LINESTRING;
        worldOpt.styles()->addStyle( style );
        map->addImageLayer( new ImageLayer( ImageLayerOptions("world", worldOpt) ) );
    }
    else //if (useGeom || useOverlay)
    {
        FeatureGeomModelOptions worldOpt;
        worldOpt.featureOptions() = featureOpt;
        worldOpt.geometryTypeOverride() = Geometry::TYPE_LINESTRING;
        worldOpt.styles()->addStyle( style );
        worldOpt.enableLighting() = false;
        worldOpt.depthTestEnabled() = false;

        ModelLayerOptions options( "my features", worldOpt );
        options.overlay() = useOverlay;
        map->addModelLayer( new ModelLayer(options) );
    }

    viewer.setSceneData( mapNode );
    viewer.setCameraManipulator( new EarthManipulator() );

    if ( !useStencil && !useOverlay )
        viewer.addEventHandler( new osgEarth::Util::AutoClipPlaneHandler );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
