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

    bool useGeom = arguments.read("--geom");

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
    featureOpt.url() = "../data/world.shp";

    // Define a style for the feature data. Since we are going to render the
    // vectors as lines, configure the line symbolizer:
    Style* style = new Style;

    LineSymbol* ls = new LineSymbol;
    ls->stroke()->color() = osg::Vec4f( 1,1,0,0.1 ); // yellow
    ls->stroke()->width() = 1.0f;
    style->addSymbol(ls);

    // Now we'll choose the AGG-Lite driver to render the features. By the way, the
    // feature data is actually polygons, so we override that to treat it as lines.
    // We apply the feature driver and set the style as well.
    if ( !useGeom )
    {
        AGGLiteOptions worldOpt;
        worldOpt.featureOptions() = featureOpt;
        worldOpt.geometryTypeOverride() = Geometry::TYPE_LINESTRING;
        worldOpt.styles()->addStyle( style );
        map->addImageLayer( new ImageLayer( ImageLayerOptions("world", worldOpt) ) );
    }
    else
    {
        FeatureGeomModelOptions worldOpt;
        worldOpt.featureOptions() = featureOpt;
        worldOpt.geometryTypeOverride() = Geometry::TYPE_LINESTRING;
        worldOpt.styles()->addStyle( style );
        worldOpt.enableLighting() = false;
        worldOpt.depthTestEnabled() = false;
        map->addModelLayer( new ModelLayer( "my features", worldOpt ) );
    }

    // That's it, the map is ready; now create a MapNode to render the Map:
    MapNode* mapNode = new MapNode( map );


    viewer.setSceneData( mapNode );

    viewer.setCameraManipulator( new EarthManipulator() );

    viewer.addEventHandler( new osgEarth::Util::AutoClipPlaneHandler );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
