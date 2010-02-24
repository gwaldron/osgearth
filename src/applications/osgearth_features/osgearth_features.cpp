/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarth/EarthFile>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>

#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/agglite/AGGLiteOptions>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarthUtil;

//
// NOTE: run this sample from the repo/tests directory.
//
int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    // Start by creating the map:
    Map* map = new Map();

    // Start with a basemap imagery layer; we'll be using the GDAL driver
    // to load a local GeoTIFF file:
    GDALOptions* basemapOpt = new GDALOptions();
    basemapOpt->url() = "../data/world.tif";
    map->addMapLayer( new ImageMapLayer( "basemap", basemapOpt ) );

    // Next we add a feature layer. First configure a feature driver to 
    // load the vectors from a shapefile:
    OGRFeatureOptions* featureOpt = new OGRFeatureOptions();
    featureOpt->url() = "../data/world.shp";

    // Define a style for the feature data. Since we are going to render the
    // vectors as lines, configure the line symbolizer:
    Style style;
    style.lineSymbolizer()->stroke()->color() = osg::Vec4f( 1,1,0,1 );
    style.lineSymbolizer()->stroke()->width() = 1.5f;

    // Now we'll choose the AGG-Lite driver to render the features. By the way, the
    // feature data is actually polygons, so we override that to treat it as lines.
    // We apply the feature driver and set the style as well.
    AGGLiteOptions* worldOpt = new AGGLiteOptions();
    worldOpt->featureOptions() = featureOpt;
    worldOpt->geometryTypeOverride() = Geometry::TYPE_LINESTRING;
    worldOpt->styles()->addStyle( style );
    map->addMapLayer( new ImageMapLayer("world", worldOpt) );

    // That's it, the map is ready; now create a MapNode to render the Map:
    MapNode* mapNode = new MapNode( map );


    viewer.setSceneData( mapNode );

    viewer.setCameraManipulator( new EarthManipulator() );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
