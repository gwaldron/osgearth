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

#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>

#include <osgEarthSymbology/Style>
#include <osgEarthFeatures/ConvertTypeFilter>

#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/agglite/AGGLiteOptions>
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>

#include <osgDB/WriteFile>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;

int usage( const std::string& app )
{
    OE_NOTICE "\n" << app << "\n"
        << "    --rasterize           : draw features as rasterized image tiles \n"
        << "    --overlay             : draw features as projection texture \n"
        << "    --mem                 : load features from memory \n"
        << "    --labels              : add feature labels \n"
        << "\n"
        << MapNodeHelper().usage();

    return -1;
}

//
// NOTE: run this sample from the repo/tests directory.
//
int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    if ( arguments.read("--help") )
        return usage( argv[0] );

    bool useRaster  = arguments.read("--rasterize");
    bool useOverlay = arguments.read("--overlay");
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
        featureOptions.url() = "../data/world.shp";
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

    osg::Group* root = new osg::Group();
    root->addChild( mapNode );
    viewer.setSceneData( root );
    viewer.setCameraManipulator( new EarthManipulator() );

    // Process cmdline args
    MapNodeHelper().parse(mapNode, arguments, &viewer, root, new LabelControl("Features Demo"));
   
    if (useRaster)
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
        map->addModelLayer( new ModelLayer(layerOptions) );
    }

    if ( useLabels )
    {
        // set up symbology for drawing labels. We're pulling the label
        // text from the name attribute, and its draw priority from the
        // population attribute.
        Style labelStyle;

        TextSymbol* text = labelStyle.getOrCreateSymbol<TextSymbol>();
        text->content() = StringExpression( "[cntry_name]" );
        text->priority() = NumericExpression( "[pop_cntry]" );
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

    
    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}
