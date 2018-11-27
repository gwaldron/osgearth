/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2018 Pelican Mapping
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

#include <osgEarth/MapNode>
#include <osgEarth/GDAL>
#include <osgEarth/ImageLayer>
#include <osgEarth/ModelLayer>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>

#include <osgEarthSymbology/Style>
#include <osgEarthFeatures/OGRFeatureLayer>
#include <osgEarthFeatures/FeatureModelLayer>

#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/agglite/AGGLiteOptions>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;

int usage( const std::string& app )
{
    OE_NOTICE "\n" << app << "\n"
        << "  --rasterize           : draw features as rasterized image tiles \n"
        << "  --drape               : draw features as projected texture \n"
        << "  --clamp               : draw features using shader clamping \n"
        << "  --mem                 : load features from memory \n"
        << "  --labels              : add feature labels \n"
        << "\n"
        << MapNodeHelper().usage()
        << std::endl;

    return 0;
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
    bool useMem     = arguments.read("--mem");
    bool useLabels  = arguments.read("--labels");
    bool useDraping = arguments.read("--drape");
    bool useClamping = arguments.read("--clamp");

    osgViewer::Viewer viewer(arguments);

    // Start by creating the map:
    Map* map = new Map();

    // Start with a basemap imagery layer; we'll be using the GDAL driver
    // to load a local GeoTIFF file:
    GDALImageLayer* basemap = new GDALImageLayer();
    basemap->setURL("../data/world.tif");
    map->addLayer(basemap);
    
    // Next we add a layer to provide the feature data. 
    OGRFeatureLayer* features = new OGRFeatureLayer();
    features->setName("vector-data");
    if ( !useMem )
    {
        features->setURL("../data/world.shp");
    }
    else
    {
        // the --mem options tells us to just make an in-memory geometry:
        Ring* line = new Ring();
        line->push_back( osg::Vec3d(-60, 20, 0) );
        line->push_back( osg::Vec3d(-120, 20, 0) );
        line->push_back( osg::Vec3d(-120, 60, 0) );
        line->push_back( osg::Vec3d(-60, 60, 0) );
        features->setGeometry(line);
    }
    map->addLayer(features);

    // Define a style for the feature data. Since we are going to render the
    // vectors as lines, configure the line symbolizer:
    Style style;

    LineSymbol* ls = style.getOrCreateSymbol<LineSymbol>();
    ls->stroke()->color() = Color::Yellow;
    ls->stroke()->width() = 2.0f;
    ls->tessellationSize()->set(100, Units::KILOMETERS);

    if (useDraping)
    {
        AltitudeSymbol* alt = style.getOrCreate<AltitudeSymbol>();
        alt->clamping() = alt->CLAMP_TO_TERRAIN;
        alt->technique() = alt->TECHNIQUE_DRAPE;
    }

    else if (useClamping)
    {
        AltitudeSymbol* alt = style.getOrCreate<AltitudeSymbol>();
        alt->clamping() = alt->CLAMP_TO_TERRAIN;
        alt->technique() = alt->TECHNIQUE_GPU;

        ls->tessellationSize()->set(100, Units::KILOMETERS);

        RenderSymbol* render = style.getOrCreate<RenderSymbol>();
        render->depthOffset()->enabled() = true;
    }
    
    if (useRaster)
    {
        //TODO
        //AGGLiteOptions rasterOptions;
        //rasterOptions.featureOptions() = featureData;
        //rasterOptions.styles() = new StyleSheet();
        //rasterOptions.styles()->addStyle( style );
        //map->addLayer(new ImageLayer(rasterOptions) );
    }

    else //if (useGeom)
    {
        FeatureModelLayer* layer = new FeatureModelLayer();
        layer->setFeatureSource(features);

        StyleSheet* styleSheet = new StyleSheet();
        styleSheet->addStyle(style);
        layer->setStyleSheet(styleSheet);

        //fml.enableLighting() = false;

        map->addLayer(layer);
    }   

    if ( useLabels && !useRaster )
    {
        // set up symbology for drawing labels. We're pulling the label
        // text from the name attribute, and its draw priority from the
        // population attribute.
        Style labelStyle;

        TextSymbol* text = labelStyle.getOrCreateSymbol<TextSymbol>();
        text->content() = StringExpression( "[cntry_name]" );
        text->priority() = NumericExpression( "[pop_cntry]" );
        text->size() = 16.0f;
        text->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
        text->fill()->color() = Color::White;
        text->halo()->color() = Color::DarkGray;

        // and configure a model layer:
        FeatureModelLayerOptions fml;
        fml.name() = "Labels";
        fml.featureLayer() = "vector-data";
        fml.styles() = new StyleSheet();
        fml.styles()->addStyle( labelStyle );
        map->addLayer(new FeatureModelLayer(fml));
    }

    // That's it, the map is ready; now create a MapNode to render the Map:
    MapNode* mapNode = new MapNode(map);

    viewer.setSceneData( mapNode );
    viewer.setCameraManipulator( new EarthManipulator() );

    // add some stock OSG handlers:
    MapNodeHelper().configureView(&viewer);

    return viewer.run();
}
