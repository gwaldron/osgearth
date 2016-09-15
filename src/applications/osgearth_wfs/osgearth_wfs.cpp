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

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

#include <osgEarth/Units>
#include <osgEarth/MapNode>

// include for WFS feature driver:
#include <osgEarthDrivers/feature_wfs/WFSFeatureOptions>

// include for feature geometry model renderer:
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>
#include <osgEarthSymbology/Style>


#define LC "[wfs example] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // general setup:
    osgViewer::Viewer viewer(arguments);
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );
    viewer.setCameraManipulator( new EarthManipulator(arguments) );
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
    viewer.getCamera()->setNearFarRatio(0.00002);

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {        
        MapNode* mapNode = MapNode::get(node);
        if ( !mapNode )
            return usage(argv[0]);

        // Create the WFS driver:
        osgEarth::Drivers::WFSFeatureOptions wfs;
        wfs.url()          = osgEarth::URI("http://demo.opengeo.org/geoserver/wfs"); 
        wfs.typeName()     = "states"; 
        wfs.outputFormat() = "json";     // JSON or GML

        // Configure a rendering style:
        Style style;
        style.setName( "states" ); 

        LineSymbol* line = style.getOrCreate<LineSymbol>(); 
        line->stroke()->color() = Color::Yellow; 
        line->stroke()->width() = 5.0f;
        line->stroke()->widthUnits() = Units::PIXELS;

        AltitudeSymbol* alt = style.getOrCreate<AltitudeSymbol>();
        alt->clamping()  = AltitudeSymbol::CLAMP_TO_TERRAIN;
        alt->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;

        // Configure a model layer to render the features:
        osgEarth::Drivers::FeatureGeomModelOptions geom; 
        geom.featureOptions() = wfs;
        geom.styles()         = new StyleSheet(); 
        geom.styles()->addStyle(style); 

        // Make the new layer and add it to the map.
        ModelLayerOptions layerOptions("states", geom); 
        ModelLayer* layer = new ModelLayer(layerOptions); 
        mapNode->getMap()->addModelLayer(layer);
        
        viewer.setSceneData( node );
        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
