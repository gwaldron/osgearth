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

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>

#include <osgEarth/Units>
#include <osgEarth/MapNode>

// include for WFS feature driver:
#include <osgEarthDrivers/feature_wfs/WFSFeatureOptions>

// include for feature geometry model renderer:
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthSymbology/Style>


#define LC "[wfs example] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
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

    // Get the bounds from the command line.
    Bounds bounds;
    double xmin = DBL_MAX, ymin = DBL_MAX, xmax = DBL_MIN, ymax = DBL_MIN;
    while (arguments.read("--bounds", xmin, ymin, xmax, ymax))
    {
        bounds.xMin() = xmin, bounds.yMin() = ymin, bounds.xMax() = xmax, bounds.yMax() = ymax;
    }        

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
        wfs.url()          = osgEarth::URI("http://demo.mapserver.org/cgi-bin/wfs"); 
        wfs.typeName()     = "cities"; 
        wfs.outputFormat() = "gml2";     // JSON or GML

        // Create the feature source from the options
        osg::ref_ptr< FeatureSource > featureSource = FeatureSourceFactory::create(wfs);
        Status s = featureSource->open();

        // Set the query with the bounds if one was specified.
        Query query;
        if (bounds.isValid())
        {
            query.bounds() = bounds;
        }

        // Get the features
        osg::ref_ptr< FeatureCursor > cursor = featureSource->createFeatureCursor(query, 0L);
        FeatureList features;
        cursor->fill(features);
        OE_NOTICE << "Got " << features.size() << " features" << std::endl;

        // Create a style
        Style style;
        style.getOrCreateSymbol<TextSymbol>()->content() = StringExpression("[NAME]");

        // Create the FeatureNode with the features and the style.
        osg::ref_ptr< FeatureNode > featureNode = new FeatureNode(features, style);
        mapNode->addChild(featureNode.get());                

        viewer.setSceneData( node );
        return viewer.run();
    }
    else
    {
        return usage(argv[0]);
    }
}
