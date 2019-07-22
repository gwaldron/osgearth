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

#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgEarth/MapNode>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/MouseCoordsTool>
#include <osgEarthUtil/MGRSFormatter>
#include <osgEarthUtil/LatLongFormatter>

#include <osgEarthUtil/GeodeticGraticule>
#include <osgEarthUtil/MGRSGraticule>
#include <osgEarthUtil/UTMGraticule>
#include <osgEarthUtil/GARSGraticule>

using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

int
usage( const std::string& msg )
{
    OE_NOTICE 
        << msg << std::endl
        << "USAGE: osgearth_graticule [options] file.earth" << std::endl
        << "   --geodetic            : display a Lat/Long graticule" << std::endl
        << "   --utm                 : display a UTM graticule" << std::endl
        << "   --mgrs                : display an MGRS graticule" << std::endl
        << "   --gars                : display a GARS graticule" << std::endl;
    return -1;
}

//------------------------------------------------------------------------

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);

    // parse command line:
    bool isUTM = arguments.read("--utm");
    bool isMGRS = arguments.read("--mgrs");
    bool isGeodetic = arguments.read("--geodetic");
    bool isGARS = arguments.read("--gars");

    // load the .earth file from the command line.
    MapNode* mapNode = MapNode::load( arguments );
    if ( !mapNode )
        return usage( "Failed to load a map from the .earth file" );

    // install our manipulator:
    viewer.setCameraManipulator( new EarthManipulator() );

    // root scene graph:
    osg::Group* root = new osg::Group();
    root->addChild( mapNode );

    Formatter* formatter = 0L;
    if ( isUTM )
    {
        UTMGraticule* gr = new UTMGraticule();
        mapNode->getMap()->addLayer(gr);
        formatter = new MGRSFormatter();
    }
    else if ( isMGRS )
    {
        MGRSGraticule* gr = new MGRSGraticule();
        mapNode->getMap()->addLayer(gr);
        formatter = new MGRSFormatter();
    }
    else if ( isGARS )
    {
        GARSGraticule* gr = new GARSGraticule();
        mapNode->getMap()->addLayer(gr);
        formatter = new LatLongFormatter();
    }
    else // if ( isGeodetic )
    {
        GeodeticGraticule* gr = new GeodeticGraticule();
        mapNode->getMap()->addLayer(gr);
        formatter = new LatLongFormatter();
    }
   
    // mouse coordinate readout:
    ControlCanvas* canvas = new ControlCanvas();
    root->addChild( canvas );
    VBox* vbox = new VBox();
    canvas->addControl( vbox );

    LabelControl* readout = new LabelControl();
    vbox->addControl( readout );

    MouseCoordsTool* tool = new MouseCoordsTool( mapNode );
    tool->addCallback( new MouseCoordsLabelCallback(readout, formatter) );
    viewer.addEventHandler( tool );


    // disable the small-feature culling
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // set a near/far ratio that is smaller than the default. This allows us to get
    // closer to the ground without near clipping. If you need more, use --logdepth
    viewer.getCamera()->setNearFarRatio(0.0001);


    // finalize setup and run.
    viewer.setSceneData( root );
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    return viewer.run();
}
