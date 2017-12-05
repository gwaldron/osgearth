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
#include <osgGA/GUIEventHandler>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarth/GeoTransform>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>
#include <osgEarthSymbology/Color>
#include <osgEarthDrivers/tms/TMSOptions>
#include <osgEarthDrivers/wms/WMSOptions>
#include <osgEarthDrivers/gdal/GDALOptions>

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Util;

/**
 * How to create a simple osgEarth map and display it.
 */
int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // create the empty map.
    Map* map = new Map();

    // add a TMS imagery layer:
    TMSOptions imagery;
    imagery.url() = "http://readymap.org/readymap/tiles/1.0.0/7/";
    map->addLayer( new ImageLayer("ReadyMap Imagery", imagery) );

    // add a TMS elevation layer:
    TMSOptions elevation;
    elevation.url() = "http://readymap.org/readymap/tiles/1.0.0/116/";
    map->addLayer( new ElevationLayer("ReadyMap Elevation", elevation) );
    
    // add a local GeoTIFF inset layer:
    GDALOptions gdal;
    gdal.url() = "../data/boston-inset.tif";
    map->addLayer(new ImageLayer("Boston", gdal));

    // add a WMS radar layer with transparency, and disable caching since
    // this layer updates on the server periodically.
    WMSOptions wms;
    wms.url() = "http://mesonet.agron.iastate.edu/cgi-bin/wms/nexrad/n0r.cgi";
    wms.format() = "png";
    wms.layers() = "nexrad-n0r";
    wms.srs() = "EPSG:4326";
    wms.transparent() = true;
    ImageLayerOptions wmsLayerOptions("WMS NEXRAD", wms);
    wmsLayerOptions.cachePolicy() = CachePolicy::NO_CACHE;
    map->addLayer(new ImageLayer(wmsLayerOptions));

    // make the map scene graph:
    MapNode* node = new MapNode( map );

    // put a model on the map atop Pike's Peak, Colorado, USA
    osg::ref_ptr<osg::Node> model = osgDB::readRefNodeFile("../data/red_flag.osg.10000.scale.osgearth_shadergen");
    if (model.valid())
    {
        GeoTransform* xform = new GeoTransform();
        xform->addChild(model.get());
        xform->setPosition(GeoPoint(map->getSRS()->getGeographicSRS(), -105.042292, 38.840829));
        node->addChild(xform);
    }

    // initialize a viewer:
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator );
    viewer.setSceneData( node );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgViewer::LODScaleHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    return viewer.run();
}