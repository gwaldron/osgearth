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

#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarth/ElevationLayer>
#include <osgEarth/ModelLayer>
#include <osgEarth/GeoTransform>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/AutoScaleCallback>

#include <osgEarthDrivers/tms/TMSOptions>
#include <osgEarthDrivers/wms/WMSOptions>
#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgEarthDrivers/osg/OSGOptions>
#include <osgEarthDrivers/xyz/XYZOptions>

#include <osg/PositionAttitudeTransform>

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

    // add a semi-transparent XYZ layer:
    XYZOptions xyz;
    xyz.url() = "http://[abc].tile.openstreetmap.org/{z}/{x}/{y}.png";
    xyz.profile()->namedProfile() = "spherical-mercator";
    ImageLayer* imageLayer = new ImageLayer("OSM", xyz);
    imageLayer->setOpacity(0.5f);
    map->addLayer(imageLayer);
    
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

    // add a local simple image as a layer using the OSG driver:
    OSGOptions osg;
    osg.url() = "../data/osgearth.gif";
    osg.profile()->srsString() = "wgs84";
    osg.profile()->bounds()->set(-90.0, 10.0, -80.0, 15.0);
    map->addLayer(new ImageLayer("Simple image", osg));

    // put a model on the map atop Pike's Peak, Colorado, USA
    osg::ref_ptr<osg::Node> model = osgDB::readRefNodeFile("cow.osgt.(0,0,3).trans.osgearth_shadergen");
    if (model.valid())
    {
        osg::PositionAttitudeTransform* pat = new osg::PositionAttitudeTransform();
        pat->addCullCallback(new AutoScaleCallback<osg::PositionAttitudeTransform>(5.0));
        pat->addChild(model.get());

        GeoTransform* xform = new GeoTransform();
        xform->setPosition(GeoPoint(SpatialReference::get("wgs84"), -105.042292, 38.840829));
        xform->addChild(pat);

        map->addLayer(new ModelLayer("Model", xform));
    }

    // make the map scene graph:
    MapNode* node = new MapNode( map );

    // initialize a viewer:
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new EarthManipulator() );
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
    viewer.setSceneData( node );

    // add some stock OSG handlers:
    MapNodeHelper().configureView(&viewer);

    return viewer.run();
}