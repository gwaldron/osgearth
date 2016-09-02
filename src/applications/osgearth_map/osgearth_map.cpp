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
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>
#include <osgEarthSymbology/Color>
#include <osgEarthDrivers/tms/TMSOptions>

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

    // create the map.
    Map* map = new Map();

    // add a TMS imager layer:
    TMSOptions imagery;
    imagery.url() = "http://readymap.org/readymap/tiles/1.0.0/7/";
    map->addImageLayer( new ImageLayer("Imagery", imagery) );

    // add a TMS elevation layer:
    TMSOptions elevation;
    elevation.url() = "http://readymap.org/readymap/tiles/1.0.0/9/";
    map->addElevationLayer( new ElevationLayer("Elevation", elevation) );

    // make the map scene graph:
    MapNode* node = new MapNode( map );

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
