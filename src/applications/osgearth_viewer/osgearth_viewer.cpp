/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/Viewpoint>
#include <osgEarth/Random>

using namespace osgEarth;
using namespace osgEarth::Util;

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // create a viewer:
    osgViewer::Viewer viewer(arguments);
    EarthManipulator* em = new EarthManipulator(arguments);
    viewer.setCameraManipulator(em);

    const SpatialReference* srs = SpatialReference::get("wgs84");
    Random rng;

    // load an earth file, and support all or our example command-line options
    // and earth file <external> tags    
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if ( node )
    {
        viewer.setSceneData( node );

        Viewpoint h;
        h.setFocalPoint(GeoPoint(srs, -159.534195, 22.0250085, 1000, ALTMODE_ABSOLUTE));
        h.setRange(63100);
        h.setHeading(1.76);
        h.setPitch(-27.45);        
        em->setViewpoint(h);

        while(viewer.done() == false)
        {
            int fn = viewer.getFrameStamp()->getFrameNumber()+1;
            if (fn % 60 == 0)
            {
                double lat = -80 + rng.next()*160.0;
                double lon = -180 + rng.next()*360.0;
                double range = pow(10.0, 3.0+rng.next()*3.0);
                double heading = rng.next()*360.0;
                double pitch = -10+rng.next()*-79.0;

                Viewpoint v;
                v.setFocalPoint(GeoPoint(srs, lon, lat, 0.0, ALTMODE_ABSOLUTE));
                v.setHeading(heading);
                v.setPitch(pitch);
                v.setRange(range);

                em->setViewpoint(v, 0.8);
            }
            viewer.frame();
        }
    }
}
