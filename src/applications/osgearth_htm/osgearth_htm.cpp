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

#include <osgViewer/Viewer>
#include <osgEarth/Notify>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/HTM>
#include <osgEarth/PlaceNode>
#include <osgEarth/Random>
#include <osgDB/ReadFile>
#include <iostream>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Contrib;

int
usage(const char* name, const char* msg =NULL)
{
    OE_NOTICE 
        << (msg ? msg : "")
        << "\nUsage: " << name << " file.earth --model <file> [--num <number>] [--debug]" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    if ( arguments.read("--help") )
        return usage(argv[0]);

    // Viewer setup
    osgViewer::Viewer viewer(arguments);
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( true, false );
    osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");
    viewer.setCameraManipulator( new EarthManipulator(arguments) );
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
    viewer.getCamera()->setNearFarRatio(0.0001);

    std::string modelPath;
    if (arguments.read("--model", modelPath) == false)
        return usage(argv[0]);

    osg::ref_ptr<osg::Node> model = osgDB::readRefNodeFile(modelPath);
    if (model.valid() == false)
        return usage(argv[0], "Cannot load model file");

    int numObjects = 10000;
    arguments.read("--num", numObjects);

    bool debug = arguments.read("--debug");

    // Load the earth file
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (!node)
        return usage(argv[0], "Cannot load earth file");

    viewer.setSceneData( node );
    MapNode* mapNode = MapNode::get(node);

    // Randomly place object instances around the US.
    const SpatialReference* wgs84 = SpatialReference::get("wgs84");
    Random prng;
    HTMGroup* htm = new HTMGroup();
    htm->setMaximumObjectsPerCell(250);
    htm->setMaximumCellSize(500000);
    htm->setMinimumCellSize(25000);
    htm->setRangeFactor(5);
    mapNode->addChild(htm);
        
    for (unsigned i = 0; i < numObjects; ++i)
    {
        GeoTransform* xform = new GeoTransform();
        xform->addChild(model.get());

        double lon = -115 + prng.next() * 40;
        double lat = 25 + prng.next() * 25;

        xform->setPosition(GeoPoint(wgs84, lon, lat, 0, ALTMODE_ABSOLUTE));

        htm->addChild(xform);

        if (i%1000 == 0)
            std::cout << "\r" << i << "/" << numObjects << std::flush;
    }
    std::cout << std::endl;

    return viewer.run();
}
