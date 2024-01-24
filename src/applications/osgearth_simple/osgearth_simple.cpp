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
#include <osgDB/ReadFile>
#include <osgEarth/MapNode>
#include <osgEarth/EarthManipulator>
#include <osgEarth/GLUtils>
#include <osgEarth/LogarithmicDepthBuffer>
#include <iostream>


int
usage(const char* name, const char* message)
{
    std::cerr << "Error: " << message << std::endl;
    std::cerr << "Usage: " << name << " file.earth" << std::endl;
    return -1;
}


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if (argc < 2)
        return usage(argv[0], "Missing earth file");

    // One time osgEarth initialization:
    osgEarth::initialize(arguments);

    // Load the earth file:
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFiles(arguments);
    if (!node.valid())
        return usage(argv[0], "File not found");

    // If the node doesn't contain a MapNode (earth file), bail out:
    auto mapNode = osgEarth::MapNode::get(node);
    if (!mapNode)
        return usage(argv[0], "No MapNode in file");

    // Set up a simple viewer with our custom manipulator:
    osgViewer::Viewer viewer(arguments);
    viewer.setSceneData(node);
    viewer.setCameraManipulator(new osgEarth::Util::EarthManipulator(arguments));

    // This is optional, but it will mitigate near-plane clipping in a whole earth scene:
    osgEarth::Util::LogarithmicDepthBuffer ldb;
    ldb.install(viewer.getCamera());

#ifndef OSG_GL3_AVAILABLE
    // If your OSG is build with a GL2 profile, install our custom realize op
    // to get all the shaders working:
    viewer.setRealizeOperation(new osgEarth::GL3RealizeOperation());
#endif

    return viewer.run();
}