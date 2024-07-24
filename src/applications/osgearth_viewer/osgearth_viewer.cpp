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
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/PhongLightingEffect>
#include <osgGA/TrackballManipulator>
#include <iostream>

#include <osgEarth/Metrics>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;

int
usage(const char* name)
{
    std::cout
        << "View an earth file: " << name << " file.earth" << std::endl
        << "View an OSG model: " << name << " [modelfile.ext] [--light]" << std::endl
        << Util::MapNodeHelper().usage() << std::endl;

    return 0;
}


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--help") )
        return usage(argv[0]);

    // start up osgEarth
    osgEarth::initialize(arguments);

    // create a simple view
    osgViewer::Viewer viewer(arguments);

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator(new EarthManipulator(arguments));

    // disable the small-feature culling; necessary for some feature rendering
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    // load an earth file, and support all or our example command-line options
    auto node = MapNodeHelper().load(arguments, &viewer);
    if (node.valid())
    {
        if (MapNode::get(node))
        {
            viewer.setSceneData(node);
            return viewer.run();
        }
        else
        {
            // not an earth file? Just view as a normal OSG node or image with basic lighting
            viewer.setCameraManipulator(new osgGA::TrackballManipulator);

            if (arguments.read("--light"))
            {
                osg::LightSource* lightSource = new osg::LightSource();
                lightSource->getLight()->setAmbient(osg::Vec4(0.75f, 0.75f, 0.75f, 1.0f));
                
                auto group = new PhongLightingGroup();
                group->addChild(lightSource);
                group->addChild(node);

                ShaderGenerator gen;
                gen.run(group);

                viewer.setSceneData(group);

                while (!viewer.done())
                {
                    auto cam = viewer.getCamera()->getInverseViewMatrix().getTrans();
                    cam.normalize();
                    lightSource->getLight()->setPosition(osg::Vec4d(cam.x(), cam.y(), cam.z(), 0));
                    viewer.frame();
                }
                return 0;
            }
            else
            {
                ShaderGenerator gen;
                gen.run(node);
                viewer.setSceneData(node);
                return viewer.run();
            }
        }
    }

    return usage(argv[0]);
}