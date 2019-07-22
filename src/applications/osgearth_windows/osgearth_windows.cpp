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

#include <osgViewer/CompositeViewer>
#include <osgEarth/Notify>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Metrics>
#include <iostream>

#define LC "[viewer] "

using namespace osgEarth;
using namespace osgEarth::Util;

int
usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " file.earth"
        << "\n          --views [num] : Number of windows to open"
        << "\n          --shared      : Use a shared graphics context"
        << "\n"
        << MapNodeHelper().usage() << std::endl;

    return 0;
}


int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    int numViews = 1;
    arguments.read("--views", numViews);

    bool sharedGC;
    sharedGC = arguments.read("--shared");

    // create a viewer:
    osgViewer::CompositeViewer viewer(arguments);
    viewer.setThreadingModel(viewer.SingleThreaded);

    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (!node)
        return usage(argv[0]);

    int size = 500;

    for(int i=0; i<numViews; ++i)
    {
        osgViewer::View* view = new osgViewer::View();
        int width = sharedGC? size*numViews : size;
        view->setUpViewInWindow(10+(i*size+30), 10, width, size);
        view->setCameraManipulator(new EarthManipulator(arguments));
        view->setSceneData(node);
        view->getDatabasePager()->setUnrefImageDataAfterApplyPolicy( true, false );
        if (sharedGC)
        {
            view->getCamera()->setViewport(i*size, 0, size, size);
            view->getCamera()->setProjectionMatrixAsPerspective(45, 1, 1, 10);
            view->getCamera()->setName(Stringify()<<"View "<<i);
        }
        MapNodeHelper().configureView(view);
        viewer.addView(view);
    }

    if (sharedGC)
    {
        for(int i=1; i<numViews; ++i)
        {
            osgViewer::View* view = viewer.getView(i);
            view->getCamera()->setGraphicsContext(viewer.getView(0)->getCamera()->getGraphicsContext());
        }
    }

    return viewer.run();
}
