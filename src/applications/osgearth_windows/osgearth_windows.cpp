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

#include <osgViewer/CompositeViewer>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <iostream>

#define LC "[osgearth_windows] "

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

struct App
{
    osgViewer::CompositeViewer _viewer;
    bool _sharedGC;
    int _size;
    osg::ref_ptr<osg::Node> _node;

    App(osg::ArgumentParser& args) :
        _viewer(args),
        _size(500)
    {
        _viewer.setThreadingModel(_viewer.SingleThreaded);
        _sharedGC = args.read("--shared");
    }

    void addView()
    {
        int i = _viewer.getNumViews();
        osgViewer::View* view = new osgViewer::View();
        int width = _sharedGC ? _size * _viewer.getNumViews() : _size;
        view->setUpViewInWindow(10 + (i*_size) + 30, 10, width, _size);
        view->setCameraManipulator(new EarthManipulator());
        view->setSceneData(_node.get());
        if (_sharedGC)
        {
            view->getCamera()->setViewport(i*_size, 0, _size, _size);
            view->getCamera()->setProjectionMatrixAsPerspective(45, 1, 1, 10);
            view->getCamera()->setName(Stringify() << "View " << i);
            if (i > 0)
            {
                view->getCamera()->setGraphicsContext(_viewer.getView(0)->getCamera()->getGraphicsContext());
            }
        }
        MapNodeHelper().configureView(view);
        _viewer.addView(view);
    }

    void releaseGLObjects()
    {
        OE_NOTICE << "Calling releaseGLObjects" << std::endl;
        _viewer.releaseGLObjects(nullptr);
    }
};


struct AddWindow : public osgGA::GUIEventHandler
{
    App& _app;
    bool _sharedGC;
    int _size;

    AddWindow(App& app) :
        _app(app)
    { }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override
    {
        if (ea.getEventType() == ea.KEYUP && ea.getKey() == ea.KEY_N)
        {
            _app.addView();
            return true;
        }
        return false;
    }
};


int
main(int argc, char** argv)
{
    osgEarth::initialize();

    osgEarth::Registry::instance()->unRefImageDataAfterApply() = false;

    osg::ArgumentParser arguments(&argc,argv);

    // help?
    if ( arguments.read("--help") )
        return usage(argv[0]);

    int numViews = 1;
    arguments.read("--views", numViews);

    // create a viewer:
    App app(arguments);

    app._node = MapNodeHelper().load(arguments, &app._viewer);
    if (!app._node.get())
        return usage(argv[0]);

    for(int i=0; i<numViews; ++i)
    {
        app.addView();
    }

    EventRouter* router = new EventRouter();
    app._viewer.getView(0)->addEventHandler(router);

    OE_NOTICE << "Press 'n' to create a new view" << std::endl;
    router->onKeyPress(EventRouter::KEY_N, [&]() { app.addView(); });

    OE_NOTICE << "Press 'r' to call releaseGLObjects" << std::endl;
    router->onKeyPress(EventRouter::KEY_R, [&]() { app.releaseGLObjects(); });

    return app._viewer.run();
}
