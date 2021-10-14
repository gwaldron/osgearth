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
#include <osgEarth/ImGui/ImGui>
#include <imgui_internal.h>

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
        << "\n          --updates [num] : Number of update traversals"
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
    osg::ref_ptr<MapNode> _mapNode;

    std::vector< osg::ref_ptr< osgViewer::View > > _viewsToRemove;

    App(osg::ArgumentParser& args) :
        _viewer(args),
        _size(800)
    {
        _viewer.setThreadingModel(_viewer.SingleThreaded);
        _sharedGC = args.read("--shared");
    }

    void addView(const std::string& name)
    {
        int i = _viewer.getNumViews();

        int x = 10 + i*(_size + 20);
        osg::GraphicsContext* gc_to_share = _sharedGC && i > 0 ? _viewer.getView(0)->getCamera()->getGraphicsContext() : nullptr;

        osgViewer::View* view = createView(name, x, 10, _size, _size, gc_to_share);

        view->setCameraManipulator(new EarthManipulator());
        view->setSceneData(_node.get());
        MapNodeHelper().configureView(view);

        _viewer.addView(view);
    }

    osgViewer::View* createView(const std::string& name, int x, int y, int width, int height, osg::GraphicsContext* sharedGC)
    {
        osg::ref_ptr<osg::DisplaySettings>& ds = osg::DisplaySettings::instance();
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits(ds.get());
        traits->readDISPLAY();
        if (traits->displayNum < 0) traits->displayNum = 0;
        traits->screenNum = 1;
        traits->x = x;
        traits->y = y;
        traits->width = width;
        traits->height = height;
        traits->windowDecoration = true;
        traits->doubleBuffer = true;
        traits->sharedContext = sharedGC;

        osg::GraphicsContext* gc = osg::GraphicsContext::createGraphicsContext(traits.get());
        gc->setName(name);

        osgViewer::View* view = new osgViewer::View();
        view->getCamera()->setGraphicsContext(gc);
        //osg::GraphicsContext::incrementContextIDUsageCount(gc->getState()->getContextID());

        view->getCamera()->setViewport(0, 0, width, height);
        view->getCamera()->setProjectionMatrixAsPerspective(45, 1, 1, 10);

        GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
        view->getCamera()->setDrawBuffer(buffer);
        view->getCamera()->setReadBuffer(buffer);
        
        return view;
    }

    void releaseGLObjects(osg::State* state)
    {
        osgViewer::ViewerBase::Cameras cameras;
        _viewer.getCameras(cameras);
        for (auto c : cameras)
        {
            c->releaseGLObjects(state);
        }
    }
};

struct GCPanel : public GUI::BaseGUI
{
    App& _app;
    GCPanel(App& app) : GUI::BaseGUI("Graphics Contexts"), _app(app) { }

    void draw(osg::RenderInfo& ri) override
    {
        if (!isVisible()) return;
        ImGui::Begin(name(), visible());
        auto gcs = osg::GraphicsContext::getAllRegisteredGraphicsContexts();
        for (auto gc : gcs)
        {
            if (gc->getState() != nullptr)
            {
                ImGui::Text("Context ID = %d", gc->getState()->getContextID());
                ImGui::Indent();
                ImGui::Text("Name = %s", gc->getName().c_str());
                ImGui::Text("Operations = %d", gc->getGraphicsThread() && gc->getGraphicsThread()->getOperationQueue() ? gc->getGraphicsThread()->getOperationQueue()->getNumOperationsInQueue() : 0);
                ImGui::Text("Size = %d x %d", gc->getTraits() ? gc->getTraits()->width : -1, gc->getTraits() ? gc->getTraits()->height : -1);
                if (ImGui::Button("release GL objects"))
                {
                    _app.releaseGLObjects(gc->getState());
                }
                ImGui::Unindent();
            }
        }

        if (ImGui::Button("New window"))
        {
            std::string name = Stringify() << "Window " << _app._viewer.getNumViews();
            _app.addView(name);
        }

        ImGui::End();
    }
};

struct ViewerPanel : public GUI::BaseGUI
{
    App& _app;
    ViewerPanel(App& app) : GUI::BaseGUI("Views"), _app(app) { }

    void draw(osg::RenderInfo& ri) override
    {
        if (!isVisible()) return;
        ImGui::Begin(name(), visible());

        if (ImGui::Button("New view"))
        {
            std::string name = Stringify() << "View " << _app._viewer.getNumViews();
            _app.addView(name);
        }

        if (ImGui::Button("Release GL Objects"))
        {
            _app.releaseGLObjects(nullptr);
        }

        ImGui::Text("Active Views");
        osgViewer::ViewerBase::Views views;
        _app._viewer.getViews(views);
        int ptr = 0;
        for (auto view : views)
        {
            ImGui::PushID(view);
            ImGui::Text("View #%d", ptr++);
            ImGui::Indent();

            if (view->getCamera() && view->getCamera()->getGraphicsContext() && view->getCamera()->getGraphicsContext()->getState())
            {
                ImGui::Text("GC = %d", view->getCamera()->getGraphicsContext()->getState()->getContextID());
            }
            else
            {
                ImGui::Text("**Invalid**");
            }

            if (ImGui::Button("close"))
            {
                OE_WARN << "Closing a view" << std::endl;
                _app._viewsToRemove.push_back(view);
            }

            ImGui::Unindent();
            ImGui::PopID();
        }            

        ImGui::End();
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

    int numUpdates = 1;
    arguments.read("--updates", numUpdates);


    // create a viewer:
    App app(arguments);

    // Setup the viewer for imgui
    app._viewer.setRealizeOperation(new GUI::ApplicationGUI::RealizeOperation);

    app._node = MapNodeHelper().loadWithoutControls(arguments, &app._viewer);
    if (!app._node.get())
        return usage(argv[0]);

    app._mapNode = MapNode::get(app._node.get());

    for(int i=0; i<numViews; ++i)
    {
        app.addView(Stringify() << "View " << i);
    }

    auto view = app._viewer.getView(0);

    // install the Gui.
    GUI::ApplicationGUI* gui = new GUI::ApplicationGUI();
    gui->addAllBuiltInTools();
    gui->add(new ViewerPanel(app), true);
    gui->add(new GCPanel(app), true);
    view->getEventHandlers().push_front(gui);

    OE_NOTICE << "Press 'n' to create a new view" << std::endl;
    EventRouter::get(view).onKeyPress(EventRouter::KEY_N, [&]() { 
        std::cout << "Creating new view" << std::endl;
        app.addView(Stringify()<<"View " << app._viewer.getNumViews()); });

    OE_NOTICE << "Press 'r' to call releaseGLObjects" << std::endl;
    EventRouter::get(view).onKeyPress(EventRouter::KEY_R, [&]() { 
        app.releaseGLObjects(nullptr);
    });

    app._viewer.realize();

    while (!app._viewer.done())
    {
        for (unsigned int i = 0; i < app._viewsToRemove.size(); ++i)
        {            
            app._viewer.removeView(app._viewsToRemove[i].get());
            // Set the scene data to null before the View is destroyed when the vector is cleared outside of this loop to prevent
            // osg from sending down a releaseGLObjects with a null state on camera destruction.  We want
            // releaseGLObjects to get called on the view that is being removed, not all of them.
            app._viewsToRemove[i]->setSceneData(nullptr);         
        }
        app._viewsToRemove.clear();

        app._viewer.advance();
        app._viewer.eventTraversal();
        for (int i = 0; i < numUpdates; ++i)
            app._viewer.updateTraversal();
        app._viewer.renderingTraversals();
    }
    return 0;
}
