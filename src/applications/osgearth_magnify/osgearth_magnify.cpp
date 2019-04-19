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

#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarth/MapNode>
#include <osgUtil/CullVisitor>

#define LC "[magnify] "

using namespace osgEarth;
using namespace osgEarth::Util;
namespace ui = osgEarth::Util::Controls;

int usage(const char* name)
{
    OE_NOTICE 
        << "\nUsage: " << name << " file.earth" << std::endl
        << MapNodeHelper().usage() << std::endl;

    return 0;
}

// Application-wide data
struct App
{
    osgViewer::View* _mainView;
    osgViewer::View* _magView;
    ui::HSliderControl* _magSlider;
    bool _useLODScale;

    App()
    {
        _useLODScale = true;
    }

    float computeRangeScale()
    {
        return 1.0f/_magSlider->getValue();
    }

    void apply()
    {
        bool isPerspective = _mainView->getCamera()->getProjectionMatrix()(3,3) == 0.0;
        if (isPerspective)
        {
            double vfov, ar, n, f;
            _mainView->getCamera()->getProjectionMatrixAsPerspective(vfov, ar, n, f);
            _magView->getCamera()->setProjectionMatrixAsPerspective(vfov * computeRangeScale(), ar, n, f);
        }
        else
        {
            double L, R, B, T, N, F;
            double M, H;
            _mainView->getCamera()->getProjectionMatrixAsOrtho(L, R, B, T, N, F);
            M = B+(T-B)/2;
            H = (T-B)*computeRangeScale()/2;
            B = M-H, T = M+H;
            M = L+(R-L)/2;
            H = (R-L)*computeRangeScale()/2;
            L = M-H, R = M+H;
            _magView->getCamera()->setProjectionMatrixAsOrtho(L, R, B, T, N, F);
        }

        if (_useLODScale)
        {
            _magView->getCamera()->setLODScale(computeRangeScale());
        }
    }
};

struct Apply : public ui::ControlEventHandler
{
    App& _app;
    Apply(App& app) : _app(app) { }
    void onValueChanged(ui::Control* control) {
        _app.apply();
    }
};

ui::Container* createUI(App& app)
{
    ui::VBox* box = new ui::VBox();
    box->setVertAlign(ui::Control::ALIGN_TOP);
    box->setAbsorbEvents(true);

    ui::HBox* sliderBox = box->addControl(new ui::HBox());
    sliderBox->addControl(new ui::LabelControl("Magnification:"));
    app._magSlider = sliderBox->addControl(new ui::HSliderControl(1.0f, 100.0f, 1.0f, new Apply(app)));
    app._magSlider->setWidth(300.0f);
    sliderBox->addControl(new ui::LabelControl(app._magSlider));

    return box;
}

//! Custom CullVisitor to test the getDistanceToViewPoint override approach.
struct MyCullVisitor : public osgUtil::CullVisitor
{
    App& _app;

    MyCullVisitor(App& app) : osgUtil::CullVisitor(), _app(app)
    {
    }

    MyCullVisitor(const MyCullVisitor& rhs) :
        osgUtil::CullVisitor(rhs),
        _app(rhs._app)
    {
    }

    virtual osgUtil::CullVisitor* clone() const
    { 
        return new MyCullVisitor(*this);
    }

    void apply(osg::Group& node)
    {
        MapNode* mapNode = dynamic_cast<MapNode*>(&node);
        if (mapNode && getCurrentCamera()->getView() == _app._magView)
        {
            // get the eyepoint in world space:
            osg::Matrix viewToWorld;
            viewToWorld.invert(*getModelViewMatrix());
            osg::Vec3d eye = osg::Vec3d(0,0,0) * viewToWorld;

            // store it:
            _eyePointStack.push_back(eye);

            // convert to geo and adjust the altitude in order to simulate a zoom-in:
            GeoPoint p;
            p.fromWorld(mapNode->getMapSRS(), eye);

            osg::Vec3d zoomedEye;
            p.alt() = p.alt() * _app.computeRangeScale();
            p.toWorld(zoomedEye);

            // store the reference view point in view space:
            _referenceViewPoints.push_back(zoomedEye*(*getModelViewMatrix()));

            // ..and the view point in world space.
            _viewPointStack.push_back(zoomedEye);
        }
        osgUtil::CullVisitor::apply(node);
    }
};


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--help") )
        return usage(argv[0]);
    
    App app;

    // optionally use a custom cull visitor instead of LOD scale:
    if (arguments.read("--cull-visitor"))
    {
        app._useLODScale = false;
        osgUtil::CullVisitor::prototype() = new MyCullVisitor(app);
        OE_NOTICE << LC << "Using a custom cull visitor" << std::endl;
    }

    osgViewer::CompositeViewer viewer(arguments);
    viewer.setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);

    // main view lets the user control the scene
    app._mainView = new osgViewer::View();   
    app._mainView->setUpViewInWindow(10, 10, 800, 800);
    app._mainView->setCameraManipulator(new EarthManipulator(arguments));
    viewer.addView(app._mainView);

    // mag view shows the magnified main view, no controls
    app._magView = new osgViewer::View();
    app._magView->setUpViewInWindow(830, 10, 800, 800);
    viewer.addView(app._magView);

    // load the earth file
    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (!node) return usage(argv[0]);
    
    // Add a UI to the main view:
    ui::ControlCanvas* canvas = new ui::ControlCanvas();
    ui::Container* ui = createUI(app);
    canvas->addControl(ui);

    osg::Group* uiGroup = new osg::Group();
    uiGroup->addChild(node);
    uiGroup->addChild(canvas);
    app._mainView->setSceneData(uiGroup);
    
    // Just the map on the magnified view:
    app._magView->setSceneData(node);

    viewer.realize();

    while(!viewer.done())
    {
        // sync magnified view to main view
        app._magView->getCamera()->setViewMatrix(app._mainView->getCamera()->getViewMatrix());
        viewer.frame();
    }
    return 0;
}
