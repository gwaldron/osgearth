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
#include <osgViewer/CompositeViewer>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarth/MapNode>

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

struct App
{
    osgViewer::View* _mainView;
    osgViewer::View* _magView;
    ui::HSliderControl* _magSlider;

    float computeLODScaleFromMag(float mag)
    {
        return 1.0f/mag;
    }

    void apply()
    {
        //const float base_vfov = 30.0f;

        double vfov, ar, n, f;
        _mainView->getCamera()->getProjectionMatrixAsPerspective(vfov, ar, n, f);

        float lodScale = computeLODScaleFromMag(_magSlider->getValue());

        _magView->getCamera()->setProjectionMatrixAsPerspective(vfov * lodScale, ar, n, f);
        _magView->getCamera()->setLODScale(lodScale);
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

int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    if ( arguments.read("--help") )
        return usage(argv[0]);

    osgViewer::CompositeViewer viewer(arguments);
    viewer.setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);

    osgViewer::View* mainView = new osgViewer::View();   
    mainView->setUpViewInWindow(10, 10, 800, 800);
    mainView->setCameraManipulator(new EarthManipulator(arguments));
    viewer.addView(mainView);

    osgViewer::View* magView = new osgViewer::View();
    magView->setUpViewInWindow(830, 10, 800, 800);
    viewer.addView(magView);

    osg::Node* node = MapNodeHelper().load(arguments, &viewer);
    if (!node) return usage(argv[0]);

    mainView->setSceneData(node);
    magView->setSceneData(node);

    App app;
    app._mainView = mainView;
    app._magView = magView;
    ui::Container* ui = createUI(app);
    ui::ControlCanvas* canvas = ui::ControlCanvas::getOrCreate(mainView);
    canvas->addControl(ui);

    while(!viewer.done())
    {
        magView->getCamera()->setViewMatrix(mainView->getCamera()->getViewMatrix());
        viewer.frame();
    }
    return 0;
}
