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

#include <osg/Notify>
#include <osg/Depth>
#include <osg/LineWidth>
#include <osg/LineStipple>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/OverlayDecorator>
#include <osgEarth/MapNode>
#include <osgEarth/CascadeDrapingDecorator>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/ViewFitter>

#define LC "[viewer] "

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Symbology;

namespace ui = osgEarth::Util::Controls;

//------------------------------------------------------------------------

namespace
{
    struct App
    {
        MapNode* _mapNode;
        ui::CheckBoxControl* _fitting;
        ui::HSliderControl* _minNearFarRatio;
        ui::ButtonControl* _centerView;
        osg::ref_ptr<osg::Node> _dumpNode;
        osg::Camera* _overlayCam;
        EarthManipulator* _manip;

        void toggleFitting()
        {
            CascadeDrapingDecorator* cdd = _mapNode->getCascadeDrapingDecorator();
            if (cdd) cdd->setUseProjectionFitting(_fitting->getValue());
        }

        void setMinNearFarRatio()
        {
            CascadeDrapingDecorator* cdd = _mapNode->getCascadeDrapingDecorator();
            if (cdd) cdd->setMinimumNearFarRatio(_minNearFarRatio->getValue());
        }

        void findFrusta()
        {
            if (_dumpNode.valid())
            {
                ViewFitter fitter(_mapNode->getMapSRS(), _overlayCam);
                const osg::BoundingSphere& bs = _dumpNode->getBound();
                GeoPoint center;
                center.fromWorld(_mapNode->getMapSRS(), bs.center());
                std::vector<GeoPoint> points;
                points.push_back(center);
                fitter.setBuffer(bs.radius()*0.85);
                Viewpoint vp;
                if (fitter.createViewpoint(points, vp))
                {
                    vp.heading() = 45, vp.pitch() = -45;
                    _manip->setViewpoint(vp, 1.0);
                }
            }
        }
    };

    OE_UI_HANDLER(toggleFitting);
    OE_UI_HANDLER(setMinNearFarRatio);
    OE_UI_HANDLER(findFrusta);

    ui::Control* makeUI(App& app)
    {
        bool usingCascade = app._mapNode->getCascadeDrapingDecorator() != 0L;

        ui::Grid* grid = new ui::Grid();
        int r = 0;

        if (usingCascade)
        {
            grid->setControl(0, r, new ui::LabelControl("Projection fitting"));
            grid->setControl(1, r, app._fitting = new ui::CheckBoxControl(true, new toggleFitting(app)));
            ++r;

            grid->setControl(0, r, new ui::LabelControl("Cascade #1 Min NF Ratio"));
            grid->setControl(1, r, app._minNearFarRatio = new ui::HSliderControl(0.0, 1.0, 0.2, new setMinNearFarRatio(app)));
            app._minNearFarRatio->setHorizFill(true, 250.0f);
            ++r;
        }

        grid->setControl(0, r, app._centerView = new ui::ButtonControl("Sync view", new findFrusta(app)));
        ++r;

        return grid;
    }

    // it's not used by osgEarth, but you can copy this code into a viewer app and
    // use it to visualize the various polyhedra created by the overlay decorator.
    // see the end of OverlayDecorator::cull for the dump types.
    struct PHDumper : public osgGA::GUIEventHandler
    {
        App& _app;
        osg::Group* _parent;
        PHDumper(App& app, osg::Group* parent) : _app(app), _parent(parent)
        {
        }

        bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
        {
            if ( ea.getEventType() == ea.FRAME )
            {
                _app._dumpNode = _app._mapNode->getDrapingDump();
                if ( !_app._dumpNode.valid() )
                {
                    _app._mapNode->getOverlayDecorator()->requestDump();
                    aa.requestRedraw();
                }
                else
                {
                    // note: the dumped geometry has some state in it (from ConvexPolyhedron::dumpGeometry)
                    // so we need to override it.
                    osg::Group* g = new osg::Group();
                    osg::StateSet* gss = g->getOrCreateStateSet();
                    gss->setAttributeAndModes(new osg::LineWidth(1.5f), 1);
                    gss->setRenderBinDetails(90210, "DepthSortedBin");

                    osg::Group* g0 = new osg::Group();
                    g->addChild( g0 );
                    osg::StateSet* g0ss = g0->getOrCreateStateSet();
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
                    g0ss->setAttributeAndModes(new osg::LineStipple(1, 0x000F), 1);
#endif
                    g0->addChild( _app._dumpNode.get() );

                    osg::Group* g1 = new osg::Group();
                    g->addChild( g1 );
                    osg::StateSet* g1ss = g1->getOrCreateStateSet();
                    g1ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OVERRIDE | 1);
                    g1->addChild( _app._dumpNode.get() );

                    _parent->removeChildren(1, _parent->getNumChildren()-1);
                    _parent->addChild( g );

                    aa.requestRedraw();
                }
            }
            return false;
        }
    };
}

int
main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::CompositeViewer viewer(arguments);
    viewer.setThreadingModel( osgViewer::CompositeViewer::SingleThreaded );
    
    App app;

    // query the screen size.
    osg::GraphicsContext::ScreenIdentifier si;
    si.readDISPLAY();
    if ( si.displayNum < 0 ) si.displayNum = 0;
    osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
    unsigned width, height;
    wsi->getScreenResolution( si, width, height );
    unsigned b = 50;

    osgViewer::View* mainView = new osgViewer::View();
    mainView->getCamera()->setName("dump");
    mainView->getCamera()->setNearFarRatio(0.00002);
    EarthManipulator* em = new EarthManipulator();
    em->getSettings()->setMinMaxPitch(-90, 0);
    mainView->setCameraManipulator( em );
    mainView->setUpViewInWindow( b, b, (width/2)-b*2, (height-b*4) );
    viewer.addView( mainView );

    osgViewer::View* overlayView = new osgViewer::View();
    overlayView->getCamera()->setNearFarRatio(0.00002);
    overlayView->setCameraManipulator( app._manip = new EarthManipulator() );
    
    overlayView->setUpViewInWindow( (width/2), b, (width/2)-b*2, (height-b*4) );
    overlayView->addEventHandler(new osgGA::StateSetManipulator(overlayView->getCamera()->getOrCreateStateSet()));
    viewer.addView( overlayView );

    std::string pathfile;
    double animationSpeed = 1.0;
    if (arguments.read("-p", pathfile))
    {
        mainView->setCameraManipulator( new osgGA::AnimationPathManipulator(pathfile) );
    }

    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    if ( node )
    {
        mainView->setSceneData( node );

        app._mapNode = MapNode::get(node);
        app._overlayCam = overlayView->getCamera();
        ui::ControlCanvas::get(mainView)->addControl(makeUI(app));

        osg::Group* group = new osg::Group();
        group->addChild(app._mapNode);
        overlayView->setSceneData( group );       
        overlayView->addEventHandler( new PHDumper(app, group) );

        return viewer.run();
    }
    else return -1;
}
