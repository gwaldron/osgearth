/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
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
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/ViewFitter>
#include <osgEarth/NodeUtils>

#define LC "[viewer] "

using namespace osgEarth::Util;
using namespace osgEarth::Contrib;

//------------------------------------------------------------------------

namespace
{

    CascadeDrapingDecorator* getCDD(osg::Node* node)
    {
        return osgEarth::Util::findTopMostNodeOfType< CascadeDrapingDecorator>(node);
    }

    osg::Node* getDrapingDump(osg::Node* node)
    {
        CascadeDrapingDecorator* cdd = getCDD(node);
        if (cdd) return cdd->getDump();
        OverlayDecorator* od = osgEarth::Util::findTopMostNodeOfType<OverlayDecorator>(node);
        if (od) return od->getDump();
        return 0L;
    }

    void requestDrapingDump(osg::Node* node)
    {
        OverlayDecorator* od = osgEarth::Util::findTopMostNodeOfType<OverlayDecorator>(node);
        if (od) od->requestDump();
    }

    struct App
    {
        MapNode* _mapNode;
        osg::ref_ptr<osg::Node> _dumpNode;
        osg::Camera* _overlayCam;
        EarthManipulator* _manip;

        void toggleFitting()
        {
            CascadeDrapingDecorator* cdd = getCDD(_mapNode);
            if (cdd) cdd->setUseProjectionFitting(!cdd->getUseProjectionFitting());
        }

        //void setMinNearFarRatio()
        //{
        //    CascadeDrapingDecorator* cdd = getCDD(_mapNode);
        //    if (cdd) cdd->setMinimumNearFarRatio(_minNearFarRatio->getValue());
        //}

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
                    vp.heading() = Angle(45, Units::DEGREES);
                    vp.pitch() = Angle(-45, Units::DEGREES);
                    _manip->setViewpoint(vp, 1.0);
                }
            }
        }
    };

#if 0
    OE_UI_HANDLER(toggleFitting);
    OE_UI_HANDLER(setMinNearFarRatio);
    OE_UI_HANDLER(findFrusta);

    ui::Control* makeUI(App& app)
    {
        bool usingCascade = getCDD(app._mapNode) != 0L;

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
#endif

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
                _app._dumpNode = getDrapingDump(_app._mapNode);
                if ( !_app._dumpNode.valid() )
                {
                    requestDrapingDump(_app._mapNode);
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
    osgEarth::initialize();
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

    auto node = MapNodeHelper().load( arguments, &viewer );
    if ( node.valid())
    {
        mainView->setSceneData( node );

        app._mapNode = MapNode::get(node);
        if (!app._mapNode)
            return -1;

        app._overlayCam = overlayView->getCamera();

        osg::Group* group = new osg::Group();
        group->addChild(app._mapNode);
        overlayView->setSceneData( group );
        overlayView->addEventHandler( new PHDumper(app, group) );

        return viewer.run();
    }
    else return -1;
}
