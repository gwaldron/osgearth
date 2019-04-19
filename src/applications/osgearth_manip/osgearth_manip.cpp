/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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

#include <string>

#include <osg/Notify>
#include <osg/Timer>
#include <osg/ShapeDrawable>
#include <osg/Depth>
#include <osg/PositionAttitudeTransform>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/GeoMath>
#include <osgEarth/GeoTransform>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Viewpoint>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/LogarithmicDepthBuffer>
#include <osgEarthUtil/ViewFitter>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthSymbology/Style>
#include <osgEarth/ScreenSpaceLayout>

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Annotation;

#define D2R (osg::PI/180.0)
#define R2D (180.0/osg::PI)

namespace
{
    /**
     * Tether callback test.
     */
    struct TetherCB : public EarthManipulator::TetherCallback
    {
        void operator()(osg::Node* node)
        {
            if ( node ) {
                OE_WARN << "Tether on\n";
            }
            else {
                OE_WARN << "Tether off\n";
            }
        }
    };

    /**
     * Builds our help menu UI.
     */
    Container* createHelp( osgViewer::View* view )
    {
        const char* text[] =
        {
            "left mouse :",        "pan",
            "middle mouse :",      "rotate",
            "right mouse :",       "continuous zoom",
            "double-click :",      "zoom to point",
            "scroll wheel :",      "zoom in/out",
            "arrows :",            "pan",
            //"1-6 :",               "fly to preset viewpoints",
            "shift-left-mouse :",  "locked pan",
            "u :",                 "toggle azimuth lock",
            "o :",                 "toggle perspective/ortho",
            "8 :",                 "Tether to thing 1",
            "9 :",                 "Tether to thing 2",
            "t :",                 "cycle tethermode",
            "b :",                 "break tether",
            "a :",                 "toggle viewpoint arcing",
            "q :",                 "toggle throwing",
            "k :",                 "toggle collision",
            "L :",                 "toggle log depth buffer"
        };

        Grid* g = new Grid();
        unsigned i, c, r;
        for( i=0; i<sizeof(text)/sizeof(text[0]); ++i )
        {
            c = i % 2;
            r = i / 2;
            g->setControl( c, r, new LabelControl(text[i]) );
        }

        VBox* v = new VBox();
        v->addControl( g );

        return v;
    }


    /**
     * Some preset viewpoints to show off the setViewpoint function.
     */
    static Viewpoint VPs[] = {
        Viewpoint( "Africa",            0.0,   0.0, 0.0, 0.0, -90.0, 10e6 ),
        Viewpoint( "California",     -121.0,  34.0, 0.0, 0.0, -90.0, 6e6 ),
        Viewpoint( "Europe",            0.0,  45.0, 0.0, 0.0, -90.0, 4e6 ),
        Viewpoint( "Washington DC",   -77.0,  38.0, 0.0, 0.0, -90.0, 1e6 ),
        Viewpoint( "Australia",       135.0, -20.0, 0.0, 0.0, -90.0, 2e6 ),
        Viewpoint( "Boston",         -71.096936, 42.332771, 0, 0.0, -90, 1e5 )
    };


    /**
     * Handler that demonstrates the "viewpoint" functionality in 
     *  osgEarthUtil::EarthManipulator. Press a number key to fly to a viewpoint.
     */
    struct FlyToViewpointHandler : public osgGA::GUIEventHandler 
    {
        FlyToViewpointHandler( EarthManipulator* manip ) : _manip(manip) { }

        bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
        {
            if ( ea.getEventType() == ea.KEYDOWN && ea.getKey() >= '1' && ea.getKey() <= '6' )
            {
                _manip->setViewpoint( VPs[ea.getKey()-'1'], 4.0 );
                aa.requestRedraw();
            }
            return false;
        }

        osg::observer_ptr<EarthManipulator> _manip;
    };
    

    /**
     * Toggles the logarithmic depth buffer
     */
    struct ToggleLDB : public osgGA::GUIEventHandler
    {
        ToggleLDB(char key) : _key(key), _installed(false) { }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                if ( !_installed )
                {
                    _nfratio = aa.asView()->getCamera()->getNearFarRatio();
                    _ldb.install(aa.asView()->getCamera());
                    aa.asView()->getCamera()->setNearFarRatio(0.00001);
                }
                else
                {
                    _ldb.uninstall(aa.asView()->getCamera());
                    aa.asView()->getCamera()->setNearFarRatio(_nfratio);
                }

                _installed = !_installed;
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Toggle LDB"));
        }

        char _key;
        float _nfratio;
        bool _installed;
        osgEarth::Util::LogarithmicDepthBuffer _ldb;
    };

    /**
     * Toggles screen space layout on the sismulated objects
     */
    struct ToggleSSL : public osgGA::GUIEventHandler
    {
        ToggleSSL(osg::Group* g, char key) : _group(g), _key(key), _installed(false) { }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                _installed = !_installed;
                ScreenSpaceLayout::setDeclutteringEnabled(_installed);
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Toggle SSL"));
        }

        char _key;
        osg::Group* _group;
        bool _installed;
    };

    /**
     * Handler to toggle "azimuth locking", which locks the camera's relative Azimuth
     * while panning. For example, it can maintain "north-up" as you pan around. The
     * caveat is that when azimuth is locked you cannot cross the poles.
     */
    struct LockAzimuthHandler : public osgGA::GUIEventHandler
    {
        LockAzimuthHandler(char key, EarthManipulator* manip)
            : _key(key), _manip(manip) { }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                bool lockAzimuth = _manip->getSettings()->getLockAzimuthWhilePanning();
                _manip->getSettings()->setLockAzimuthWhilePanning(!lockAzimuth);
                aa.requestRedraw();
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Toggle azimuth locking"));
        }

        char _key;
        osg::ref_ptr<EarthManipulator> _manip;
    };


    /**
     * Handler to toggle "viewpoint transition arcing", which causes the camera to "arc"
     * as it travels from one viewpoint to another.
     */
    struct ToggleArcViewpointTransitionsHandler : public osgGA::GUIEventHandler
    {
        ToggleArcViewpointTransitionsHandler(char key, EarthManipulator* manip)
            : _key(key), _manip(manip) { }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                bool arc = _manip->getSettings()->getArcViewpointTransitions();
                _manip->getSettings()->setArcViewpointTransitions(!arc);
                aa.requestRedraw();
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Arc viewpoint transitions"));
        }

        char _key;
        osg::ref_ptr<EarthManipulator> _manip;
    };


    /**
     * Toggles the throwing feature.
     */
    struct ToggleThrowingHandler : public osgGA::GUIEventHandler
    {
        ToggleThrowingHandler(char key, EarthManipulator* manip)
            : _key(key), _manip(manip)
        {
        }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                bool throwing = _manip->getSettings()->getThrowingEnabled();
                _manip->getSettings()->setThrowingEnabled( !throwing );
                aa.requestRedraw();
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Toggle throwing"));
        }

        char _key;
        osg::ref_ptr<EarthManipulator> _manip;
    };

    /**
     * Toggles the collision feature.
     */
    struct ToggleCollisionHandler : public osgGA::GUIEventHandler
    {
        ToggleCollisionHandler(char key, EarthManipulator* manip)
            : _key(key), _manip(manip)
        {
        }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                bool value = _manip->getSettings()->getTerrainAvoidanceEnabled();
                _manip->getSettings()->setTerrainAvoidanceEnabled( !value );
                aa.requestRedraw();
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Toggle terrain avoidance"));
        }

        char _key;
        osg::ref_ptr<EarthManipulator> _manip;
    };

    /**
     * Breaks a tether.
     */
    struct CycleTetherMode : public osgGA::GUIEventHandler
    {
        CycleTetherMode(char key, EarthManipulator* manip)
            : _key(key), _manip(manip) { }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                EarthManipulator::TetherMode mode = _manip->getSettings()->getTetherMode();
                if ( mode == _manip->TETHER_CENTER ) {
                    _manip->getSettings()->setTetherMode( _manip->TETHER_CENTER_AND_HEADING );
                    OE_NOTICE << "Tether mode = TETHER_CENTER_AND_HEADING" << std::endl;
                }
                else if ( mode == _manip->TETHER_CENTER_AND_HEADING ) {
                    _manip->getSettings()->setTetherMode( _manip->TETHER_CENTER_AND_ROTATION );
                    OE_NOTICE << "Tether mode = TETHER_CENTER_AND_ROTATION" << std::endl;
                }
                else {
                    _manip->getSettings()->setTetherMode( _manip->TETHER_CENTER );
                    OE_NOTICE << "Tether mode = CENTER" << std::endl;
                }

                aa.requestRedraw();
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Cycle Tether Mode"));
        }

        char _key;
        osg::ref_ptr<EarthManipulator> _manip;
    };

    /**
     * Breaks a tether.
     */
    struct BreakTetherHandler : public osgGA::GUIEventHandler
    {
        BreakTetherHandler(char key, EarthManipulator* manip)
            : _key(key), _manip(manip) { }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                _manip->clearViewpoint();
                aa.requestRedraw();
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Break a tether"));
        }

        char _key;
        osg::ref_ptr<EarthManipulator> _manip;
    };
    

    /**
     * Adjusts the position offset.
     */
    struct SetPositionOffset : public osgGA::GUIEventHandler
    {
        SetPositionOffset(EarthManipulator* manip)
            : _manip(manip) { }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && (ea.getModKeyMask() & ea.MODKEY_SHIFT) )
            {
                Viewpoint oldvp = _manip->getViewpoint();

                double seconds = 0.5;

                if ( ea.getKey() == ea.KEY_Left )
                {
                    Viewpoint vp;
                    vp.positionOffset() = oldvp.positionOffset().get() + osg::Vec3f(-1000,0,0);
                    _manip->setViewpoint( vp, seconds );
                }
                else if ( ea.getKey() == ea.KEY_Right )
                {
                    Viewpoint vp;
                    vp.positionOffset() = oldvp.positionOffset().get() + osg::Vec3f(1000,0,0);
                    _manip->setViewpoint( vp, seconds );
                }
                else if ( ea.getKey() == ea.KEY_Up )
                {
                    Viewpoint vp;
                    vp.positionOffset() = oldvp.positionOffset().get() + osg::Vec3f(0,0,1000);
                    _manip->setViewpoint( vp, seconds );
                }
                else if ( ea.getKey() == ea.KEY_Down )
                {
                    Viewpoint vp;
                    vp.positionOffset() = oldvp.positionOffset().get() + osg::Vec3f(0,0,-1000);
                    _manip->setViewpoint( vp, seconds );
                }
                aa.requestRedraw();
                return true;
            }
            return false;
        }

        osg::ref_ptr<EarthManipulator> _manip;
    };


    /**
     * Toggles perspective/ortho projection matrix.
     */
    struct ToggleProjMatrix : public osgGA::GUIEventHandler
    {
        ToggleProjMatrix(char key, EarthManipulator* manip)
            : _key(key), _manip(manip)
        {
        }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                osg::Matrix proj = aa.asView()->getCamera()->getProjectionMatrix();
                if ( proj(3,3) == 0 )
                {
                    OE_NOTICE << "Switching to orthographc.\n";
                    proj.getPerspective(_vfov, _ar, _zn, _zf);
                    aa.asView()->getCamera()->setProjectionMatrixAsOrtho(-1, 1, -1, 1, _zn, _zf);
                }
                else
                {
                    OE_NOTICE << "Switching to perspective.\n";
                    aa.asView()->getCamera()->setProjectionMatrixAsPerspective(_vfov, _ar, _zn, _zf);
                }
                aa.requestRedraw();
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Toggle projection matrix type"));
        }

        char _key;
        osg::ref_ptr<EarthManipulator> _manip;
        double _vfov, _ar, _zn, _zf;
    };
    
    struct FitViewToPoints : public osgGA::GUIEventHandler
    {
        std::vector<GeoPoint> _points;
        const SpatialReference* _mapSRS;

        FitViewToPoints(char key, EarthManipulator* manip, const SpatialReference* mapSRS)
            : _key(key), _manip(manip), _mapSRS(mapSRS)
        {
            // Set up a list of control points
            const SpatialReference* srs = SpatialReference::get("wgs84");
            _points.push_back(GeoPoint(srs, -120, 30, 0));
            _points.push_back(GeoPoint(srs, -100, 45, 0));
        }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                ViewFitter fitter(_mapSRS, aa.asView()->getCamera());
                fitter.setBuffer( 100000.0 );
                Viewpoint vp;
                if (fitter.createViewpoint(_points, vp))
                {
                    _manip->setViewpoint(vp);
                    aa.requestRedraw();
                }
                return true;
            }
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("FitViewToPoints"));
        }

        char _key;
        osg::ref_ptr<EarthManipulator> _manip;
    };
        

    /**
     * A simple simulator that moves an object around the Earth. We use this to
     * demonstrate/test tethering.
     */
    struct Simulator : public osgGA::GUIEventHandler
    {
        Simulator( osg::Group* root, EarthManipulator* manip, MapNode* mapnode, osg::Node* model, const char* name, char key)
            : _manip(manip), _mapnode(mapnode), _model(model), _name(name), _key(key), _varyAngles(false)
        {
            if ( !model )
            { 
                _model = AnnotationUtils::createHemisphere(250.0, osg::Vec4(1,.7,.4,1));
            }

            _attachPoint = new osg::Group();
            _attachPoint->addChild(_model);

            Style style;
            TextSymbol* text = style.getOrCreate<TextSymbol>();
            text->size() = 32.0f;
            text->declutter() = false;
            text->pixelOffset()->set(50, 50);
            
            _label = new LabelNode(_name, style);
            _label->setDynamic( true );
            _label->setHorizonCulling(false);
            _attachPoint->addChild(_label);

            _geo = new GeoPositionNode();
            _geo->getPositionAttitudeTransform()->addChild(_attachPoint);

            mapnode->addChild(_geo.get());
        }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if ( ea.getEventType() == ea.FRAME )
            {
                double t0 = osg::Timer::instance()->time_s();
                double t = fmod( t0, 6000.0 ) / 6000.0;
                GeoPoint p = _start.interpolate(_end, t);
                double bearing = GeoMath::bearing(_start.y(), _start.x(), p.y(), p.x());

                float pitch = 0.0;

                if (_varyAngles)
                {
                    float a = sin(t0*0.2);
                    float b = 0.4 * cos(t0*0.5);
                    bearing += a;
                    pitch += b;
                }

                _geo->setPosition(p);

                _geo->setLocalRotation(
                    osg::Quat(pitch, osg::Vec3d(1, 0, 0)) *
                    osg::Quat(bearing, osg::Vec3d(0, 0, -1)));
            }
            else if ( ea.getEventType() == ea.KEYDOWN )
            {
                if ( ea.getKey() == _key )
                {                                
                    Viewpoint vp = _manip->getViewpoint();
                    vp.setNode(_label);
                    vp.range() = 25000.0;
                    vp.pitch() = -45.0;
                    _manip->setViewpoint(vp, 2.0);
                }
                return true;
            }
            return false;
        }

        std::string                        _name;
        char                               _key;
        MapNode*                           _mapnode;
        EarthManipulator*                  _manip;
        GeoPoint                           _start, _end;
        LabelNode*                         _label;
        osg::Node*                         _model;
        float                              _heading;
        float                              _pitch;
        osg::ref_ptr<GeoPositionNode>      _geo;
        osg::Group*                        _attachPoint;
        bool                               _varyAngles;
    };

    /**
     * Place an X at the sim entity position, in screen space.
     * The point of this is to test the EarthManipulator::UpdateCameraCallback
     * which provides a frame-synched camera matrix (post-update traversal)
     */
    struct CalculateWindowCoords : public osgGA::GUIEventHandler
                                   
    {
        CalculateWindowCoords(char key, EarthManipulator* manip, Simulator* sim)
            : _key(key), _active(false), _sim(sim), _xform(0L)
        {
            //nop
        }

        void onUpdateCamera(const osg::Camera* cam)
        {
            if (_active)
            {
                if (!_xform)
                {
                    osg::Geometry* geom = new osg::Geometry();
                    osg::Vec3Array* verts = new osg::Vec3Array();
                    verts->push_back(osg::Vec3(-10000, 0, 0));
                    verts->push_back(osg::Vec3( 10000, 0, 0));
                    verts->push_back(osg::Vec3( 0, -10000, 0));
                    verts->push_back(osg::Vec3( 0,  10000, 0));
                    verts->push_back(osg::Vec3( 0, 0, -10000));
                    verts->push_back(osg::Vec3( 0, 0,  10000));
                    geom->setVertexArray(verts);
                    osg::Vec4Array* colors = new osg::Vec4Array();
                    colors->push_back(osg::Vec4(1, 1, 0, 1));
                    colors->setBinding(colors->BIND_OVERALL);
                    geom->setColorArray(colors);
                    geom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 6));
                    geom->setCullingActive(false);
                    geom->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::ALWAYS, 0, 1, true), 1);

                    _xform = new osg::MatrixTransform();
                    _xform->addChild(geom);

                    osg::View* view = const_cast<osg::View*>(cam->getView());
                    ControlCanvas::getOrCreate(view)->addChild(_xform);
                }

                GeoPoint p = _sim->_geo->getPosition();

                osg::Vec3d world;
                p.toWorld(world);

                osg::Matrix worldToWindow =
                    cam->getViewMatrix() *
                    cam->getProjectionMatrix() *
                    cam->getViewport()->computeWindowMatrix();

                osg::Vec3d win = world * worldToWindow;

                _xform->setMatrix(osg::Matrix::translate(win));
            }
        }

        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
        {
            if (ea.getEventType() == ea.KEYDOWN && ea.getKey() == _key)
            {
                _active = !_active;
                aa.requestRedraw();
                return true;
            }
            
            return false;
        }

        void getUsage(osg::ApplicationUsage& usage) const
        {
            using namespace std;
            usage.addKeyboardMouseBinding(string(1, _key), string("Show Window Coords"));
        }

        osg::MatrixTransform* _xform;
        Simulator* _sim;
        bool _active;
        char _key;
    };

    struct CameraUpdater : public EarthManipulator::UpdateCameraCallback
    {
        CalculateWindowCoords* _calc;

        CameraUpdater(CalculateWindowCoords* calc) : _calc(calc) { }
        
        void onUpdateCamera(const osg::Camera* cam)
        {
            _calc->onUpdateCamera(cam);
        }
    };
}


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    if (arguments.read("--help") || argc==1)
    {
        OE_WARN << "Usage: " << argv[0] << " [earthFile] [--model modelToLoad]"
            << std::endl;
        return 0;
    }

    osgViewer::Viewer viewer(arguments);

    // install the programmable manipulator.
    EarthManipulator* manip = new EarthManipulator();
    viewer.setCameraManipulator( manip );

    // UI:
    Container* help = createHelp(&viewer);

    osg::Node* earthNode = MapNodeHelper().load( arguments, &viewer, help );
    if (!earthNode)
    {
        OE_WARN << "Unable to load earth model." << std::endl;
        return -1;
    }

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );

    osgEarth::MapNode* mapNode = osgEarth::MapNode::findMapNode( earthNode );

    // user model?
    osg::ref_ptr<osg::Node> model;
    std::string modelFile;
    if (arguments.read("--model", modelFile))
        model = osgDB::readRefNodeFile(modelFile + ".osgearth_shadergen");

    osg::Group* sims = new osg::Group();
    root->addChild( sims );

    const SpatialReference* wgs84 = SpatialReference::get("wgs84");

    // Simulator for tethering:
    Simulator* sim1 = new Simulator(sims, manip, mapNode, model.get(), "Thing 1", '8');
    sim1->_start = GeoPoint(wgs84, 45.0, 55.0, 10000);
    sim1->_end = GeoPoint(wgs84, -45, -55.0, 10000);
    sim1->_varyAngles = false;
    viewer.addEventHandler(sim1);

    Simulator* sim2 = new Simulator(sims, manip, mapNode, model.get(), "Thing 2", '9');
    sim2->_start = GeoPoint(wgs84, 45.0, 54.0, 10000);
    sim2->_end = GeoPoint(wgs84, -44.0, -54.0, 10000);
    sim2->_varyAngles = true;
    viewer.addEventHandler(sim2);

    manip->getSettings()->getBreakTetherActions().push_back( EarthManipulator::ACTION_GOTO );    

    // Set the minimum distance to something larger than the default
    manip->getSettings()->setMinMaxDistance(10.0, manip->getSettings()->getMaxDistance());

    // Sets the maximum focal point offsets (usually for tethering)
    manip->getSettings()->setMaxOffset(5000.0, 5000.0);
    
    // Pitch limits.
    manip->getSettings()->setMinMaxPitch(-90, 90);


    viewer.setSceneData( root );

    manip->getSettings()->bindMouse(
        EarthManipulator::ACTION_EARTH_DRAG,
        osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON,
        osgGA::GUIEventAdapter::MODKEY_SHIFT );

    manip->getSettings()->bindMouseClick(
        EarthManipulator::ACTION_GOTO,
        osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON,
        osgGA::GUIEventAdapter::MODKEY_SHIFT);

    manip->getSettings()->setArcViewpointTransitions( true );    

    manip->setTetherCallback( new TetherCB() );
    
    //viewer.addEventHandler(new FlyToViewpointHandler( manip ));
    viewer.addEventHandler(new LockAzimuthHandler('u', manip));
    viewer.addEventHandler(new ToggleArcViewpointTransitionsHandler('a', manip));
    viewer.addEventHandler(new ToggleThrowingHandler('q', manip));
    viewer.addEventHandler(new ToggleCollisionHandler('k', manip));
    viewer.addEventHandler(new ToggleProjMatrix('o', manip));
    viewer.addEventHandler(new BreakTetherHandler('b', manip));
    viewer.addEventHandler(new CycleTetherMode('t', manip));
    viewer.addEventHandler(new SetPositionOffset(manip));
    viewer.addEventHandler(new ToggleLDB('L'));
    viewer.addEventHandler(new ToggleSSL(sims, ')'));
    viewer.addEventHandler(new FitViewToPoints('j', manip, mapNode->getMapSRS()));
    
    CalculateWindowCoords* calc = new CalculateWindowCoords('W', manip, sim1);
    viewer.addEventHandler(calc);
    manip->setUpdateCameraCallback(new CameraUpdater(calc));

    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

    while(!viewer.done())
    {
        viewer.frame();

        // simulate slow frame rate
        //OpenThreads::Thread::microSleep(1000*1000);
    }
    return 0;
}
