/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "ViewpointsExtension"
#include <osgEarth/Viewpoint>
#include <osgEarth/XmlUtils>
#include <osgEarth/EarthManipulator>
#include <osgEarth/StringUtils>
#include <osgEarth/Math>
#include <osgViewer/View>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Viewpoints;

#ifdef OSGEARTH_HAVE_CONTROLS_API
using namespace osgEarth::Util::Controls;
#endif

#define LC "[ViewpointsExtension] "


#define VP_MIN_DURATION       2.0     // minimum fly time.
#define VP_METERS_PER_SECOND  2500.0  // fly speed
#define VP_MAX_DURATION       2.0     // maximum fly time.
#define VP_DEFAULT_DELAY_TIME 2.0     // default when auto-flying between viewpoints

namespace
{
    void flyToViewpoint(EarthManipulator* manip, const Viewpoint& vp, float t)
    {
        Viewpoint currentVP = manip->getViewpoint();
        GeoPoint vp0 = currentVP.focalPoint().get();
        GeoPoint vp1 = vp.focalPoint().get();
        auto distance = vp0.geodesicDistanceTo(vp1);
        double duration = clamp(distance.as(Units::METERS) / VP_METERS_PER_SECOND, VP_MIN_DURATION, (double)t);
        manip->setViewpoint(vp, duration);
    }


    struct ViewpointsHandler : public osgGA::GUIEventHandler
    {
        std::vector<Viewpoint> _viewpoints;
        optional<Viewpoint>    _flyTo;
        float                  _transitionTime = 0.0f;
        float                  _autoRunDelay = 0.0f;
        osg::Timer_t           _autoRunStartWaitTime;
        int                    _autoRunIndex = 0;
        int                    _homeIndex = -1;
        int                    _count = 0;
        bool                   _mapKeys = true;

        ViewpointsHandler(const std::vector<Viewpoint>& viewpoints, float t, bool mapKeys)
            : _viewpoints( viewpoints ), _transitionTime(t), _mapKeys(mapKeys)
        {
            _autoRunStartWaitTime = osg::Timer::instance()->tick();
        }

        bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
        {
            if (_count == 0 && _homeIndex >= 0 && _homeIndex < _viewpoints.size())
            {
                EarthManipulator* manip = getManip(aa);
                if (manip)
                {
                    flyToViewpoint(manip, _viewpoints[_homeIndex], _transitionTime);
                    ++_count;
                }
            }

            if (ea.getHandled())
                return false;

            if ( _mapKeys && ea.getEventType() == ea.KEYDOWN )
            {
                if ( !_viewpoints.empty() && _autoRunDelay <= 0.0f )
                {
                    int index = (int)ea.getKey() - (int)'1';
                    if ( index >= 0 && index < (int)_viewpoints.size() )
                    {
                        EarthManipulator* manip = getManip(aa);
                        if ( manip )
                            flyToViewpoint( manip, _viewpoints[index], _transitionTime );
                    }
                }
                if ( ea.getKey() == 'v' )
                {
                    osgViewer::View* view = dynamic_cast<osgViewer::View*>(aa.asView());
                    if ( view )
                    {
                        EarthManipulator* manip = getManip(aa);
                        if ( manip )
                        {
                            XmlDocument xml( manip->getViewpoint().getConfig() );
                            xml.store( std::cout );
                            std::cout << std::endl;
                        }
                    }
                }
                aa.requestRedraw();
            }

            else if ( ea.getEventType() == ea.FRAME && _viewpoints.size() > 0 )
            {
                if (_flyTo.isSet())
                {
                    EarthManipulator* manip = getManip(aa);
                    if ( manip )
                        flyToViewpoint(manip, *_flyTo, _transitionTime);
                    _flyTo.unset();
                }

                else if (_autoRunDelay > 0.0)
                {
                    osg::Timer_t now = osg::Timer::instance()->tick();
                    float dt = osg::Timer::instance()->delta_s(_autoRunStartWaitTime, now);

                    //OE_WARN << "now="<<now << ", t+auto=" << ((_transitionTime + _autoRunStartWaitTime)) << ",dt=" << now - (_transitionTime + _autoRunStartWaitTime) << ", delay="<<_autoRunDelay<<std::endl;
                    if (dt > (_transitionTime + _autoRunDelay))
                    {
                        int i = (_autoRunIndex++ % _viewpoints.size());
                        _flyTo = _viewpoints[i];
                        _autoRunStartWaitTime = now;
                    }
                }
            }

            return false;
        }

        EarthManipulator* getManip(osgGA::GUIActionAdapter& aa)
        {
            osgViewer::View* view = dynamic_cast<osgViewer::View*>(aa.asView());
            return view ? dynamic_cast<EarthManipulator*>(view->getCameraManipulator()) : 0L;
        }

        void setAutoRunDelayTime(float t)
        {
            _autoRunDelay = t;
        }
    };


#ifdef OSGEARTH_HAVE_CONTROLS_API
    // flies to a viewpoint in response to control event (click)
    struct ClickViewpointHandler : public ControlEventHandler
    {
        ClickViewpointHandler(const Viewpoint& vp, ViewpointsHandler* handler) :
            _vp(vp), _handler(handler) { }

        Viewpoint          _vp;
        ViewpointsHandler* _handler;

        virtual void onClick(Control* control)
        {
            _handler->_flyTo = _vp;
        }
    };


    Control* createViewpointControl(ViewpointsHandler* handler)
    {
        Grid* grid = 0L;

        if ( handler->_viewpoints.size() > 0 )
        {
            // the viewpoint container:
            grid = new Grid();
            grid->setBackColor(osg::Vec4(0,0,0,0.1));
            grid->setChildSpacing( 0 );
            grid->setChildVertAlign( Control::ALIGN_CENTER );

            for( unsigned i=0; i<handler->_viewpoints.size(); ++i )
            {
                const Viewpoint& vp = handler->_viewpoints[i];
                Control* num = new LabelControl(Stringify() << (i+1), 16.0f, osg::Vec4f(1,1,0,1));
                num->setPadding( 4 );
                grid->setControl( 0, i, num );

                Control* vpc = new LabelControl(vp.name()->empty() ? "<no name>" : vp.name().get(), 16.0f);
                vpc->setPadding( 4 );
                vpc->setHorizFill( true );
                vpc->setActiveColor( osg::Vec4(0.4,0.4,1.0,1.0) ); // blue
                vpc->addEventHandler( new ClickViewpointHandler(vp, handler) );
                grid->setControl( 1, i, vpc );
            }
        }

        return grid;
    }
#endif 
}

//.........................................................................


ViewpointsExtension::ViewpointsExtension()
{
    //NOP
}

ViewpointsExtension::ViewpointsExtension(const ConfigOptions& options) :
ConfigOptions( options )
{
    // backwards-compatibility: read viewpoints at the top level???
    const Config& viewpointsConf = options.getConfig();
    float t = viewpointsConf.value("time", VP_MAX_DURATION);
    int home = viewpointsConf.value("home", (int)-1);
    bool mapKeys = viewpointsConf.value("map_keys", true) == true;

    std::vector<Viewpoint> viewpoints;

    const ConfigSet& children = viewpointsConf.children("viewpoint");
    if ( children.size() > 0 )
    {
        for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
        {
            viewpoints.push_back( Viewpoint(*i) );
        }
    }

    OE_DEBUG << LC << "Read " << viewpoints.size() << " viewpoints\n";

    ViewpointsHandler* handler = new ViewpointsHandler(viewpoints, t, mapKeys);
    handler->_homeIndex = home;

    if (viewpointsConf.hasValue("autorun"))
    {
        float t = osgEarth::as<float>(viewpointsConf.value("autorun"), VP_DEFAULT_DELAY_TIME);
        handler->setAutoRunDelayTime(t);
    }

    _handler = handler;
}

ViewpointsExtension::~ViewpointsExtension()
{
    //nop
}

void
ViewpointsExtension::setDBOptions(const osgDB::Options* dbOptions)
{
    _dbOptions = dbOptions;
}

bool
ViewpointsExtension::connect(osg::View* view)
{
    osgViewer::View* v = dynamic_cast<osgViewer::View*>(view);
    if ( v && _handler.valid() )
    {
        v->addEventHandler( _handler.get() );
    }
    return true;
}

bool
ViewpointsExtension::disconnect(osg::View* view)
{
    osgViewer::View* v = dynamic_cast<osgViewer::View*>(view);
    if ( v && _handler.valid() )
    {
        v->removeEventHandler( _handler.get() );
    }
    return true;
}


#ifdef OSGEARTH_HAVE_CONTROLS_API
bool
ViewpointsExtension::connect(Control* control)
{
    Container* container = dynamic_cast<Container*>(control);
    if ( container )
    {
        ViewpointsHandler* vh = static_cast<ViewpointsHandler*>(_handler.get());
        if ( vh->_viewpoints.size() > 0 )
        {
            Control* c = createViewpointControl( vh );
            if ( c )
                container->addControl( c );
        }
    }
    return true;
}

bool
ViewpointsExtension::disconnect(Control* control)
{
    // TODO: remove the UI
    return true;
}
#endif
