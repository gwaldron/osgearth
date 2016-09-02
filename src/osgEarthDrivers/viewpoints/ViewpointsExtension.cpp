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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include "ViewpointsExtension"
#include <osgEarth/Viewpoint>
#include <osgEarth/XmlUtils>
#include <osgEarthUtil/EarthManipulator>
#include <osgViewer/View>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Viewpoints;

#define LC "[ViewpointsExtension] "


#define VP_MIN_DURATION      2.0     // minimum fly time.
#define VP_METERS_PER_SECOND 2500.0  // fly speed
#define VP_MAX_DURATION      2.0     // maximum fly time.

namespace
{
    void flyToViewpoint(EarthManipulator* manip, const Viewpoint& vp, float t)
    {
        Viewpoint currentVP = manip->getViewpoint();
        GeoPoint vp0 = currentVP.focalPoint().get();
        GeoPoint vp1 = vp.focalPoint().get();
        double distance = vp0.distanceTo(vp1);
        double duration = osg::clampBetween(distance / VP_METERS_PER_SECOND, VP_MIN_DURATION, (double)t);
        manip->setViewpoint( vp, duration );
    }


    struct ViewpointsHandler : public osgGA::GUIEventHandler
    {
        ViewpointsHandler(const std::vector<Viewpoint>& viewpoints, float t)
            : _viewpoints( viewpoints ), _t(t) { }

        bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
        {
            if ( ea.getEventType() == ea.KEYDOWN )
            {
                if ( !_viewpoints.empty() )
                {
                    int index = (int)ea.getKey() - (int)'1';
                    if ( index >= 0 && index < (int)_viewpoints.size() )
                    {
                        EarthManipulator* manip = getManip(aa);
                        if ( manip )
                            flyToViewpoint( manip, _viewpoints[index], _t );
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

            else if ( ea.getEventType() == ea.FRAME && _flyTo.isSet() )
            {
                EarthManipulator* manip = getManip(aa);
                if ( manip )
                    flyToViewpoint(manip, *_flyTo, _t);
                _flyTo.unset();
            }

            return false;
        }

        EarthManipulator* getManip(osgGA::GUIActionAdapter& aa)
        {
            osgViewer::View* view = dynamic_cast<osgViewer::View*>(aa.asView());
            return view ? dynamic_cast<EarthManipulator*>(view->getCameraManipulator()) : 0L;
        }

        std::vector<Viewpoint> _viewpoints;
        optional<Viewpoint>    _flyTo;
        float                  _t;
    };


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

    std::vector<Viewpoint> viewpoints;

    const ConfigSet& children = viewpointsConf.children("viewpoint");
    if ( children.size() > 0 )
    {
        for( ConfigSet::const_iterator i = children.begin(); i != children.end(); ++i )
        {
            viewpoints.push_back( Viewpoint(*i) );
        }
    }

    OE_INFO << LC << "Read " << viewpoints.size() << " viewpoints\n";

    _handler = new ViewpointsHandler(viewpoints, t);
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
    //TODO: remove the event handler
    osgViewer::View* v = dynamic_cast<osgViewer::View*>(view);
    if ( v && _handler.valid() )
    {
        v->removeEventHandler( _handler.get() );
    }
    return true;
}

bool
ViewpointsExtension::connect(Control* control)
{
    //TODO add a UI.
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
