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
#include <osgViewer/Viewer>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthSymbology/Color>

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
    class AddGuiOperation : public osg::Operation
    {
    public:

        AddGuiOperation(MapNode* _mapNode)
            : mapNode(_mapNode)
        {
        }

        virtual void operator () (osg::Object* callingObject)
        {
            osgViewer::Viewer* view = dynamic_cast<osgViewer::Viewer*>(callingObject);

            if(view)
            {
                //check if there is an instance of the ToggleCanvasEventHandler class, which indicates that the earth file was loaded with the osgearth_viewer
                const osgViewer::View::EventHandlers& handlers = view->getEventHandlers();
                for(osgViewer::View::EventHandlers::const_iterator it = handlers.begin(); it!=handlers.end(); it++)
                {
                    const ToggleCanvasEventHandler *canvasHandler = dynamic_cast<const ToggleCanvasEventHandler*>(it->get());
                    if(canvasHandler!=0)
                        return;
                }

                // Install a new Canvas for our UI controls, or use one that already exists.
                ControlCanvas* canvas = ControlCanvas::getOrCreate( view );

                Container* mainContainer;
                mainContainer = new VBox();
                mainContainer->setAbsorbEvents( true );
                mainContainer->setBackColor( Color(Color::Black, 0.8) );
                mainContainer->setHorizAlign( Control::ALIGN_LEFT );
                mainContainer->setVertAlign( Control::ALIGN_BOTTOM );
                canvas->addControl( mainContainer );

                // Add an event handler to toggle the canvas with a key press;
                view->addEventHandler(new ToggleCanvasEventHandler(canvas, 'y'));
                // install our default manipulator (do this before calling load)
                EarthManipulator *manip = dynamic_cast<EarthManipulator*>(view->getCameraManipulator());
                if(!manip)
                    view->setCameraManipulator( new EarthManipulator );

                // Hook up the extensions!
                for(std::vector<osg::ref_ptr<Extension> >::const_iterator eiter = mapNode->getExtensions().begin();
                    eiter != mapNode->getExtensions().end();
                    ++eiter)
                {
                    Extension* e = eiter->get();

                    // Check for a View interface:
                    ExtensionInterface<osg::View>* viewIF = ExtensionInterface<osg::View>::get( e );
                    if ( viewIF )
                        viewIF->connect( view );

                    // Check for a Control interface:
                    ExtensionInterface<Control>* controlIF = ExtensionInterface<Control>::get( e );
                    if ( controlIF )
                        controlIF->connect( mainContainer );
                }
            }
        }
    private:
        osg::ref_ptr<MapNode> mapNode;
    };


    class RemoveEventHandler : public osg::Operation
    {
    public:

        RemoveEventHandler(osg::Node *node, osgGA::GUIEventHandler *eventHandler)
            : _node(node), _eventHandler(eventHandler)
        {
        }

        virtual void operator () (osg::Object* callingObject)
        {
            osgViewer::Viewer* view = dynamic_cast<osgViewer::Viewer*>(callingObject);

            if(view)
            {
                _node->removeEventCallback(_eventHandler);
            }
        }
        osg::ref_ptr<osgGA::GUIEventHandler> _eventHandler;
        osg::ref_ptr<osg::Node> _node;
    };


    struct AddGuiEventHandler : public osgGA::GUIEventHandler
    {
        AddGuiEventHandler(osg::Node* node)
            : _node(node)
        {
        }

        bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
        {
            osgViewer::Viewer* viewer = static_cast<osgViewer::Viewer*>(&aa);

            if(viewer)
            {
                switch (ea.getEventType())
                {
                    case osgGA::GUIEventAdapter::FRAME:
                    {
                        MapNode *mapNode = MapNode::get(_node);
                        if(mapNode)
                        {
                            viewer->addUpdateOperation(new AddGuiOperation(mapNode));
                            viewer->addUpdateOperation(new RemoveEventHandler(_node.get(), this));
                        }
                        return false;
                    }
                    break;
                    default:
                        break;
                }
                return false;
            }
            return false;
        }
        osg::ref_ptr<osg::Node> _node;
    };


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

bool ViewpointsExtension::connect(MapNode *mapnode)
{
    mapnode->addEventCallback(new AddGuiEventHandler(mapnode));
    return true;
}

bool ViewpointsExtension::disconnect(MapNode *mapnode)
{
    return true;
}
