/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
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

#include <osgEarthUtil/AnnotationEvents>
#include <osgEarthUtil/Pickers>
#include <osgGA/GUIEventAdapter>
#include <osgGA/EventVisitor>
#include <osgViewer/View>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

AnnotationEventCallback::AnnotationEventCallback() :
_hoverEnabled( true )
{
    //nop
}

void
AnnotationEventCallback::addHandler( AnnotationEventHandler* handler )
{
    _handlers.push_back( handler );
}

void
AnnotationEventCallback::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
    osgGA::EventVisitor* ev = static_cast<osgGA::EventVisitor*>(nv);
    osgGA::EventVisitor::EventList& events = ev->getEvents();
    osgViewer::View* view = static_cast<osgViewer::View*>(ev->getActionAdapter());

    for( osgGA::EventVisitor::EventList::const_iterator e = events.begin(); e != events.end(); ++e )
    {
        osgGA::GUIEventAdapter* ea = e->get();

        if ( ea->getEventType() == osgGA::GUIEventAdapter::MOVE ||
             ea->getEventType() == osgGA::GUIEventAdapter::DRAG )
        {
            _mx = ea->getX();
            _my = ea->getY();
        }

        else if ( ea->getEventType() == osgGA::GUIEventAdapter::PUSH )
        {
            Picker picker( view, node );
            Picker::Hits hits;
            if ( picker.pick( _mx, _my, hits ) )
            {
                for( Picker::Hits::const_iterator h = hits.begin(); h != hits.end(); ++h )
                {
                    AnnotationNode* anno = picker.getNode<AnnotationNode>( *h );
                    if ( anno )
                    {
                        fireEvent( &AnnotationEventHandler::onClick, anno );
                        //break;
                    }
                }
            }
        }

        else if ( ea->getEventType() == osgGA::GUIEventAdapter::FRAME && _hoverEnabled )
        {
            std::set<AnnotationNode*> toUnHover;
            toUnHover.swap( _hovered );

            Picker picker( view, node );
            Picker::Hits hits;

            if ( picker.pick( _mx, _my, hits ) )
            {
                for( Picker::Hits::const_iterator h = hits.begin(); h != hits.end(); ++h )
                {
                    const Picker::Hit& hit = *h;

                    AnnotationNode* anno = picker.getNode<AnnotationNode>( hit );
                    if ( anno )
                    {
                        _hovered.insert( anno );
                        fireEvent( &AnnotationEventHandler::onHoverEnter, anno );
                        toUnHover.erase( anno );
                        //break;
                    }
                }
            }                

            for( std::set<AnnotationNode*>::iterator i = toUnHover.begin(); i != toUnHover.end(); ++i )
            {
                fireEvent( &AnnotationEventHandler::onHoverLeave, *i );
            }
        }
    }

    traverse(node,nv);
}

void
AnnotationEventCallback::fireEvent( EventHandlerMethodPtr method, AnnotationNode* node )
{
    for( Handlers::iterator i = _handlers.begin(); i != _handlers.end(); ++i )
    {
        (i->get()->*method)( node );
    }
}
