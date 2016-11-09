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

#include <osgEarthUtil/AnnotationEvents>
#include <osgEarth/IntersectionPicker>
#include <osgGA/GUIEventAdapter>
#include <osgGA/EventVisitor>
#include <osgViewer/View>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Annotation;

AnnotationEventCallback::AnnotationEventCallback( AnnotationEventHandler* handler ) :
_hoverEnabled( true ),
_mouseDown   ( false )
{
    if ( handler )
        addHandler( handler );
}

void
AnnotationEventCallback::addHandler( AnnotationEventHandler* handler )
{
    if ( handler )
        _handlers.push_back( handler );
}

void
AnnotationEventCallback::setHoverEnabled( bool hoverEnabled )
{
    // Does not unhover currently hovered annotations, so don't turn hovering off if there
    // are currently items hovered
    if(hoverEnabled || !_hovered.size() )
    {
      _hoverEnabled = hoverEnabled;
    }
}

void
AnnotationEventCallback::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
    osgGA::EventVisitor* ev = static_cast<osgGA::EventVisitor*>(nv);
    osgGA::EventQueue::Events& events = ev->getEvents();
    osgViewer::View* view = static_cast<osgViewer::View*>(ev->getActionAdapter());

    for( osgGA::EventQueue::Events::const_iterator e = events.begin(); e != events.end(); ++e )
    {
        osgGA::GUIEventAdapter* ea = dynamic_cast<osgGA::GUIEventAdapter*>(e->get());
        if ( !ea )
            continue;

        if ( ea->getEventType() == osgGA::GUIEventAdapter::MOVE ||
             ea->getEventType() == osgGA::GUIEventAdapter::DRAG )
        {
            _args.x = ea->getX();
            _args.y = ea->getY();
        }

        else if ( ea->getEventType() == osgGA::GUIEventAdapter::PUSH )
        {
            _mouseDown = true;
            _args.x = ea->getX();
            _args.y = ea->getY();
            _args.buttons = ea->getButtonMask();
            _args.modkeys = ea->getModKeyMask();

            IntersectionPicker picker( view, node );
            IntersectionPicker::Hits hits;
            if ( picker.pick( _args.x, _args.y, hits ) )
            {
                std::set<AnnotationNode*> fired; // prevent multiple hits on the same instance

                for( IntersectionPicker::Hits::const_iterator h = hits.begin(); h != hits.end(); ++h )
                {
                    AnnotationNode* anno = picker.getNode<AnnotationNode>( *h );
                    if ( anno && fired.find(anno) == fired.end() )
                    {
                        fireEvent( &AnnotationEventHandler::onClick, anno );
                        fired.insert( anno );
                        //break;
                    }
                }
            }
        }

        else if ( ea->getEventType() == osgGA::GUIEventAdapter::RELEASE )
        {
            _mouseDown = false;
        }

        else if ( ea->getEventType() == osgGA::GUIEventAdapter::FRAME && _hoverEnabled && !_mouseDown )
        {
            //Insert all the currently hovered annotations into a set to be unhoverd
            std::set<AnnotationNode*> toUnHover;
            for( std::set<AnnotationNode*>::iterator i = _hovered.begin(); i != _hovered.end(); ++i )
            {
                toUnHover.insert( *i );
            }

            IntersectionPicker picker( view, node );
            IntersectionPicker::Hits hits;

            if ( picker.pick( _args.x, _args.y, hits ) )
            {
                for( IntersectionPicker::Hits::const_iterator h = hits.begin(); h != hits.end(); ++h )
                {
                    const IntersectionPicker::Hit& hit = *h;

                    AnnotationNode* anno = picker.getNode<AnnotationNode>( hit );
                    if ( anno )
                    {
                        //If the annotation ins't current hovered, add it to the list of new hovered items
                        if ( _hovered.find(anno) == _hovered.end() )
                        {
                            _hovered.insert( anno );
                            fireEvent( &AnnotationEventHandler::onHoverEnter, anno );
                        }
                        //It's still hovered, so don't unhover it
                        toUnHover.erase( anno );
                        //break;
                    }
                }
            }                

            //The unhovered list now contains all the annotations that were hovered on the previous frame that need to be unhovered
            //and removed from the previous hover list
            for( std::set<AnnotationNode*>::iterator i = toUnHover.begin(); i != toUnHover.end(); ++i )
            {
                _hovered.erase( *i );
                fireEvent( &AnnotationEventHandler::onHoverLeave, *i );
            }
        }
    }

    traverse(node,nv);
}

void
AnnotationEventCallback::fireEvent(EventHandlerMethodPtr   method, 
                                   AnnotationNode*         node )
{
    for( Handlers::iterator i = _handlers.begin(); i != _handlers.end(); ++i )
    {
        (i->get()->*method)( node, _args );
    }
}
