/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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

#include <osgEarthUtil/FeatureQueryTool>
#include <osgEarth/Pickers>
#include <osgViewer/View>
#include <osgEarth/DPLineSegmentIntersector>

#define LC "[FeatureQueryTool] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Util;

//#undef OE_DEBUG
//#define OE_DEBUG OE_INFO

//-----------------------------------------------------------------------

FeatureQueryTool::FeatureQueryTool(MapNode*                    mapNode,
                                   FeatureQueryTool::Callback* callback) :
_mapNode( mapNode )
{
    if ( callback )
        addCallback( callback );
}

void
FeatureQueryTool::addCallback( FeatureQueryTool::Callback* cb )
{
    if ( cb )
        _callbacks.push_back( cb );
}

void
FeatureQueryTool::setMapNode( MapNode* mapNode )
{
    _mapNode = mapNode;
}

bool
FeatureQueryTool::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    bool handled = false;
    bool attempt;

    if ( _inputPredicate.valid() )
    {
        attempt = _inputPredicate->accept(ea);
    }
    else
    {
        attempt =
            ea.getEventType() == osgGA::GUIEventAdapter::RELEASE &&
            _mouseDown && 
            fabs(ea.getX()-_mouseDownX) <= 3.0 && 
            fabs(ea.getY()-_mouseDownY) <= 3.0;
    }

    if ( attempt && getMapNode() )
    {
        osg::View* view = aa.asView();

        Picker picker(
            dynamic_cast<osgViewer::View*>(view),
            getMapNode()->getModelLayerGroup() );

        Picker::Hits hits;

        if ( picker.pick( ea.getX(), ea.getY(), hits ) )
        {
            // find the closest indexed feature to the camera. It must be a feature
            // that is not only closest, but exists in the index as well.

            FeatureSourceIndexNode* closestIndex    = 0L;
            FeatureID               closestFID;
            double                  closestDistance = DBL_MAX;
            osg::Vec3d              closestWorldPt;

            for(Picker::Hits::iterator hit = hits.begin(); hit != hits.end(); ++hit )
            {
                FeatureSourceIndexNode* index = picker.getNode<FeatureSourceIndexNode>( *hit );
                if ( index && (hit->ratio < closestDistance) )
                {
                    if ( hit->indexList.size() == 0 )
                    {
                        OE_INFO << LC << "Index list is empty\n";
                        return true;
                    }

                    FeatureID fid;
                    if ( index->getFID( hit->drawable, *hit->indexList.begin(), fid ) )
                    {
                        closestIndex    = index;
                        closestFID      = fid;
                        closestDistance = hit->ratio;
                        closestWorldPt  = hit->matrix.valid() ? hit->localIntersectionPoint * (*hit->matrix.get()) : hit->localIntersectionPoint;
                    }
                }
            }

            if ( closestIndex )
            {
                OE_DEBUG << LC << "HIT: feature ID = " << (unsigned)closestFID << std::endl;

                Callback::EventArgs args;
                args._ea = &ea;
                args._aa = &aa;
                args._worldPoint = closestWorldPt;

                for( Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); )
                {
                    if ( i->valid() )
                    {
                        i->get()->onHit( closestIndex, closestFID, args );
                        ++i;
                    }
                    else
                    {
                        i = _callbacks.erase( i );
                    }
                }

                handled = true;
            }
        }

        if ( !handled )
        {
            OE_DEBUG << LC << "miss" << std::endl;

            Callback::EventArgs args;
            args._ea = &ea;
            args._aa = &aa;

            for( Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); )
            {
                if ( i->valid() )
                {
                    i->get()->onMiss( args );
                    ++i;
                }
                else
                {
                    i = _callbacks.erase( i );
                }
            }
        }

        _mouseDown = false;
    }

    // unmodified left mouse click
    else if (
        ea.getEventType()  == osgGA::GUIEventAdapter::PUSH &&
        ea.getModKeyMask() == 0 &&
        ea.getButtonMask() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
    {
        _mouseDown = true;
        _mouseDownX = ea.getX();
        _mouseDownY = ea.getY();
    }

    return handled;
}
