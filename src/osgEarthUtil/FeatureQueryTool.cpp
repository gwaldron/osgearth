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

#include <osgEarthUtil/FeatureQueryTool>
#include <osgEarth/Pickers>
#include <osgEarth/Registry>
#include <osgEarthFeatures/FeatureSourceNode>
#include <osgViewer/View>
#include <osg/Depth>

#define LC "[FeatureQueryTool] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Util;

#undef OE_DEBUG
#define OE_DEBUG OE_INFO

//-----------------------------------------------------------------------

FeatureQueryTool::FeatureQueryTool(MapNode*                    mapNode,
                                   FeatureQueryTool::Callback* callback) :
_mapNode( mapNode )
{
    _mapNodePath.push_back( mapNode->getTerrainEngine() );

    if ( callback )
        addCallback( callback );
}

void
FeatureQueryTool::addCallback( FeatureQueryTool::Callback* cb )
{
    if ( cb )
        _callbacks.push_back( cb );
}

bool
FeatureQueryTool::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    bool handled = false;

    // on a mouse click. Perhaps later we can parameterize this.
    if ( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
    {
        _mouseDown = true;
        _mouseDownX = ea.getX();
        _mouseDownY = ea.getY();
    }

    else if ( ea.getEventType() == osgGA::GUIEventAdapter::RELEASE )
    {
        if ( _mouseDown && ( fabs(ea.getX()-_mouseDownX) <= 3.0 && fabs(ea.getY()-_mouseDownY) <= 3.0) )
        {
            osg::View* view = aa.asView();

            //TODO: optimize so we don't bother searching the terrain
            Picker picker(
                dynamic_cast<osgViewer::View*>(view),
                _mapNode->getModelLayerGroup(),
                5.0f,
                Picker::NO_LIMIT);

            Picker::Hits hits;
            if ( picker.pick( ea.getX(), ea.getY(), hits ) )
            {
                // find the closest indexed feature to the camera. It must be a feature
                // that is not only closest, but exists in the index as well.

                FeatureSourceMultiNode* closestIndex    = 0L;
                FeatureID               closestFID;
                double                  closestDistance = DBL_MAX;

                for(Picker::Hits::iterator hit = hits.begin(); hit != hits.end(); ++hit )
                {
                    FeatureSourceMultiNode* index = picker.getNode<FeatureSourceMultiNode>( *hit );
                    if ( index && (hit->distance < closestDistance) )
                    {
                        FeatureID fid;
                        if ( index->getFID( hit->drawable, hit->primitiveIndex, fid ) )
                        {
                            closestIndex    = index;
                            closestFID      = fid;
                            closestDistance = hit->distance;
                        }
                    }
                }

                if ( closestIndex )
                {
                    OE_DEBUG << LC << "HIT: feature ID = " << (unsigned)closestFID << std::endl;

                    for( Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
                        i->get()->onHit( view, closestIndex, closestFID );

                    handled = true;
                }
            }

            if ( !handled )
            {
                for( Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
                    i->get()->onMiss( view );
            }
        }
        _mouseDown = false;
    }

    return handled;
}

//-----------------------------------------------------------------------


void
FeatureHighlightCallback::onHit( osg::View* view, FeatureSourceMultiNode* index, FeatureID fid )
{
    clear();

    FeatureSourceMultiNode::FeatureDrawSet& drawSet = index->getDrawSet(fid);
    if ( !drawSet.empty() )
    {
        osg::Geode* geode = new osg::Geode();

        osg::Group* container = 0L;

        for( FeatureSourceMultiNode::FeatureDrawSet::iterator d = drawSet.begin(); d != drawSet.end(); ++d )
        {
            osg::Geometry* featureGeom = d->first->asGeometry();
            osg::Geometry* highlightGeom = new osg::Geometry( *featureGeom, osg::CopyOp::SHALLOW_COPY );
            osg::Vec4Array* highlightColor = new osg::Vec4Array(1);
            (*highlightColor)[0] = osg::Vec4f(0,1,1,0.5);
            highlightGeom->setColorArray(highlightColor);
            highlightGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
            highlightGeom->setPrimitiveSetList( d->second );
            geode->addDrawable(highlightGeom);

            if ( !container )
            {
                // establishes a container for the highlight geometry.
                osg::Geode* featureGeode = dynamic_cast<osg::Geode*>( featureGeom->getParent(0) );
                container = featureGeode->getParent(0);
                if ( featureGeom->getStateSet() )
                    geode->getOrCreateStateSet()->merge( *featureGeom->getStateSet() );
            }
        }

        osg::StateSet* sset = geode->getOrCreateStateSet();

        // set up to overwrite the real geometry:
        sset->setAttributeAndModes( new osg::Depth(osg::Depth::LEQUAL,0,1,false), osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE );
        sset->setRenderBinDetails( 42, "DepthSortedBin" );

        // turn off texturing:
        for( int ii = 0; ii < Registry::instance()->getCapabilities().getMaxFFPTextureUnits(); ++ii )
        {
            sset->setTextureMode( ii, GL_TEXTURE_2D, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
            sset->setTextureMode( ii, GL_TEXTURE_3D, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
            //sset->setTextureMode( ii, GL_TEXTURE_RECTANGLE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
            //sset->setTextureMode( ii, GL_TEXTURE_CUBE_MAP, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
        }

        sset->setMode( GL_BLEND,    1 );
        sset->setMode( GL_LIGHTING, 0 );

        container->addChild( geode );


        Selection selection;
        selection._index     = index;
        selection._fid       = fid;
        selection._geode     = geode;

        _selections.insert( selection );
    }
}

void
FeatureHighlightCallback::onMiss( osg::View* view )
{
    clear();
}

void
FeatureHighlightCallback::clear()
{
    for( SelectionSet::iterator i = _selections.begin(); i != _selections.end(); ++i )
    {
        Selection& selection = *i;
        selection._geode->getParent(0)->removeChild( selection._geode );
    }
    _selections.clear();
}
