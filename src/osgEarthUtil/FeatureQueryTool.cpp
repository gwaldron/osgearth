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
#include <osgViewer/View>
#include <osg/Depth>

#define LC "[FeatureQueryTool] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

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

bool
FeatureQueryTool::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    bool handled = false;
    bool attempt;

    if ( _inputPredicate.valid() )
    {
        attempt = _inputPredicate->test(ea);
    }
    else
    {
        attempt =
            ea.getEventType() == osgGA::GUIEventAdapter::RELEASE &&
            _mouseDown && 
            fabs(ea.getX()-_mouseDownX) <= 3.0 && 
            fabs(ea.getY()-_mouseDownY) <= 3.0;
    }

    if ( attempt )
    {
        osg::View* view = aa.asView();

        Picker picker(
            dynamic_cast<osgViewer::View*>(view),
            _mapNode->getModelLayerGroup() );

        Picker::Hits hits;

        if ( picker.pick( ea.getX(), ea.getY(), hits ) )
        {
            // find the closest indexed feature to the camera. It must be a feature
            // that is not only closest, but exists in the index as well.

            FeatureSourceIndexNode* closestIndex    = 0L;
            FeatureID               closestFID;
            double                  closestDistance = DBL_MAX;

            for(Picker::Hits::iterator hit = hits.begin(); hit != hits.end(); ++hit )
            {
                FeatureSourceIndexNode* index = picker.getNode<FeatureSourceIndexNode>( *hit );
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

                Callback::EventArgs args;
                args._ea = &ea;
                args._aa = &aa;

                for( Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
                    i->get()->onHit( closestIndex, closestFID, args );

                handled = true;
            }
        }

        if ( !handled )
        {
            OE_DEBUG << LC << "miss" << std::endl;

            Callback::EventArgs args;
            args._ea = &ea;
            args._aa = &aa;

            for( Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i )
                i->get()->onMiss( args );
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

//-----------------------------------------------------------------------

void
FeatureHighlightCallback::onHit( FeatureSourceIndexNode* index, FeatureID fid, const EventArgs& args )
{
    clear();

    FeatureSourceIndexNode::FeatureDrawSet& drawSet = index->getDrawSet(fid);
    if ( !drawSet.empty() )
    {
        osg::Group* container = 0L;
        osg::Group* group = new osg::Group();
        osg::Geode* geode = 0L;

        for( FeatureSourceIndexNode::PrimitiveSetGroups::iterator d = drawSet.primSetGroups.begin(); d != drawSet.primSetGroups.end(); ++d )
        {
            osg::Geometry* featureGeom = d->first->asGeometry();
            osg::Geometry* highlightGeom = new osg::Geometry( *featureGeom, osg::CopyOp::SHALLOW_COPY );
            osg::Vec4Array* highlightColor = new osg::Vec4Array(1);
            (*highlightColor)[0] = osg::Vec4f(0,1,1,0.5);
            highlightGeom->setColorArray(highlightColor);
            highlightGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
            highlightGeom->setPrimitiveSetList( d->second );

            if ( !geode )
            {
                geode = new osg::Geode();
                group->addChild( geode );
            }

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

        for( FeatureSourceIndexNode::NodeVector::iterator n = drawSet.nodes.begin(); n != drawSet.nodes.end(); ++n )
        {
            group->addChild( *n );
            if ( !container )
                container = (*n)->getParent(0);
        }

        osg::StateSet* sset = group->getOrCreateStateSet();

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

        sset->setMode( GL_BLEND,    osg::StateAttribute::ON  | osg::StateAttribute::OVERRIDE );
        sset->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );

        container->addChild( group );

        Selection selection;
        selection._index     = index;
        selection._fid       = fid;
        selection._group     = group;

        _selections.insert( selection );
    }
}

void
FeatureHighlightCallback::onMiss( const EventArgs& args )
{
    clear();
}

void
FeatureHighlightCallback::clear()
{
    for( SelectionSet::iterator i = _selections.begin(); i != _selections.end(); ++i )
    {
        Selection& selection = *i;
        osg::ref_ptr<osg::Group> safeGroup = selection._group.get();
        if ( safeGroup.valid() && safeGroup->getNumParents() > 0 )
        {
            osg::Group* parent = safeGroup->getParent(0);
            if ( parent ) 
                parent->removeChild( safeGroup.get() );
        }
    }
    _selections.clear();
}


//-----------------------------------------------------------------------

FeatureReadoutCallback::FeatureReadoutCallback( Container* container )
{
    _grid = new Grid();
    _grid->setBackColor( Color(Color::Black,0.7f) );
    container->addControl( _grid );
}

void
FeatureReadoutCallback::onHit( FeatureSourceIndexNode* index, FeatureID fid, const EventArgs& args )
{
    clear();
    if ( index && index->getFeatureSource() )
    {
        Feature* f = index->getFeatureSource()->getFeature( fid );
        if ( f )
        {
            unsigned r=0;
            const AttributeTable& attrs = f->getAttrs();
            for( AttributeTable::const_iterator i = attrs.begin(); i != attrs.end(); ++i, ++r )
            {
                _grid->setControl( 0, r, new LabelControl(i->first, 14.0f, Color::Yellow) );
                _grid->setControl( 1, r, new LabelControl(i->second.getString(), 14.0f, Color::White) );
            }
            _grid->setVisible( true );
        }
    }
    args._aa->requestRedraw();
}

void
FeatureReadoutCallback::onMiss( const EventArgs& args )
{
    clear();
    args._aa->requestRedraw();
}

void
FeatureReadoutCallback::clear()
{
    _grid->clearControls();
    _grid->setVisible( false );
}
