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
#include <osgEarthFeatures/FeatureDrawSet>
#include <osg/Geode>
#include <osg/MatrixTransform>

using namespace osgEarth;
using namespace osgEarth::Features;

#define LC "[FeatureDrawSet] "

//-----------------------------------------------------------------------------

namespace
{
    // walks a node path, accumulating the state, when merges in the final set and returns the result.
    osg::StateSet* accumulateStateSet( const osg::NodePath& path, osg::StateSet* final )
    {
        osg::StateSet* s = new osg::StateSet();
        for( osg::NodePath::const_iterator i = path.begin(); i != path.end(); ++i )
        {
            if ( (*i)->getStateSet() )
                s->merge( *(*i)->getStateSet() );
        }

        if ( final )
            s->merge( *final );
        return s;
    }
}

//-----------------------------------------------------------------------------

FeatureDrawSet::FeatureDrawSet() :
_visible( true )
{
    //nop
}


FeatureDrawSet::PrimitiveSets&
FeatureDrawSet::getOrCreateSlice(osg::Drawable* d)
{
    for( DrawableSlices::iterator i = _slices.begin(); i != _slices.end(); ++i )
    {
        if ( i->first.get() == d )
        {
            return i->second;
        }
    }

    _slices.push_back( DrawableSlice(d, PrimitiveSets()) );
    return _slices.back().second;
}

FeatureDrawSet::DrawableSlices::iterator 
FeatureDrawSet::slice(osg::Drawable* d)
{
    for( DrawableSlices::iterator i = _slices.begin(); i != _slices.end(); ++i )
    {
        if ( i->first.get() == d )
        {
            return i;
        }
    }
    return _slices.end();
}

FeatureDrawSet::DrawableSlices::const_iterator 
FeatureDrawSet::slice(osg::Drawable* d) const
{
    for( DrawableSlices::const_iterator i = _slices.begin(); i != _slices.end(); ++i )
    {
        if ( i->first.get() == d )
        {
            return i;
        }
    }
    return _slices.end();
}


void
FeatureDrawSet::setVisible( bool visible )
{
    if ( _visible )
    {
        _invisibleMasks.clear();
        for( unsigned i=0; i<_nodes.size(); ++i )
        {
            _invisibleMasks.push_back( _nodes[i]->getNodeMask() );
            _nodes[i]->setNodeMask( 0 );
        }

        for( unsigned i=0; i < _slices.size(); ++i )
        {
            DrawableSlice& slice = _slices[i];
            osg::Geometry* geom = slice.first->asGeometry();
            for( PrimitiveSets::iterator p = slice.second.begin(); p != slice.second.end(); ++p )
                geom->removePrimitiveSet( geom->getPrimitiveSetIndex(p->get()) );
        }
    }

    else // (!_visible)
    {
        for( unsigned i=0; i<_nodes.size(); ++i )
        {
            _nodes[i]->setNodeMask( _invisibleMasks[i] );
        }
        _invisibleMasks.clear();

        for( unsigned i=0; i < _slices.size(); ++i )
        {
            DrawableSlice& slice = _slices[i];
            osg::Geometry* geom = slice.first->asGeometry();
            for( PrimitiveSets::iterator p = slice.second.begin(); p != slice.second.end(); ++p )
                geom->addPrimitiveSet( p->get() );
        }
    }

    _visible = visible;
}


void
FeatureDrawSet::clear()
{
    _nodes.clear();
    _slices.clear();
    _invisibleMasks.clear();
    _visible = true;
}


osg::Node*
FeatureDrawSet::createCopy()
{
    osg::Group* group = new osg::Group();

    for( Nodes::iterator n = _nodes.begin(); n != _nodes.end(); ++n )
    {
        osg::Node* node = n->get();
        osg::Node* nodeCopy = osg::clone(node, osg::CopyOp::SHALLOW_COPY);
        osg::Matrix local2world = osg::computeLocalToWorld( node->getParentalNodePaths()[0] );
        if ( !local2world.isIdentity() )
        {
            osg::MatrixTransform* xform = new osg::MatrixTransform(local2world);
            xform->addChild( nodeCopy );
            group->addChild( xform );
        }
        else
        {
            group->addChild( nodeCopy );
        }
    }

    osg::Geode* geode = 0L;
    for( DrawableSlices::iterator p = _slices.begin(); p != _slices.end(); ++p )
    {
        DrawableSlice& slice = *p;

        osg::Drawable* d = slice.first.get();

        const PrimitiveSets& psets = slice.second;
        if ( psets.size() > 0 )
        {        
            osg::Geometry* featureGeom = d->asGeometry();

            osg::NodePath path = featureGeom->getParent(0)->getParentalNodePaths()[0];

            // make a shallow copy of the geometry (share all the buffer arrays)
            osg::Geometry* copiedGeom = new osg::Geometry( *featureGeom, osg::CopyOp::SHALLOW_COPY );
            copiedGeom->setPrimitiveSetList( psets );

            // build the state set do it matches:
            copiedGeom->setStateSet( accumulateStateSet(path, copiedGeom->getStateSet()) );

            // add to our geode
            if ( !geode )
                geode = new osg::Geode();

            geode->addDrawable( copiedGeom );

            // include a matrix transform if necessary:
            osg::Matrix local2world = osg::computeLocalToWorld( path );
            if ( !local2world.isIdentity() )
            {
                osg::MatrixTransform* xform = new osg::MatrixTransform(local2world);
                xform->addChild( geode );
                group->addChild( xform );
            }
            else
            {
                group->addChild( geode );
            }
        }
    }

    return group;
}