/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

#include <osgEarth/ClampableNode>
#include <osgEarth/ClampingTechnique>
#include <osgEarth/DepthOffset>
#include <osgEarth/OverlayDecorator>
#include <osgEarth/MapNode>

#define LC "[ClampableNode] "

using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    static osg::Group* getTechniqueGroup(MapNode* m)
    {
        return m ? m->getOverlayDecorator()->getGroup<ClampingTechnique>() : 0L;
    }
}

//------------------------------------------------------------------------

ClampableNode::ClampableNode( MapNode* mapNode, bool active ) :
OverlayNode( mapNode, active, &getTechniqueGroup )
{
    init();
}

void
ClampableNode::init()
{
    // auto-bias starts out true, but if you set the depth offset options
    // it will toggle to false.
    _autoBias = true;

    _doDirty  = false;

    osg::StateSet* s = this->getOrCreateStateSet();

    _biasUniform = s->getOrCreateUniform( "oe_clamp_bias", osg::Uniform::FLOAT_VEC2 );
    _biasUniform->set( osg::Vec2f(*_do.minBias(), *_do.maxBias()) );

    _rangeUniform = s->getOrCreateUniform( "oe_clamp_range", osg::Uniform::FLOAT_VEC2 );
    _rangeUniform->set( osg::Vec2f(*_do.minRange(), *_do.maxRange()) );
}

void
ClampableNode::dirtyDepthOffsetOptions()
{
    if ( !_doDirty )
    {
        _doDirty = true;
        _autoBias = false;
        ADJUST_UPDATE_TRAV_COUNT( this, 1 );
    }
}

void
ClampableNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.UPDATE_VISITOR )
    {
        applyDepthOffsetOptions();
        _doDirty = false;
        ADJUST_UPDATE_TRAV_COUNT( this, -1 );
    }
    OverlayNode::traverse(nv);
}

void
ClampableNode::applyDepthOffsetOptions()
{
    if ( _do.enabled() == true )
    {
        _biasUniform->set( osg::Vec2f(*_do.minBias(), *_do.maxBias()) );
        _rangeUniform->set( osg::Vec2f(*_do.minRange(), *_do.maxRange()) );
        dirtyBound();
    }
    else
    {
        _biasUniform->set( osg::Vec2f(0.0f, 0.0f) );
    }
}

void
ClampableNode::setAutoCalculateDepthOffset()
{
    // prompts OSG to call computeBound() on the next pass which
    // will recalculate the minimum bias.
    _autoBias = true;
    dirtyBound();
}

osg::BoundingSphere
ClampableNode::computeBound() const
{
    if ( _autoBias && _do.enabled() == true )
    {
        _do.minBias() = DepthOffsetUtils::recalculate( this );
        _biasUniform->set( osg::Vec2f(*_do.minBias(), *_do.maxBias()) );
    }

    return OverlayNode::computeBound();
}
