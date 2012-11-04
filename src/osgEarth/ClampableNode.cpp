/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
OverlayNode( mapNode, active, &getTechniqueGroup ),
_bias      ( 100.0f, 10000.0f ),
_range     ( 1000.0f, 10000000.f ),
_autoBias  ( true )
{
    osg::StateSet* s = this->getOrCreateStateSet();

    _biasUniform = s->getOrCreateUniform( "oe_clamp_bias", osg::Uniform::FLOAT_VEC2 );
    _biasUniform->set( _bias );

    _rangeUniform = s->getOrCreateUniform( "oe_clamp_range", osg::Uniform::FLOAT_VEC2 );
    _rangeUniform->set( _range );
}

void
ClampableNode::setDepthOffsetBias( float minBias, float maxBias )
{
    _autoBias = false;
    _bias[0] = minBias;
    _bias[1] = maxBias;
    _biasUniform->set( _bias );
}

void
ClampableNode::setDepthOffsetRange( float minRange, float maxRange )
{
    _range[0] = minRange;
    _range[1] = maxRange;
    _rangeUniform->set( _range );
}

void
ClampableNode::calculateDepthOffsetBiasFromSubgraph()
{
    _bias[0] = DepthOffsetUtils::recalculate( this );
    _biasUniform->set( _bias );
}

osg::BoundingSphere
ClampableNode::computeBound() const
{
    if ( _autoBias )
    {
        _bias[0] = DepthOffsetUtils::recalculate( this );
        _biasUniform->set( _bias );
    }

    return osg::Group::computeBound();
}
