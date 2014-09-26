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

#include "TritonNode"
#include "TritonContext"
#include "TritonDrawable"
#include <osgEarth/CullingUtils>
#include <Triton.h>

#define LC "[TritonNode] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Drivers::Triton;

TritonNode::TritonNode(MapNode*           mapNode,
                       const TritonOptions& options) :
OceanNode( options ),
_options ( options )
{
    const Map* map = mapNode->getMap();
    if ( map )
        setSRS( map->getSRS() );

    _TRITON = new TritonContext( options );

    if ( map )
        _TRITON->setSRS( map->getSRS() );

    TritonDrawable* tritonDrawable = new TritonDrawable(mapNode,_TRITON);
    _drawable = tritonDrawable;
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( _drawable );
    geode->setNodeMask( OCEAN_MASK );

    this->addChild( geode );

    this->setNumChildrenRequiringUpdateTraversal(1);
}

TritonNode::~TritonNode()
{
    //nop
}

void
TritonNode::onSetSeaLevel()
{
    if ( _TRITON->ready() )
    {
        _TRITON->getEnvironment()->SetSeaLevel( getSeaLevel() );
    }
    dirtyBound();
}

osg::BoundingSphere
TritonNode::computeBound() const
{
    return osg::BoundingSphere();
}

void
TritonNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.UPDATE_VISITOR && _TRITON->ready() )
    {
        _TRITON->update(nv.getFrameStamp()->getSimulationTime());
    }
    OceanNode::traverse(nv);
}
