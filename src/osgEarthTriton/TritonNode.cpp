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
#include <osgEarth/NodeUtils>

#define LC "[TritonNode] "

using namespace osgEarth::Triton;


TritonNode::TritonNode(const TritonOptions& options,
                       Callback* callback) :
OceanNode( options ),
_options ( options ),
_callback( callback ),
_needsMapNode( true )
{
    // Triton requires a constant update traversal.
    ADJUST_UPDATE_TRAV_COUNT(this, +1);
}


// @deprecated ctor
TritonNode::TritonNode(osgEarth::MapNode* mapNode,
                       const TritonOptions& options,
                       Callback* callback) :
OceanNode( options ),
_options ( options ),
_callback( callback )
{
    // Triton requires a constant update traversal.
    ADJUST_UPDATE_TRAV_COUNT(this, +1);

    setMapNode(mapNode);
    _needsMapNode = (mapNode == 0L);
}

void
TritonNode::setMaskLayer(const osgEarth::ImageLayer* maskLayer)
{
    _maskLayer = maskLayer;
    create();
}

void
TritonNode::setMapNode(osgEarth::MapNode* mapNode)
{
    if (!mapNode)
    {
        this->removeChildren(0, this->getNumChildren());
        _drawable = 0L;
        _TRITON = 0L;
        setSRS(0L);
        _needsMapNode = true;
    }
    else
    {
        _mapNode = mapNode;
        create();
    }
}

void
TritonNode::create()
{    
    this->removeChildren(0, this->getNumChildren());
    _drawable = 0L;

    osg::ref_ptr<MapNode> mapNode;
    if (!_mapNode.lock(mapNode))
        return;

    const osgEarth::Map* map = mapNode->getMap();
    if ( map )
        setSRS( map->getSRS() );

    // Remember the resource releaser so we can properly destroy 
    // Triton objects in a graphics context.
    _releaser = mapNode->getResourceReleaser();

    // create an object to house Triton data and resources.
    if (!_TRITON.valid())
        _TRITON = new TritonContext(_options);

    if ( map )
        _TRITON->setSRS( map->getSRS() );

    if ( _callback.valid() )
        _TRITON->setCallback( _callback.get() );

    TritonDrawable* drawable = new TritonDrawable(mapNode.get(), _TRITON.get());
    _drawable = drawable;
    _alphaUniform = getOrCreateStateSet()->getOrCreateUniform("oe_ocean_alpha", osg::Uniform::FLOAT);
    _alphaUniform->set(getAlpha());
    _drawable->setNodeMask( TRITON_OCEAN_MASK );
    drawable->setMaskLayer(_maskLayer.get());
    this->addChild(_drawable);

    drawable->_heightCameraParent = this;

    //this->setNumChildrenRequiringUpdateTraversal(1);

    // Place in the depth-sorted bin and set a rendering order.
    // We want Triton to render after the terrain.
    _drawable->getOrCreateStateSet()->setRenderBinDetails( _options.renderBinNumber().get(), "DepthSortedBin" );
}

TritonNode::~TritonNode()
{
    // submit the TRITON context to the releaser so it can shut down Triton
    // objects in a valid graphics context.
    if (_TRITON.valid())
    {
        osg::ref_ptr<ResourceReleaser> releaser;
        if (_releaser.lock(releaser))
        {
            releaser->push(_TRITON.get());
        }
    }
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

void
TritonNode::onSetAlpha()
{
    _alphaUniform->set(getAlpha());
}

osg::BoundingSphere
TritonNode::computeBound() const
{
    return osg::BoundingSphere();
}

void
TritonNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.UPDATE_VISITOR )
    {
        // Find a MapNode in the traversal path if necessary:
        if (_needsMapNode)
        {
            MapNode* mapNode = osgEarth::findInNodePath<MapNode>(nv);
            if (mapNode)
            {
                setMapNode(mapNode);
                _needsMapNode = false;
            }
        }

        // Tick Triton each frame:
        if (_TRITON->ready())
        {
            _TRITON->update(nv.getFrameStamp()->getSimulationTime());
        }
    }

    osgEarth::Util::OceanNode::traverse(nv);
}
