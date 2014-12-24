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
#include "TileNode"

#include <osg/ClusterCullingCallback>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Uniform>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/DrawInstanced>
#include <osgEarth/Registry>
#include <osgEarth/CullingUtils>
#include <osgUtil/Optimizer>

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TileNode] "

TileNode::TileNode() :
_model( 0L )
{
    //NOP
}

TileNode::TileNode(const TerrainTileModel* model) :
_model             ( model ),
_lastTraversalFrame( 0 ),
_dirty             ( false ),
_outOfDate         ( false )
{
    // model required.
    if ( !model )
    {
        OE_WARN << LC << "Illegal: Created a tile node with no model\n";
        return;
    }

    this->setName( _model->getKey().str() );

    // revisions are initially in sync:
    if ( model )
    {
        _maprevision = model->getRevision();

        if ( model->requiresUpdateTraverse() )
        {
            this->setNumChildrenRequiringUpdateTraversal(1);
        }
    }
}


void
TileNode::setLastTraversalFrame(unsigned frame)
{
    _lastTraversalFrame = frame;
}

osg::Group*
TileNode::getPayloadGroup() const
{
    return _payload.get();
}

osg::Group*
TileNode::getOrCreatePayloadGroup()
{
    if ( !_payload.valid() )
    {
        osg::StateSet* stateSet = new osg::StateSet();
        std::string binName = Stringify() << "oe.PayloadBin." << _engineUID;
        stateSet->setRenderBinDetails(1, binName);
        stateSet->setNestRenderBins( false );

        _payload = new osg::Group();
        _payload->setStateSet( stateSet );
        this->addChild( _payload.get() );
    }
    return _payload.get();
}
