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

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TileNode] "


//----------------------------------------------------------------------------

TileNode::TileNode( const TileKey& key, const TileModel* model ) :
_key               ( key ),
_model             ( model ),
_lastTraversalFrame( 0 ),
_dirty             ( false ),
_outOfDate         ( false )
{
    this->setName( key.str() );

    // revisions are initially in sync:
    if ( model )
    {
        _maprevision = model->_revision;
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


void
TileNode::traverse( osg::NodeVisitor& nv )
{
    if ( _model.valid() )
    {
        if ( nv.getVisitorType() == nv.CULL_VISITOR )
        {
            osg::ClusterCullingCallback* ccc = dynamic_cast<osg::ClusterCullingCallback*>(getCullCallback());
            if (ccc)
            {
                if (ccc->cull(&nv,0,static_cast<osg::State *>(0))) return;
            }

            // if this tile is marked dirty, bump the marker so the engine knows it
            // needs replacing.
            if ( _dirty || _model->_revision != _maprevision )
            {
                _outOfDate = true;
            }
        }
        else if (nv.getVisitorType() == nv.UPDATE_VISITOR)
        {
            _model->updateTraverse(nv);
        }
    }    

    osg::MatrixTransform::traverse( nv );
}

void
TileNode::releaseGLObjects(osg::State* state) const
{
    osg::MatrixTransform::releaseGLObjects( state );

    if ( _model.valid() )
        _model->releaseGLObjects( state );
}

void
TileNode::resizeGLObjectBuffers(unsigned maxSize)
{
    osg::MatrixTransform::resizeGLObjectBuffers( maxSize );

    if ( _model.valid() )
        const_cast<TileModel*>(_model.get())->resizeGLObjectBuffers( maxSize );
}
