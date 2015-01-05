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


TileNode::TileNode(const TileKey& key, const TileModel* model, const osg::Matrixd& matrix) :
_key               ( key ),
_model             ( model ),
_lastTraversalFrame( 0 ),
_dirty             ( false ),
_outOfDate         ( false )
{
    this->setName( key.str() );
    this->setMatrix( matrix );

    // revisions are initially in sync:
    if ( model )
    {
        _maprevision = model->_revision;
        if ( model->requiresUpdateTraverse() )
        {
            this->setNumChildrenRequiringUpdateTraversal(1);
        }
        
        if (model->_elevationTexture.valid() && model->_elevationData.getLocator())
        {
            osg::Matrixd elevMatrix;

            model->_tileLocator->createScaleBiasMatrix(
                model->_elevationData.getLocator()->getDataExtent(),
                elevMatrix);

            _elevTexMat = new osg::RefMatrix(elevMatrix);
        }

        if (model->_normalTexture.valid() && model->_normalData.getLocator())
        {
            osg::Matrixd normalMatrix;

            model->_tileLocator->createScaleBiasMatrix(
                model->_normalData.getLocator()->getDataExtent(),
                normalMatrix);

            _normalTexMat = new osg::RefMatrix(normalMatrix);
        }
    }
}

osg::Texture*
TileNode::getElevationTexture() const
{
    return _model.valid() ?
        _model->_elevationTexture.get() :
        0L;
}

osg::RefMatrix*
TileNode::getElevationTextureMatrix() const
{
    return _elevTexMat.get();
}

osg::Texture*
TileNode::getNormalTexture() const
{
    return _model.valid() ?
        _model->_normalTexture.get() :
        0L;
}

osg::RefMatrix*
TileNode::getNormalTextureMatrix() const
{
    return _normalTexMat.get();
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

void
TileNode::traverse( osg::NodeVisitor& nv )
{
    if ( _model.valid() )
    {
        if ( nv.getVisitorType() == nv.CULL_VISITOR )
        {
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
