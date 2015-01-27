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
#include <osgEarth/ImageUtils>
#include <osgUtil/Optimizer>

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TileNode] "


TileNode::TileNode(const TileKey& key, TileModel* model, const osg::Matrixd& matrix) :
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

            _elevTexMat = new osg::RefMatrixf( osg::Matrixf(elevMatrix) );
            
            // just stick this here for now.
            osg::StateSet* stateSet = getOrCreateStateSet();

            stateSet->setTextureAttribute(
                2,
                _model->_elevationTexture.get() );

            stateSet->addUniform( new osg::Uniform(
                "oe_terrain_tex_matrix",
                osg::Matrixf(elevMatrix)) );
        }

        if (model->_normalTexture.valid() && model->_normalData.getLocator())
        {
            osg::Matrixd normalMatrix;

            model->_tileLocator->createScaleBiasMatrix(
                model->_normalData.getLocator()->getDataExtent(),
                normalMatrix);

            _normalTexMat = new osg::RefMatrixf( osg::Matrixf(normalMatrix) );
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

osg::RefMatrixf*
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

osg::RefMatrixf*
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

void
TileNode::notifyOfArrival(TileNode* that)
{
    OE_DEBUG << LC << this->getKey().str()
        << " was waiting on "
        << that->getKey().str() << " and it arrived.\n";
        
    osg::Texture* thisTex = this->getNormalTexture();
    osg::Texture* thatTex = that->getNormalTexture();
    if ( !thisTex || !thatTex ) {
        OE_DEBUG << LC << "bailed on " << getKey().str() << " - null normal texture\n";
        return;
    }

    osg::RefMatrixf* thisTexMat = this->getNormalTextureMatrix();
    osg::RefMatrixf* thatTexMat = that->getNormalTextureMatrix();
    if ( !thisTexMat || !thatTexMat || (*thisTexMat != *thatTexMat) ) {
        OE_DEBUG << LC << "bailed on " << getKey().str() << " - null texmat\n";
        return;
    }

    osg::Image* thisImage = thisTex->getImage(0);
    osg::Image* thatImage = thatTex->getImage(0);
    if ( !thisImage || !thatImage ) {
        OE_DEBUG << LC << "bailed on " << getKey().str() << " - null image\n";
        return;
    }

    int width = thisImage->s();
    int height = thisImage->t();
    if ( width != thatImage->s() || height != thatImage->t() ) {
        OE_DEBUG << LC << "bailed on " << getKey().str() << " - mismatched sizes\n";
        return;
    }

    if (_model->_normalData.isFallbackData()) {
        OE_DEBUG << LC << "bailed on " << getKey().str() << " - fallback data\n";
        return;
    }

    ImageUtils::PixelReader readThis(thisImage);
    ImageUtils::PixelWriter writeThis(thisImage);

    ImageUtils::PixelReader readThat(thatImage);
    ImageUtils::PixelWriter writeThat(thatImage);

    bool dirty = false;

    if ( that->getKey() == getKey().createNeighborKey(1,0) )
    {
        // "that" is to the east:
        for(int t=0; t<height; ++t)
        {
            osg::Vec4f average = (readThis(width-1, t) + readThat(0, t))*0.5;
            writeThis(average, width-1, t);
            writeThat(average, 0, t);
        }
        dirty = true;
    }

    else if ( that->getKey() == getKey().createNeighborKey(0,1) )
    {
        // neighbor is to the south:
        for(int s=0; s<width; ++s)
        {
            osg::Vec4f average = (readThis(s, 0) + readThat(s, height-1))*0.5;
            writeThis(average, s, 0);
            writeThat(average, s, height-1);
        }
        dirty = true;
    }

    else
    {
        OE_WARN << LC << "That's weird.\n";
    }

    if ( dirty )
    {
        thisImage->dirty();
        thatImage->dirty();
    }
}
