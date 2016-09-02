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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
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

            model->_elevationData.getLocator()->createScaleBiasMatrix(
                key.getExtent(),
                elevMatrix );

            _elevTexMat = new osg::RefMatrixf( osg::Matrixf(elevMatrix) );
            
            osg::StateSet* stateSet = getOrCreateStateSet();

            // TEMPORARY.
            stateSet->setTextureAttribute(
                2,
                _model->_elevationTexture.get() );

            stateSet->addUniform( new osg::Uniform(
                "oe_terrain_tex_matrix",
                osg::Matrixf(elevMatrix)) );
        }

        if (model->_normalTexture.valid() && model->_normalData.getLocator())
        {
            osg::Matrixf normalMatrix;

            model->_normalData.getLocator()->createScaleBiasMatrix(
                getKey().getExtent(),
                normalMatrix );

            // apply a small scale/bias that will center the sampling coords
            // on the texels. This will prevent "seams" from forming between
            // tiles then using a non-identity texture matrix.
            float size = (float)_model->_normalTexture->getImage(0)->s();
            osg::Matrixf samplingScaleBias =
                osg::Matrixf::translate(0.5f/(size-1.0f), 0.5f/(size-1.0f), 0.0) *
                osg::Matrixf::scale((size-1.0f)/size, (size-1.0f)/size, 1.0f);

            normalMatrix.postMult( samplingScaleBias );

            _normalTexMat = new osg::RefMatrixf(normalMatrix);
            
            osg::StateSet* stateSet = getOrCreateStateSet();

            stateSet->setTextureAttribute(
                model->_normalData.getUnit(),
                model->_normalTexture.get() );

            stateSet->addUniform( new osg::Uniform(
                "oe_tile_normalTexMatrix", *_normalTexMat.get() ) );
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

#define OE_TEST OE_DEBUG
void
TileNode::notifyOfArrival(TileNode* that)
{
    OE_TEST << LC << this->getKey().str()
        << " was waiting on "
        << that->getKey().str() << " and it arrived.\n";
        
    osg::Texture* thisTex = this->getNormalTexture();
    osg::Texture* thatTex = that->getNormalTexture();
    if ( !thisTex || !thatTex ) {
        OE_TEST << LC << "bailed on " << getKey().str() << " - null normal texture\n";
        return;
    }

    osg::RefMatrixf* thisTexMat = this->getNormalTextureMatrix();
    osg::RefMatrixf* thatTexMat = that->getNormalTextureMatrix();
    if ( !thisTexMat || !thatTexMat || !thisTexMat->isIdentity() || !thatTexMat->isIdentity() ) {
        OE_TEST << LC << "bailed on " << getKey().str() << " - null texmat\n";
        return;
    }

    osg::Image* thisImage = thisTex->getImage(0);
    osg::Image* thatImage = thatTex->getImage(0);
    if ( !thisImage || !thatImage ) {
        OE_TEST << LC << "bailed on " << getKey().str() << " - null image\n";
        return;
    }

    int width = thisImage->s();
    int height = thisImage->t();
    if ( width != thatImage->s() || height != thatImage->t() ) {
        OE_TEST << LC << "bailed on " << getKey().str() << " - mismatched sizes\n";
        return;
    }

    if (_model->_normalData.isFallbackData()) {
        OE_TEST << LC << "bailed on " << getKey().str() << " - fallback data\n";
        return;
    }

    // Just copy the neighbor's edge normals over to our texture.
    // Averaging them would be more accurate, but then we'd have to
    // re-generate each texture multiple times instead of just once.
    // Besides, there's almost no visual difference anyway.
    ImageUtils::PixelReader readThat(thatImage);
    ImageUtils::PixelWriter writeThis(thisImage);

    bool thisDirty = false;

    if ( that->getKey() == getKey().createNeighborKey(1,0) )
    {
        // neighbor is to the east:
        for(int t=0; t<height; ++t)
        {
            writeThis(readThat(0,t), width-1, t);
        }
        thisDirty = true;
    }

    else if ( that->getKey() == getKey().createNeighborKey(0,1) )
    {
        // neighbor is to the south:
        for(int s=0; s<width; ++s)
        {
            writeThis(readThat(s, height-1), s, 0);
        }
        thisDirty = true;
    }

    else
    {
        OE_INFO << LC << "Unhandled notify\n";
    }

    if ( thisDirty )
    {
        // so heavy handed. Wish we could just write the row
        // or column that changed.
        thisImage->dirty();
    }
}
