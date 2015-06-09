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
#include "SurfaceNode"
#include "TileGroupFactory"
#include "Loader"
#include "LoadTileData"
#include "ExpireTiles"

#include <osgEarth/CullingUtils>
#include <osg/Uniform>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[TileNode] "


namespace
{
    // Scale and bias matrices, one for each TileKey quadrant.
    const osg::Matrixf scaleBias[4] =
    {
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.0f,0.5f,0,1.0f),
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.5f,0.5f,0,1.0f),
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.0f,0.0f,0,1.0f),
        osg::Matrixf(0.5f,0,0,0, 0,0.5f,0,0, 0,0,1.0f,0, 0.5f,0.0f,0,1.0f)
    };

    void applyScaleBias(osg::Matrixf* m, unsigned quadrant)
    {
        m->preMult( scaleBias[quadrant] );
    }

    void applyScaleBias(osg::Matrix& m, unsigned quadrant)
    {
        m.preMult( scaleBias[quadrant] );
    }
}



TileNode::TileNode() :
_dirty( false )
{
    //nop
}

void
TileNode::create(const TileKey& key, TileGroupFactory* context)
{
    _key = key;

    // Next, build the surface geometry for the node.
    SurfaceNode* surface = new SurfaceNode(
        key,
        context->getMapFrame().getMapInfo(),
        context->getRenderBindings(),
        context->getGeometryPool() );

    _payload = surface;

    // add a cluster-culling callback:                
    if ( context->getMapFrame().getMapInfo().isGeocentric() )
    {
        addCullCallback( ClusterCullingFactory::create(key.getExtent()) );
    }

    // need to recompute the bounds after adding payload:
    dirtyBound();

    // signal the tile to start loading data:
    setDirty( true );

    // register me.
    context->liveTiles()->add( this );
}

osg::BoundingSphere
TileNode::computeBound() const
{
    return _payload.valid() ? _payload->computeBound() : osg::BoundingSphere();
}

float
TileNode::getSubdivideRange(osg::NodeVisitor& nv) const
{
    // TODO. This is just a placeholder.
    return getBound().radius() * 6.0;
}

bool
TileNode::isDormant(osg::NodeVisitor& nv) const
{
    return
        nv.getFrameStamp() &&
        nv.getFrameStamp()->getFrameNumber() - _lastTraversalFrame > 2u;
}

bool
TileNode::isReadyToTraverse() const
{
    // Later, we might replace this if the tile isn't immediately created at cull time.
    return true;
}

void
TileNode::setDirty(bool value)
{
    _dirty = value;
}

void
TileNode::releaseGLObjects(osg::State* state) const
{
    if ( getStateSet() )
        getStateSet()->releaseGLObjects(state);
    if ( _payload.valid() )
        _payload->releaseGLObjects(state);

    osg::Group::releaseGLObjects(state);
}

void
TileNode::traverse(osg::NodeVisitor& nv)
{
    // Cull
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        if ( nv.getFrameStamp() )
        {
            _lastTraversalFrame.exchange( nv.getFrameStamp()->getFrameNumber() );
        }

        const osg::Vec3& center = getBound().center();

        float cameraRange = nv.getDistanceToViewPoint( center, true );

        // If we're in range of the children, try to cull them:
        if ( cameraRange < getSubdivideRange(nv) && getTileKey().getLOD() < 25 )
        {
            // We are in range of the child nodes. Either draw them or load them.
            unsigned numChildrenReady = 0;

            // If the children don't exist, create them and inherit the parent's data.
            if ( getNumChildren() == 0 )
            {
                Threading::ScopedMutexLock exclusive(_mutex);
                if ( getNumChildren() == 0 )
                {
                    createChildren( nv );
                }
            }

            // All 4 children must be ready before we can traverse any of them:
            for(unsigned i = 0; i < 4; ++i)
            {                
                if ( getSubTile(i)->isReadyToTraverse() )
                {
                    ++numChildrenReady;
                }
            }

            // If all are ready, traverse them now.
            if ( numChildrenReady == 4 )
            {
                osg::CullStack* cull = dynamic_cast<osg::CullStack*>(&nv);

                for(unsigned i=0; i<4; ++i)
                {
                    TileNode* child = getSubTile(i);
                    if (cull->isCulled( *child ) && child->isDormant( nv ) && child->getNumChildren() > 0)
                    {
                        child->expireChildren( nv );
                    }
                    else
                    {
                        child->accept( nv );
                    }
                }
            }

            // Not all children are ready, so cull the current payload.
            else if ( _payload.valid() )
            {
                _payload->accept( nv );
            }
        }

        // If children are outside camera range, cull the payload.
        else if ( _payload.valid() )
        {
            _payload->accept( nv );

            if ( getNumChildren() > 0 )
            {
                if (getSubTile(0)->isDormant( nv ) &&
                    getSubTile(1)->isDormant( nv ) &&
                    getSubTile(2)->isDormant( nv ) &&
                    getSubTile(3)->isDormant( nv ))
                {
                    expireChildren( nv );
                }
            }
        }

        // If this tile is marked dirty, try loading data.
        if ( _dirty )
        {
            load( nv );
        }
    }

    // Update, Intersection, ComputeBounds, CompileGLObjects, etc.
    else
    {
        for(unsigned i=0; i<getNumChildren(); ++i)
        {
            _children[i]->accept( nv );
        }

        if ( _payload.valid() )
        {
            _payload->accept( nv );
        }
    }
}


void
TileNode::createChildren(osg::NodeVisitor& nv)
{
    TileGroupFactory* context = static_cast<TileGroupFactory*>( nv.getUserData() );

    // Create the four child nodes.
    for(unsigned quadrant=0; quadrant<4; ++quadrant)
    {
        TileNode* node = new TileNode();

        // Build the surface geometry:
        node->create( getTileKey().createChildKey(quadrant), context );

        // Inherit the samplers with new scale/bias matrices:
        node->inheritState( this, context->getRenderBindings() );

        // Add to the scene graph.
        addChild( node );
    }
}

bool
TileNode::inheritState(TileNode* parent, const RenderBindings& bindings)
{
    bool changesMade = false;

    unsigned quadrant = getTileKey().getQuadrant();

    // Find all the sampler matrix uniforms and scale/bias them to the current quadrant.
    // This will inherit textures and use the proper sub-quadrant until new data arrives (later).
    for( RenderBindings::const_iterator binding = bindings.begin(); binding != bindings.end(); ++binding )
    {
        osg::Matrixf matrix;
        if ( parent && parent->getStateSet() )
        {
            osg::Uniform* matrixUniform = parent->getStateSet()->getUniform( binding->matrixName() );
            if ( matrixUniform )
            {
                matrixUniform->get( matrix );
                matrix.preMult( scaleBias[quadrant] );
            }
        }

        // If this node doesn't have a texture for this binding, that means it's inheriting one:
        if ( getStateSet() == 0L || getStateSet()->getTextureAttribute(binding->unit(), osg::StateAttribute::TEXTURE) == 0L )
        {
            getOrCreateStateSet()->addUniform( new osg::Uniform(binding->matrixName().c_str(), matrix) );
            changesMade = true;
        }
    }

    return changesMade;
}

void
TileNode::load(osg::NodeVisitor& nv)
{
    // Pull the factory from the traversal data.
    TileGroupFactory* context = static_cast<TileGroupFactory*>( nv.getUserData() );

    if ( !_loadRequest.valid() )
    {
        Threading::ScopedMutexLock lock(_mutex);
        if ( !_loadRequest.valid() )
        {
            _loadRequest = new LoadTileData( this, context );
        }
    }
        
    // Prioritize by LOD.
    float priority = (float)getTileKey().getLOD();
    context->getLoader()->load( _loadRequest.get(), priority, nv );
}

void
TileNode::expireChildren(osg::NodeVisitor& nv)
{
    TileGroupFactory* context = static_cast<TileGroupFactory*>( nv.getUserData() );
    if ( !_expireRequest.valid() )
    {
        Threading::ScopedMutexLock lock(_mutex);
        if ( !_expireRequest.valid() )
        {
            _expireRequest = new ExpireTiles(this, context);
        }
    }
       
    // Low priority for expiry requests.
    const float lowPriority = 0.0f;
    context->getLoader()->load( _expireRequest.get(), lowPriority, nv );
}


#if 0

void
TileNode::notifyOfArrival(TileNode* that)
{
    OE_TEST << LC << this->getKey().str()
        << " was waiting on "
        << that->getKey().str() << " and it arrived.\n";
        
    osg::Texture* thisTex = this->getModel()->getNormalTexture();
    osg::Texture* thatTex = that->getModel()->getNormalTexture();
    if ( !thisTex || !thatTex ) {
        OE_TEST << LC << "bailed on " << getKey().str() << " - null normal texture\n";
        return;
    }

    osg::RefMatrixf* thisTexMat = this->getModel()->getNormalTextureMatrix();
    osg::RefMatrixf* thatTexMat = that->getModel()->getNormalTextureMatrix();
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

    //if (_model->_normalData.isFallbackData()) {
    //    OE_TEST << LC << "bailed on " << getKey().str() << " - fallback data\n";
    //    return;
    //}

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
        for(int t=1; t<height; ++t) // start at 1 to skip the corner piece
        {
            writeThis(readThat(0,t), width-1, t);
        }
        thisDirty = true;
    }

    else if ( that->getKey() == getKey().createNeighborKey(0,1) )
    {
        // neighbor is to the south:
        for(int s=0; s<width-1; ++s) // -1 because of the corner piece
        {
            writeThis(readThat(s, height-1), s, 0);
        }
        thisDirty = true;
    }

    else if ( that->getKey() == getKey().createNeighborKey(1,1) )
    {
        // the corner
        writeThis(readThat(0,height-1), width-1, 0);
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

#endif