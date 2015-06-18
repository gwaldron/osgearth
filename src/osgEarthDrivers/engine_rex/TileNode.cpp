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
#include "EngineContext"
#include "Loader"
#include "LoadTileData"
#include "ExpireTiles"

#include <osgEarth/CullingUtils>
#include <osgEarth/ImageUtils>

#include <osg/Uniform>
#include <osg/ComputeBoundsVisitor>

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

    void applyScaleBias(osg::Matrix& m, unsigned quadrant)
    {
        m.preMult( scaleBias[quadrant] );
    }
}



TileNode::TileNode() :
_dirty      ( false )
{
    // The StateSet must have a dynamic data variance since we plan to alter it
    // as new data becomes available.^
    getOrCreateStateSet()->setDataVariance(osg::Object::DYNAMIC);

    _count = 0;
}

void
TileNode::create(const TileKey& key, EngineContext* context)
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
    // TODO - placeholder for now
    const float factor = 6.0f;

    //return getBound().radius() * factor;
    const osg::BoundingBox& box = _payload->getAlignedBoundingBox();
    return factor * 0.5*std::max( box.xMax()-box.xMin(), box.yMax()-box.yMin() );
}

bool
TileNode::isDormant(osg::NodeVisitor& nv) const
{
    return
        nv.getFrameStamp() &&
        nv.getFrameStamp()->getFrameNumber() - _lastTraversalFrame > 2u;
}

void
TileNode::setElevationExtrema(const osg::Vec2f& value)
{
    if ( _payload.valid() )
    {
        _payload->setElevationExtrema(value);
    }
    _extrema = value;
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

        osg::CullStack* cull = dynamic_cast<osg::CullStack*>(&nv);

        // If the children are visible, subdivide.
        if (cameraRange < getSubdivideRange(nv) && getTileKey().getLOD() < 22)
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
                // TODO:
                // If we do thing, we need to quite sure that all 4 children will be accepted into
                // the draw set. Perhaps isReadyToTraverse() needs to check that.
                _children[0]->accept( nv );
                _children[1]->accept( nv );
                _children[2]->accept( nv );
                _children[3]->accept( nv );
            }

            // Not all children are ready, so cull the current payload.
            else if ( _payload.valid() )
            {
                _payload->accept( nv );
            }
        }

        // If children are outside camera range, draw the payload and expire the children.
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
        if ( getNumChildren() > 0 )
        {
            for(unsigned i=0; i<getNumChildren(); ++i)
            {
                _children[i]->accept( nv );
            }
        }
        else if ( _payload.valid() )
        {
            _payload->accept( nv );
        }
    }
}


void
TileNode::createChildren(osg::NodeVisitor& nv)
{
    EngineContext* context = static_cast<EngineContext*>( nv.getUserData() );

    // Create the four child nodes.
    for(unsigned quadrant=0; quadrant<4; ++quadrant)
    {
        TileNode* node = new TileNode();

        // Build the surface geometry:
        node->create( getTileKey().createChildKey(quadrant), context );

        // Inherit the samplers with new scale/bias information.
        node->inheritState( this, context->getRenderBindings() );

        // Add to the scene graph.
        addChild( node );
    }
    
    OE_DEBUG << LC << "Creating children of: " << getTileKey().str() << "; count = " << (++_count) << "\n";
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
        osg::StateAttribute* sa = getStateSet()->getTextureAttribute(binding->unit(), osg::StateAttribute::TEXTURE);

        // If this node doesn't have a texture for this binding, that means it's inheriting one:
        osg::Matrixf matrix;
        if ( sa == 0L )
        {
            // Find the parent's matrix and scale/bias it to this quadrant:
            if ( parent && parent->getStateSet() )
            {
                osg::Uniform* matrixUniform = parent->getStateSet()->getUniform( binding->matrixName() );
                if ( matrixUniform )
                {
                    matrixUniform->get( matrix );
                    matrix.preMult( scaleBias[quadrant] );
                }
            }

            // Add a new uniform with the scale/bias'd matrix:
            getOrCreateStateSet()->addUniform( new osg::Uniform(binding->matrixName().c_str(), matrix) );
            changesMade = true;
        }
    }

    if ( parent )
    {
        setElevationExtrema( parent->getElevationExtrema() );
    }

#if 0
        if ( binding->usage() == binding->ELEVATION )
        {
            // Find the elevation texture and calculate the min/max elevations.
            // TODO: this is not completely thread-safe, querying the parent node path during the CULL traversal. Do this later.
            if (recalculateExterma)
            {
                bool found = false;
                TileNode* nextParent = 0L;
                for(TileNode* node = this; node != 0L && !found; node = nextParent)
                {
                    osg::Texture* elevTex = dynamic_cast<osg::Texture*>(node->getStateSet()->getTextureAttribute(binding->unit(), osg::StateAttribute::TEXTURE));
                    if ( elevTex )
                    {
                        osg::Vec2f extrema;
                        found = findExtrema(elevTex, matrix, extrema);
                        if ( found )
                        {
                            setElevationExtrema( extrema );
                            OE_DEBUG << LC << getTileKey().str() << " : found min=" << extrema[0] << ", max=" << extrema[1] << "\n";
                            break;
                        }
                        else
                        {
                            setElevationExtrema( parent->getElevationExtrema() );
                        }
                    }
                    nextParent = nextParent ? dynamic_cast<TileNode*>(node->getParent(0)) : parent;
                }

                if ( !found )
                {
                    // This will happen sometimes, like when the initial levels are loading and there
                    // is no elevation texture in the graph yet. It's normal.
                    OE_DEBUG << LC << "Failed to locate the elevation texture for extrema calculation, " << getTileKey().str() << "\n";
                }
            }
            else
            {
                setElevationExtrema( parent->getElevationExtrema() );
            }
        }
#endif

    return changesMade;
}

void
TileNode::recalculateExtrema(const RenderBindings& bindings)
{
    const SamplerBinding* elevBinding = SamplerBinding::findUsage(bindings, SamplerBinding::ELEVATION);
    if ( !elevBinding )
        return;

    osg::Matrixf matrix;
    const osg::Uniform* u = getStateSet()->getUniform(elevBinding->matrixName());
    if ( !u )
    {
        OE_WARN << LC << getTileKey().str() << " : recalculateExtrema: illegal state\n";
        return;
    }

    u->get(matrix);

    bool found = false;
    TileNode* parent = 0L;
    for(TileNode* node = this; node != 0L && !found; node = parent)
    {
        osg::Texture* elevTex = dynamic_cast<osg::Texture*>(node->getStateSet()->getTextureAttribute(elevBinding->unit(), osg::StateAttribute::TEXTURE));
        if ( elevTex )
        {
            osg::Vec2f extrema;
            found = findExtrema(elevTex, matrix, extrema);
            if ( found )
            {
                setElevationExtrema( extrema );
                OE_DEBUG << LC << getTileKey().str() << " : found min=" << extrema[0] << ", max=" << extrema[1] << "\n";
                break;
            }
        }
        parent = node->getNumParents() > 0 ? dynamic_cast<TileNode*>(node->getParent(0)) : 0L;
    }

    if ( !found )
    {
        // This will happen sometimes, like when the initial levels are loading and there
        // is no elevation texture in the graph yet. It's normal.
        OE_DEBUG << LC << "Failed to locate the elevation texture for extrema calculation, " << getTileKey().str() << "\n";
    }
}

void
TileNode::load(osg::NodeVisitor& nv)
{
    // Access the context:
    EngineContext* context = static_cast<EngineContext*>( nv.getUserData() );

    // Create a new load request on demand:
    if ( !_loadRequest.valid() )
    {
        Threading::ScopedMutexLock lock(_mutex);
        if ( !_loadRequest.valid() )
        {
            _loadRequest = new LoadTileData( this, context );
        }
    }
        
    // Prioritize by LOD.
    float priority = -(float)getTileKey().getLOD();

    // Submit to the loader.
    //OE_INFO << LC << getTileKey().str() << "load\n";
    context->getLoader()->load( _loadRequest.get(), priority, nv );
}

void
TileNode::expireChildren(osg::NodeVisitor& nv)
{
    OE_DEBUG << "Expiring children of " << getTileKey().str() << "\n";

    EngineContext* context = static_cast<EngineContext*>( nv.getUserData() );
    if ( !_expireRequest.valid() )
    {
        Threading::ScopedMutexLock lock(_mutex);
        if ( !_expireRequest.valid() )
        {
            _expireRequest = new ExpireTiles(this, context);
        }
    }
       
    // Low priority for expiry requests.
    //OE_INFO << LC << getTileKey().str() << "expire\n";
    const float lowPriority = -100.0f;
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

bool
TileNode::findExtrema(osg::Texture* tex, const osg::Matrix& m, osg::Vec2f& extrema) const
{
    // Searches a texture image (using a texture matrix) for the min and max elevation values.
    extrema.set( FLT_MAX, -FLT_MAX );

    osg::Image* image = tex->getImage(0);
    if ( image )
    {
        double s_offset = m(3,0) * (double)image->s();
        double t_offset = m(3,1) * (double)image->t();
        double s_span   = m(0,0) * (double)image->s();
        double t_span   = m(1,1) * (double)image->t();

        if ( s_span < 1.0 || t_span < 1.0 )
            return false;

        ImageUtils::PixelReader read(image);

        // starting column and row:
        int c0 = osg::clampAbove( ((int)s_offset)-1, 0 );
        int r0 = osg::clampAbove( ((int)t_offset)-1, 0 );

        int c1 = osg::clampBelow( c0 + ((int)s_span) + 1, image->s()-1 );
        int r1 = osg::clampBelow( r0 + ((int)t_span) + 1, image->t()-1 );

        OE_DEBUG << LC << "find: key=" << getTileKey().str() << std::dec << 
            "scale=" << s_offset << ", " << t_offset
            << "; span = " << s_span << ", " << t_span
            << "; c0=" << c0 << ", r0=" << r0 << "; c1=" << c1 << ", r1=" << r1 << "\n";
        
        for(int c=c0; c <= c1; ++c)
        {
            for(int r=r0; r <= r1; ++r)
            {
                float e = read(c, r).r();
                if ( e < extrema[0] ) extrema[0] = e;
                if ( e > extrema[1] ) extrema[1] = e;
            }
        }

        if ( extrema[0] > extrema[1] )
        {
            OE_WARN << LC << "findExtrema ERROR (" << getTileKey().str() << ") c0=" << c0 << ", r0=" << r0 << "; c1=" << c1 << ", r1=" << r1 << ", s=" << image->s() << ", t=" << image->t() << "\n";
        }
    }
    else
    {
        OE_WARN << LC << "findExtrema ERROR (" << getTileKey().str() << ") no tex image available\n";
    }

    return extrema[0] <= extrema[1];
}