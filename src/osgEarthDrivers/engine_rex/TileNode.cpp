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
#include "SelectionInfo"

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
_dirty      ( false ),
_fMorphStartDistance(-1),
_fMorphEndDistance(-1)
{
    osg::StateSet* stateSet = getOrCreateStateSet();

    // The StateSet must have a dynamic data variance since we plan to alter it
    // as new data becomes available.
    stateSet->setDataVariance(osg::Object::DYNAMIC);

    _count = 0;
}

void
TileNode::create(const TileKey& key, EngineContext* context)
{
    _key = key;

    // Get a shared geometry from the pool that corresponds to this tile key:
    osg::ref_ptr<osg::Geometry> geom;
    context->getGeometryPool()->getPooledGeometry(key, context->getSelectionInfo()._uiLODForMorphing, context->getMapFrame().getMapInfo(), geom);

    TileDrawable* surfaceDrawable = new TileDrawable(key, context->getRenderBindings(), geom.get());
    surfaceDrawable->setDrawAsPatches(false);

    _surface = new SurfaceNode(
        key,
        context->getMapFrame().getMapInfo(),
        context->getRenderBindings(),
        surfaceDrawable );
    
    osg::StateSet* surfaceSS = _surface->getOrCreateStateSet();
    surfaceSS->setRenderBinDetails(0, "oe.SurfaceBin");
    surfaceSS->setNestRenderBins(false);

    // Surface node for rendering land cover geometry.
    TileDrawable* patchDrawable = new TileDrawable(key, context->getRenderBindings(), geom.get());
    patchDrawable->setDrawAsPatches(true);

    // PPP: Is this correct???
    _landCover = new SurfaceNode(
        key,
        context->getMapFrame().getMapInfo(),
        context->getRenderBindings(),
        patchDrawable );

    // add a cluster-culling callback:
    if ( context->getMapFrame().getMapInfo().isGeocentric() )
    {
        addCullCallback( ClusterCullingFactory::create(key.getExtent()) );
    }

    setMorphStartDistance(getSubDivisionRange());

    createTileSpecificUniforms();
    updateTileSpecificUniforms(context->getSelectionInfo());

    // Set up a data container for multipass layer rendering.
    _mptex = new MPTexture();
    surfaceDrawable->setMPTexture( _mptex.get() );

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
    return _surface.valid() ? _surface->computeBound() : osg::BoundingSphere();
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
    if ( _surface.valid() )
        _surface->setElevationExtrema(value);

    if ( _landCover.valid() )
        _landCover->setElevationExtrema(value);

    _extrema = value;
}

void
TileNode::createTileSpecificUniforms(void)
{
    // Install the tile key uniform.
    _tileKeyUniform = new osg::Uniform("oe_tile_key", osg::Vec4f(0,0,0,0));
    getStateSet()->addUniform( _tileKeyUniform.get() );

    _tileMorphUniform = new osg::Uniform("oe_tile_morph_constants", osg::Vec4f(0,0,0,0));
    getStateSet()->addUniform( _tileMorphUniform.get() );

    _tileGridDimsUniform = new osg::Uniform("oe_tile_grid_dimensions", osg::Vec4f(0,0,0,0));
    getStateSet()->addUniform( _tileGridDimsUniform.get() );

    _tileExtentsUniform = new osg::Uniform("oe_tile_extents", osg::Vec4f(0,0,0,0));
    getStateSet()->addUniform( _tileExtentsUniform.get() );

    _tileCenterToCameraDistUniform = new osg::Uniform("oe_tile_camera_to_tilecenter", osg::Vec4f(0,0,0,0));
    getStateSet()->addUniform( _tileCenterToCameraDistUniform.get() );
}

void
TileNode::updateTileSpecificUniforms(const SelectionInfo& selectionInfo)
{
    if(_surface.valid()==false)
    {
        return;
    }
    
    // update the tile key uniform
    const osg::BoundingBox& bbox = _surface->getAlignedBoundingBox();
    float width = std::max( (bbox.xMax()-bbox.xMin()), (bbox.yMax()-bbox.yMin()) );

    unsigned tw, th;
    _key.getProfile()->getNumTiles(_key.getLOD(), tw, th);

    _tileKeyUniform->set(osg::Vec4f(_key.getTileX(), th-_key.getTileY()-1.0f, _key.getLOD(), width));

    // update the morph constants
    if(_fMorphStartDistance>0&&_fMorphEndDistance>0)
    {
        float one_by_end_minus_start = _fMorphEndDistance - _fMorphStartDistance;
        one_by_end_minus_start = 1.0f/one_by_end_minus_start;

        osg::Vec4f vMorphConstants(
            _fMorphStartDistance
            , one_by_end_minus_start
            , _fMorphEndDistance * one_by_end_minus_start
            , one_by_end_minus_start
            );

        _tileMorphUniform->set((vMorphConstants));
    }

    // Update grid dims
    float fGridDims = selectionInfo._uiGridDimensions.first-1;
    _tileGridDimsUniform->set(osg::Vec4f(fGridDims, fGridDims*0.5f, 2.0/fGridDims, selectionInfo._uiLODForMorphing));

    // update tile extents
    float fXExtents = abs(bbox.xMax()-bbox.xMin());
    float fYExtents = abs(bbox.yMax()-bbox.yMin());
    _tileExtentsUniform->set(osg::Vec4f(fXExtents,fYExtents,0,0));
}

void
TileNode::updatePerFrameUniforms(const osg::NodeVisitor& nv)
{
    if (_tileCenterToCameraDistUniform.get())
    {
        osg::Vec4f vPerFrameUniform(getTileCenterToCameraDistance(nv),0,0,0);
        _tileCenterToCameraDistUniform->set((vPerFrameUniform));
    }
}

void
TileNode::setMorphStartDistance(float fVal)
{
    _fMorphStartDistance = fVal;
}
void
TileNode::setMorphEndDistance(float fVal)
{
    _fMorphEndDistance = fVal;
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
    if ( _surface.valid() )
        _surface->releaseGLObjects(state);

    osg::Group::releaseGLObjects(state);
}

float
TileNode::getTileCenterToCameraDistance(const osg::NodeVisitor& nv) const
{
    const osg::Vec3& vTileCenter = getBound().center();
    return nv.getDistanceToViewPoint( vTileCenter, true );
}

float
TileNode::getSubDivisionRange(void) const
{
    // TODO - placeholder for now
    const float factor = 6.0f;

    const osg::BoundingBox& box = _surface->getAlignedBoundingBox();
    return factor * 0.5*std::max( box.xMax()-box.xMin(), box.yMax()-box.yMin() );
}

bool
TileNode::shouldSubDivide(osg::NodeVisitor& nv)
{
    if ( getTileCenterToCameraDistance(nv) < getSubDivisionRange() && _key.getLOD() < 23 )
    {
        return true;
    }
    return false;
}

#define OSGEARTH_REX_TILE_NODE_DEBUG_TRAVERSAL 0

void TileNode::lodSelect(osg::NodeVisitor& nv)
{
    if ( nv.getFrameStamp() )
    {
        _lastTraversalFrame.exchange( nv.getFrameStamp()->getFrameNumber() );
    }

    unsigned currLOD = getTileKey().getLOD();

    updatePerFrameUniforms(nv);

#if OSGEARTH_REX_TILE_NODE_DEBUG_TRAVERSAL
    if (currLOD==0)
    {
        OE_INFO << LC <<"Traversing: "<<"\n";    
    }
#endif
    osgUtil::CullVisitor* cull = dynamic_cast<osgUtil::CullVisitor*>( &nv );

    EngineContext* context = static_cast<EngineContext*>( nv.getUserData() );
    const SelectionInfo& selectionInfo = context->getSelectionInfo();

    bool bShouldSubDivide = shouldSubDivide(nv);

    // If *any* of the children are visible, subdivide.
    if (bShouldSubDivide)
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
        else if ( _surface.valid() )
        {
            _surface->accept( nv );
        }
    }

    // If children are outside camera range, draw the payload and expire the children.
    else if ( _surface.valid() )
    {
        _surface->accept( nv );

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

    // Traverse land cover bins at this LOD.
    for(int i=0; i<context->landCoverBins()->size(); ++i)
    {
        const LandCoverBin& bin = context->landCoverBins()->at(i);
        if ( bin._lod == getTileKey().getLOD() )
        {
            cull->pushStateSet( bin._stateSet.get() );
            _landCover->accept( nv );
            cull->popStateSet();
        }
    }

    // If this tile is marked dirty, try loading data.
    if ( _dirty )
    {
        load( nv );
    }
}
void TileNode::regularUpdate(osg::NodeVisitor& nv)
{
    if ( getNumChildren() > 0 )
    {
        for(unsigned i=0; i<getNumChildren(); ++i)
        {
            _children[i]->accept( nv );
        }
    }

    else 
    {
        _surface->accept( nv );
        //_landCover->accept( nv );
    }
}

void
TileNode::traverse(osg::NodeVisitor& nv)
{
    // Cull
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        lodSelect(nv);
    }

    // Update, Intersection, ComputeBounds, CompileGLObjects, etc.
    else
    {
        regularUpdate(nv);
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

        node->setMorphEndDistance(getSubDivisionRange());

        // Build the surface geometry:
        node->create( getTileKey().createChildKey(quadrant), context );

        // Inherit the samplers with new scale/bias information.
        node->inheritState( this, context->getRenderBindings(), context->getSelectionInfo() );

        // Add to the scene graph.
        addChild( node );
    }
    
    OE_DEBUG << LC << "Creating children of: " << getTileKey().str() << "; count = " << (++_count) << "\n";
}

bool
TileNode::inheritState(TileNode* parent, const RenderBindings& bindings, const SelectionInfo& selectionInfo)
{
    bool changesMade = false;

    // which quadrant is this tile in?
    unsigned quadrant = getTileKey().getQuadrant();

    // Find all the sampler matrix uniforms and scale/bias them to the current quadrant.
    // This will inherit textures and use the proper sub-quadrant until new data arrives (later).
    for( RenderBindings::const_iterator binding = bindings.begin(); binding != bindings.end(); ++binding )
    {
        if ( binding->usage().isSetTo(binding->COLOR) )
        {
            if ( parent && parent->getStateSet() )
            {
                MPTexture* parentMPTex = parent->getMPTexture();
                _mptex->inheritState( parentMPTex, scaleBias[quadrant] );
                changesMade = true;
            }
        }

        else
        {
            osg::StateAttribute* sa = getStateSet()->getTextureAttribute(binding->unit(), osg::StateAttribute::TEXTURE);        
            if ( sa == 0L )
            {
                osg::Matrixf matrix;

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
    }

    if ( parent )
    {
        setElevationExtrema( parent->getElevationExtrema() );
    }

    updateTileSpecificUniforms(selectionInfo);

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
TileNode::mergeStateSet(osg::StateSet* stateSet, MPTexture* mptex, const RenderBindings& bindings)
{
    _mptex->merge( mptex );    
    getStateSet()->merge(*stateSet);
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
    float priority = - (float)getTileKey().getLOD();

    // Submit to the loader.
    //OE_INFO << LC << getTileKey().str() << "load\n";
    //if ( getTileKey().getLOD() < 6 )
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

        // if the window is smaller than one pixel, forget it.
        if ( s_span < 4.0 || t_span < 4.0 )
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