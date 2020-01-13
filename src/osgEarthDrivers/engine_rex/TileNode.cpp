/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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
#include "ProxySurfaceNode"
#include "EngineContext"
#include "Loader"
#include "LoadTileData"
#include "SelectionInfo"
#include "ElevationTextureUtils"
#include "TerrainCuller"
#include "RexTerrainEngineNode"

#include <osgEarth/CullingUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Utils>
#include <osgEarth/NodeUtils>

using namespace osgEarth::REX;
using namespace osgEarth;

#define OSGEARTH_TILE_NODE_PROXY_GEOMETRY_DEBUG 0

// Whether to check the child nodes for culling before traversing them.
// This could prevent premature Loader requests, but it increases cull time.
//#define VISIBILITY_PRECHECK

#define LC "[TileNode] "

#define REPORT(name,timer) if(context->progress()) { \
    context->progress()->stats()[name] += OE_GET_TIMER(timer); }

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
}

TileNode::TileNode() : 
_dirty        ( false ),
_childrenReady( false ),
_lastTraversalTime(0.0),
_lastTraversalFrame(0.0),
_empty(false),              // an "empty" node exists but has no geometry or children.,
_imageUpdatesActive(false)
{
    //nop
}

void
TileNode::create(const TileKey& key, TileNode* parent, EngineContext* context)
{
    if (!context)
        return;

    _context = context;

    _key = key;

    osg::ref_ptr<const Map> map = _context->getMap();
    if (!map.valid())
        return;

    unsigned tileSize = options().tileSize().get();

    // Mask generator creates geometry from masking boundaries when they exist.
    osg::ref_ptr<MaskGenerator> masks = new MaskGenerator(key, tileSize, map.get());

    // Get a shared geometry from the pool that corresponds to this tile key:
    osg::ref_ptr<SharedGeometry> geom;
    context->getGeometryPool()->getPooledGeometry(
        key,
        tileSize,
        masks.get(), 
        geom);

    // If we donget an empty, that most likely means the tile was completely
    // contained by a masking boundary. Mark as empty and we are done.
    if (geom->empty())
    {
        OE_DEBUG << LC << "Tile " << _key.str() << " is empty.\n";
        _empty = true;
        return;
    }

    // Create the drawable for the terrain surface:
    TileDrawable* surfaceDrawable = new TileDrawable(
        key, 
        geom.get(),
        options().tileSize().get() );

    // Give the tile Drawable access to the render model so it can properly
    // calculate its bounding box and sphere.
    surfaceDrawable->setModifyBBoxCallback(context->getModifyBBoxCallback());

    // Create the node to house the tile drawable:
    _surface = new SurfaceNode(key, surfaceDrawable);
    
    // create a data load request for this new tile:
    _loadRequest = new LoadTileData( this, context );
    _loadRequest->setName( _key.str() );
    _loadRequest->setTileKey( _key );

    // Encode the tile key in a uniform. Note! The X and Y components are presented
    // modulo 2^16 form so they don't overrun single-precision space.
    unsigned tw, th;
    _key.getProfile()->getNumTiles(_key.getLOD(), tw, th);

    const double m = 65536; //pow(2.0, 16.0);

    double x = (double)_key.getTileX();
    double y = (double)(th - _key.getTileY()-1);

    _tileKeyValue.set(
        (float)fmod(x, m),
        (float)fmod(y, m),
        (float)_key.getLOD(),
        -1.0f);

    // initialize all the per-tile uniforms the shaders will need:
    float range, morphStart, morphEnd;
    context->getSelectionInfo().get(_key, range, morphStart, morphEnd);

    float one_over_end_minus_start = 1.0f/(morphEnd - morphStart);
    _morphConstants.set(morphEnd * one_over_end_minus_start, one_over_end_minus_start);

    // Make a tilekey to use for testing whether to subdivide.
    if (_key.getTileY() <= th/2)
        _subdivideTestKey = _key.createChildKey(0);
    else
        _subdivideTestKey = _key.createChildKey(3);

    // Initialize the data model by copying the parent's rendering data
    // and scale/biasing the matrices.
    if (parent)
    {
        unsigned quadrant = getKey().getQuadrant();

        const RenderBindings& bindings = context->getRenderBindings();

        bool setElevation = false;

        for (unsigned p = 0; p < parent->_renderModel._passes.size(); ++p)
        {
            const RenderingPass& parentPass = parent->_renderModel._passes[p];

            // If the key is now out of the layer's valid min/max range, skip this pass.
            if (!passInLegalRange(parentPass))
                continue;

            // Copy the parent pass:
            _renderModel._passes.push_back(parentPass);
            RenderingPass& myPass = _renderModel._passes.back();

            // Scale/bias each matrix for this key quadrant.
            Samplers& samplers = myPass.samplers();
            for (unsigned s = 0; s < samplers.size(); ++s)
            {
                samplers[s]._matrix.preMult(scaleBias[quadrant]);
            }

            // Are we using image blending? If so, initialize the color_parent 
            // to the color texture.
            if (bindings[SamplerBinding::COLOR_PARENT].isActive())
            {
                samplers[SamplerBinding::COLOR_PARENT] = samplers[SamplerBinding::COLOR];
            }
        }

        // Copy the parent's shared samplers and scale+bias each matrix to the new quadrant:
        _renderModel._sharedSamplers = parent->_renderModel._sharedSamplers;

        for (unsigned s = 0; s<_renderModel._sharedSamplers.size(); ++s)
        {
            Sampler& sampler = _renderModel._sharedSamplers[s];
            sampler._matrix.preMult(scaleBias[quadrant]);
        }

        // Use the elevation sampler to initialize the elevation raster
        // (used for primitive functors, intersection, etc.)
        if (!setElevation && bindings[SamplerBinding::ELEVATION].isActive())
        {
            const Sampler& elevation = _renderModel._sharedSamplers[SamplerBinding::ELEVATION];
            if (elevation._texture.valid())
            {
                setElevationRaster(elevation._texture->getImage(0), elevation._matrix);
                setElevation = true;
            }
        }
    }

    // need to recompute the bounds after adding payload:
    dirtyBound();

    // signal the tile to start loading data:
    setDirty( true );

    // register me.
    context->liveTiles()->add( this );

    // tell the world.
    OE_DEBUG << LC << "notify (create) key " << getKey().str() << std::endl;
    context->getEngine()->getTerrain()->notifyTileAdded(getKey(), this);
}

osg::BoundingSphere
TileNode::computeBound() const
{
    osg::BoundingSphere bs;
    if (_surface.valid())
    {
        bs = _surface->getBound();
        const osg::BoundingBox& bbox = _surface->getAlignedBoundingBox();
        _tileKeyValue.a() = osg::maximum( (bbox.xMax()-bbox.xMin()), (bbox.yMax()-bbox.yMin()) );
    }    
    return bs;
}

bool
TileNode::isDormant(const osg::FrameStamp* fs) const
{
    const unsigned minMinExpiryFrames = 3u;

    bool dormant = 
           fs &&
           fs->getFrameNumber() - _lastTraversalFrame > osg::maximum(options().minExpiryFrames().get(), minMinExpiryFrames) &&
           fs->getReferenceTime() - _lastTraversalTime > options().minExpiryTime().get();
    return dormant;
}

void
TileNode::setElevationRaster(const osg::Image* image, const osg::Matrixf& matrix)
{
    if (image == 0L)
    {
        OE_WARN << LC << "TileNode::setElevationRaster: image is NULL!\n";
    }

    if (image != getElevationRaster() || matrix != getElevationMatrix())
    {
        if ( _surface.valid() )
            _surface->setElevationRaster( image, matrix );
    }
}

const osg::Image*
TileNode::getElevationRaster() const
{
    return _surface.valid() ? _surface->getElevationRaster() : 0L;
}

const osg::Matrixf&
TileNode::getElevationMatrix() const
{
    static osg::Matrixf s_identity;
    return _surface.valid() ? _surface->getElevationMatrix() : s_identity;
}

void
TileNode::setDirty(bool value)
{
    _loadRequest->clearLayerFilter();
    _dirty = value;
}

void
TileNode::refreshLayers(const std::set<UID>& layers)
{
    for(std::set<UID>::const_iterator i = layers.begin(); i != layers.end(); ++i)
        _loadRequest->addLayerToFilter(*i);

    _dirty = true;
}

void
TileNode::releaseGLObjects(osg::State* state) const
{
    osg::Group::releaseGLObjects(state);

    if ( _surface.valid() )
        _surface->releaseGLObjects(state);

    _renderModel.releaseGLObjects(state);
}

void
TileNode::resizeGLObjectBuffers(unsigned maxSize)
{
    osg::Group::resizeGLObjectBuffers(maxSize);

    if ( _surface.valid() )
        _surface->resizeGLObjectBuffers(maxSize);

    _renderModel.resizeGLObjectBuffers(maxSize);
}

bool
TileNode::shouldSubDivide(TerrainCuller* culler, const SelectionInfo& selectionInfo)
{    
    unsigned currLOD = _key.getLOD();

    EngineContext* context = culler->getEngineContext();
    
    if (currLOD < selectionInfo.getNumLODs() && currLOD != selectionInfo.getNumLODs()-1)
    {
        // In PSOS mode, subdivide when the on-screen size of a tile exceeds the maximum
        // allowable on-screen tile size in pixels.
        if (options().rangeMode() == osg::LOD::PIXEL_SIZE_ON_SCREEN)
        {
            float tileSizeInPixels = -1.0;

            if (context->getEngine()->getComputeRangeCallback())
            {
                tileSizeInPixels = (*context->getEngine()->getComputeRangeCallback())(this, *culler->_cv);
            }    

            if (tileSizeInPixels <= 0.0)
            {
                tileSizeInPixels = _surface->getPixelSizeOnScreen(culler);
            }
        
            return (tileSizeInPixels > options().tilePixelSize().get());
        }

        // In DISTANCE-TO-EYE mode, use the visibility ranges precomputed in the SelectionInfo.
        else
        {
            float range = context->getSelectionInfo().getRange(_subdivideTestKey);
#if 1
            // slightly slower than the alternate block below, but supports a user overriding
            // CullVisitor::getDistanceToViewPoint -gw
            return _surface->anyChildBoxWithinRange(range, *culler);
#else
            return _surface->anyChildBoxIntersectsSphere(
                culler->getViewPointLocal(), 
                range*range / culler->getLODScale());
#endif
        }
    }                 
    return false;
}

bool
TileNode::cull_spy(TerrainCuller* culler)
{
    bool visible = false;

    EngineContext* context = culler->getEngineContext();

    // Shows all culled tiles. All this does is traverse the terrain
    // and add any tile that's been "legitimately" culled (i.e. culled
    // by a non-spy traversal) in the last 2 frames. We use this
    // trick to spy on another camera.
    unsigned frame = culler->getFrameStamp()->getFrameNumber();

    if ( frame - _surface->getLastFramePassedCull() < 2u)
    {
        _surface->accept( *culler );
    }

    else if ( _childrenReady )
    {
        for(int i=0; i<4; ++i)
        {
            TileNode* child = getSubTile(i);
            if (child)
                child->accept( *culler );
        }
    }

    return visible;
}

bool
TileNode::cull(TerrainCuller* culler)
{
    EngineContext* context = culler->getEngineContext();

    // Horizon check the surface first:
    if (!_surface->isVisibleFrom(culler->getViewPointLocal()))
    {
        return false;
    }
    
    // determine whether we can and should subdivide to a higher resolution:
    bool childrenInRange = shouldSubDivide(culler, context->getSelectionInfo());

    // whether it is OK to create child TileNodes is necessary.
    bool canCreateChildren = childrenInRange;

    // whether it is OK to load data if necessary.
    bool canLoadData = true;

    // whether to accept the current surface node and not the children.
    bool canAcceptSurface = false;
    
    // Don't load data in progressive mode until the parent is up to date
    if (options().progressive() == true)
    {
        TileNode* parent = getParentTile();
        if ( parent && parent->isDirty() )
        {
            canLoadData = false;
        }
    }
    
    // If this is an inherit-viewpoint camera, we don't need it to invoke subdivision
    // because we want only the tiles loaded by the true viewpoint.
    const osg::Camera* cam = culler->getCamera();
    if ( cam && cam->getReferenceFrame() == osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT )
    {
        canCreateChildren = false;
        canLoadData       = false;
    }

    if (childrenInRange)
    {
        // We are in range of the child nodes. Either draw them or load them.

        // If the children don't exist, create them and inherit the parent's data.
        if ( !_childrenReady && canCreateChildren )
        {
            _mutex.lock();

            if ( !_childrenReady ) // double check inside mutex
            {
                OE_START_TIMER(createChildren);
                createChildren( context );
                REPORT("TileNode::createChildren", createChildren);
                _childrenReady = true;

                // This means that you cannot start loading data immediately; must wait a frame.
                canLoadData = false;
            }

            _mutex.unlock();
        }

        // If all are ready, traverse them now.
        if ( _childrenReady )
        {
            for(int i=0; i<4; ++i)
            {
                TileNode* child = getSubTile(i);
                if (child)
                    child->accept(*culler);
            }
        }

        // If we don't traverse the children, traverse this node's payload.
        else
        {
            canAcceptSurface = true;
        }
    }

    // If children are outside camera range, draw the payload and expire the children.
    else
    {
        canAcceptSurface = true;
    }

    // accept this surface if necessary.
    if ( canAcceptSurface )
    {
        _surface->accept( *culler );
    }

    // If this tile is marked dirty, try loading data.
    if ( _dirty && canLoadData )
    {
        load( culler );
    }

    return true;
}

bool
TileNode::accept_cull(TerrainCuller* culler)
{
    bool visible = false;
    
    if (culler)
    {
        // update the timestamp so this tile doesn't become dormant.
        _lastTraversalFrame.exchange( culler->getFrameStamp()->getFrameNumber() );
        _lastTraversalTime = culler->getFrameStamp()->getReferenceTime();

        if ( !culler->isCulled(*this) )
        {
            visible = cull( culler );
        }
    }

    return visible;
}

bool
TileNode::accept_cull_spy(TerrainCuller* culler)
{
    bool visible = false;
    
    if (culler)
    {
        visible = cull_spy( culler );
    }

    return visible;
}

void
TileNode::traverse(osg::NodeVisitor& nv)
{
    // Cull only:
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        if (_empty == false)
        {
            TerrainCuller* culler = dynamic_cast<TerrainCuller*>(&nv);
        
            if (culler->_isSpy)
            {
                accept_cull_spy( culler );
            }
            else
            {
                accept_cull( culler );
            }
        }
    }

    // Everything else: update, GL compile, intersection, compute bound, etc.
    else
    {
        // Check for image updates.
        if (nv.getVisitorType() == nv.UPDATE_VISITOR && _imageUpdatesActive)
        {
            unsigned numUpdated = 0u;

            for (unsigned p = 0; p < _renderModel._passes.size(); ++p)
            {
                RenderingPass& pass = _renderModel._passes[p];
                Samplers& samplers = pass.samplers();
                for (unsigned s = 0; s < samplers.size(); ++s)
                {
                    Sampler& sampler = samplers[s];
                    if (sampler._texture.valid() && sampler._matrix.isIdentity())
                    {
                        for(unsigned i = 0; i < sampler._texture->getNumImages(); ++i)
                        {
                            osg::Image* image = sampler._texture->getImage(i);
                            if (image && image->requiresUpdateCall())
                            {
                                image->update(&nv);
                                numUpdated++;
                            }
                        }
                    }
                }
            }

            // if no updates were detected, don't check next time.
            if (numUpdated == 0)
            {
                ADJUST_UPDATE_TRAV_COUNT(this, -1);
                _imageUpdatesActive = false;
            }
        }

        // If there are child nodes, traverse them:
        int numChildren = getNumChildren();
        if ( numChildren > 0 )
        {
            for(int i=0; i<numChildren; ++i)
            {
                _children[i]->accept( nv );
            }
        }

        // Otherwise traverse the surface.
        else if (_surface.valid())
        {
            _surface->accept( nv );
        }
    }
}

void
TileNode::createChildren(EngineContext* context)
{
    // NOTE: Ensure that _mutex is locked before calling this function!
    //OE_WARN << "Creating children for " << _key.str() << std::endl;

    // Create the four child nodes.
    for(unsigned quadrant=0; quadrant<4; ++quadrant)
    {
        TileNode* node = new TileNode();

        // Build the surface geometry:
        node->create( getKey().createChildKey(quadrant), this, context );

        // Add to the scene graph.
        addChild( node );
    }
}

void
TileNode::merge(const TerrainTileModel* model, const RenderBindings& bindings)
{
    bool newElevationData = false;

    const SamplerBinding& color = bindings[SamplerBinding::COLOR];
    if (color.isActive())
    {
        for(TerrainTileColorLayerModelVector::const_iterator i = model->colorLayers().begin();
            i != model->colorLayers().end();
            ++i)
        {
            TerrainTileImageLayerModel* model = dynamic_cast<TerrainTileImageLayerModel*>(i->get());
            if (model)
            {
                if (model->getTexture())
                {
                    RenderingPass* pass = _renderModel.getPass(model->getImageLayer()->getUID());
                    if (!pass)
                    {
                        pass = &_renderModel.addPass();
                        pass->setLayer(model->getLayer());

                        // This is a new pass that just showed up at this LOD
                        // Since it just arrived at this LOD, make the parent the same as the color.
                        if (bindings[SamplerBinding::COLOR_PARENT].isActive())
                        {
                            pass->samplers()[SamplerBinding::COLOR_PARENT]._texture = model->getTexture();
                            pass->samplers()[SamplerBinding::COLOR_PARENT]._matrix = *model->getMatrix();
                        }
                    }
                    pass->samplers()[SamplerBinding::COLOR]._texture = model->getTexture();
                    pass->samplers()[SamplerBinding::COLOR]._matrix = *model->getMatrix();

                    // check to see if this data requires an image update traversal.
                    if (_imageUpdatesActive == false)
                    {
                        osg::Texture* texture = model->getTexture();
                        for(unsigned i=0; i<texture->getNumImages(); ++i)
                        {
                            const osg::Image* image = texture->getImage(i);
                            if (image && image->requiresUpdateCall())
                            {
                                ADJUST_UPDATE_TRAV_COUNT(this, +1);
                                _imageUpdatesActive = true;
                                break;
                            }
                        }
                    }
                }
            }

            else // non-image color layer (like splatting, e.g.)
            {
                TerrainTileColorLayerModel* model = i->get();
                if (model && model->getLayer())
                {
                    RenderingPass* pass = _renderModel.getPass(model->getLayer()->getUID());
                    if (!pass)
                    {
                        pass = &_renderModel.addPass();
                        pass->setLayer(model->getLayer());
                    }
                }
            }
        }
    }

    // Elevation:
    const SamplerBinding& elevation = bindings[SamplerBinding::ELEVATION];
    if (elevation.isActive() && model->elevationModel().valid() && model->elevationModel()->getTexture())
    {
        osg::Texture* tex = model->elevationModel()->getTexture();

        _renderModel._sharedSamplers[SamplerBinding::ELEVATION]._texture = tex;
        _renderModel._sharedSamplers[SamplerBinding::ELEVATION]._matrix.makeIdentity();

        setElevationRaster(tex->getImage(0), osg::Matrixf::identity());

        newElevationData = true;
    } 

    // Normals:
    const SamplerBinding& normals = bindings[SamplerBinding::NORMAL];
    if (normals.isActive() && model->normalModel().valid() && model->normalModel()->getTexture())
    {
        osg::Texture* tex = model->normalModel()->getTexture();

        if (_context->options().normalizeEdges() == true)
        {
            // keep the normal map around because we might update it later in "ping"
            tex->setUnRefImageDataAfterApply(false);
        }

        _renderModel._sharedSamplers[SamplerBinding::NORMAL]._texture = tex;
        _renderModel._sharedSamplers[SamplerBinding::NORMAL]._matrix.makeIdentity();

        updateNormalMap();
    }

    // Land Cover:
    const SamplerBinding& landCover = bindings[SamplerBinding::LANDCOVER];
    if (landCover.isActive() && model->landCoverModel().valid() && model->landCoverModel()->getTexture())
    {
        osg::Texture* tex = model->landCoverModel()->getTexture();

        _renderModel._sharedSamplers[SamplerBinding::LANDCOVER]._texture = tex;
        _renderModel._sharedSamplers[SamplerBinding::LANDCOVER]._matrix.makeIdentity();
    }

    // Other Shared Layers:
    for (unsigned i = 0; i < model->sharedLayers().size(); ++i)
    {
        TerrainTileImageLayerModel* layerModel = model->sharedLayers()[i].get();
        if (layerModel->getTexture())
        {
            // locate the shared binding corresponding to this layer:
            UID uid = layerModel->getImageLayer()->getUID();
            unsigned bindingIndex = INT_MAX;
            for(unsigned i=SamplerBinding::SHARED; i<bindings.size() && bindingIndex==INT_MAX; ++i) {
                if (bindings[i].isActive() && bindings[i].sourceUID().isSetTo(uid)) {
                    bindingIndex = i;
                }                   
            }

            if (bindingIndex < INT_MAX)
            {
                osg::Texture* tex = layerModel->getTexture();
                _renderModel._sharedSamplers[bindingIndex]._texture = tex;
                _renderModel._sharedSamplers[bindingIndex]._matrix.makeIdentity();
            }
        }
    }

    // Patch Layers
    for (unsigned i = 0; i < model->patchLayers().size(); ++i)
    {
        TerrainTilePatchLayerModel* layerModel = model->patchLayers()[i].get();
    }

    if (_childrenReady)
    {
        for (int i = 0; i < 4; ++i)
        {
            TileNode* child = getSubTile(i);
            if (child)
                child->refreshInheritedData(this, bindings);
        }
    }

    if (newElevationData && _context->options().normalizeEdges() == true)
    {
        _context->getEngine()->getTerrain()->notifyTileAdded(getKey(), this);
    }
}

void TileNode::loadChildren()
{
    _mutex.lock();

    if ( !_childrenReady )
    {        
        // Create the children
        createChildren( _context.get() );        
        _childrenReady = true;        
        int numChildren = getNumChildren();
        if ( numChildren > 0 )
        {
            for(int i=0; i<numChildren; ++i)
            {
                TileNode* child = getSubTile(i);
                if (child)
                {
                    // Load the children's data.
                    child->loadSync();
                }
            }
        }
    }
    _mutex.unlock();    
}

void
TileNode::refreshSharedSamplers(const RenderBindings& bindings)
{    
    for (unsigned i = 0; i < _renderModel._sharedSamplers.size(); ++i)
    {
        if (bindings[i].isActive() == false)
        {
            _renderModel._sharedSamplers[i]._texture = 0L;
        }
    }
}

void
TileNode::refreshInheritedData(TileNode* parent, const RenderBindings& bindings)
{
    // Run through this tile's rendering data and re-inherit textures and matrixes
    // from the parent. When a TileNode gets new data (via a call to merge), any
    // children of that tile that are inheriting textures or matrixes need to 
    // refresh to inherit that new data. In turn, those tile's children then need
    // to update as well. This method does that.

    // which quadrant is this tile in?
    unsigned quadrant = getKey().getQuadrant();

    // Count the number of inherited samplers so we know when to stop. If none of the
    // samplers in this tile inherit from the parent, there is no need to continue
    // down the Tile tree.
    unsigned changes = 0;

    RenderingPasses& parentPasses = parent->_renderModel._passes;

    for (unsigned p = 0; p<parentPasses.size(); ++p)
    {
        const RenderingPass& parentPass = parentPasses[p];

        RenderingPass* myPass = _renderModel.getPass(parentPass.sourceUID());

        // Inherit the samplers for this pass.
        if (myPass)
        {
            Samplers& samplers = myPass->samplers();
            for (unsigned s = 0; s < samplers.size(); ++s)
            {
                Sampler& mySampler = samplers[s];
                
                // the color-parent gets special treatment, since it is not included
                // in the TileModel (rather it is always derived here).
                if (s == SamplerBinding::COLOR_PARENT && bindings[SamplerBinding::COLOR_PARENT].isActive())
                {
                    const Samplers& parentSamplers = parentPass.samplers();
                    const Sampler& parentSampler = parentSamplers[SamplerBinding::COLOR];
                    osg::Matrixf newMatrix = parentSampler._matrix;
                    newMatrix.preMult(scaleBias[quadrant]);

                    // Did something change?
                    if (mySampler._texture.get() != parentSampler._texture.get() ||
                        mySampler._matrix != newMatrix)
                    {
                        if (parentSampler._texture.valid() && passInLegalRange(parentPass))
                        {
                            // set the parent-color texture to the parent's color texture
                            // and scale/bias the matrix.
                            mySampler._texture = parentSampler._texture.get();
                            mySampler._matrix = newMatrix;
                        }
                        else
                        {
                            // parent has no color texture? Then set our parent-color
                            // equal to our normal color texture.
                            mySampler._texture = samplers[SamplerBinding::COLOR]._texture.get();
                            mySampler._matrix = samplers[SamplerBinding::COLOR]._matrix;
                        }
                        ++changes;
                    }
                }

                // all other samplers just need to inherit from their parent 
                // and scale/bias their texture matrix.
                else if (!mySampler._texture.valid() || !mySampler._matrix.isIdentity())
                {
                    mySampler = parentPass.samplers()[s];
                    mySampler._matrix.preMult(scaleBias[quadrant]);
                    ++changes;
                }
            }
        }
        else
        {
            // Pass exists in the parent node, but not in this node, so add it now.
            if (passInLegalRange(parentPass))
            {
                myPass = &_renderModel.addPass();
                *myPass = parentPass;

                for (unsigned s = 0; s < myPass->samplers().size(); ++s)
                {
                    Sampler& sampler = myPass->samplers()[s];
                    sampler._matrix.preMult(scaleBias[quadrant]);
                }
                ++changes;
            }
        }
    }

    // Handle all the shared samplers (elevation, normal, etc.)
    const Samplers& parentSharedSamplers = parent->_renderModel._sharedSamplers;
    Samplers& mySharedSamplers = _renderModel._sharedSamplers;
    for (unsigned s = 0; s<mySharedSamplers.size(); ++s)
    {        
        Sampler& mySampler = mySharedSamplers[s];
        if (!mySampler._texture.valid() || !mySampler._matrix.isIdentity())
        {
            mySampler = parentSharedSamplers[s];
            mySampler._matrix.preMult(scaleBias[quadrant]);
            ++changes;

            // Update the local elevation raster cache (for culling and intersection testing).
            if (s == SamplerBinding::ELEVATION && mySampler._texture.valid())
            {
                this->setElevationRaster(mySampler._texture->getImage(0), mySampler._matrix);
            }
        }
    }

    // Handle any shared samplers that exist in the parent but not here:
    for (unsigned s = mySharedSamplers.size(); s < parentSharedSamplers.size(); ++s)
    {
        mySharedSamplers[s] = parentSharedSamplers[s];
        mySharedSamplers[s]._matrix.preMult(scaleBias[quadrant]);
        ++changes;
    }

    if (changes > 0)
    {
        dirtyBound(); // only for elev/patch changes maybe?

        if (_childrenReady)
        {
            for (int i = 0; i < 4; ++i)
            {
                TileNode* child = getSubTile(i);
                if (child)
                    child->refreshInheritedData(this, bindings);
            }
        }
    }
    else
    {
        //OE_INFO << LC << _key.str() << ": refreshInheritedData, stopped short.\n";
    }
}

bool
TileNode::passInLegalRange(const RenderingPass& pass) const
{
    return 
        pass.terrainLayer() == 0L ||
        pass.terrainLayer()->isKeyInVisualRange(getKey());
}

void
TileNode::load(TerrainCuller* culler)
{    
    const SelectionInfo& si = _context->getSelectionInfo();
    int lod     = getKey().getLOD();
    int numLods = si.getNumLODs();
    
    // LOD priority is in the range [0..numLods]
    float lodPriority = (float)lod;

    // If progressive mode is enabled, lower LODs get higher priority since
    // we want to load them in order
    if (options().progressive() == true)
    {
        lodPriority = (float)(numLods - lod);
    }

    // dist priority is in the range [0..1]
    float distance = culler->getDistanceToViewPoint(getBound().center(), true);
    float maxRange = si.getLOD(0)._visibilityRange;
    float distPriority = 1.0 - distance/maxRange;

    // add them together, and you get tiles sorted first by lodPriority
    // (because of the biggest range), and second by distance.
    float priority = lodPriority + distPriority;

    // Submit to the loader.
    _context->getLoader()->load( _loadRequest.get(), priority, *culler );
}

void
TileNode::loadSync()
{
    osg::ref_ptr<LoadTileData> loadTileData = new LoadTileData(this, _context.get());
    loadTileData->setEnableCancelation(false);
    loadTileData->invoke(0L);
    loadTileData->apply(0L);
}

bool
TileNode::areSubTilesDormant(const osg::FrameStamp* fs) const
{
    return
        getNumChildren() >= 4           &&
        getSubTile(0)->isDormant( fs )  &&
        getSubTile(1)->isDormant( fs )  &&
        getSubTile(2)->isDormant( fs )  &&
        getSubTile(3)->isDormant( fs );
}

void
TileNode::removeSubTiles()
{
    _childrenReady = false;
    this->removeChildren(0, this->getNumChildren());
}


void
TileNode::notifyOfArrival(TileNode* that)
{
    if (options().normalizeEdges() == true)
    {
        if (_key.createNeighborKey(1, 0) == that->getKey())
            _eastNeighbor = that;

        if (_key.createNeighborKey(0, 1) == that->getKey())
            _southNeighbor = that;

        updateNormalMap();
    }
}

void
TileNode::updateNormalMap()
{
    if (options().normalizeEdges() == false)
        return;

    Sampler& thisNormalMap = _renderModel._sharedSamplers[SamplerBinding::NORMAL];
    if (!thisNormalMap._texture.valid() || !thisNormalMap._matrix.isIdentity() || !thisNormalMap._texture->getImage(0))
        return;

    if (!_eastNeighbor.valid() || !_southNeighbor.valid())
        return;

    osg::ref_ptr<TileNode> east;
    if (_eastNeighbor.lock(east))
    {
        const Sampler& thatNormalMap = east->_renderModel._sharedSamplers[SamplerBinding::NORMAL];
        if (!thatNormalMap._texture.valid() || !thatNormalMap._matrix.isIdentity() || !thatNormalMap._texture->getImage(0))
            return;

        osg::Image* thisImage = thisNormalMap._texture->getImage(0);
        osg::Image* thatImage = thatNormalMap._texture->getImage(0);

        int width = thisImage->s();
        int height = thisImage->t();
        if ( width != thatImage->s() || height != thatImage->t() )
            return;

        // Just copy the neighbor's edge normals over to our texture.
        // Averaging them would be more accurate, but then we'd have to
        // re-generate each texture multiple times instead of just once.
        // Besides, there's almost no visual difference anyway.
        ImageUtils::PixelReader readThat(thatImage);
        ImageUtils::PixelWriter writeThis(thisImage);
        
        for (int t=0; t<height; ++t)
        {
            writeThis(readThat(0, t), width-1, t);
        }

        thisImage->dirty();
    }

    osg::ref_ptr<TileNode> south;
    if (_southNeighbor.lock(south))
    {
        const Sampler& thatNormalMap = south->_renderModel._sharedSamplers[SamplerBinding::NORMAL];
        if (!thatNormalMap._texture.valid() || !thatNormalMap._matrix.isIdentity() || !thatNormalMap._texture->getImage(0))
            return;

        osg::Image* thisImage = thisNormalMap._texture->getImage(0);
        osg::Image* thatImage = thatNormalMap._texture->getImage(0);

        int width = thisImage->s();
        int height = thisImage->t();
        if ( width != thatImage->s() || height != thatImage->t() )
            return;

        // Just copy the neighbor's edge normals over to our texture.
        // Averaging them would be more accurate, but then we'd have to
        // re-generate each texture multiple times instead of just once.
        // Besides, there's almost no visual difference anyway.
        ImageUtils::PixelReader readThat(thatImage);
        ImageUtils::PixelWriter writeThis(thisImage);

        for (int s=0; s<width; ++s)
            writeThis(readThat(s, height-1), s, 0);

        thisImage->dirty();
    }

    //OE_INFO << LC << _key.str() << " : updated normal map.\n";
}

const TerrainOptions&
TileNode::options() const
{
    return _context->options();
}