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
#include <osgEarth/Metrics>

using namespace osgEarth::REX;
using namespace osgEarth;
using namespace osgEarth::Util;

#define OSGEARTH_TILE_NODE_PROXY_GEOMETRY_DEBUG 0

// Whether to check the child nodes for culling before traversing them.
// This could prevent premature Loader requests, but it increases cull time.
//#define VISIBILITY_PRECHECK

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
}

TileNode::TileNode() : 
_loadsInQueue(0u),
_childrenReady( false ),
_lastTraversalTime(0.0),
_lastTraversalFrame(0),
_empty(false),              // an "empty" node exists but has no geometry or children.,
_imageUpdatesActive(false),
_doNotExpire(false),
_revision(0),
_mutex("TileNode(OE)"),
_loadQueue("TileNode LoadQueue(OE)"),
_createChildAsync(true)
{
    //nop
}

TileNode::~TileNode()
{
    //OE_INFO << LC << "Tile " << _key.str() << " destroyed" << std::endl;
}

void
TileNode::setDoNotExpire(bool value)
{
    _doNotExpire = value;
}

void
TileNode::create(const TileKey& key, TileNode* parent, EngineContext* context, Cancelable* progress)
{
    if (!context)
        return;

    _key = key;
    _context = context;

    createGeometry(progress);

    // Encode the tile key in a uniform. Note! The X and Y components are presented
    // modulo 2^16 form so they don't overrun single-precision space.
    unsigned tw, th;
    _key.getProfile()->getNumTiles(_key.getLOD(), tw, th);

    const double m = 65536; //pow(2.0, 16.0);

    double x = (double)_key.getTileX();
    double y = (double)(th - _key.getTileY() - 1);

    _tileKeyValue.set(
        (float)fmod(x, m),
        (float)fmod(y, m),
        (float)_key.getLOD(),
        -1.0f);

    // initialize all the per-tile uniforms the shaders will need:
    float range, morphStart, morphEnd;
    context->getSelectionInfo().get(_key, range, morphStart, morphEnd);

    float one_over_end_minus_start = 1.0f / (morphEnd - morphStart);
    _morphConstants.set(morphEnd * one_over_end_minus_start, one_over_end_minus_start);

    // Make a tilekey to use for testing whether to subdivide.
    if (_key.getTileY() <= th / 2)
        _subdivideTestKey = _key.createChildKey(0);
    else
        _subdivideTestKey = _key.createChildKey(3);
}

void
TileNode::createGeometry(Cancelable* progress)
{
    osg::ref_ptr<const Map> map = _context->getMap();
    if (!map.valid())
        return;

    _empty = false;

    unsigned tileSize = options().tileSize().get();

    // Get a shared geometry from the pool that corresponds to this tile key:
    osg::ref_ptr<SharedGeometry> geom;

    _context->getGeometryPool()->getPooledGeometry(
        _key,
        tileSize,
        map.get(),
        geom,
        progress);

    if (progress && progress->isCanceled())
        return;

    if (geom.valid())
    {
        // Create the drawable for the terrain surface:
        TileDrawable* surfaceDrawable = new TileDrawable(
            _key,
            geom.get(),
            tileSize);

        // Give the tile Drawable access to the render model so it can properly
        // calculate its bounding box and sphere.
        surfaceDrawable->setModifyBBoxCallback(_context->getModifyBBoxCallback());

        osg::ref_ptr<const osg::Image> elevationRaster = getElevationRaster();
        osg::Matrixf elevationMatrix = getElevationMatrix();

        // Create the node to house the tile drawable:
        _surface = new SurfaceNode(_key, surfaceDrawable);

        if (elevationRaster.valid())
            _surface->setElevationRaster(elevationRaster.get(), elevationMatrix);
    }
    else
    {
        _empty = true;
    }

    dirtyBound();
}

void
TileNode::initializeData()
{
    // Initialize the data model by copying the parent's rendering data
    // and scale/biasing the matrices.

    TileNode* parent = getParentTile();
    if (parent)
    {
        unsigned quadrant = getKey().getQuadrant();

        const RenderBindings& bindings = _context->getRenderBindings();

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
        if (bindings[SamplerBinding::ELEVATION].isActive())
        {
            updateElevationRaster();
            //const Sampler& elevation = _renderModel._sharedSamplers[SamplerBinding::ELEVATION];
            //if (elevation._texture.valid())
            //{
            //    setElevationRaster(elevation._texture->getImage(0), elevation._matrix);
            //}
        }
    }

    // register me.
    _context->liveTiles()->add( this );

    // signal the tile to start loading data:
    refreshAllLayers();

    // tell the world.
    OE_DEBUG << LC << "notify (create) key " << getKey().str() << std::endl;
    _context->getEngine()->getTerrain()->notifyTileUpdate(getKey(), this);
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
TileNode::isDormant() const
{
    const unsigned minMinExpiryFrames = 3u;
    unsigned frame = _context->getClock()->getFrame();
    double now = _context->getClock()->getTime();

    bool dormant = 
        frame - _lastTraversalFrame > osg::maximum(options().minExpiryFrames().get(), minMinExpiryFrames) &&
        now - _lastTraversalTime > options().minExpiryTime().get();

    return dormant;
}

bool
TileNode::areSiblingsDormant() const
{
    const TileNode* parent = getParentTile();
    return parent ? parent->areSubTilesDormant() : true;
}

void
TileNode::setElevationRaster(const osg::Image* image, const osg::Matrixf& matrix)
{
    if (image != getElevationRaster() || matrix != getElevationMatrix())
    {
        if ( _surface.valid() )
            _surface->setElevationRaster( image, matrix );
    }
}

void
TileNode::updateElevationRaster()
{
    const Sampler& elev = _renderModel._sharedSamplers[SamplerBinding::ELEVATION];
    if (elev._texture.valid())
        setElevationRaster(elev._texture->getImage(0), elev._matrix);
    else
        setElevationRaster(NULL, osg::Matrixf::identity());
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
TileNode::refreshAllLayers()
{
    refreshLayers(CreateTileManifest());
}

void
TileNode::refreshLayers(const CreateTileManifest& manifest)
{
    LoadTileData* r = new LoadTileData(manifest, this, _context.get());
    r->setName(_key.str());
    r->setTileKey(_key);

    _loadQueue.lock();
    _loadQueue.push(r);
    _loadsInQueue = _loadQueue.size();
    _loadQueue.unlock();
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
    unsigned frame = context->getClock()->getFrame();

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

    const TerrainOptions& opt = _context->options();
    canLoadData =
        _doNotExpire ||
        _key.getLOD() == opt.firstLOD().get() ||
        _key.getLOD() >= opt.minLOD().get();

    // whether to accept the current surface node and not the children.
    bool canAcceptSurface = false;

    // If this is an inherit-viewpoint camera, we don't need it to invoke subdivision
    // because we want only the tiles loaded by the true viewpoint.
    const osg::Camera* cam = culler->getCamera();
    if (cam && cam->getReferenceFrame() == osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT)
    {
        canCreateChildren = false;
        canLoadData = false;
    }
    
    else
    {
        // Don't load data OR geometry in progressive mode until the parent is up to date
        if (options().progressive() == true)
        {
            TileNode* parent = getParentTile();
            if (parent && parent->dirty())
            {
                canLoadData = false;

                // comment this out if you want to load the geometry, but not the data --
                // this will allow the terrain to always show the higest tessellation level
                // even as the data is still loading ..
                canCreateChildren = false;
            }
        }
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
                _childrenReady = createChildren( context );

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
    if ( dirty() && canLoadData )
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
        TerrainCuller* culler = dynamic_cast<TerrainCuller*>(&nv);

        // update the timestamp so this tile doesn't become dormant.
        _lastTraversalFrame.exchange(_context->getClock()->getFrame());
        _lastTraversalTime = _context->getClock()->getTime();

        _context->liveTiles()->update(this, nv);

        if (_empty)
        {
            // even if the tile's empty, we need to process its load queue
            if (dirty())
            {
                load(culler);
            }
        }
        else
        {
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

                    if (sampler.ownsTexture())
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

bool
TileNode::createChildren(EngineContext* context)
{
    if (_createChildAsync)
    {
        if (_createChildJobs.empty())
        {
            TileKey parentkey(_key);

            for (unsigned quadrant = 0; quadrant < 4; ++quadrant)
            {
                TileKey childkey = getKey().createChildKey(quadrant);

                CreateTileJob job([context, parentkey, childkey](Cancelable* state) {
                    osg::ref_ptr<TileNode> tile = context->liveTiles()->get(parentkey);
                    if (tile.valid() && !state->isCanceled())
                        return tile->createChild(childkey, context, state);
                    else
                        return (TileNode*)nullptr;
                });

                _createChildJobs.emplace_back(job.schedule());
            }
        }

        else
        {
            int numChildrenReady = 0;

            for (int i = 0; i < 4; ++i)
            {
                if (_createChildJobs[i].isAvailable())
                    ++numChildrenReady;
            }

            if (numChildrenReady == 4)
            {
                for (int i = 0; i < 4; ++i)
                {
                    osg::ref_ptr<TileNode> child = _createChildJobs[i].get();
                    addChild(child);
                    child->initializeData();
                }

                _createChildJobs.clear();
            }
        }
    }

    else
    {
        for (unsigned quadrant = 0; quadrant < 4; ++quadrant)
        {
            TileKey childkey = getKey().createChildKey(quadrant);
            osg::ref_ptr<TileNode> node = createChild(childkey, context, nullptr);
            addChild(node.get());
            node->initializeData();
        }
    }

    return _createChildJobs.empty();
}

TileNode*
TileNode::createChild(const TileKey& childkey, EngineContext* context, Cancelable* progress)
{
    OE_PROFILING_ZONE;

    // Note! There's a chance that the TileNode we want to create here
    // already exists in the registry as an orphan. We should probably 
    // check that out and try to re-attach it...?
    osg::ref_ptr<TileNode> node = new TileNode();

    // Build the surface geometry:
    node->create(childkey, this, context, progress);

    return 
        progress && progress->isCanceled() ? nullptr : node.release();
}

void
TileNode::merge(const TerrainTileModel* model, LoadTileData* request)
{
    bool newElevationData = false;
    const RenderBindings& bindings = _context->getRenderBindings();
    RenderingPasses& myPasses = _renderModel._passes;
    const CreateTileManifest& manifest = request->getManifest();
    vector_set<UID> uidsLoaded;

    // if terrain constraints are in play, regenerate the tile's geometry.
    // this could be kinda slow, but meh, if you are adding and removing
    // constraints, frame drops are not a big concern
    if (manifest.includesConstraints())
    {
        // todo: progress callback here? I believe progress gets
        // checked before merge() anyway.
        createGeometry(nullptr);
    }

    // First deal with the rendering passes (for color data):
    const SamplerBinding& color = bindings[SamplerBinding::COLOR];
    if (color.isActive())
    {
        // loop over all the layers included in the new data model and
        // add them to our render model (or update them if they already exist)
        for(TerrainTileColorLayerModelVector::const_iterator i = model->colorLayers().begin();
            i != model->colorLayers().end();
            ++i)
        {
            TerrainTileImageLayerModel* model = dynamic_cast<TerrainTileImageLayerModel*>(i->get());
            if (model && model->getLayer() && model->getTexture())
            {
                RenderingPass* pass = _renderModel.getPass(model->getLayer()->getUID());
                bool isNewPass = (pass == NULL);

                if (isNewPass)
                {
                    // Pass didn't exist here, so add it now.
                    pass = &_renderModel.addPass();
                    pass->setLayer(model->getLayer());
                }

                pass->setSampler(SamplerBinding::COLOR, model->getTexture(), *model->getMatrix(), model->getRevision());

                // If this is a new rendering pass, just copy the color into the color-parent.
                if (isNewPass && bindings[SamplerBinding::COLOR_PARENT].isActive())
                {
                    pass->samplers()[SamplerBinding::COLOR_PARENT] = pass->samplers()[SamplerBinding::COLOR];
                }
                    
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

                uidsLoaded.insert(pass->sourceUID());
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

                    uidsLoaded.insert(pass->sourceUID());
                }
            }
        }

        // Next loop over all the passes that we OWN, we asked for, but we didn't get.
        // That means they no longer exist at this LOD, and we need to convert them
        // into inherited samplers (or delete them entirely)
        for(int p=0; p<myPasses.size(); ++p)
        {
            RenderingPass& myPass = myPasses[p];

            if (myPass.ownsTexture() && 
                manifest.includes(myPass.layer()) &&
                !uidsLoaded.contains(myPass.sourceUID()))
            {
                OE_DEBUG << LC << "Releasing orphaned layer " << myPass.layer()->getName() << std::endl;

                // release the GL objects associated with this pass.
                // taking this out...can cause "flashing" issues
                //myPass.releaseGLObjects(NULL);
                
                bool deletePass = true;

                TileNode* parent = getParentTile();
                if (parent)
                {
                    const RenderingPass* parentPass = parent->_renderModel.getPass(myPass.sourceUID());
                    if (parentPass)
                    {
                        myPass.inheritFrom(*parentPass, scaleBias[_key.getQuadrant()]);
                        deletePass = false;
                    }
                }

                if (deletePass)
                {
                    myPasses.erase(myPasses.begin() + p);
                    --p;
                }
            }
        }
    }

    // Elevation data:
    const SamplerBinding& elevation = bindings[SamplerBinding::ELEVATION];
    if (elevation.isActive())
    {
        if (model->elevationModel().valid() && model->elevationModel()->getTexture())
        {
            osg::Texture* tex = model->elevationModel()->getTexture();
            int revision = model->elevationModel()->getRevision();

            _renderModel.setSharedSampler(SamplerBinding::ELEVATION, tex, revision);

            //setElevationRaster(tex->getImage(0), osg::Matrixf::identity());
            updateElevationRaster();

            newElevationData = true;
        }

        else if (
            manifest.includesElevation() && 
            _renderModel._sharedSamplers[SamplerBinding::ELEVATION].ownsTexture())
        {
            // We OWN elevation data, requested new data, and didn't get any.
            // That means it disappeared and we need to delete what we have.
            inheritSharedSampler(SamplerBinding::ELEVATION);

            updateElevationRaster();

            newElevationData = true;
        }
    } 

    // Normals:
    const SamplerBinding& normals = bindings[SamplerBinding::NORMAL];
    if (normals.isActive())
    {
        if (model->elevationModel().valid() && model->elevationModel()->getTexture())
        {
            ElevationTexture* etex = static_cast<ElevationTexture*>(model->elevationModel()->getTexture());
            if (etex->getNormalMapTexture())
            {
                osg::Texture* tex = etex->getNormalMapTexture();
                int revision = model->elevationModel()->getRevision();

                if (_context->options().normalizeEdges() == true)
                {
                    // keep the normal map around because we might update it later
                    tex->setUnRefImageDataAfterApply(false);
                }

                _renderModel.setSharedSampler(SamplerBinding::NORMAL, tex, revision);
                updateNormalMap();
            }
        }

        //if (model->normalModel().valid() && model->normalModel()->getTexture())
        //{
        //    osg::Texture* tex = model->normalModel()->getTexture();
        //    int revision = model->normalModel()->getRevision();

        //    if (_context->options().normalizeEdges() == true)
        //    {
        //        // keep the normal map around because we might update it later
        //        tex->setUnRefImageDataAfterApply(false);
        //    }

        //    _renderModel.setSharedSampler(SamplerBinding::NORMAL, tex, revision);
        //    updateNormalMap();
        //}

        // If we OWN normal data, requested new data, and didn't get any,
        // that means it disappeared and we need to delete what we have:
        else if (
            manifest.includesElevation() && // not a typo, check for elevation
            _renderModel._sharedSamplers[SamplerBinding::NORMAL].ownsTexture())
        {
            inheritSharedSampler(SamplerBinding::NORMAL);
            updateNormalMap();
        }
    }

    // Land Cover:
    const SamplerBinding& landCover = bindings[SamplerBinding::LANDCOVER];
    if (landCover.isActive())
    {
        if (model->landCoverModel().valid() && model->landCoverModel()->getTexture())
        {
            osg::Texture* tex = model->landCoverModel()->getTexture();
            int revision = model->landCoverModel()->getRevision();
        
            _renderModel.setSharedSampler(SamplerBinding::LANDCOVER, tex, revision);
        }

        else if (
            manifest.includesLandCover() &&
            _renderModel._sharedSamplers[SamplerBinding::LANDCOVER].ownsTexture())
        {
            // We OWN landcover data, requested new data, and didn't get any.
            // That means it disappeared and we need to delete what we have.
            inheritSharedSampler(SamplerBinding::LANDCOVER);
        }
    }

    // Other Shared Layers:
    uidsLoaded.clear();
    for (unsigned i = 0; i < model->sharedLayers().size(); ++i)
    {
        TerrainTileImageLayerModel* layerModel = model->sharedLayers()[i].get();
        if (layerModel->getTexture())
        {
            // locate the shared binding corresponding to this layer:
            UID uid = layerModel->getImageLayer()->getUID();
            unsigned bindingIndex = INT_MAX;
            for(unsigned i=SamplerBinding::SHARED; i<bindings.size() && bindingIndex==INT_MAX; ++i)
            {
                if (bindings[i].isActive() && bindings[i].sourceUID().isSetTo(uid))
                {
                    bindingIndex = i;
                }                   
            }

            if (bindingIndex < INT_MAX)
            {
                osg::Texture* tex = layerModel->getTexture();
                int revision = layerModel->getRevision();
                _renderModel.setSharedSampler(bindingIndex, tex, revision);
                uidsLoaded.insert(uid);
            }
        }
    }

    // Look for shared layers we need to remove because we own them,
    // requested them, and didn't get updates for them:
    for(unsigned i=SamplerBinding::SHARED; i<bindings.size(); ++i)
    {
        if (bindings[i].isActive() && 
            manifest.includes(bindings[i].sourceUID().get()) &&
            !uidsLoaded.contains(bindings[i].sourceUID().get()))
        {
            inheritSharedSampler(i);
        }
    }

    // Patch Layers - NOP for now
#if 0
    for (unsigned i = 0; i < model->patchLayers().size(); ++i)
    {
        TerrainTilePatchLayerModel* layerModel = model->patchLayers()[i].get();
    }
#endif

    // Propagate changes we made down to this tile's children.
    if (_childrenReady)
    {
        for (int i = 0; i < 4; ++i)
        {
            TileNode* child = getSubTile(i);
            if (child)
            {
                child->refreshInheritedData(this, bindings);
            }
        }
    }

    if (newElevationData)
    {
        _context->getEngine()->getTerrain()->notifyTileUpdate(getKey(), this);
    }

    // Remove the load request that spawned this merge.
    // The only time the request will NOT be in the queue is if it was
    // loadSync() was called.
    _loadQueue.lock();
    if (_loadQueue.empty() == false)
        _loadQueue.pop();
    _loadsInQueue = _loadQueue.size();
    _loadQueue.unlock();

    // Bump the data revision for the tile.
    ++_revision;
}

void TileNode::inheritSharedSampler(int binding)
{
    TileNode* parent = getParentTile();
    if (parent)
    {
        TileRenderModel& parentModel = parent->_renderModel;
        Sampler& mySampler = _renderModel._sharedSamplers[binding];
        mySampler = parentModel._sharedSamplers[binding];
        if (mySampler._texture.valid())
            mySampler._matrix.preMult(scaleBias[_key.getQuadrant()]);
    }
    else
    {
        _renderModel.clearSharedSampler(binding);
    }

    // Bump the data revision for the tile.
    ++_revision;
}

//void TileNode::loadChildren()
//{
//    _mutex.lock();
//
//    if ( !_childrenReady )
//    {        
//        // Create the children
//        createChildren( _context.get() );
//        _childrenReady = true;        
//        int numChildren = getNumChildren();
//        if ( numChildren > 0 )
//        {
//            for(int i=0; i<numChildren; ++i)
//            {
//                TileNode* child = getSubTile(i);
//                if (child)
//                {
//                    // Load the children's data.
//                    child->loadSync();
//                }
//            }
//        }
//    }
//    _mutex.unlock();
//}

void
TileNode::refreshSharedSamplers(const RenderBindings& bindings)
{    
    for (unsigned i = 0; i < _renderModel._sharedSamplers.size(); ++i)
    {
        if (bindings[i].isActive() == false)
        {
            _renderModel.clearSharedSampler(i);
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

    // Count the number of changes we make so we can stop early if it's OK.
    unsigned changes = 0;

    RenderingPasses& parentPasses = parent->_renderModel._passes;
    RenderingPasses& myPasses = _renderModel._passes;

    // Delete any inherited pass whose parent pass no longer exists:
    for(int p=0; p<myPasses.size(); ++p)
    {
        RenderingPass& myPass = myPasses[p];
        if (myPass.inheritsTexture())
        {
            RenderingPass* myParentPass = parent->_renderModel.getPass(myPass.sourceUID());
            if (myParentPass == NULL)
            {
                //OE_WARN << _key.str() << " removing orphaned pass " << myPass.sourceUID() << std::endl;
                myPasses.erase(myPasses.begin()+p);
                --p;
                ++changes;
            }
        }
    }

    // Look for passes in the parent that need to be inherited by this node.
    for (unsigned p=0; p<parentPasses.size(); ++p)
    {
        const RenderingPass& parentPass = parentPasses[p];

        // the corresponsing pass in this node:
        RenderingPass* myPass = _renderModel.getPass(parentPass.sourceUID());

        // Inherit the samplers for this pass.
        if (myPass)
        {
            // Handle the main color:
            if (bindings[SamplerBinding::COLOR].isActive())
            {
                Sampler& mySampler = myPass->samplers()[SamplerBinding::COLOR];
                if (mySampler.inheritsTexture())
                {
                    mySampler.inheritFrom(parentPass.samplers()[SamplerBinding::COLOR], scaleBias[quadrant]);
                    ++changes;
                }
            }

            // Handle the parent color. This is special case -- the parent
            // sampler is always set to the parent's color sampler with a
            // one-time scale/bias.
            if (bindings[SamplerBinding::COLOR_PARENT].isActive())
            {
                Sampler& mySampler = myPass->samplers()[SamplerBinding::COLOR_PARENT];
                const Sampler& parentColorSampler = parentPass.samplers()[SamplerBinding::COLOR];
                osg::Matrixf newMatrix = parentColorSampler._matrix;
                newMatrix.preMult(scaleBias[quadrant]);

                // Did something change?
                if (mySampler._texture.get() != parentColorSampler._texture.get() ||
                    mySampler._matrix != newMatrix ||
                    mySampler._revision != parentColorSampler._revision)
                {
                    if (parentColorSampler._texture.valid() && passInLegalRange(parentPass))
                    {
                        // set the parent-color texture to the parent's color texture
                        // and scale/bias the matrix.
                        mySampler._texture = parentColorSampler._texture.get();
                        mySampler._matrix = newMatrix;
                        mySampler._revision = parentColorSampler._revision;
                    }
                    else
                    {
                        // parent has no color texture? Then set our parent-color
                        // equal to our normal color texture.
                        mySampler = myPass->samplers()[SamplerBinding::COLOR];
                    }
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
                myPass->inheritFrom(parentPass, scaleBias[quadrant]);
                ++changes;
            }
        }
    }

    // Update all the shared samplers (elevation, normal, etc.)
    const Samplers& parentSharedSamplers = parent->_renderModel._sharedSamplers;
    Samplers& mySharedSamplers = _renderModel._sharedSamplers;

    for (unsigned binding=0; binding<parentSharedSamplers.size(); ++binding)
    {        
        Sampler& mySampler = mySharedSamplers[binding];

        if (mySampler.inheritsTexture())
        {
            mySampler.inheritFrom(parentSharedSamplers[binding], scaleBias[quadrant]);
            ++changes;

            // Update the local elevation raster cache (for culling and intersection testing).
            if (binding == SamplerBinding::ELEVATION)
            {
                //osg::Image* raster = mySampler._texture.valid() ? mySampler._texture->getImage(0) : NULL;
                //this->setElevationRaster(raster, mySampler._matrix);
                updateElevationRaster();
            }
        }
    }

    if (changes > 0)
    {
        // Bump the data revision for the tile.
        ++_revision;

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
}

bool
TileNode::passInLegalRange(const RenderingPass& pass) const
{
    return 
        pass.tileLayer() == 0L ||
        pass.tileLayer()->isKeyInVisualRange(getKey());
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
    //if (options().progressive() == true)
    //{
    //    lodPriority = (float)(numLods - lod);
    //}

    // dist priority is in the range [0..1]
    float distance = culler->getDistanceToViewPoint(getBound().center(), true);
    float maxRange = si.getLOD(0)._visibilityRange;
    float distPriority = 1.0 - distance/maxRange;

    // add them together, and you get tiles sorted first by lodPriority
    // (because of the biggest range), and second by distance.
    float priority = lodPriority + distPriority;

    // Submit to the loader.
    _loadQueue.lock(); // lock the load queue
    if (_loadQueue.empty() == false)
    {
        LoadTileData* r = _loadQueue.front().get();
        _context->getLoader()->load(r, priority, *culler);
    }
    _loadQueue.unlock(); // unlock the load queue
}

void
TileNode::loadSync()
{
    osg::ref_ptr<LoadTileData> loadTileData = new LoadTileData(this, _context.get());
    loadTileData->setEnableCancelation(false);
    loadTileData->run(0L);
    loadTileData->merge();
}

bool
TileNode::areSubTilesDormant() const
{
    return
        getNumChildren() >= 4       &&
        getSubTile(0)->isDormant()  &&
        getSubTile(1)->isDormant()  &&
        getSubTile(2)->isDormant()  &&
        getSubTile(3)->isDormant();
}

void
TileNode::removeSubTiles()
{
    _childrenReady = false;
    for(int i=0; i<getNumChildren(); ++i)
    {
        getChild(i)->releaseGLObjects(NULL);
    }
    this->removeChildren(0, this->getNumChildren());

    _createChildJobs.clear();
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
    if (thisNormalMap.inheritsTexture() || !thisNormalMap._texture->getImage(0))
        return;

    if (!_eastNeighbor.valid() || !_southNeighbor.valid())
        return;

    osg::ref_ptr<TileNode> east;
    if (_eastNeighbor.lock(east))
    {
        const Sampler& thatNormalMap = east->_renderModel._sharedSamplers[SamplerBinding::NORMAL];
        if (thatNormalMap.inheritsTexture() || !thatNormalMap._texture->getImage(0))
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
        osg::Vec4 pixel;
        ImageUtils::PixelReader readThat(thatImage);
        ImageUtils::PixelWriter writeThis(thisImage);
        
        for (int t=0; t<height; ++t)
        {
            readThat(pixel, 0, t);
            writeThis(pixel, width-1, t);
            //writeThis(readThat(0, t), width-1, t);
        }

        thisImage->dirty();
    }

    osg::ref_ptr<TileNode> south;
    if (_southNeighbor.lock(south))
    {
        const Sampler& thatNormalMap = south->_renderModel._sharedSamplers[SamplerBinding::NORMAL];
        if (thatNormalMap.inheritsTexture() || !thatNormalMap._texture->getImage(0))
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
        osg::Vec4 pixel;
        ImageUtils::PixelReader readThat(thatImage);
        ImageUtils::PixelWriter writeThis(thisImage);

        for (int s=0; s<width; ++s)
        {
            readThat(pixel, s, height-1);
            writeThis(pixel, s, 0);
            //writeThis(readThat(s, height-1), s, 0);
        }

        thisImage->dirty();
    }

    //OE_INFO << LC << _key.str() << " : updated normal map.\n";
}

const TerrainOptions&
TileNode::options() const
{
    return _context->options();
}
