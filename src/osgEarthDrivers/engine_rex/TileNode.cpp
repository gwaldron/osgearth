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
#include "ProxySurfaceNode"
#include "EngineContext"
#include "Loader"
#include "LoadTileData"
#include "SelectionInfo"
#include "ElevationTextureUtils"
#include "TerrainCuller"

#include <osgEarth/CullingUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/TraversalData>
#include <osgEarth/Shadowing>
#include <osgEarth/Utils>
#include <osgEarth/TraversalData>

#include <osg/Uniform>
#include <osg/ComputeBoundsVisitor>
#include <osg/ValueObject>

using namespace osgEarth::Drivers::RexTerrainEngine;
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
_minExpiryTime( 0.0 ),
_minExpiryFrames( 0 ),
_lastTraversalTime(0.0),
_lastTraversalFrame(0.0),
_count(0)
{
    //nop
}

void
TileNode::create(const TileKey& key, TileNode* parent, EngineContext* context)
{
    if (!context)
        return;

    _key = key;

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

    // Mask generator creates geometry from masking boundaries when they exist.
    osg::ref_ptr<MaskGenerator> masks = new MaskGenerator(
        key, 
        context->getOptions().tileSize().get(), 
        context->getMapFrame());

    // Get a shared geometry from the pool that corresponds to this tile key:
    osg::ref_ptr<osg::Geometry> geom;
    context->getGeometryPool()->getPooledGeometry(key, context->getMapFrame().getMapInfo(), geom, masks.get());

    // Create the drawable for the terrain surface:
    TileDrawable* surfaceDrawable = new TileDrawable(
        key, 
        geom.get(),
        context->getOptions().tileSize().get() );

    // Create the node to house the tile drawable:
    _surface = new SurfaceNode(
        key,
        context->getMapFrame().getMapInfo(),
        context->getRenderBindings(),
        surfaceDrawable );

    // This was the old patch drawable.
    // Future: This will be the same thing at the TileDrawCommand but with a "drawAsPatch" boolean
    // and will be a separate layer.
#if 0
    // Create a drawable for "patch" geometry, which is rendered as GL patches, not triangles.
    // Patch geometry can be used to place land cover or render other tile-specific data.
    TileDrawable* patchDrawable = new TileDrawable(
        key, 
        geom.get(),
        context->getOptions().tileSize().get() );
    
    //patchDrawable->setDrawAsPatches(true);

    // And a node to house that as well:
    _patch = new SurfaceNode(
        key,
        context->getMapFrame().getMapInfo(),
        context->getRenderBindings(),
        patchDrawable );
#endif

    // initialize all the per-tile uniforms the shaders will need:
    float start = (float)context->getSelectionInfo().visParameters(_key.getLOD())._fMorphStart;
    float end   = (float)context->getSelectionInfo().visParameters(_key.getLOD())._fMorphEnd;
    float one_by_end_minus_start = end - start;
    one_by_end_minus_start = 1.0f/one_by_end_minus_start;
    _morphConstants.set( end * one_by_end_minus_start, one_by_end_minus_start );

    // Initialize the data model by copying the parent's rendering data
    // and scale/biasing the matrices.
    if (parent)
    {    
        unsigned quadrant = getTileKey().getQuadrant();

        // Copy the parent's rendering model.
        _renderingData = parent->_renderingData;

        for (unsigned p = 0; p < _renderingData._passes.size(); ++p)
        {
            // Scale/bias each matrix for this key quadrant.
            RenderingPass& pass = _renderingData._passes[p];
            for (unsigned s = 0; s < pass._samplers.size(); ++s)
            {
                Sampler& sampler = pass._samplers[s];
                sampler._matrix.preMult(scaleBias[quadrant]);
            }

            // Are we using image blending? If so, initialize the color_parent 
            // to the color texture.
            if (context->getRenderBindings()[SamplerBinding::COLOR_PARENT].isActive())
            {
                pass._samplers[SamplerBinding::COLOR_PARENT] = pass._samplers[SamplerBinding::COLOR];
            }

            // Use the elevation sampler in the first pass to initialize
            // the elevation raster (used for primitive functors, intersection, etc.)
            if (p == 0)
            {
                const Sampler& elevation = pass._samplers[SamplerBinding::ELEVATION];
                if (elevation._texture.valid())
                {
                    setElevationRaster(elevation._texture->getImage(0), elevation._matrix);
                }
            }
        }
    }
    else
    {
        //OE_INFO << LC << _key.str() << ": No parent found in create\n";
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
    osg::BoundingSphere bs;
    if (_surface.valid())
    {
        bs = _surface->getBound();
        const osg::BoundingBox& bbox = _surface->getAlignedBoundingBox();
        _tileKeyValue.a() = std::max( (bbox.xMax()-bbox.xMin()), (bbox.yMax()-bbox.yMin()) );
    }    
    return bs;
}

bool
TileNode::isDormant(const osg::FrameStamp* fs) const
{
    const unsigned minMinExpiryFrames = 3u;

    bool dormant = 
           fs &&
           fs->getFrameNumber() - _lastTraversalFrame > std::max(_minExpiryFrames, minMinExpiryFrames) &&
           fs->getReferenceTime() - _lastTraversalTime > _minExpiryTime;
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

        if ( _patch.valid() )
            _patch->setElevationRaster( image, matrix );
    }
}

const osg::Image*
TileNode::getElevationRaster() const
{
    return _surface->getElevationRaster();
}

const osg::Matrixf&
TileNode::getElevationMatrix() const
{
    return _surface->getElevationMatrix();
}

void
TileNode::setDirty(bool value)
{
    _dirty = value;
}

void
TileNode::releaseGLObjects(osg::State* state) const
{
    //OE_WARN << LC << "Tile " << _key.str() << " : Release GL objects\n";

    if ( _surface.valid() )
        _surface->releaseGLObjects(state);

    if ( _patch.valid() )
        _patch->releaseGLObjects(state);

    _renderingData.releaseGLObjects(state);

    osg::Group::releaseGLObjects(state);
}

bool
TileNode::shouldSubDivide(TerrainCuller* culler, const SelectionInfo& selectionInfo)
{    
    unsigned currLOD = _key.getLOD();
    if (currLOD < selectionInfo.numLods() && currLOD != selectionInfo.numLods()-1)
    {
        return _surface->anyChildBoxIntersectsSphere(
            culler->getViewPointLocal(), 
            (float)selectionInfo.visParameters(currLOD+1)._visibilityRange2,
            culler->getLODScale());
    }
    return false;
}

bool
TileNode::cull_stealth(TerrainCuller* culler)
{
    bool visible = false;

    EngineContext* context = culler->getEngineContext();

    // Shows all culled tiles, good for testing culling
    unsigned frame = culler->getFrameStamp()->getFrameNumber();

    if ( frame - _lastAcceptSurfaceFrame < 2u )
    {
        _surface->accept( *culler );
    }

    else if ( _childrenReady )
    {
        for(int i=0; i<4; ++i)
        {
            getSubTile(i)->accept_cull_stealth( culler );
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
    
    // Don't create children in progressive mode until content is in place
    if ( _dirty && context->getOptions().progressive() == true )
    {
        canCreateChildren = false;
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

            if ( !_childrenReady )
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
                getSubTile(i)->accept(*culler);
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
        _lastAcceptSurfaceFrame.exchange( culler->getFrameStamp()->getFrameNumber() );
    }

       
    // Run any patch callbacks.
#if 0 //TODO!
    context->invokeTilePatchCallbacks( cv, getTileKey(), _payloadStateSet.get(), _patch.get() );
#else
    //OE_WARN << "TODO: patch callbacks\n";
#endif

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
TileNode::accept_cull_stealth(TerrainCuller* culler)
{
    bool visible = false;
    
    if (culler)
    {
        visible = cull_stealth( culler );
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
        
        if (VisitorData::isSet(nv, "osgEarth.Stealth"))
        {
            accept_cull_stealth( culler );
        }
        else
        {
            accept_cull( culler );
        }
    }

    // Everything else: update, GL compile, intersection, compute bound, etc.
    else
    {
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
        // TODO: in what situations should we traverse the landcover as well? GL compile?
        else 
        {
            _surface->accept( nv );
        }
    }
}

void
TileNode::createChildren(EngineContext* context)
{
    // NOTE: Ensure that _mutex is locked before calling this fucntion!
    //OE_WARN << "Creating children for " << _key.str() << std::endl;

    // Create the four child nodes.
    for(unsigned quadrant=0; quadrant<4; ++quadrant)
    {
        TileNode* node = new TileNode();
        if (context->getOptions().minExpiryFrames().isSet())
        {
            node->setMinimumExpiryFrames( *context->getOptions().minExpiryFrames() );
        }
        if (context->getOptions().minExpiryTime().isSet())
        {         
            node->setMinimumExpiryTime( *context->getOptions().minExpiryTime() );
        }

        // Build the surface geometry:
        node->create( getTileKey().createChildKey(quadrant), this, context );

        // Add to the scene graph.
        addChild( node );
    }
}

void
TileNode::merge(const TerrainTileModel* model, const RenderBindings& bindings)
{
    // Add color passes:
    const SamplerBinding& color = bindings[SamplerBinding::COLOR];
    if (color.isActive())
    {
        for (TerrainTileImageLayerModelVector::const_iterator i = model->colorLayers().begin();
            i != model->colorLayers().end();
            ++i)
        {
            TerrainTileImageLayerModel* layer = i->get();
            if (layer && layer->getTexture())
            {
                RenderingPass* pass = _renderingData.getPass(layer->getImageLayer()->getUID());
                if (!pass)
                {
                    pass = &_renderingData.addPass();
                    pass->_sourceUID = layer->getImageLayer()->getUID();
                    pass->_valid = true;
                    //OE_INFO << LC << _key.str() << ": addPass for layer " << layer->getImageLayer()->getName() << "\n";
                }
                pass->_samplers[SamplerBinding::COLOR]._texture = layer->getTexture();
                pass->_samplers[SamplerBinding::COLOR]._matrix.makeIdentity();
            }
        }
    }
    else
    {
        // No color layers? We need a rendering pass with a null texture then
        // to accomadate the other samplers.
        RenderingPass& pass = _renderingData.addPass();
        pass._valid = true;
    }

    // Elevation:
    const SamplerBinding& elevation = bindings[SamplerBinding::ELEVATION];
    if (elevation.isActive() && model->elevationModel().valid() && model->elevationModel()->getTexture())
    {
        osg::Texture* tex = model->elevationModel()->getTexture();
        // always keep the elevation image around because we use it for bounding box computation:
        tex->setUnRefImageDataAfterApply(false);
        for (unsigned p = 0; p < _renderingData._passes.size(); ++p)
        {
            _renderingData._passes[p]._samplers[SamplerBinding::ELEVATION]._texture = tex;
            _renderingData._passes[p]._samplers[SamplerBinding::ELEVATION]._matrix.makeIdentity();
        }

        setElevationRaster(tex->getImage(0), osg::Matrixf::identity());
    }    

    // Normals:
    const SamplerBinding& normals = bindings[SamplerBinding::NORMAL];
    if (normals.isActive() && model->normalModel().valid() && model->normalModel()->getTexture())
    {
        osg::Texture* tex = model->normalModel()->getTexture();
        for (unsigned p = 0; p < _renderingData._passes.size(); ++p)
        {
            _renderingData._passes[p]._samplers[SamplerBinding::NORMAL]._texture = tex;
            _renderingData._passes[p]._samplers[SamplerBinding::NORMAL]._matrix.makeIdentity();
        }
    }

    // Shared Layers:
    for (unsigned i = 0; i < model->sharedLayers().size(); ++i)
    {
        unsigned bindingIndex = SamplerBinding::SHARED + i;
        const SamplerBinding& binding = bindings[bindingIndex];

        TerrainTileImageLayerModel* layerModel = model->sharedLayers().at(i);
        if (layerModel->getTexture())
        {
            osg::Texture* tex = layerModel->getTexture();
            //applyDefaultUnRefPolicy(tex);
            for (unsigned p = 0; p < _renderingData._passes.size(); ++p)
            {
                _renderingData._passes[p]._samplers[bindingIndex]._texture = tex;
                _renderingData._passes[p]._samplers[bindingIndex]._matrix.makeIdentity();
            }
        }
    }

    if (_childrenReady)
    {
        getSubTile(0)->refreshInheritedData(this, bindings);
        getSubTile(1)->refreshInheritedData(this, bindings);
        getSubTile(2)->refreshInheritedData(this, bindings);
        getSubTile(3)->refreshInheritedData(this, bindings);
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
    unsigned quadrant = getTileKey().getQuadrant();

    // Count the number of inherited samplers so we know when to stop. If none of the
    // samplers in this tile inherit from the parent, there is no need to continue
    // down the Tile tree.
    unsigned changes = 0;

    RenderingPassList& parentPasses = parent->_renderingData._passes;

    for (unsigned p = 0; p<parentPasses.size(); ++p)
    {
        const RenderingPass& parentPass = parentPasses[p];

        RenderingPass* myPass = _renderingData.getPass(parentPass._sourceUID);
        if (myPass)
        {
            for (unsigned s = 0; s < myPass->_samplers.size(); ++s)
            {
                Sampler& mySampler = myPass->_samplers[s];
                
                // the color-parent gets special treatment, since it is not included
                // in the TileModel (rather it is always derived here).
                if (s == SamplerBinding::COLOR_PARENT)
                {
                    const Sampler& parentSampler = parentPass._samplers[SamplerBinding::COLOR];
                    osg::Matrixf newMatrix = parentSampler._matrix;
                    newMatrix.preMult(scaleBias[quadrant]);

                    // Did something change?
                    if (mySampler._texture.get() != parentSampler._texture.get() ||
                        mySampler._matrix != newMatrix)
                    {
                        if (parentSampler._texture.valid())
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
                            mySampler._texture = myPass->_samplers[SamplerBinding::COLOR]._texture.get();
                            mySampler._matrix = myPass->_samplers[SamplerBinding::COLOR]._matrix;
                        }
                        ++changes;
                    }
                }

                else
                if (!mySampler._texture.valid() || !mySampler._matrix.isIdentity())
                {
                    const Sampler& parentSampler = parentPass._samplers[s];
                    mySampler._texture = parentSampler._texture.get();
                    mySampler._matrix = parentSampler._matrix;
                    mySampler._matrix.preMult(scaleBias[quadrant]);
                    ++changes;
                }
            }
        }
        else
        {
            myPass = &_renderingData.addPass();
            *myPass = parentPass;
            for (unsigned s = 0; s < myPass->_samplers.size(); ++s)
            {
                Sampler& sampler = myPass->_samplers[s];
                sampler._matrix.preMult(scaleBias[quadrant]);
            }
            ++changes;
        }
    }

    if (!_renderingData._passes.empty())
    {
        // Locate the elevation sampler and pass its raster data along to use
        // for intersections, primitive functors, etc.
        const Sampler& elevation = _renderingData._passes[0]._samplers[SamplerBinding::ELEVATION];
        if (elevation._texture.valid())
        {
            const osg::Image* elevRaster = elevation._texture->getImage(0);
            osg::Matrixf elevMatrix = elevation._matrix;

            if (elevRaster)
            {
                setElevationRaster(elevRaster, elevMatrix);
            }
        }
    }

    if (changes > 0)
    {
        dirtyBound();
        
        if (_childrenReady)
        {
            getSubTile(0)->refreshInheritedData(this, bindings);
            getSubTile(1)->refreshInheritedData(this, bindings);
            getSubTile(2)->refreshInheritedData(this, bindings);
            getSubTile(3)->refreshInheritedData(this, bindings);
        }
    }
    else
    {
        //OE_INFO << LC << _key.str() << ": refreshInheritedData, stopped short.\n";
    }
}

void
TileNode::load(TerrainCuller* culler)
{
    // Access the context:
    EngineContext* context = culler->getEngineContext();

    // Create a new load request on demand:
    if ( !_loadRequest.valid() )
    {
        Threading::ScopedMutexLock lock(_mutex);
        if ( !_loadRequest.valid() )
        {
            _loadRequest = new LoadTileData( this, context );
            _loadRequest->setName( _key.str() );
            _loadRequest->setTileKey( _key );
        }
    }

    
    // Construct the load PRIORITY: 0=lowest, 1=highest.
    
    const SelectionInfo& si = context->getSelectionInfo();
    int lod     = getTileKey().getLOD();
    int numLods = si.numLods();
    
    // LOD priority is in the range [0..numLods]
    float lodPriority = (float)lod;
    if ( context->getOptions().highResolutionFirst() == false )
        lodPriority = (float)(numLods - lod);

    float distance = culler->getDistanceToViewPoint(getBound().center(), true);

    // dist priority uis in the range [0..1]
    float distPriority = 1.0 - distance/si.visParameters(0)._visibilityRange;

    // add them together, and you get tiles sorted first by lodPriority
    // (because of the biggest range), and second by distance.
    float priority = lodPriority + distPriority;

    // normalize the composite priority to [0..1].
    priority /= (float)(numLods+1);

    // Submit to the loader.
    context->getLoader()->load( _loadRequest.get(), priority, *culler );
}

void
TileNode::loadSync(EngineContext* context)
{
    osg::ref_ptr<LoadTileData> loadTileData = new LoadTileData(this, context);
    loadTileData->invoke();
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
