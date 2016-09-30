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
_lastTraversalFrame(0.0)
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
    if (!context)
        return;

    _key = key;

    // Create mask records
    osg::ref_ptr<MaskGenerator> masks = new MaskGenerator(key, context->getOptions().tileSize().get(), context->getMapFrame());

    // Get a shared geometry from the pool that corresponds to this tile key:
    osg::ref_ptr<osg::Geometry> geom;
    context->getGeometryPool()->getPooledGeometry(key, context->getMapFrame().getMapInfo(), geom, masks.get());


    // Create the drawable for the terrain surface:
    TileDrawable* surfaceDrawable = new TileDrawable(
        key, 
        context->getRenderBindings(), 
        geom.get(),
        context->getOptions().tileSize().get(),
        context->getGeometryPool()->getNumSkirtElements() );

    surfaceDrawable->setDrawAsPatches(false);

    // Create the node to house the tile drawable:
    _surface = new SurfaceNode(
        key,
        context->getMapFrame().getMapInfo(),
        context->getRenderBindings(),
        surfaceDrawable );

    // Create a drawable for "patch" geometry, which is rendered as GL patches, not triangles.
    // Patch geometry can be used to place land cover or render other tile-specific data.
    TileDrawable* patchDrawable = new TileDrawable(
        key, 
        context->getRenderBindings(),
        geom.get(),
        context->getOptions().tileSize().get(),
        context->getGeometryPool()->getNumSkirtElements() );
    
    patchDrawable->setDrawAsPatches(true);

    // And a node to house that as well:
    _patch = new SurfaceNode(
        key,
        context->getMapFrame().getMapInfo(),
        context->getRenderBindings(),
        patchDrawable );

    // initialize all the per-tile uniforms the shaders will need:
    createPayloadStateSet(context);

    updateTileUniforms(context->getSelectionInfo());

    // Set up a data container for multipass layer rendering:
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
    if ( _surface.valid() )
        _surface->setElevationRaster( image, matrix );

    if ( _patch.valid() )
        _patch->setElevationRaster( image, matrix );
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
TileNode::createPayloadStateSet(EngineContext* context)
{
    _payloadStateSet = new osg::StateSet();

    // Install the tile key uniform.
    _tileKeyUniform = new osg::Uniform("oe_tile_key", osg::Vec4f(0,0,0,0));
    _payloadStateSet->addUniform( _tileKeyUniform.get() );

    _tileMorphUniform = new osg::Uniform("oe_tile_morph", osg::Vec2f(0,0));
    _payloadStateSet->addUniform( _tileMorphUniform.get() );
}

void
TileNode::updateTileUniforms(const SelectionInfo& selectionInfo)
{
    //assert(_surface.valid());
    // update the tile key uniform
    const osg::BoundingBox& bbox = _surface->getAlignedBoundingBox();
    float width = std::max( (bbox.xMax()-bbox.xMin()), (bbox.yMax()-bbox.yMin()) );


    // Encode the tile key in a uniform. Note! The X and Y components are presented
    // modulo 2^16 form so they don't overrun single-precision space.
    unsigned tw, th;
    _key.getProfile()->getNumTiles(_key.getLOD(), tw, th);

    const double m = pow(2.0, 16.0);

    double x = (double)_key.getTileX();
    double y = (double)(th - _key.getTileY()-1);

    _tileKeyUniform->set(osg::Vec4f(
        (float)fmod(x, m),
        (float)fmod(y, m),
        (float)_key.getLOD(),
        width));

    // update the morph constants

    float start = (float)selectionInfo.visParameters(_key.getLOD())._fMorphStart;
    float end   = (float)selectionInfo.visParameters(_key.getLOD())._fMorphEnd;

    float one_by_end_minus_start = end - start;
    one_by_end_minus_start = 1.0f/one_by_end_minus_start;
    osg::Vec2f morphConstants( end * one_by_end_minus_start, one_by_end_minus_start );
    _tileMorphUniform->set( morphConstants );

    const osg::Image* er = getElevationRaster();
    if ( er )
    {
        // pre-calculate texel-sampling scale and bias coefficients that allow us to sample
        // elevation textures on texel-center instead of edge:
        float size = (float)er->s();
        osg::Vec2f elevTexelOffsets( (size-1.0f)/size, 0.5/size );
        getOrCreateStateSet()->getOrCreateUniform("oe_tile_elevTexelCoeff", osg::Uniform::FLOAT_VEC2)->set(elevTexelOffsets);
    }
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
    OE_DEBUG << LC << "Tile " << _key.str() << " : Release GL objects\n";

    if ( getStateSet() )
        getStateSet()->releaseGLObjects(state);
    if ( _payloadStateSet.valid() )
        _payloadStateSet->releaseGLObjects(state);
    if ( _surface.valid() )
        _surface->releaseGLObjects(state);
    if ( _patch.valid() )
        _patch->releaseGLObjects(state);
    if ( _mptex.valid() )
        _mptex->releaseGLObjects(state);

    osg::Group::releaseGLObjects(state);
}

bool
TileNode::shouldSubDivide(osgUtil::CullVisitor* cv, const SelectionInfo& selectionInfo)
{
    unsigned currLOD = _key.getLOD();
    if (currLOD < selectionInfo.numLods() && currLOD != selectionInfo.numLods()-1)
    {
        return _surface->anyChildBoxIntersectsSphere(
            cv->getViewPointLocal(), 
            (float)selectionInfo.visParameters(currLOD+1)._visibilityRange2,
            cv->getLODScale());
    }
    return false;
}


bool
TileNode::isVisible(osg::CullStack* stack) const
{
#ifdef VISIBILITY_PRECHECK
    return _surface->isVisible( stack );
#else
    return true;
#endif
}

bool
TileNode::cull_stealth(osgUtil::CullVisitor* cv)
{
    bool visible = false;

    EngineContext* context = VisitorData::fetch<EngineContext>(*cv, ENGINE_CONTEXT_TAG);

    // Shows all culled tiles, good for testing culling
    unsigned frame = cv->getFrameStamp()->getFrameNumber();

    if ( frame - _lastAcceptSurfaceFrame < 2u )
    {
        acceptSurface( cv, context );
    }

    else if ( _childrenReady )
    {
        for(int i=0; i<4; ++i)
        {
            getSubTile(i)->accept_cull_stealth( cv );
        }
    }

    return visible;
}

bool
TileNode::cull(osgUtil::CullVisitor* cv)
{
    EngineContext* context = VisitorData::fetch<EngineContext>(*cv, ENGINE_CONTEXT_TAG);
    const SelectionInfo& selectionInfo = context->getSelectionInfo();

    // Horizon check the surface first:
    if ( !_surface->isVisible(cv) )
    {
        return false;
    }
    
    // determine whether we can and should subdivide to a higher resolution:
    bool childrenInRange = shouldSubDivide(cv, selectionInfo);

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
    const osg::Camera* cam = cv->getCurrentCamera();
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
                getSubTile(i)->accept_cull(cv);
            }

            // if we traversed all children, but they all return "not visible",
            // that means it's a horizon-culled tile anyway and we don't need
            // to add any drawables.
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
        acceptSurface( cv, context );
        _lastAcceptSurfaceFrame.exchange( cv->getFrameStamp()->getFrameNumber() );
    }

       
    // Run any patch callbacks.
    context->invokeTilePatchCallbacks( cv, getTileKey(), _payloadStateSet.get(), _patch.get() );

    // If this tile is marked dirty, try loading data.
    if ( _dirty && canLoadData )
    {
        load( *cv );
    }

    return true;
}

bool
TileNode::acceptSurface(osgUtil::CullVisitor* cv, EngineContext* context)
{
    OE_START_TIMER(acceptSurface);

    // The reason we push the top-leel surface SS for every node is because
    // of the patch callbacks. Instead of doing this we need a way to put
    // patch traversals in their own top-level bin...

    cv->pushStateSet( context->_surfaceSS.get() );
    cv->pushStateSet( _payloadStateSet.get() );
    _surface->accept( *cv );
    cv->popStateSet();
    cv->popStateSet();

    REPORT("TileNode::acceptSurface", acceptSurface);

    return true; //visible;
}

bool
TileNode::accept_cull(osgUtil::CullVisitor* cv)
{
    bool visible = false;
    
    if (cv)
    {
        // update the timestamp so this tile doesn't become dormant.
        _lastTraversalFrame.exchange( cv->getFrameStamp()->getFrameNumber() );
        _lastTraversalTime = cv->getFrameStamp()->getReferenceTime();

        if ( !cv->isCulled(*this) )
        {
            cv->pushStateSet( getStateSet() );

            visible = cull( cv );

            cv->popStateSet();
        }
    }

    return visible;
}

bool
TileNode::accept_cull_stealth(osgUtil::CullVisitor* cv)
{
    bool visible = false;
    
    if (cv)
    {
        cv->pushStateSet( getStateSet() );

        visible = cull_stealth( cv );

        cv->popStateSet();
    }

    return visible;
}

void
TileNode::traverse(osg::NodeVisitor& nv)
{
    // Cull only:
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);

        if (VisitorData::isSet(nv, "osgEarth.Stealth"))
        {
            accept_cull_stealth( cv );
        }
        else
        {
            accept_cull( cv );
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
        node->create( getTileKey().createChildKey(quadrant), context );

        // Add to the scene graph.
        addChild( node );

        // Inherit the samplers with new scale/bias information.
        node->inheritState( context );
    }
}

bool
TileNode::inheritState(EngineContext* context)
{
    // Find the parent node. It will only be null if this is a "first LOD" tile.
    TileNode* parent = getNumParents() > 0 ? dynamic_cast<TileNode*>(getParent(0)) : 0L;

    bool changesMade = false;

    // which quadrant is this tile in?
    unsigned quadrant = getTileKey().getQuadrant();

    // default inheritance of the elevation data for bounding purposes:
    osg::ref_ptr<const osg::Image> elevRaster;
    osg::Matrixf                   elevMatrix;
    if ( parent )
    {
        elevRaster = parent->getElevationRaster();
        elevMatrix = parent->getElevationMatrix();
        elevMatrix.preMult( scaleBias[quadrant] );
    }

    // Find all the sampler matrix uniforms and scale/bias them to the current quadrant.
    // This will inherit textures and use the proper sub-quadrant until new data arrives (later).
    for( RenderBindings::const_iterator binding = context->getRenderBindings().begin(); binding != context->getRenderBindings().end(); ++binding )
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

        else if ( binding->usage().isSetTo(binding->COLOR_PARENT) )
        {
            //nop -- this is handled as part of the COLOR binding
        }

        else
        {
            osg::StateAttribute* sa = getStateSet()->getTextureAttribute(binding->unit(), osg::StateAttribute::TEXTURE);        

            // If the attribute isn't present, that means we are inheriting it from above.
            // So construct a new scale/bias matrix.
            if ( sa == 0L )
            {
                osg::Matrixf matrix;

                // Find the parent's matrix and scale/bias it to this quadrant:
                if ( parent && parent->getStateSet() )
                {
                    const osg::Uniform* matrixUniform = parent->getStateSet()->getUniform( binding->matrixName() );
                    if ( matrixUniform )
                    {
                        matrixUniform->get( matrix );
                        matrix.preMult( scaleBias[quadrant] );
                    }
                }

                // Add a new uniform with the scale/bias'd matrix:
                osg::StateSet* stateSet = getOrCreateStateSet();
                stateSet->removeUniform( binding->matrixName() );
                stateSet->addUniform( context->getOrCreateMatrixUniform(binding->matrixName(), matrix) );
                changesMade = true;
            }

            // If this is elevation data, record the new raster so we can apply it to the node.
            else if ( binding->usage().isSetTo(binding->ELEVATION) )
            {
                osg::Texture* t = static_cast<osg::Texture*>(sa);
                elevRaster = t->getImage(0);
                elevMatrix = osg::Matrixf::identity();
            }
        }
    }

    // If we found one, communicate it to the node and its children.
    if (elevRaster.valid())
    {
        if (elevRaster.get() != getElevationRaster() || elevMatrix != getElevationMatrix() )
        {
            setElevationRaster( elevRaster.get(), elevMatrix );
            changesMade = true;
        }
    }

    // finally, update the uniforms for terrain morphing
    updateTileUniforms( context->getSelectionInfo() );

    if ( !changesMade )
    {
        OE_INFO << LC << _key.str() << ", good, no changes :)\n";
    }
    else
    {
        dirtyBound();
    }

    return changesMade;
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
    EngineContext* context = VisitorData::fetch<EngineContext>(nv, ENGINE_CONTEXT_TAG);

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

    float distance = nv.getDistanceToViewPoint(getBound().center(), true);

    // dist priority uis in the range [0..1]
    float distPriority = 1.0 - distance/si.visParameters(0)._visibilityRange;

    // add thenm together, and you get tiles sorted first by lodPriority (because of
    // the biggest range), and second by distance.
    float priority = lodPriority + distPriority;

    // normalize the composite priority to [0..1].
    priority /= (float)(numLods+1);

    // Submit to the loader.
    context->getLoader()->load( _loadRequest.get(), priority, nv );
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
