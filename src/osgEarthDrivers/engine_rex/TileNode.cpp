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
_dirty( false )
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

    // Create mask records
    osg::ref_ptr<MaskGenerator> masks = context ? new MaskGenerator(key, context->getOptions().tileSize().get(), context->getMapFrame()) : 0L;

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

    // PPP: Better way to do this rather than here?
    // Can't do it at RexTerrainEngineNode level, because the SurfaceNode is not valid yet
    if (context->getSelectionInfo().initialized()==false)
    {
        static Threading::Mutex s_selInfoMutex;
        Threading::ScopedMutexLock lock(s_selInfoMutex);
        if ( context->getSelectionInfo().initialized()==false)
        {
            SelectionInfo& selectionInfo = const_cast<SelectionInfo&>(context->getSelectionInfo());

            unsigned uiFirstLOD = *(context->_options.firstLOD());
            unsigned uiMaxLod   = std::min( context->_options.maxLOD().get(), 19u ); // beyond LOD 19 or 20, morphing starts to lose precision.
            unsigned uiTileSize = *(context->_options.tileSize());

            selectionInfo.initialize(uiFirstLOD, uiMaxLod, uiTileSize, getVisibilityRangeHint(context));
        }
    }

    // initialize all the per-tile uniforms the shaders will need:
    createTileUniforms();
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
    return fs && fs->getFrameNumber() - _lastTraversalFrame > 2u;
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
TileNode::createTileUniforms()
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

    unsigned tw, th;
    _key.getProfile()->getNumTiles(_key.getLOD(), tw, th);

    _tileKeyUniform->set(osg::Vec4f(_key.getTileX(), th-_key.getTileY()-1.0f, _key.getLOD(), width));

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
    if ( getStateSet() )
        getStateSet()->releaseGLObjects(state);
    if ( _payloadStateSet.valid() )
        _payloadStateSet->releaseGLObjects(state);
    if ( _surface.valid() )
        _surface->releaseGLObjects(state);
    if ( _patch.valid() )
        _patch->releaseGLObjects(state);
    //if ( _mptex.valid() )
    //    _mptex->releaseGLObjects(state);

    osg::Group::releaseGLObjects(state);
}

float
TileNode::getVisibilityRangeHint(EngineContext* context) const
{    
    unsigned firstLOD = context->_options.firstLOD().get();
    float mtrf = context->_options.minTileRangeFactor().get();

    if (getTileKey().getLOD()!=firstLOD)
    {
        OE_INFO << LC <<"Error: Visibility Range hint can be computed only using the first LOD"<<std::endl;
        return -1;
    }
    // The motivation here is to use the extents of the tile at lowest resolution
    // along with a factor as an estimate of the visibility range
    const float factor = mtrf * 3.214f; //2.5f;
    const osg::BoundingBox& box = _surface->getAlignedBoundingBox();
    return factor * 0.5*std::max( box.xMax()-box.xMin(), box.yMax()-box.yMin() );
}


bool
TileNode::shouldSubDivide(osg::NodeVisitor& nv, const SelectionInfo& selectionInfo, float lodScale)
{
    unsigned currLOD = _key.getLOD();
    if (   currLOD <  selectionInfo.numLods()
        && currLOD != selectionInfo.numLods()-1)
    {
        osg::Vec3 cameraPos = nv.getViewPoint();

        float radius2 = (float)selectionInfo.visParameters(currLOD+1)._visibilityRange2;
        return _surface->anyChildBoxIntersectsSphere(cameraPos, radius2, lodScale);
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

void
TileNode::cull_stealth(osg::NodeVisitor& nv)
{
    if ( !isDormant(nv.getFrameStamp()) )
    {
        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>( &nv );
        EngineContext* context = static_cast<EngineContext*>( nv.getUserData() );
       
        if ( getNumChildren() == 4 )
        {
            unsigned before = RenderBinUtils::getTotalNumRenderLeaves( cv->getRenderStage() );
            for(int i=0; i<4; ++i)
            {
                _children[i]->accept( nv );
            }
            unsigned after = RenderBinUtils::getTotalNumRenderLeaves( cv->getRenderStage() );
            if ( after == before )
            {
                acceptSurface( cv, context );
            }
        }

        else if ( _surface.valid() )
        {
            acceptSurface( cv, context );
        }
    }
}


void TileNode::cull(osg::NodeVisitor& nv)
{
    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>( &nv );

    EngineContext* context = static_cast<EngineContext*>( nv.getUserData() );
    const SelectionInfo& selectionInfo = context->getSelectionInfo();

    // record the number of drawables before culling this node:
    unsigned before = RenderBinUtils::getTotalNumRenderLeaves( cv->getRenderStage() );

    if ( context->progress() )
        context->progress()->stats()["TileNode::cull"]++;

    // determine whether we can and should subdivide to a higher resolution:
    bool subdivide = shouldSubDivide(nv, selectionInfo, cv->getLODScale());

    // whether it is OK to create child TileNodes is necessary.
    bool canCreateChildren = subdivide;

    // whether it is OK to load data if necessary.
    bool canLoadData = true;


    if ( _dirty && context->getOptions().progressive() == true )
    {
        // Don't create children in progressive mode until content is in place
        canCreateChildren = false;
    }
    
    // If this is an inherit-viewpoint camera, we don't need it to invoke subdivision
    // because we want only the tiles loaded by the true viewpoint.
    const osg::Camera* cam = cv->getCurrentCamera();
    if ( cam && cam->getReferenceFrame() == osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT )
    {
        canCreateChildren = false;
        canLoadData = false;
    }

    optional<bool> surfaceVisible( false );

    if (subdivide)
    {
        // We are in range of the child nodes. Either draw them or load them.

        // If the children don't exist, create them and inherit the parent's data.
        if ( getNumChildren() == 0 && canCreateChildren )
        {
            Threading::ScopedMutexLock exclusive(_mutex);
            if ( getNumChildren() == 0 )
            {
                createChildren( context );
            }
        }

        // If all are ready, traverse them now.
        if ( getNumChildren() == 4 )
        {
            for(int i=0; i<4; ++i)
            {
                // pre-check each child for simple bounding sphere culling, and if the check 
                // fails, unload it's children if them exist. This lets us unload dormant
                // tiles from memory as we go. If those children are visible from another
                // camera, no worries, the unload attempt will fail gracefully.
                if (!cv->isCulled(*_children[i].get()))
                {
                    _children[i]->accept( nv );
                }
                else
                {
                    context->getUnloader()->unloadChildren(getSubTile(i)->getTileKey());
                }
            }
        }

        // If we don't traverse the children, traverse this node's payload.
        else if ( _surface.valid() )
        {
            surfaceVisible = acceptSurface( cv, context );
        }
    }

    // If children are outside camera range, draw the payload and expire the children.
    else if ( _surface.valid() )
    {
        surfaceVisible = acceptSurface( cv, context );

        // if children exists, and are not in the process of loading, unload them now.
        if ( !_dirty && _children.size() > 0 )
        {
            context->getUnloader()->unloadChildren( this->getTileKey() );
        }
    }

    // See whether we actually added any drawables.
    unsigned after = RenderBinUtils::getTotalNumRenderLeaves( cv->getRenderStage() );
    bool addedDrawables = (after > before);
    
    // Only continue if we accepted at least one surface drawable.
    if ( addedDrawables )
    {
        // update the timestamp so this tile doesn't become dormant.
        _lastTraversalFrame.exchange( nv.getFrameStamp()->getFrameNumber() );
    }

    context->invokeTilePatchCallbacks( cv, getTileKey(), _payloadStateSet.get(), _patch.get() );

    // If this tile is marked dirty, try loading data.
    if ( addedDrawables && _dirty && canLoadData )
    {
        // Only load data if the surface would be visible to the camera
        if ( !surfaceVisible.isSet() )
        {
            surfaceVisible = _surface->isVisible(cv);
        }

        if ( surfaceVisible == true )
        {
            load( nv );
        }
        else
        {
            OE_DEBUG << LC << "load skipped for " << _key.str() << std::endl;
        }
    }
}

bool
TileNode::acceptSurface(osgUtil::CullVisitor* cv, EngineContext* context)
{
    bool visible = _surface->isVisible(cv);
    if ( visible )
    {
        if ( context->progress() )
            context->progress()->stats()["TileNode::acceptSurface"]++;

        cv->pushStateSet(context->_surfaceSS.get());

        cv->pushStateSet( _payloadStateSet.get() );
        _surface->accept( *cv );
        cv->popStateSet();

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
        if (VisitorData::isSet(nv, "osgEarth.Stealth"))
        {
            cull_stealth( nv );
        }
        else
        {
            cull(nv);
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
    // Create the four child nodes.
    for(unsigned quadrant=0; quadrant<4; ++quadrant)
    {
        TileNode* node = new TileNode();

        // Build the surface geometry:
        node->create( getTileKey().createChildKey(quadrant), context );

        // Add to the scene graph.
        addChild( node );

        // Inherit the samplers with new scale/bias information.
        node->inheritState( context );
    }
    
    //OE_NOTICE << LC << "Creating children of: " << getTileKey().str() << "; count = " << (++_count) << "\n";
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
    EngineContext* context = static_cast<EngineContext*>( nv.getUserData() );

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

#if 0
    double range0, range1;
    int lod = getTileKey().getLOD();
    if ( lod > context->getOptions().firstLOD().get() )
        range0 = context->getSelectionInfo().visParameters(lod-1)._visibilityRange;
    else
        range0 = 0.0;
    double range1 = context->getSelectionInfo().visParameters(lod)._visibilityRange;

    priority = 
#endif

    // Prioritize by LOD. (negated because lower order gets priority)
    float priority = - (float)getTileKey().getLOD();

    if ( context->getOptions().highResolutionFirst() == true )
    {
        priority = context->getSelectionInfo().numLods() - priority;
        //priority = -priority;
    }

    // then sort by distance within each LOD.
    float distance = nv.getDistanceToViewPoint( getBound().center(), true );
    priority = 10.0f*priority - log10(distance);

    // testing intermediate loading idea...
    //if ( getTileKey().getLOD() == 5 )
    //    priority += 100.0f;

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
    this->removeChildren(0, this->getNumChildren());
}
