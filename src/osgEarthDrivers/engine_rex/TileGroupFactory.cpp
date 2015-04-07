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
#include "TileGroupFactory"
#include "FileLocationCallback"
#include "TilePagedLOD"
#include "TileGroup"
#include "SurfaceNodeFactory"

#include <osgEarth/GeoData>
#include <osgEarth/Registry>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Progress>
#include <osgEarth/Containers>
#include <osgEarth/CullingUtils>
#include <osgEarth/TerrainEngineNode>

#include <osgUtil/CullVisitor>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[TileGroupFactory] "

//..............................................................

namespace
{
    // Cull callback that uses a tight bounding box around the surface node.
    struct SurfaceNodeCullCallback : public osg::NodeCallback
    {
        void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
            if ( !cv->isCulled(static_cast<SurfaceNode*>(node)->getBoundingBox()) )
                traverse(node, nv);
        }
    };

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
#if 1
        m->preMult( scaleBias[quadrant] );
#else
        osg::Matrixf scaleBias;
        scaleBias(0,0) = 0.5f;
        scaleBias(1,1) = 0.5f;
        if ( quadrant == 1 || quadrant == 3 )
            scaleBias(3,0) = 0.5f;
        if ( quadrant == 0 || quadrant == 1 )
            scaleBias(3,1) = 0.5f;

        m->preMult( scaleBias );
#endif
    }
}

//..............................................................


TileGroupFactory::TileGroupFactory(const Map*                     map,
                                   TerrainEngine*                 terrainEngine,
                                   GeometryPool*                  geometryPool,
                                   TileNodeRegistry*              liveTiles,
                                   TileNodeRegistry*              deadTiles,
                                   const RenderBindings&          renderBindings,
                                   const RexTerrainEngineOptions& options) :
_frame         ( map ),
_terrainEngine ( terrainEngine ),
_geometryPool  ( geometryPool ),
_liveTiles     ( liveTiles ),
_deadTiles     ( deadTiles ),
_renderBindings( renderBindings ),
_options       ( options )
{
    //NOP
}

unsigned
TileGroupFactory::getMinimumRequiredLevel()
{
    // highest required level in the map:
    unsigned minLevel = _frame.getHighestMinLevel();

    return _options.minLOD().isSet() ?
        std::max( _options.minLOD().value(), minLevel ) :
        minLevel;
}

TileNode*
TileGroupFactory::createTileNode(TerrainTileModel* model,
                                 ProgressCallback* progress)
{
    TileNode* tileNode = new TileNode(model);
    
    for(TerrainTileImageLayerModelVector::const_iterator i = model->colorLayers().begin();
        i != model->colorLayers().end();
        ++i)
    {
        const TerrainTileLayerModel* layer = i->get();

        if ( layer->getTexture() )
        {
            osg::StateSet* stateSet = tileNode->getOrCreateStateSet();

            stateSet->setTextureAttribute(
                _renderBindings.color().unit(),
                layer->getTexture() );

            // (note: sampler uniform is set at the top level by the engine)
        }

        if ( layer->getMatrix() )
        {
            osg::StateSet* stateSet = tileNode->getOrCreateStateSet();

            stateSet->addUniform( new osg::Uniform(
                _renderBindings.color().matrixName().c_str(),
                *(layer->getMatrix())) );
        }
    }

    if ( model->elevationModel().valid() )
    {
        const TerrainTileElevationModel* em = model->elevationModel();
        
        if ( em->getTexture() )
        {
            osg::StateSet* stateSet = tileNode->getOrCreateStateSet();

            stateSet->setTextureAttribute(
                _renderBindings.elevation().unit(),
                em->getTexture() );

            osg::Matrixf elevMatrix;
            if ( em->getMatrix() )
                elevMatrix = *(em->getMatrix());

            stateSet->addUniform( new osg::Uniform(
                _renderBindings.elevation().matrixName().c_str(),
                elevMatrix ));

            // (note: sampler uniform is set at the top level by the engine)
        }
    }

    if ( model->normalModel().valid() )
    {
        const TerrainTileLayerModel* m = model->normalModel().get();

        if ( m->getTexture() )
        {
            osg::StateSet* stateSet = tileNode->getOrCreateStateSet();

            stateSet->setTextureAttribute(
                _renderBindings.normal().unit(),
                m->getTexture() );

            osg::Matrixf matrix;
            if ( m->getMatrix() )
                matrix = *(m->getMatrix());

            stateSet->addUniform( new osg::Uniform(
                _renderBindings.normal().matrixName().c_str(),
                matrix ));
        }
    }
    
    for(TerrainTileImageLayerModelVector::const_iterator i = model->sharedLayers().begin();
        i != model->sharedLayers().end();
        ++i)
    {
        const TerrainTileImageLayerModel* layerModel = i->get();
        
        const ImageLayer* imageLayer = layerModel->getImageLayer();
        if ( imageLayer )
        {
            if ( layerModel->getTexture() )
            {
                osg::StateSet* stateSet = tileNode->getOrCreateStateSet();

                stateSet->setTextureAttribute(
                    imageLayer->shareImageUnit().get(),
                    layerModel->getTexture() );
                
                //TODO: don't really need this if we set it up in the Engine 
                //  once at the top level when adding the layer.
                stateSet->addUniform( new osg::Uniform(
                    imageLayer->shareSamplerName()->c_str(),
                    imageLayer->shareImageUnit().get() ));
            }

            if ( layerModel->getMatrix() )
            {
                osg::StateSet* stateSet = tileNode->getOrCreateStateSet();

                stateSet->addUniform( new osg::Uniform(
                    imageLayer->shareMatrixName()->c_str(),
                    *(layerModel->getMatrix())) );
            }
        }
    }

    //get parent tile data for lod blending
    osg::ref_ptr<const TerrainTileModel> parentModel;
    TileKey parentKey = model->getKey().createParentKey();

    if ( _liveTiles->get(parentKey, parentModel) )
    {
        for(TerrainTileImageLayerModelVector::const_iterator i = parentModel->colorLayers().begin();
            i != parentModel->colorLayers().end();
            ++i)
        {
            const TerrainTileLayerModel* layer = i->get();

            if ( layer->getTexture() )
            {
                osg::StateSet* stateSet = tileNode->getOrCreateStateSet();

                stateSet->setTextureAttribute(
                    _renderBindings.parentColor().unit(),
                    layer->getTexture() );

                // (note: sampler uniform is set at the top level by the engine)
            }

            if ( layer->getMatrix() )
            {
                osg::StateSet* stateSet = tileNode->getOrCreateStateSet();

                osg::Matrixf parentTexMatrix;
                if ( layer->getMatrix() )
                    parentTexMatrix = *(osg::clone(layer->getMatrix()));

                applyScaleBias( &parentTexMatrix, model->getKey().getQuadrant() );

                stateSet->addUniform( new osg::Uniform(
                    _renderBindings.parentColor().matrixName().c_str(),
                    parentTexMatrix) );
            }
        }

        if ( parentModel->elevationModel() )
        {
            const TerrainTileElevationModel* parentEM = parentModel->elevationModel();

            if (parentEM->getTexture())
            {
                osg::StateSet* stateSet = tileNode->getOrCreateStateSet();

                stateSet->setTextureAttribute(
                    _renderBindings.parentElevation().unit(),
                    parentEM->getTexture() );

                osg::Matrixf parentElevMatrix;
                if ( parentEM->getMatrix() )
                    parentElevMatrix = *(osg::clone(parentEM->getMatrix()));

                applyScaleBias( &parentElevMatrix, model->getKey().getQuadrant() );

                stateSet->addUniform( new osg::Uniform(
                    _renderBindings.parentElevation().matrixName().c_str(),
                    parentElevMatrix ));
            }
        }
    }

    return tileNode;
}

TileNode*
TileGroupFactory::createTileNodeGraph(TerrainTileModel* model,
                                      bool              setupChildrenIfNecessary,
                                      ProgressCallback* progress)
{
    // TODO: fix this
    const unsigned tileSize = 17;

    // Build the surface node.
    SurfaceNodeFactory factory(model, _frame, _renderBindings, _geometryPool);
    SurfaceNode* surfaceNode = factory.createSurfaceNode();

    surfaceNode->setEngineUID( _terrainEngine->getUID() );

    // see if this tile might have children.
    bool prepareForChildren =
        setupChildrenIfNecessary &&
        model->getKey().getLOD() < *_options.maxLOD();
    
    // Build the Tile Node that will hold all the textures and texture matrices.
    TileNode* tileNode = createTileNode(
        model,
        progress );

    // Build the paging node that will load subtiles, if necessary:
    if ( prepareForChildren )
    {
        osg::BoundingSphere bs = surfaceNode->getBound();
        TilePagedLOD* plod = new TilePagedLOD( _terrainEngine->getUID(), _liveTiles, _deadTiles );
        plod->setCenter  ( bs.center() );
        plod->addChild   ( surfaceNode );
        plod->setFileName( 1, Stringify() 
            << model->getKey().str()
            << "." << _terrainEngine->getUID()
            << ".osgearth_engine_rex_tile" );

        if ( _options.rangeMode().value() == osg::LOD::DISTANCE_FROM_EYE_POINT )
        {
            // Compute the world coordinates of the SE and NE corners of the tile:
            GeoExtent extent = model->getKey().getExtent();
            GeoPoint lowerLeft(extent.getSRS(), extent.xMin(), extent.yMin(), 0.0, ALTMODE_ABSOLUTE);
            GeoPoint upperRight(extent.getSRS(), extent.xMax(), extent.yMax(), 0.0, ALTMODE_ABSOLUTE);
            osg::Vec3d ll, ur;
            lowerLeft.toWorld( ll );
            upperRight.toWorld( ur );

            //Compute the min range based on the 2D size of the tile
            double radius = (ur - ll).length() / 2.0;
            float minRange = (float)(radius * _options.minTileRangeFactor().value());

            plod->setRange( 0, minRange, FLT_MAX );
            plod->setRange( 1, 0, minRange );
            plod->setRangeMode( osg::LOD::DISTANCE_FROM_EYE_POINT );
        }
        else
        {
            plod->setRange( 0, 0.0f, _options.tilePixelSize().value() );
            plod->setRange( 1, _options.tilePixelSize().value(), FLT_MAX );
            plod->setRangeMode( osg::LOD::PIXEL_SIZE_ON_SCREEN );
        }
        
        // Install a tile-aligned bounding box in the pager node itself so we can do
        // visibility testing before paging in subtiles.
        // TODO: consider making this optional because it might be useful to pre-fetch
        // a higher LOD than we need in order to facilitate better LOD blending.
        plod->setChildBoundingBoxAndMatrix(
            1,
            surfaceNode->getBoundingBox(),
            surfaceNode->getMatrix() );

#if USE_FILELOCATIONCALLBACK
        osgDB::Options* options = plod->getOrCreateDBOptions();
        options->setFileLocationCallback( new FileLocationCallback() );
#endif
        
        tileNode->addChild( plod );

        // Install a callback to reject back-facing tiles.
        if ( _frame.getMapInfo().isGeocentric() && _options.clusterCulling() == true )
        {
            const osg::HeightField* heightField = model->elevationModel()->getHeightField();
            if ( heightField )
            {
                tileNode->addCullCallback( HeightFieldUtils::createClusterCullingCallback(
                    heightField,
                    tileNode->getKey().getProfile()->getSRS()->getEllipsoid(),
                    *_options.verticalScale() ) );
            }
            else
            {
                // no heightfield; generate a CCC just using the extent.
                tileNode->addCullCallback( ClusterCullingFactory::create(model->getKey().getExtent()) );
            }
        }
    }
    else
    {
        tileNode->addChild( surfaceNode );
    }

    return tileNode;
}


osg::Node*
TileGroupFactory::createTileGroup(const TileKey&    parentKey, 
                                  bool              accumulate,
                                  bool              setupChildren,
                                  ProgressCallback* progress )
{
    if ( progress && progress->isCanceled() )
        return 0L;

    _frame.sync();
    
    OE_START_TIMER(create_model);

    osg::ref_ptr<TerrainTileModel> model[4];
    for(unsigned q=0; q<4; ++q)
    {
        if ( progress && progress->isCanceled() )
            return 0L;
        
        TileKey quadrantKey = parentKey.createChildKey(q);

        // assemble a new tile model for this child.
        model[q] = _terrainEngine->createTileModel(
            _frame,
            quadrantKey,
            _liveTiles.get(),
            progress );

        // if any one of the TileModel creations fail, we will be unable to build
        // this quadtile. So goodbye.
        if ( !model[q].valid() )
        {
            OE_DEBUG << LC << "Bailed on key " << parentKey.str() << " due to a NULL quadrant model." << std::endl;
            return 0L;
        }
    }

    if (progress)
        progress->stats()["create_tilemodel_time"] += OE_STOP_TIMER(create_model);

    // See whether we can make a new tile family from the four models.
    bool makeGroup;

    // If this is a request for a root tile, make it no matter what.
    if ( parentKey.getLOD() == 0 || (parentKey.getLOD()-1) == _options.firstLOD().value() )
    {
        makeGroup = true;
    }

    // If there's a minimum LOD set, and we haven't reached it yet, make the tile.
    else if ( parentKey.getLOD() <= getMinimumRequiredLevel() )
    {
        makeGroup = true;
    }

    // Otherwise, only make the tile if at least one quadrant has REAL data
    // (not fallback data).
    else
    {
        makeGroup = false;
        for(unsigned q=0; q<4; ++q)
        {
            if ( model[q]->containsNewData() )
            {
                makeGroup = true;
                break;
            }
        }
    }
    
    if ( progress && progress->isCanceled() )
        return 0L;

    OE_START_TIMER(compile_tile);

    osg::ref_ptr<TileGroup> tileGroup;

    if ( makeGroup )
    {
        tileGroup = new TileGroup();

        for( unsigned q=0; q<4; ++q )
        {
            // create the new node:
            osg::ref_ptr<TileNode> tileGraph = createTileNodeGraph(
                model[q].get(),
                setupChildren,
                progress);

            // notify the engine so it can fire callbacks, etc.
            _terrainEngine->notifyOfTerrainTileNodeCreation(
                model[q]->getKey(),
                tileGraph.get() );

            // welcome to the family.
            tileGroup->addChild( tileGraph.get() );
        }
    }

    if (progress)
        progress->stats()["compile_tilemodel_time"] += OE_STOP_TIMER(compile_tile);

    return tileGroup.release();
}
