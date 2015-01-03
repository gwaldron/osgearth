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
#include <osgEarth/TerrainEngineNode>

#include <osgUtil/CullVisitor>

using namespace osgEarth::Drivers::MPTerrainEngine;
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
}

//..............................................................


TileGroupFactory::TileGroupFactory(const Map*                    map,
                                   TerrainEngine*                terrainEngine,
                                   GeometryPool*                 geometryPool,
                                   TileNodeRegistry*             liveTiles,
                                   TileNodeRegistry*             deadTiles,
                                   const RenderBindings&         renderBindings,
                                   const MPTerrainEngineOptions& options) :
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

    if ( model->elevationModel() )
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
    SurfaceNodeFactory factory(model, _frame, _renderBindings, _geometryPool, tileSize, _options);
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
            << ".osgearth_engine_mp_tile" );

        if ( _options.rangeMode().value() == osg::LOD::DISTANCE_FROM_EYE_POINT )
        {
            //Compute the min range based on the 2D size of the tile
            GeoExtent extent = model->getKey().getExtent();
            GeoPoint lowerLeft(extent.getSRS(), extent.xMin(), extent.yMin(), 0.0, ALTMODE_ABSOLUTE);
            GeoPoint upperRight(extent.getSRS(), extent.xMax(), extent.yMax(), 0.0, ALTMODE_ABSOLUTE);
            osg::Vec3d ll, ur;
            lowerLeft.toWorld( ll );
            upperRight.toWorld( ur );
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
        
#if 0 // TODO: reinstate this!

        // Install a tile-aligned bounding box in the pager node itself so we can do
        // visibility testing before paging in subtiles.
        plod->setChildBoundingBoxAndMatrix(
            1,
            surfaceNode->getTerrainBoundingBox(),
            surfaceNode->getLocalToWorldMatrix() );
#endif

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
