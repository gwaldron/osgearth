/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include "SerialKeyNodeFactory"
#include "DynamicLODScaleCallback"
#include "FileLocationCallback"
#include "LODFactorCallback"
#include "CustomPagedLOD"

#include <osgEarth/Registry>
#include <osgEarth/HeightFieldUtils>
#include <osg/PagedLOD>
#include <osg/CullStack>
#include <osg/Uniform>

#include <osgEarth/MapNode>

using namespace osgEarth_engine_quadtree;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[SerialKeyNodeFactory] "


SerialKeyNodeFactory::SerialKeyNodeFactory(TileModelFactory*        modelFactory,
                                           TileModelCompiler*       modelCompiler,
                                           TileNodeRegistry*        liveTiles,
                                           TileNodeRegistry*        deadTiles,
                                           const QuadTreeTerrainEngineOptions& options,
                                           const MapInfo&           mapInfo,
                                           TerrainNode*             terrain,
                                           UID                      engineUID ) :
_modelFactory    ( modelFactory ),
_modelCompiler   ( modelCompiler ),
_liveTiles       ( liveTiles ),
_deadTiles       ( deadTiles ),
_options         ( options ),
_mapInfo         ( mapInfo ),
_terrain         ( terrain ),
_engineUID       ( engineUID )
{
    //nop
}

void
SerialKeyNodeFactory::addTile(TileModel* model, bool tileHasRealData, bool tileHasLodBlending, osg::Group* parent )
{
    // create a node:
    TileNode* tileNode = new TileNode( model->_tileKey, model->_tileLocator );

    // install the tile model and compile it:
    tileNode->setTileModel( model );
    tileNode->compile( _modelCompiler.get() );

    // assemble a URI for this tile's child group:
    std::string uri = Stringify() << model->_tileKey.str() << "." << _engineUID << ".osgearth_engine_quadtree_tile";

    osg::Node* result = 0L;

    // Only add the next tile if all the following are true:
    // 1. Either there's real tile data, or a minLOD is explicity set in the options;
    // 2. The tile isn't blacklisted; and
    // 3. We are still below the maximim LOD.
    bool wrapInPagedLOD =
        (tileHasRealData || (_options.minLOD().isSet() && model->_tileKey.getLOD() < *_options.minLOD())) &&
        !osgEarth::Registry::instance()->isBlacklisted( uri ) &&
        model->_tileKey.getLOD() < *_options.maxLOD();

    if ( wrapInPagedLOD )
    {
        osg::BoundingSphere bs = tileNode->getBound();
      
        float maxRange = FLT_MAX;
        
        //Compute the min range based on the 2D size of the tile
        GeoExtent extent = model->_tileKey.getExtent();
        GeoPoint lowerLeft(extent.getSRS(), extent.xMin(), extent.yMin(), 0.0, ALTMODE_ABSOLUTE);
        GeoPoint upperRight(extent.getSRS(), extent.xMax(), extent.yMax(), 0.0, ALTMODE_ABSOLUTE);
        osg::Vec3d ll, ur;
        lowerLeft.toWorld( ll );
        upperRight.toWorld( ur );
        double radius = (ur - ll).length() / 2.0;
        float minRange = (float)(radius * _options.minTileRangeFactor().value());

        
        // create a PLOD so we can keep subdividing:
        osg::PagedLOD* plod = new CustomPagedLOD( _liveTiles.get(), _deadTiles.get() );
        plod->setCenter( bs.center() );
        plod->addChild( tileNode );
        plod->setRangeMode( *_options.rangeMode() );
        plod->setFileName( 1, uri );
  

        if (plod->getRangeMode() == osg::LOD::PIXEL_SIZE_ON_SCREEN)
        {
            static const float sqrt2 = sqrt(2.0f);

            minRange = 0;
            maxRange = (*_options.tilePixelSize()) * sqrt2;
            plod->setRange( 0, minRange, maxRange  );
            plod->setRange( 1, maxRange, FLT_MAX );            
        }
        else
        {
            plod->setRange( 0, minRange, maxRange );                
            plod->setRange( 1, 0, minRange );        
        }        
                        

        plod->setUserData( new MapNode::TileRangeData(minRange, maxRange) );

#if USE_FILELOCATIONCALLBACK
        osgDB::Options* options = Registry::instance()->cloneOrCreateOptions();
        options->setFileLocationCallback( new FileLocationCallback() );
        plod->setDatabaseOptions( options );

#endif
        result = plod;
        
        if ( tileHasLodBlending )
        {
            // Make the LOD transition distance, and a measure of how
            // close the tile is to an LOD change, to shaders.
            result->addCullCallback(new LODFactorCallback);
        }
    }
    else
    {
        result = tileNode;
    }

    // this cull callback dynamically adjusts the LOD scale based on distance-to-camera:
    if ( _options.lodFallOff().isSet() && *_options.lodFallOff() > 0.0 )
    {
        result->addCullCallback( new DynamicLODScaleCallback(*_options.lodFallOff()) );
    }

    // this one rejects back-facing tiles:
    if ( _mapInfo.isGeocentric() && _options.clusterCulling() == true )
    {
        osg::HeightField* hf =
            model->_elevationData.getHFLayer()->getHeightField();

        result->addCullCallback( HeightFieldUtils::createClusterCullingCallback(
            hf,
            tileNode->getLocator()->getEllipsoidModel(),
            *_options.verticalScale() ) );
    }

    parent->addChild( result );
}

osg::Node*
SerialKeyNodeFactory::createRootNode( const TileKey& key )
{
    osg::ref_ptr<TileModel> model;
    bool                    real;
    bool                    lodBlending;

    _modelFactory->createTileModel( key, model, real, lodBlending );

    // yes, must put the single tile under a tile node group so that it
    // gets registered in the tile node registry
    osg::Group* root = new TileNodeGroup();

    addTile( model.get(), real, lodBlending, root );
    
    return root;
}

osg::Node*
SerialKeyNodeFactory::createNode( const TileKey& parentKey )
{
    osg::ref_ptr<TileModel> models[4];
    bool                   realData[4];
    bool                   lodBlending[4];
    bool                   tileHasAnyRealData = false;

    for( unsigned i = 0; i < 4; ++i )
    {
        TileKey child = parentKey.createChildKey( i );

        _modelFactory->createTileModel( child, models[i], realData[i], lodBlending[i] );

        if ( models[i].valid() && realData[i] )
        {
            tileHasAnyRealData = true;
        }
    }

    osg::Group* root = 0L;

    // assemble the tile.
    if ( tileHasAnyRealData || _options.minLOD().isSet() || parentKey.getLevelOfDetail() == 0 )
    {
        // Now create TileNodes for them and assemble into a tile group.
        root = new TileNodeGroup();

        for( unsigned i = 0; i < 4; ++i )
        {
            if ( models[i].valid() )
            {
                addTile( models[i].get(), realData[i], lodBlending[i], root );
            }
        }
    }

    return root;
}
