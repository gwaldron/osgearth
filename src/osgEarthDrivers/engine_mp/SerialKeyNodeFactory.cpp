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
#include "TilePagedLOD"

#include <osgEarth/Registry>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Progress>
#include <osg/PagedLOD>
#include <osg/CullStack>
#include <osg/Uniform>

#include <osgEarth/MapNode>

using namespace osgEarth_engine_mp;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[SerialKeyNodeFactory] "

namespace
{
    struct MyProgressCallback : public osgEarth::ProgressCallback
    {
        osg::observer_ptr<osg::PagedLOD> _plod;

        MyProgressCallback( osg::PagedLOD* plod )
            : _plod(plod) { }

        bool isCanceled() const
        {
            if ( _canceled )
                return true;

            if ( !_plod.valid() )
            {
                _canceled = true;
                OE_INFO << "CANCEL, plod = null." << std::endl;
            }
            else
            {
                osg::ref_ptr<osg::PagedLOD> plod = _plod.get();
                if ( !plod.valid() )
                {
                    _canceled = true;
                    OE_INFO << "CANCEL, plod = null." << std::endl;
                }
                else
                {
                    osg::ref_ptr<osg::Referenced> dbr = plod->getDatabaseRequest( 1 );
                    if ( !dbr.valid() || dbr->referenceCount() < 2 )
                    {
                        _canceled = true;
                        OE_INFO << "CANCEL, REFCOUNT = " << dbr->referenceCount() << std::endl;
                    }
                }
            }

            return _canceled;
        }
    };
}


SerialKeyNodeFactory::SerialKeyNodeFactory(TileModelFactory*        modelFactory,
                                           TileModelCompiler*       modelCompiler,
                                           TileNodeRegistry*        liveTiles,
                                           TileNodeRegistry*        deadTiles,
                                           const MPTerrainEngineOptions& options,
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


osg::Node*
SerialKeyNodeFactory::createTile(TileModel* model,
                                 bool       tileHasRealData, 
                                 bool       tileHasLodBlending)
{
    // see if the parent model is available.
    osg::ref_ptr<const TileModel> parentModel;
    if ( model->_tileKey.getLOD() > 0 )
    {
        osg::ref_ptr<TileNode> parentTile;
        if (_liveTiles->get(model->_tileKey.createParentKey(), parentTile))
            parentModel = parentTile->getTileModel();
    }

    // compile the model into a node:
    TileNode* tileNode = _modelCompiler->compile( model, parentModel.get() );

    // see if this tile might have children.
    bool prepareForChildren =
        (tileHasRealData || (_options.minLOD().isSet() && model->_tileKey.getLOD() < *_options.minLOD())) &&
        model->_tileKey.getLOD() < *_options.maxLOD();

    osg::Node* result = 0L;

    if ( prepareForChildren )
    {
        //Compute the min range based on the 2D size of the tile
        osg::BoundingSphere bs = tileNode->getBound();
        GeoExtent extent = model->_tileKey.getExtent();
        GeoPoint lowerLeft(extent.getSRS(), extent.xMin(), extent.yMin(), 0.0, ALTMODE_ABSOLUTE);
        GeoPoint upperRight(extent.getSRS(), extent.xMax(), extent.yMax(), 0.0, ALTMODE_ABSOLUTE);
        osg::Vec3d ll, ur;
        lowerLeft.toWorld( ll );
        upperRight.toWorld( ur );
        double radius = (ur - ll).length() / 2.0;
        float minRange = (float)(radius * _options.minTileRangeFactor().value());

        osgDB::Options* dbOptions = Registry::instance()->cloneOrCreateOptions();

        TileGroup* plod = new TileGroup(tileNode, _engineUID, _liveTiles.get(), _deadTiles.get(), dbOptions);
        plod->setSubtileRange( minRange );

#if 0
        // custom PLOD that holds the tile:
        TilePagedLOD* plod = new TilePagedLOD(
            tileNode,
            _engineUID,
            _liveTiles.get(), 
            _deadTiles.get() );

        //Compute the min range based on the 2D size of the tile
        osg::BoundingSphere bs = tileNode->getBound();
        GeoExtent extent = model->_tileKey.getExtent();
        GeoPoint lowerLeft(extent.getSRS(), extent.xMin(), extent.yMin(), 0.0, ALTMODE_ABSOLUTE);
        GeoPoint upperRight(extent.getSRS(), extent.xMax(), extent.yMax(), 0.0, ALTMODE_ABSOLUTE);
        osg::Vec3d ll, ur;
        lowerLeft.toWorld( ll );
        upperRight.toWorld( ur );

        double radius = (ur - ll).length() / 2.0;

        float maxRange = FLT_MAX;
        float minRange = (float)(radius * _options.minTileRangeFactor().value());

        plod->setCenter( bs.center() );
        //plod->addChild( tileNode );
        plod->setRangeMode( *_options.rangeMode() );
    //    plod->setFileName( 1, uri );
#endif

#if 0
        if (plod->getRangeMode() == osg::LOD::PIXEL_SIZE_ON_SCREEN)
        {
            static const float sqrt2 = sqrt(2.0f);
            minRange = 0;
            maxRange = (*_options.tilePixelSize()) * sqrt2;
            //plod->setRange( 0, minRange, maxRange  );
            //plod->setRange( 1, maxRange, FLT_MAX );
        }
        else
        {
        }
        plod->setRange( 0, 0, FLT_MAX );

        plod->setMinRange( minRange );
#endif

        //for( unsigned q=0; q<4; ++q)
        //{
        //    TileKey childKey = model->_tileKey.createChildKey(q);

        //    std::string uri = Stringify() 
        //        << childKey.str() 
        //        << "." 
        //        << _engineUID 
        //        << ".osgearth_engine_mp_tile";

        //    plod->setFileName( 1+q, uri );
        //    plod->setRange   ( 1+q, 0, minRange );
        //}
        
        //osgDB::Options* dbOptions = Registry::instance()->cloneOrCreateOptions();
        //dbOptions->setUserData( new MyProgressCallback(plod) );
        //plod->setDatabaseOptions( dbOptions );

#if USE_FILELOCATIONCALLBACK
        dbOptions->setFileLocationCallback( new FileLocationCallback() );
#endif
        
        result = plod;
    

    //    if ( tileHasLodBlending )
    //    {
    //        // Make the LOD transition distance, and a measure of how
    //        // close the tile is to an LOD change, to shaders.
    //        result->addCullCallback(new LODFactorCallback);
    //    }
    }
    else
    {
        result = tileNode;
    }

    //// this cull callback dynamically adjusts the LOD scale based on distance-to-camera:
    //if ( _options.lodFallOff().isSet() && *_options.lodFallOff() > 0.0 )
    //{
    //    result->addCullCallback( new DynamicLODScaleCallback(*_options.lodFallOff()) );
    //}


    // this one rejects back-facing tiles:
    if ( _mapInfo.isGeocentric() && _options.clusterCulling() == true )
    {
        osg::HeightField* hf =
            model->_elevationData.getHeightField(); //.getHFLayer()->getHeightField();

        result->addCullCallback( HeightFieldUtils::createClusterCullingCallback(
            hf,
            tileNode->getKey().getProfile()->getSRS()->getEllipsoid(),
            *_options.verticalScale() ) );
    }

    return result;
}


osg::Node*
SerialKeyNodeFactory::createRootNode( const TileKey& key )
{
    osg::ref_ptr<TileModel> model;
    bool                    real;
    bool                    lodBlending;

    _modelFactory->createTileModel( key, model, real, lodBlending );
    return createTile( model.get(), real, lodBlending );
}


osg::Node*
SerialKeyNodeFactory::createNode( const TileKey& key, ProgressCallback* progress )
{
    osg::ref_ptr<TileModel> model;
    bool                    isReal;
    bool                    hasBlending;

    if ( progress && progress->isCanceled() )
        return 0L;

    _modelFactory->createTileModel(key, model, isReal, hasBlending);

    if ( progress && progress->isCanceled() )
        return 0L;

    if ( isReal || _options.minLOD().isSet() || key.getLOD() == 0 )
    {
        return createTile( model.get(), isReal, hasBlending );
    }
    else
    {
        return 0L;
    }
}
