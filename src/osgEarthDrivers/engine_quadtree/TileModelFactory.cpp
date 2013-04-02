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
#include "TileModelFactory"
#include <osgEarth/MapFrame>
#include <osgEarth/MapInfo>
#include <osgEarth/ImageUtils>
#include <osgEarth/HeightFieldUtils>

using namespace osgEarth_engine_quadtree;
using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace OpenThreads;

#define LC "[TileModelFactory] "

//------------------------------------------------------------------------

namespace
{
    struct BuildColorData
    {
        void init( const TileKey&                      key, 
                   ImageLayer*                         layer, 
                   const MapInfo&                      mapInfo,
                   const QuadTreeTerrainEngineOptions& opt, 
                   TileModel*                          model )
        {
            _key      = key;
            _layer    = layer;
            _mapInfo  = &mapInfo;
            _opt      = &opt;
            _model    = model;
            //_repo     = &repo;
        }

        void execute()
        {
            GeoImage geoImage;
            bool isFallbackData = false;

            bool useMercatorFastPath =
                _opt->enableMercatorFastPath() != false &&
                _mapInfo->isGeocentric()                &&
                _layer->getProfile()                    &&
                _layer->getProfile()->getSRS()->isSphericalMercator();

            // fetch the image from the layer, falling back on parent keys utils we are 
            // able to find one that works.

            bool autoFallback = _key.getLevelOfDetail() <= 1;

            TileKey imageKey( _key );
            TileSource*    tileSource   = _layer->getTileSource();
            const Profile* layerProfile = _layer->getProfile();

            //Only try to get data from the source if it actually intersects the key extent
            bool hasDataInExtent = true;
            if (tileSource && layerProfile)
            {
                GeoExtent ext = _key.getExtent();
                if (!layerProfile->getSRS()->isEquivalentTo( ext.getSRS()))
                {
                    ext = layerProfile->clampAndTransformExtent( ext );
                }
                hasDataInExtent = tileSource->hasDataInExtent( ext );
            }
            
            if (hasDataInExtent)
            {
                while( !geoImage.valid() && imageKey.valid() && _layer->isKeyValid(imageKey) )
                {
                    if ( useMercatorFastPath )
                    {
                        bool mercFallbackData = false;
                        geoImage = _layer->createImageInNativeProfile( imageKey, 0L, autoFallback, mercFallbackData );
                        if ( geoImage.valid() && mercFallbackData )
                        {
                            isFallbackData = true;
                        }
                    }
                    else
                    {
                        geoImage = _layer->createImage( imageKey, 0L, autoFallback );
                    }

                    if ( !geoImage.valid() )
                    {
                        imageKey = imageKey.createParentKey();
                        isFallbackData = true;
                    }
                }
            }

            GeoLocator* locator = 0L;

            if ( !geoImage.valid() )
            {
                // no image found, so make an empty one (one pixel alpha).
                geoImage = GeoImage( ImageUtils::createEmptyImage(), _key.getExtent() );
                locator = GeoLocator::createForKey( _key, *_mapInfo );
                isFallbackData = true;
            }
            else
            {
                if ( useMercatorFastPath )
                    locator = new MercatorLocator(geoImage.getExtent());
                else
                    locator = GeoLocator::createForExtent(geoImage.getExtent(), *_mapInfo);
            }

            // add the color layer to the repo.
            _model->_colorData[_layer->getUID()] = TileModel::ColorData(
                _layer,
                geoImage.getImage(),
                locator,
                _key.getLevelOfDetail(),
                _key,
                isFallbackData );
        }

        TileKey        _key;
        const MapInfo* _mapInfo;
        ImageLayer*    _layer;
        TileModel*     _model;
        const QuadTreeTerrainEngineOptions* _opt;
    };
}

//------------------------------------------------------------------------

namespace
{
    struct BuildElevationData
    {
        void init(const TileKey& key, const MapFrame& mapf, const QuadTreeTerrainEngineOptions& opt, TileModel* model, HeightFieldCache* hfCache) //sourceTileNodeBuilder::SourceRepo& repo)
        {
            _key   = key;
            _mapf  = &mapf;
            _opt   = &opt;
            _model = model;
            _hfCache = hfCache;
            //_repo = &repo;
        }

        void execute()
        {            
            const MapInfo& mapInfo = _mapf->getMapInfo();

            // Request a heightfield from the map, falling back on lower resolution tiles
            // if necessary (fallback=true)
            osg::ref_ptr<osg::HeightField> hf;
            bool isFallback = false;

            //if ( _mapf->getHeightField( _key, true, hf, &isFallback ) )
            if (_hfCache->getOrCreateHeightField( *_mapf, _key, true, hf, &isFallback) )
            {                

                // Put it in the repo
                osgTerrain::HeightFieldLayer* hfLayer = new osgTerrain::HeightFieldLayer( hf.get() );

                // Generate a locator.
                hfLayer->setLocator( GeoLocator::createForKey( _key, mapInfo ) );

                _model->_elevationData = TileModel::ElevationData(hfLayer, isFallback);
                
                if ( *_opt->normalizeEdges() )
                {
                    // next, query the neighboring tiles to get adjacency information.
                    for( int x=-1; x<=1; x++ )
                    {
                        for( int y=-1; y<=1; y++ )
                        {
                            if ( x != 0 || y != 0 )
                            {
                                TileKey nk = _key.createNeighborKey(x, y);
                                if ( nk.valid() )
                                {
                                    if (_hfCache->getOrCreateHeightField( *_mapf, nk, true, hf, &isFallback) )
                                    //if ( _mapf->getHeightField(nk, true, hf, &isFallback) )
                                    {               
                                        if ( mapInfo.isPlateCarre() )
                                        {
                                            HeightFieldUtils::scaleHeightFieldToDegrees( hf.get() );
                                        }

                                        _model->_elevationData.setNeighbor( x, y, hf.get() );
                                    }
                                }
                            }
                        }
                    }
                }

#if 0
                if ( *_opt->morphLODs() )
                {
                    // find the parent tile as well, for LOD morphing!!
                    if ( _key.getLevelOfDetail() > 0 )
                    {
                        if (_hfCache->getOrCreateHeightField( *_mapf, _key.createParentKey(), true, hf, &isFallback ))
                        {       
                            if ( mapInfo.isPlateCarre() )
                                HeightFieldUtils::scaleHeightFieldToDegrees( hf.get() );

                            _model->_elevationData.setParent( hf.get() );
                        }
                    }
                }
#endif
            }
        }

        TileKey                  _key;
        const MapFrame*          _mapf;
        const QuadTreeTerrainEngineOptions* _opt;
        TileModel* _model;
        osg::ref_ptr< HeightFieldCache> _hfCache;
    };
}

//------------------------------------------------------------------------

TileModelFactory::TileModelFactory(const Map*                          map, 
                                   TileNodeRegistry*                   liveTiles,
                                   const QuadTreeTerrainEngineOptions& terrainOptions ) :
_map           ( map ),
_liveTiles     ( liveTiles ),
_terrainOptions( terrainOptions )
{
    _hfCache = new HeightFieldCache();
}

HeightFieldCache*
TileModelFactory::getHeightFieldCache() const
{
    return _hfCache;
}


void
TileModelFactory::createTileModel(const TileKey&           key, 
                                  osg::ref_ptr<TileModel>& out_model,
                                  bool&                    out_hasRealData,
                                  bool&                    out_hasLodBlendedLayers )
{
    MapFrame mapf( _map, Map::MASKED_TERRAIN_LAYERS );
    
    const MapInfo& mapInfo = mapf.getMapInfo();

    osg::ref_ptr<TileModel> model = new TileModel();
    model->_tileKey = key;
    model->_tileLocator = GeoLocator::createForKey(key, mapInfo);

    // init this to false, then search for real data. "Real data" is data corresponding
    // directly to the key, as opposed to fallback data, which is derived from a lower
    // LOD key.
    out_hasRealData = false;
    out_hasLodBlendedLayers = false;
    
    // Fetch the image data and make color layers.
    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
    {
        ImageLayer* layer = i->get();

        if ( layer->getEnabled() )
        {
            BuildColorData build;
            build.init( key, layer, mapInfo, _terrainOptions, model.get() );
            build.execute();

            if ( layer->getImageLayerOptions().lodBlending() == true )
            {
                out_hasLodBlendedLayers = true;
            }
        }
    }

    // make an elevation layer.
    BuildElevationData build;
    build.init( key, mapf, _terrainOptions, model.get(), _hfCache );
    build.execute();


    // Bail out now if there's no data to be had.
    if ( model->_colorData.size() == 0 && !model->_elevationData.getHFLayer() )
    {
        return;
    }

    // OK we are making a tile, so if there's no heightfield yet, make an empty one.
    if ( !model->_elevationData.getHFLayer() )
    {
        osg::HeightField* hf = HeightFieldUtils::createReferenceHeightField( key.getExtent(), 8, 8 );
        osgTerrain::HeightFieldLayer* hfLayer = new osgTerrain::HeightFieldLayer( hf );
        hfLayer->setLocator( GeoLocator::createForKey(key, mapInfo) );
        model->_elevationData = TileModel::ElevationData( hfLayer, true );
    }

    // Now, if there are any color layers that did not get built, create them with an empty
    // image so the shaders have something to draw.
    osg::ref_ptr<osg::Image> emptyImage;
    osgTerrain::Locator* locator = model->_elevationData.getHFLayer()->getLocator();

    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
    {
        ImageLayer* layer = i->get();

        if ( layer->getEnabled() && !layer->isKeyValid(key) )
        {
            if ( !emptyImage.valid() )
                emptyImage = ImageUtils::createEmptyImage();

            model->_colorData[i->get()->getUID()] = TileModel::ColorData(
                layer,
                emptyImage.get(),
                locator,
                key.getLevelOfDetail(),
                key,
                true );
        }
    }

    // Ready to create the actual tile.
    //AssembleTile assemble;
    //assemble.init( key, mapInfo, _terrainOptions, model.get(), mapf.terrainMaskLayers() );
    //assemble.execute();

    // if we're using LOD blending, find and add the parent's state set.
    if ( out_hasLodBlendedLayers && key.getLevelOfDetail() > 0 && _liveTiles.valid() )
    {
        osg::ref_ptr<TileNode> parent;
        if ( _liveTiles->get( key.createParentKey(), parent ) )
        {
            model->_parentStateSet = parent->getPublicStateSet();
        }
    }

    if (!out_hasRealData)
    {
        // Check the results and see if we have any real data.
        for( TileModel::ColorDataByUID::const_iterator i = model->_colorData.begin(); i != model->_colorData.end(); ++i )
        {
            if ( !i->second.isFallbackData() ) 
            {
                out_hasRealData = true;
                break;
            }
        }
    }

    if ( !out_hasRealData && !model->_elevationData.isFallbackData() )
    {
        out_hasRealData = true;
    }

    out_model = model.release();
    //out_tile = assemble._node;
}
