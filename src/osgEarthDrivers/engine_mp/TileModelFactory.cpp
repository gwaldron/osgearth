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
#include <osgEarth/Progress>

using namespace osgEarth::Drivers::MPTerrainEngine;
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
                   unsigned                            order,
                   const MapInfo&                      mapInfo,
                   const MPTerrainEngineOptions&       opt, 
                   TileNodeRegistry*                   tiles,
                   TileModel*                          model)
        {
            _key      = key;
            _layer    = layer;
            _order    = order;
            _mapInfo  = &mapInfo;
            _opt      = &opt;
            _tiles    = tiles;
            _model    = model;
        }

        bool execute(ProgressCallback* progress)
        {
            GeoImage geoImage;
            bool isFallbackData = false;

            bool useMercatorFastPath =
                _opt->enableMercatorFastPath() != false &&
                _mapInfo->isGeocentric()                &&
                _layer->getProfile()                    &&
                _layer->getProfile()->getSRS()->isSphericalMercator();

            // fetch the image from the layer.

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
            
            if (hasDataInExtent && _layer->isKeyValid(_key))
            {
                if ( useMercatorFastPath )
                {
                    geoImage = _layer->createImageInNativeProfile( _key, progress );
                }
                else
                {
                    geoImage = _layer->createImage( _key, progress );
                }
            }

            if ( geoImage.valid() )
            {
                GeoLocator* locator = 0L;
                
                if ( useMercatorFastPath )
                    locator = new MercatorLocator(geoImage.getExtent());
                else
                    locator = GeoLocator::createForExtent(geoImage.getExtent(), *_mapInfo);

                // convert the image to PMA. This must be done in the CPU; for some
                // reason (which we could not determine) it fails to try this after the
                // texture lookup in the shader.
                if ( _opt->premultipliedAlpha() == true )
                {
                    ImageUtils::convertToPremultipliedAlpha( geoImage.getImage() );
                }

                // add the color layer to the repo.
                _model->_colorData[_layer->getUID()] = TileModel::ColorData(
                    _layer,
                    _order,
                    geoImage.getImage(),
                    locator,
                    isFallbackData );

                return true;
            }

            else // fall back on parent tile.
            {
                TileKey parentKey = _key.createParentKey();
                osg::ref_ptr<TileNode> parentNode;
                _tiles->get(parentKey, parentNode);
                if ( parentNode.valid() )
                {
                    const TileModel* parentModel = parentNode->getTileModel();
                    if ( parentModel )
                    {
                        TileModel::ColorData parentColorData;
                        if ( parentModel->getColorData(_layer->getUID(), parentColorData) )
                        {
                            TileModel::ColorData& colorData = _model->_colorData[_layer->getUID()];
                            colorData = TileModel::ColorData(parentColorData);
                            colorData.setIsFallbackData( true );
                            return true;
                        }
                    }
                }
            }

            return false;
        }

        TileKey        _key;
        const MapInfo* _mapInfo;
        TileNodeRegistry* _tiles;
        ImageLayer*    _layer;
        unsigned       _order;
        TileModel*     _model;
        const MPTerrainEngineOptions* _opt;
    };
}

//------------------------------------------------------------------------

namespace
{
    struct BuildElevationData
    {
        void init(const TileKey& key, const MapFrame& mapf, const MPTerrainEngineOptions& opt, TileModel* model, HeightFieldCache* hfCache) //sourceTileNodeBuilder::SourceRepo& repo)
        {
            _key   = key;
            _mapf  = &mapf;
            _opt   = &opt;
            _model = model;
            _hfCache = hfCache;
        }

        void execute(ProgressCallback* progress)
        {            
            const MapInfo& mapInfo = _mapf->getMapInfo();

            // Request a heightfield from the map, falling back on lower resolution tiles
            // if necessary (fallback=true)
            osg::ref_ptr<osg::HeightField> hf;
            bool isFallback = false;

            if (_hfCache->getOrCreateHeightField( *_mapf, _key, true, hf, &isFallback, true, SAMPLE_FIRST_VALID, progress))
            {
                _model->_elevationData = TileModel::ElevationData(
                    hf,
                    GeoLocator::createForKey( _key, mapInfo ),
                    isFallback );

                if ( _opt->normalizeEdges().value() == true )
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
                                    if (_hfCache->getOrCreateHeightField( *_mapf, nk, true, hf, &isFallback, true, SAMPLE_FIRST_VALID, progress) )
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

                    // parent too.
                    if ( _key.getLOD() > 0 )
                    {
                        if ( _hfCache->getOrCreateHeightField( *_mapf, _key.createParentKey(), true, hf, &isFallback, true, SAMPLE_FIRST_VALID, progress) )
                        {
                            if ( mapInfo.isPlateCarre() )
                            {
                                HeightFieldUtils::scaleHeightFieldToDegrees( hf.get() );
                            }

                            _model->_elevationData.setParent( hf.get() );
                        }
                    }
                }
            }
        }

        TileKey                  _key;
        const MapFrame*          _mapf;
        const MPTerrainEngineOptions* _opt;
        TileModel* _model;
        osg::ref_ptr< HeightFieldCache> _hfCache;
    };
}

//------------------------------------------------------------------------

TileModelFactory::TileModelFactory(//const Map*                          map, 
                                   TileNodeRegistry*                   liveTiles,
                                   const MPTerrainEngineOptions& terrainOptions ) :
//_map           ( map ),
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
                                  const MapFrame&          frame,
                                  osg::ref_ptr<TileModel>& out_model,
                                  ProgressCallback*        progress)
{

    osg::ref_ptr<TileModel> model = new TileModel( frame.getRevision(), frame.getMapInfo() );
    model->_tileKey = key;
    model->_tileLocator = GeoLocator::createForKey(key, frame.getMapInfo());

    OE_START_TIMER(fetch_imagery);

    // Fetch the image data and make color layers.
    unsigned index = 0;
    unsigned order = 0;
    for( ImageLayerVector::const_iterator i = frame.imageLayers().begin(); i != frame.imageLayers().end(); ++i )
    {
        ImageLayer* layer = i->get();

        if ( layer->getEnabled() )
        {
            BuildColorData build;
            build.init( key, layer, order, frame.getMapInfo(), _terrainOptions, _liveTiles.get(), model.get() );

            bool addedToModel = build.execute(progress);
            if ( addedToModel )
            {
                // only bump the order if we added something to the data model.
                order++;
            }
        }
    }

    if (progress)
        progress->stats()["fetch_imagery_time"] += OE_STOP_TIMER(fetch_imagery);

    
    OE_START_TIMER(fetch_elevation);

    // make an elevation layer.
    BuildElevationData build;
    build.init( key, frame, _terrainOptions, model.get(), _hfCache );
    build.execute(progress);

    if (progress)
        progress->stats()["fetch_elevation_time"] += OE_STOP_TIMER(fetch_elevation);


    // Bail out now if there's no data to be had.
    if ( model->_colorData.size() == 0 && !model->_elevationData.getHeightField() )
    {
        return;
    }

    // OK we are making a tile, so if there's no heightfield yet, make an empty one (and mark it
    // as fallback data of course)
    if ( !model->_elevationData.getHeightField() )
    {
        osg::HeightField* hf = HeightFieldUtils::createReferenceHeightField( key.getExtent(), 15, 15 );
        model->_elevationData = TileModel::ElevationData(
            hf,
            GeoLocator::createForKey(key, frame.getMapInfo()),
            true );
    }

    // look up the parent model and cache it.
    osg::ref_ptr<TileNode> parentTile;
    if ( _liveTiles->get(key.createParentKey(), parentTile) )
        model->_parentModel = parentTile->getTileModel();

    out_model = model.release();
}
