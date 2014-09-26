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
            bool ok = false;

            // This will only go true if we are requesting a ROOT TILE but we have to
            // fall back on lower resolution data to create it.
            bool isFallback = false;

            GeoImage geoImage;

            // The "fast path" preserves mercator tiles without reprojection.
            bool useMercatorFastPath =
                _opt->enableMercatorFastPath() != false &&
                _mapInfo->isGeocentric()                &&
                _layer->getProfile()                    &&
                _layer->getProfile()->getSRS()->isSphericalMercator();

            // If this is a ROOT tile, we will try to fall back on lower-resolution
            // data if we can't find something at the optimal LOD.
            bool isRootKey =
                (_key.getLOD() == 0) || // should never be
                (_key.getLOD()-1 == _opt->firstLOD().value());

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
            
            // fetch the image from the layer.
            if (hasDataInExtent && _layer->isKeyInRange(_key))
            {
                if ( useMercatorFastPath )
                {
                    geoImage = _layer->createImageInNativeProfile( _key, progress );

                    // If this is a root tile, try to find lower-resolution data to
                    // fulfill the request.
                    if ( isRootKey && !geoImage.valid() )
                    {
                        isFallback = true;

                        for(TileKey fallbackKey = _key.createParentKey();
                            fallbackKey.valid() && !geoImage.valid();
                            fallbackKey = fallbackKey.createParentKey())
                        {
                            geoImage = _layer->createImageInNativeProfile( fallbackKey, progress );
                            if ( geoImage.valid() )
                            {
                                OE_DEBUG << LC << "Fell back from (" 
                                    << _key.str() << ") to ("
                                    << fallbackKey.str() << ") for a root tile request."
                                    << std::endl;
                            }
                        }
                    }
                }
                else
                {
                    geoImage = _layer->createImage( _key, progress );

                    // If this is a root tile, try to find lower-resolution data to
                    // fulfill the request.
                    if ( isRootKey && !geoImage.valid() )
                    {
                        isFallback = true;

                        for(TileKey fallbackKey = _key.createParentKey();
                            fallbackKey.valid() && !geoImage.valid();
                            fallbackKey = fallbackKey.createParentKey())
                        {
                            geoImage = _layer->createImage( fallbackKey, progress );
                            if ( geoImage.valid() )
                            {
                                OE_DEBUG << LC << "Fell back from (" 
                                    << _key.str() << ") to ("
                                    << fallbackKey.str() << ") for a root tile request."
                                    << std::endl;
                            }
                        }
                    }
                }
            }

            if ( geoImage.valid() )
            {
                GeoLocator* locator = 0L;
                
                if ( useMercatorFastPath )
                    locator = new MercatorLocator(geoImage.getExtent());
                else
                    locator = GeoLocator::createForExtent(geoImage.getExtent(), *_mapInfo);

                // add the color layer to the repo.
                _model->_colorData[_layer->getUID()] = TileModel::ColorData(
                    _layer,
                    _order,
                    geoImage.getImage(),
                    locator,
                    isFallback ); // isFallbackData

                ok = true;
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
                            colorData._order = _order;
                            colorData.setIsFallbackData( true );
                            ok = true;
                        }
                    }
                }
            }

            return ok;
        }

        TileKey           _key;
        const MapInfo*    _mapInfo;
        TileNodeRegistry* _tiles;
        ImageLayer*       _layer;
        unsigned          _order;
        TileModel*        _model;
        const MPTerrainEngineOptions* _opt;
    };
}

//------------------------------------------------------------------------

TileModelFactory::TileModelFactory(TileNodeRegistry*             liveTiles,
                                   const MPTerrainEngineOptions& terrainOptions ) :
_liveTiles     ( liveTiles ),
_terrainOptions( terrainOptions )
{
    _hfCache = new HeightFieldCache(liveTiles, terrainOptions);
}

HeightFieldCache*
TileModelFactory::getHeightFieldCache() const
{
    return _hfCache;
}


void
TileModelFactory::buildElevation(const TileKey&    key,
                                 const MapFrame&   frame,
                                 bool              accumulate,
                                 TileModel*        model,
                                 ProgressCallback* progress)
{     
    const MapInfo& mapInfo = frame.getMapInfo();

    const osgEarth::ElevationInterpolation& interp =
        frame.getMapOptions().elevationInterpolation().get();

    // Request a heightfield from the map, falling back on lower resolution tiles
    // if necessary (fallback=true)
    osg::ref_ptr<osg::HeightField> hf;

    bool isFallback = false;

    if (_hfCache->getOrCreateHeightField(frame, key, accumulate, hf, isFallback, SAMPLE_FIRST_VALID, interp, progress))
    {
        model->_elevationData = TileModel::ElevationData(
            hf,
            GeoLocator::createForKey( key, mapInfo ),
            isFallback );

        // Edge normalization: requires adjacency information
        if ( _terrainOptions.normalizeEdges() == true )
        {
            for( int x=-1; x<=1; x++ )
            {
                for( int y=-1; y<=1; y++ )
                {
                    if ( x != 0 || y != 0 )
                    {
                        TileKey nk = key.createNeighborKey(x, y);
                        if ( nk.valid() )
                        {
                            osg::ref_ptr<osg::HeightField> hf;
                            if (_hfCache->getOrCreateHeightField(frame, nk, accumulate, hf, isFallback, SAMPLE_FIRST_VALID, interp, progress) )
                            {
                                model->_elevationData.setNeighbor( x, y, hf.get() );
                            }
                        }
                    }
                }
            }

            // parent too.
            if ( key.getLOD() > 0 )
            {
                osg::ref_ptr<osg::HeightField> hf;
                if ( _hfCache->getOrCreateHeightField(frame, key.createParentKey(), accumulate, hf, isFallback, SAMPLE_FIRST_VALID, interp, progress) )
                {
                    model->_elevationData.setParent( hf.get() );
                }
            }
        }
    }
}


void
TileModelFactory::createTileModel(const TileKey&           key, 
                                  const MapFrame&          frame,
                                  bool                     accumulate,
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

        if ( layer->getEnabled() && layer->isKeyInRange(key) )
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
    buildElevation(key, frame, accumulate, model.get(), progress);

    if (progress)
        progress->stats()["fetch_elevation_time"] += OE_STOP_TIMER(fetch_elevation);


    // If nothing was added, not even a fallback heightfield, something went
    // horribly wrong. Leave without a tile model. Chances are that a parent tile
    // not not found in the live-tile registry.
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
