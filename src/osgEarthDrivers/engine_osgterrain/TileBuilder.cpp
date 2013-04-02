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
#include "TileBuilder"
#include "TransparentLayer"
#include <osgEarth/ImageUtils>
#include <osgEarth/TaskService>
#include <osgEarth/HeightFieldUtils>

using namespace osgEarth_engine_osgterrain;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TileBuilder] "

//------------------------------------------------------------------------

struct BuildColorLayer
{
    void init( const TileKey& key, ImageLayer* layer, const MapInfo& mapInfo,
               const OSGTerrainOptions& opt, TileBuilder::SourceRepo& repo )
    {
        _key      = key;
        _layer    = layer;
        _mapInfo  = &mapInfo;
        _opt      = &opt;
        _repo     = &repo;
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
        TileSource* tileSource = _layer->getTileSource();
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
            hasDataInExtent = ext.isValid() && tileSource->hasDataInExtent( ext );
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

        bool isStreaming = _opt->loadingPolicy()->mode() == LoadingPolicy::MODE_PREEMPTIVE || _opt->loadingPolicy()->mode() == LoadingPolicy::MODE_SEQUENTIAL;

        if (geoImage.getImage() && isStreaming)
        {
            // protected against multi threaded access. This is a requirement in sequential/preemptive mode, 
            // for example. This used to be in TextureCompositorTexArray::prepareImage.
            // TODO: review whether this affects performance.    
            geoImage.getImage()->setDataVariance( osg::Object::DYNAMIC );
        }

        // add the color layer to the repo.
        _repo->add( CustomColorLayer(
            _layer,
            geoImage.getImage(),
            locator,
            _key.getLevelOfDetail(),
            _key,
            isFallbackData ) );
    }

    TileKey        _key;
    const MapInfo* _mapInfo;
    ImageLayer*    _layer;
    const OSGTerrainOptions* _opt;
    TileBuilder::SourceRepo* _repo;
};

//------------------------------------------------------------------------

struct BuildElevLayer
{
    void init(const TileKey& key, const MapFrame& mapf, const OSGTerrainOptions& opt, TileBuilder::SourceRepo& repo)
    {
        _key  = key;
        _mapf = &mapf;
        _opt  = &opt;
        _repo = &repo;
    }

    void execute()
    {
        const MapInfo& mapInfo = _mapf->getMapInfo();

        // Request a heightfield from the map, falling back on lower resolution tiles
        // if necessary (fallback=true)
        osg::ref_ptr<osg::HeightField> hf;
        bool isFallback = false;

        if ( _mapf->getHeightField( _key, true, hf, &isFallback ) )
        {
            // Treat Plate Carre specially by scaling the height values. (There is no need
            // to do this with an empty heightfield)
            if ( mapInfo.isPlateCarre() )
            {
                HeightFieldUtils::scaleHeightFieldToDegrees( hf.get() );
            }

            // Put it in the repo
            osgTerrain::HeightFieldLayer* hfLayer = new osgTerrain::HeightFieldLayer( hf.get() );

            // Generate a locator.
            hfLayer->setLocator( GeoLocator::createForKey( _key, mapInfo ) );

            _repo->set( CustomElevLayer(hfLayer, isFallback) );
        }
    }

    TileKey                  _key;
    const MapFrame*          _mapf;
    const OSGTerrainOptions* _opt;
    TileBuilder::SourceRepo* _repo;
};

//------------------------------------------------------------------------

struct AssembleTile
{
    void init(const TileKey& key, const MapInfo& mapInfo, const OSGTerrainOptions& opt, TileBuilder::SourceRepo& repo, const MaskLayerVector& masks=MaskLayerVector() )
    {
        _key     = key;
        _mapInfo = &mapInfo;
        _opt     = &opt;
        _repo    = &repo;
        _tile    = 0L;
        _masks.clear();
        std::copy( masks.begin(), masks.end(), std::back_inserter(_masks) );
    }

    void execute()
    {
        _tile = new Tile( _key, GeoLocator::createForKey(_key, *_mapInfo), *_opt->quickReleaseGLObjects() );
        _tile->setVerticalScale( *_opt->verticalScale() );

        //_tile->setRequiresNormals( true );
        _tile->setDataVariance( osg::Object::DYNAMIC );
        _tile->setTerrainMasks(_masks);

        // copy over the source data.
        _tile->setCustomColorLayers( _repo->_colorLayers );
        _tile->setElevationLayer( _repo->_elevLayer.getHFLayer() );

        osg::BoundingSphere bs = _tile->getBound();

        // a skirt hides cracks when transitioning between LODs:
        osg::HeightField* hf = _repo->_elevLayer.getHFLayer()->getHeightField();
        hf->setSkirtHeight(bs.radius() * _opt->heightFieldSkirtRatio().get() );
    }

    TileKey                  _key;
    const MapInfo*           _mapInfo;
    const OSGTerrainOptions* _opt;
    TileBuilder::SourceRepo* _repo;
    Tile*                    _tile;
    MaskLayerVector          _masks;
};

//------------------------------------------------------------------------

TileBuilder::TileBuilder(const Map* map, const OSGTerrainOptions& terrainOptions, TaskService* service) :
_map( map ),
_terrainOptions( terrainOptions ),
_service( service )
{
    //nop
}

TileBuilder::Job*
TileBuilder::createJob( const TileKey& key, Threading::MultiEvent& semaphore )
{
    Job* job = new Job( key, _map );

    // create the image layer tasks:
    for( ImageLayerVector::const_iterator i = job->_mapf.imageLayers().begin(); i != job->_mapf.imageLayers().end(); ++i )
    {
        ImageLayer* layer = i->get();

        if ( layer->getEnabled() && layer->isKeyValid(key) )
        {
            ParallelTask<BuildColorLayer>* j = new ParallelTask<BuildColorLayer>( &semaphore );
            j->init( key, layer, job->_mapf.getMapInfo(), _terrainOptions, job->_repo );
            j->setPriority( -(float)key.getLevelOfDetail() );
            job->_tasks.push_back( j );
        }
    }

    // If we have elevation layers, start an elevation job as well. Otherwise just create an
    // empty one while we're waiting for the images to load.
    if ( job->_mapf.elevationLayers().size() > 0 )
    {
        ParallelTask<BuildElevLayer>* ej = new ParallelTask<BuildElevLayer>( &semaphore );
        ej->init( key, job->_mapf, _terrainOptions, job->_repo );
        ej->setPriority( -(float)key.getLevelOfDetail() );
        job->_tasks.push_back( ej );
    }

    return job;
}

void
TileBuilder::runJob( TileBuilder::Job* job )
{
    for( TaskRequestVector::iterator i = job->_tasks.begin(); i != job->_tasks.end(); ++i )
        _service->add( i->get() );
}

void
TileBuilder::finalizeJob(TileBuilder::Job*   job, 
                         osg::ref_ptr<Tile>& out_tile,
                         bool&               out_hasRealData,
                         bool&               out_hasLodBlending)
{
    SourceRepo& repo = job->_repo;

    out_hasRealData = false;
    out_hasLodBlending = false;

    // Bail out now if there's no data to be had.
    if ( repo._colorLayers.size() == 0 && !repo._elevLayer.getHFLayer() )
    {
        return;
    }

    const TileKey& key = job->_key;
    const MapInfo& mapInfo = job->_mapf.getMapInfo();

    // OK we are making a tile, so if there's no heightfield yet, make an empty one.
    if ( !repo._elevLayer.getHFLayer() )
    {
        osg::HeightField* hf = HeightFieldUtils::createReferenceHeightField( key.getExtent(), 8, 8 );
        osgTerrain::HeightFieldLayer* hfLayer = new osgTerrain::HeightFieldLayer( hf );
        hfLayer->setLocator( GeoLocator::createForKey(key, mapInfo) );
        repo._elevLayer = CustomElevLayer( hfLayer, true );
    }

    // Now, if there are any color layers that did not get built, create them with an empty
    // image so the shaders have something to draw.
    osg::ref_ptr<osg::Image> emptyImage;
    osgTerrain::Locator* locator = repo._elevLayer.getHFLayer()->getLocator();

    for( ImageLayerVector::const_iterator i = job->_mapf.imageLayers().begin(); i != job->_mapf.imageLayers().end(); ++i )
    {
        ImageLayer* layer = i->get();

        if ( layer->getEnabled() )
        {
            if ( !layer->isKeyValid(key) )
            {
                if ( !emptyImage.valid() )
                    emptyImage = ImageUtils::createEmptyImage();

                repo.add( CustomColorLayer(
                    i->get(), emptyImage.get(),
                    locator,
                    key.getLevelOfDetail(),
                    key,
                    true ) );
            }

            if ( i->get()->getImageLayerOptions().lodBlending() == true )
                out_hasLodBlending = true;
        }
    }

    // Ready to create the actual tile.
    AssembleTile assemble;
    assemble.init( key, mapInfo, _terrainOptions, repo );
    assemble.execute();

    // Check the results and see if we have any real data.
    for( ColorLayersByUID::const_iterator i = repo._colorLayers.begin(); i != repo._colorLayers.end(); ++i )
    {
        if ( !i->second.isFallbackData() ) 
        {
            out_hasRealData = true;
            break;
        }
    }
    if ( !out_hasRealData && !repo._elevLayer.isFallbackData() )
    {
        out_hasRealData = true;
    }

    out_tile = assemble._tile;
}

void
TileBuilder::createTile(const TileKey&      key, 
                        bool                parallelize, 
                        osg::ref_ptr<Tile>& out_tile, 
                        bool&               out_hasRealData,
                        bool&               out_hasLodBlendedLayers )
{
    MapFrame mapf( _map, Map::MASKED_TERRAIN_LAYERS );

    SourceRepo repo;

    // init this to false, then search for real data. "Real data" is data corresponding
    // directly to the key, as opposed to fallback data, which is derived from a lower
    // LOD key.
    out_hasRealData = false;
    out_hasLodBlendedLayers = false;

    const MapInfo& mapInfo = mapf.getMapInfo();

    // If we need more than one layer, fetch them in parallel.
    // TODO: change the test based on isKeyValid total.
    if ( parallelize && (mapf.imageLayers().size() + mapf.elevationLayers().size() > 1) )
    {
        // count the valid layers.
        int jobCount = 0;

        for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
        {
            if ( i->get()->isKeyValid( key ) )
                ++jobCount;

            if ( i->get()->getImageLayerOptions().lodBlending() == true )
                out_hasLodBlendedLayers = true;
        }

        if ( mapf.elevationLayers().size() > 0 )
            ++jobCount;

        // A thread job monitoring event:
        Threading::MultiEvent semaphore( jobCount );

        // Start the image layer jobs:
        for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
        {
            ImageLayer* layer = i->get();
            if ( layer->isKeyValid(key) )
            {
                ParallelTask<BuildColorLayer>* j = new ParallelTask<BuildColorLayer>( &semaphore );
                j->init( key, layer, mapInfo, _terrainOptions, repo );
                j->setPriority( -(float)key.getLevelOfDetail() );
                _service->add( j );
            }
        }

        // If we have elevation layers, start an elevation job as well. Otherwise just create an
        // empty one while we're waiting for the images to load.
        if ( mapf.elevationLayers().size() > 0 )
        {
            ParallelTask<BuildElevLayer>* ej = new ParallelTask<BuildElevLayer>( &semaphore );
            ej->init( key, mapf, _terrainOptions, repo );
            ej->setPriority( -(float)key.getLevelOfDetail() );
            _service->add( ej );
        }
        else
        {
            BuildElevLayer build;
            build.init( key, mapf, _terrainOptions, repo );
            build.execute();
        }

        // Wait for all the jobs to finish.
        semaphore.wait();
    }
    
    // Fetch the image data serially:
    else
    {
        // gather all the image layers serially.
        for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
        {
            ImageLayer* layer = i->get();
            //if ( layer->isKeyValid(key) )  // Wrong. no guarantee key is in the same profile.

            if ( layer->getEnabled() )
            {
                BuildColorLayer build;
                build.init( key, layer, mapInfo, _terrainOptions, repo );
                build.execute();

                if ( layer->getImageLayerOptions().lodBlending() == true )
                    out_hasLodBlendedLayers = true;
            }
        }
        
        // make an elevation layer.
        BuildElevLayer build;
        build.init( key, mapf, _terrainOptions, repo );
        build.execute();
    }

    // Bail out now if there's no data to be had.
    if ( repo._colorLayers.size() == 0 && !repo._elevLayer.getHFLayer() )
    {
        return;
    }

    // OK we are making a tile, so if there's no heightfield yet, make an empty one.
    if ( !repo._elevLayer.getHFLayer() )
    {
        osg::HeightField* hf = HeightFieldUtils::createReferenceHeightField( key.getExtent(), 8, 8 );
        //osg::HeightField* hf = key.getProfile()->getVerticalSRS()->createReferenceHeightField( key.getExtent(), 8, 8 );
        osgTerrain::HeightFieldLayer* hfLayer = new osgTerrain::HeightFieldLayer( hf );
        hfLayer->setLocator( GeoLocator::createForKey(key, mapInfo) );
        repo._elevLayer = CustomElevLayer( hfLayer, true );
    }

    // Now, if there are any color layers that did not get built, create them with an empty
    // image so the shaders have something to draw.
    osg::ref_ptr<osg::Image> emptyImage;
    osgTerrain::Locator* locator = repo._elevLayer.getHFLayer()->getLocator();

    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
    {
        ImageLayer* layer = i->get();

        if ( layer->getEnabled() && !layer->isKeyValid(key) )
        {
            if ( !emptyImage.valid() )
                emptyImage = ImageUtils::createEmptyImage();

            repo.add( CustomColorLayer(
                layer,
                emptyImage.get(),
                locator,
                key.getLevelOfDetail(),
                key,
                true ) );
        }
    }

    //osg::Vec3dArray* maskBounds = 0L;
    //osgEarth::MaskLayer* mask = mapf.getTerrainMaskLayer();
    //if (mask)
    //  maskBounds = mask->getOrCreateBoundary();

    // Ready to create the actual tile.
    AssembleTile assemble;
    assemble.init( key, mapInfo, _terrainOptions, repo, mapf.terrainMaskLayers() );
    assemble.execute();

    if (!out_hasRealData)
    {
        // Check the results and see if we have any real data.
        for( ColorLayersByUID::const_iterator i = repo._colorLayers.begin(); i != repo._colorLayers.end(); ++i )
        {
            if ( !i->second.isFallbackData() ) 
            {
                out_hasRealData = true;
                break;
            }
        }
    }

    if ( !out_hasRealData && !repo._elevLayer.isFallbackData() )
    {
        out_hasRealData = true;
    }

    out_tile = assemble._tile;
}
