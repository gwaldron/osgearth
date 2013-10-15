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

#include <osgEarth/CacheSeed>
#include <osgEarth/CacheEstimator>
#include <osgEarth/MapFrame>
#include <OpenThreads/ScopedLock>
#include <limits.h>

#define LC "[CacheSeed] "

using namespace osgEarth;
using namespace OpenThreads;





/******************************************************************/
CacheTileOperation::CacheTileOperation(const MapFrame& mapFrame, CacheSeed& cacheSeed, const TileKey& key):
_mapFrame( mapFrame ),
_cacheSeed( cacheSeed ),
_key( key )
{
}

void CacheTileOperation::operator()(osg::Object*)
{
    unsigned int x, y, lod;
    _key.getTileXY(x, y);
    lod = _key.getLevelOfDetail();

    bool gotData = true;

    if ( _cacheSeed.getMinLevel() <= lod && _cacheSeed.getMaxLevel() >= lod )
    {
        gotData = _cacheSeed.cacheTile( _mapFrame, _key );
        if (gotData)
        {                
            _cacheSeed.incrementCompleted();
            _cacheSeed.reportProgress( std::string("Cached tile: ") + _key.str() );
        }       
    }

    if ( gotData && lod <= _cacheSeed.getMaxLevel() )
    {
        TileKey k0 = _key.createChildKey(0);
        TileKey k1 = _key.createChildKey(1);
        TileKey k2 = _key.createChildKey(2);
        TileKey k3 = _key.createChildKey(3); 

        bool intersectsKey = false;
        if ( _cacheSeed.getExtents().empty()) intersectsKey = true;
        else
        {
            for (unsigned int i = 0; i < _cacheSeed.getExtents().size(); ++i)
            {
                const GeoExtent& extent = _cacheSeed.getExtents()[i];
                if (extent.intersects( k0.getExtent() ) ||
                    extent.intersects( k1.getExtent() ) ||
                    extent.intersects( k2.getExtent() ) ||
                    extent.intersects( k3.getExtent() ))
                {
                    intersectsKey = true;
                }

            }
        }

        //Check to see if the bounds intersects ANY of the tile's children.  If it does, then process all of the children
        //for this level
        if (intersectsKey)
        {
            // Queue the task up for the children
            _cacheSeed._queue.get()->add( new CacheTileOperation( _mapFrame, _cacheSeed, k0) );
            _cacheSeed._queue.get()->add( new CacheTileOperation( _mapFrame, _cacheSeed, k1) );
            _cacheSeed._queue.get()->add( new CacheTileOperation( _mapFrame, _cacheSeed, k2) );
            _cacheSeed._queue.get()->add( new CacheTileOperation( _mapFrame, _cacheSeed, k3) );                
        }
    }
}

/******************************************************************/





CacheSeed::CacheSeed():
_minLevel (0),
_maxLevel (12),
_total    (0),
_completed(0),
_numThreads(1)
{
}

CacheSeed::CacheSeed( const CacheSeed& rhs):
_minLevel( rhs._minLevel),
_maxLevel( rhs._maxLevel),
_numThreads( rhs._numThreads )
{
}

void CacheSeed::seed( Map* map )
{
    // We must do this to avoid an error message in OpenSceneGraph b/c the findWrapper method doesn't appear to be threadsafe.
    // This really isn't a big deal b/c this only effects data that is already cached.
    osgDB::ObjectWrapper* wrapper = osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper( "osg::Image" );

    osg::Timer_t startTime = osg::Timer::instance()->tick();
    if ( !map->getCache() )
    {
        OE_WARN << LC << "Warning: No cache defined; aborting." << std::endl;
        return;
    }

    std::vector<TileKey> keys;
    map->getProfile()->getRootKeys(keys);

    //Add the map's entire extent if we don't have one specified.
    if (_extents.empty())
    {
        addExtent( map->getProfile()->getExtent() );
    }

    bool hasCaches = false;
    int src_min_level = INT_MAX;
    unsigned int src_max_level = 0;

    MapFrame mapf( map, Map::TERRAIN_LAYERS, "CacheSeed::seed" );

    //Assumes the the TileSource will perform the caching for us when we call createImage
    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); i++ )
    {
        ImageLayer* layer = i->get();
        TileSource* src   = layer->getTileSource();

        const ImageLayerOptions& opt = layer->getImageLayerOptions();

        if ( layer->isCacheOnly() )
        {
            OE_WARN << LC << "Warning: Layer \"" << layer->getName() << "\" is set to cache-only; skipping." << std::endl;
        }
        else if ( !src )
        {
            OE_WARN << "Warning: Layer \"" << layer->getName() << "\" could not create TileSource; skipping." << std::endl;
        }
        //else if ( src->getCachePolicyHint(0L) == CachePolicy::NO_CACHE )
        //{
        //    OE_WARN << LC << "Warning: Layer \"" << layer->getName() << "\" does not support seeding; skipping." << std::endl;
        //}
        else if ( !layer->getCache() )
        {
            OE_WARN << LC << "Notice: Layer \"" << layer->getName() << "\" has no cache defined; skipping." << std::endl;
        }
        else
        {
            hasCaches = true;

            if (opt.minLevel().isSet() && (int)opt.minLevel().get() < src_min_level)
                src_min_level = opt.minLevel().get();
            if (opt.maxLevel().isSet() && opt.maxLevel().get() > src_max_level)
                src_max_level = opt.maxLevel().get();
        }
    }

    for( ElevationLayerVector::const_iterator i = mapf.elevationLayers().begin(); i != mapf.elevationLayers().end(); i++ )
    {
        ElevationLayer* layer = i->get();
        TileSource*     src   = layer->getTileSource();
        const ElevationLayerOptions& opt = layer->getElevationLayerOptions();

        if ( layer->isCacheOnly() )
        {
            OE_WARN << LC << "Warning: Layer \"" << layer->getName() << "\" is set to cache-only; skipping." << std::endl;
        }
        else if (!src)
        {
            OE_WARN << "Warning: Layer \"" << layer->getName() << "\" could not create TileSource; skipping." << std::endl;
        }
        //else if ( src->getCachePolicyHint(0L) == CachePolicy::NO_CACHE )
        //{
        //    OE_WARN << LC << "Warning: Layer \"" << layer->getName() << "\" does not support seeding; skipping." << std::endl;
        //}
        else if ( !layer->getCache() )
        {
            OE_WARN << LC << "Notice: Layer \"" << layer->getName() << "\" has no cache defined; skipping." << std::endl;
        }
        else
        {
            hasCaches = true;

            if (opt.minLevel().isSet() && (int)opt.minLevel().get() < src_min_level)
                src_min_level = opt.minLevel().get();
            if (opt.maxLevel().isSet() && opt.maxLevel().get() > src_max_level)
                src_max_level = opt.maxLevel().get();
        }
    }

    if ( !hasCaches )
    {
        OE_WARN << LC << "There are either no caches defined in the map, or no sources to cache; aborting." << std::endl;
        return;
    }

    if ( src_max_level > 0 && src_max_level < _maxLevel )
    {
        _maxLevel = src_max_level;
    }

    OE_NOTICE << LC << "Maximum cache level will be " << _maxLevel << std::endl;

    //Estimate the number of tiles
    _total = 0;    
    CacheEstimator est;
    est.setMinLevel( _minLevel );
    est.setMaxLevel( _maxLevel );
    est.setProfile( map->getProfile() ); 
    for (unsigned int i = 0; i < _extents.size(); i++)
    {                
        est.addExtent( _extents[ i ] );
    } 
    _total = est.getNumTiles();

    OE_INFO << "Processing ~" << _total << " tiles" << std::endl;


    // Initialize the operations queue
    _queue = new osg::OperationQueue;

    osg::Timer_t endTime = osg::Timer::instance()->tick();

    // Start the threads
    std::vector< osg::ref_ptr< osg::OperationsThread > > threads;
    for (unsigned int i = 0; i < _numThreads; i++)
    {        
        osg::OperationsThread* thread = new osg::OperationsThread();
        thread->setOperationQueue(_queue.get());
        thread->start();
        threads.push_back( thread );
    }

    OE_NOTICE << "Startup time " << osg::Timer::instance()->delta_s( startTime, endTime ) << std::endl;

    
    // Add the root keys to the queue
    for (unsigned int i = 0; i < keys.size(); ++i)
    {
        //processKey( mapf, keys[i] );
        _queue.get()->add( new CacheTileOperation( mapf, *this, keys[i]) );
    }    

    bool done = false;
    while (!done)
    {
        OpenThreads::Thread::microSleep(500000); // sleep for half a second
        done = true;
        if (_queue->getNumOperationsInQueue() > 0)
        {
            done = false;
            continue;
        }
        else
        {
            // Make sure no threads are currently working on an operation, which actually might add MORE operations since we are doing a quadtree traversal
            for (unsigned int i = 0; i < threads.size(); i++)
            {
                if (threads[i]->getCurrentOperation())
                {
                    done = false;
                    continue;
                }
            }
        }
    }    

    _total = _completed;

    if ( _progress.valid()) _progress->reportProgress(_completed, _total, 0, 1, "Finished");
}

unsigned int CacheSeed::getNumThreads() const
{
    return _numThreads;
}

void CacheSeed::setNumThreads( unsigned int numThreads )
{
    _numThreads = numThreads;
}

void CacheSeed::incrementCompleted( ) const
{            
    CacheSeed* nonconst_this = const_cast<CacheSeed*>(this);    
    ++nonconst_this->_completed;    
}

void CacheSeed::reportProgress( const std::string& message ) const
{
    if ( _progress.valid() )
    {
        CacheSeed* nonconst_this = const_cast<CacheSeed*>(this);    
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock( nonconst_this->_mutex );
        _progress->reportProgress(_completed, _total, message );
    }
}

bool
CacheSeed::cacheTile(const MapFrame& mapf, const TileKey& key ) const
{
    bool gotData = false;

    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); i++ )
    {
        ImageLayer* layer = i->get();
        if ( layer->isKeyValid( key ) )
        {
            GeoImage image = layer->createImage( key );
            if ( image.valid() )
                gotData = true;
        }
    }

    if ( mapf.elevationLayers().size() > 0 )
    {
        osg::ref_ptr<osg::HeightField> hf;
        mapf.getHeightField( key, false, hf );
        if ( hf.valid() )
            gotData = true;
    }

    return gotData;
}

void
CacheSeed::addExtent( const GeoExtent& value)
{
    _extents.push_back( value );
}
