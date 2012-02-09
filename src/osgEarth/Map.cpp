/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarth/Map>
#include <osgEarth/Registry>
#include <osgEarth/TileSource>
#include <osgEarth/URI>
#include <iterator>

using namespace osgEarth;

#define LC "[Map] "

//------------------------------------------------------------------------

void
MapCallback::onMapModelChanged( const MapModelChange& change )
{
    switch( change.getAction() )
    {
    case MapModelChange::ADD_ELEVATION_LAYER: 
        onElevationLayerAdded( change.getElevationLayer(), change.getFirstIndex() ); break;
    case MapModelChange::ADD_IMAGE_LAYER:
        onImageLayerAdded( change.getImageLayer(), change.getFirstIndex() ); break;
    case MapModelChange::ADD_MASK_LAYER:
        onMaskLayerAdded( change.getMaskLayer() ); break;
    case MapModelChange::ADD_MODEL_LAYER:
        onModelLayerAdded( change.getModelLayer(), change.getFirstIndex() ); break;
    case MapModelChange::REMOVE_ELEVATION_LAYER:
        onElevationLayerRemoved( change.getElevationLayer(), change.getFirstIndex() ); break;
    case MapModelChange::REMOVE_IMAGE_LAYER:
        onImageLayerRemoved( change.getImageLayer(), change.getFirstIndex() ); break;
    case MapModelChange::REMOVE_MASK_LAYER:
        onMaskLayerRemoved( change.getMaskLayer() ); break;
    case MapModelChange::REMOVE_MODEL_LAYER:
        onModelLayerRemoved( change.getModelLayer() ); break;
    case MapModelChange::MOVE_ELEVATION_LAYER:
        onElevationLayerMoved( change.getElevationLayer(), change.getFirstIndex(), change.getSecondIndex() ); break;
    case MapModelChange::MOVE_IMAGE_LAYER:
        onImageLayerMoved( change.getImageLayer(), change.getFirstIndex(), change.getSecondIndex() ); break;
		case MapModelChange::MOVE_MODEL_LAYER:
				onModelLayerMoved( change.getModelLayer(), change.getFirstIndex(), change.getSecondIndex() ); break;
    }
}

//------------------------------------------------------------------------

Map::Map( const MapOptions& options ) :
osg::Referenced   ( true ),
_mapOptions       ( options ),
_dataModelRevision( 0 )
{            
    if (_mapOptions.cachePolicy().isSet() &&
        _mapOptions.cachePolicy()->usage() == CachePolicy::USAGE_CACHE_ONLY )
    {
        OE_INFO << LC << "CACHE-ONLY MODE activated from map" << std::endl;
    }

    // if the map was a cache policy set, make this the system-wide default, UNLESS
    // there ALREADY IS a registry default, in which case THAT will override THIS one.
    // (In other words, whichever one is set first wins.)
    if ( _mapOptions.cachePolicy().isSet() )
    {
        if ( Registry::instance()->defaultCachePolicy().empty() )
            Registry::instance()->setDefaultCachePolicy( *_mapOptions.cachePolicy() );
        else
            _mapOptions.cachePolicy() = Registry::instance()->defaultCachePolicy();
    }
    else if ( ! Registry::instance()->defaultCachePolicy().empty() )
    {
        _mapOptions.cachePolicy() = Registry::instance()->defaultCachePolicy();
    }

    // the map-side dbOptions object holds I/O information for all components.
    _dbOptions = osg::clone( Registry::instance()->getDefaultOptions() );

    // we do our own caching
    _dbOptions->setObjectCacheHint( osgDB::Options::CACHE_NONE );
    
    // store the top-level referrer context in the options
    URIContext( _mapOptions.referrer() ).store( _dbOptions );
}

bool
Map::isGeocentric() const
{
    return 
        _mapOptions.coordSysType() == MapOptions::CSTYPE_GEOCENTRIC ||
        _mapOptions.coordSysType() == MapOptions::CSTYPE_GEOCENTRIC_CUBE;
}

const osgDB::Options*
Map::getGlobalOptions() const
{
    return _globalOptions.get();
}

void
Map::setGlobalOptions( const osgDB::Options* options )
{
    _globalOptions = options;
}

Revision
Map::getImageLayers( ImageLayerVector& out_list ) const
{
    out_list.reserve( _imageLayers.size() );

    Threading::ScopedReadLock lock( const_cast<Map*>(this)->_mapDataMutex );
    for( ImageLayerVector::const_iterator i = _imageLayers.begin(); i != _imageLayers.end(); ++i )
        out_list.push_back( i->get() );

    return _dataModelRevision;
}

int
Map::getNumImageLayers() const
{
    Threading::ScopedReadLock lock( const_cast<Map*>(this)->_mapDataMutex );
    return _imageLayers.size();
}

ImageLayer*
Map::getImageLayerByName( const std::string& name ) const
{
    Threading::ScopedReadLock( const_cast<Map*>(this)->_mapDataMutex );
    for( ImageLayerVector::const_iterator i = _imageLayers.begin(); i != _imageLayers.end(); ++i )
        if ( i->get()->getName() == name )
            return i->get();
    return 0L;
}

ImageLayer*
Map::getImageLayerByUID( UID layerUID ) const
{
    Threading::ScopedReadLock( const_cast<Map*>(this)->_mapDataMutex );
    for( ImageLayerVector::const_iterator i = _imageLayers.begin(); i != _imageLayers.end(); ++i )
        if ( i->get()->getUID() == layerUID )
            return i->get();
    return 0L;
}

ImageLayer*
Map::getImageLayerAt( int index ) const
{
    Threading::ScopedReadLock( const_cast<Map*>(this)->_mapDataMutex );
    if ( index >= 0 && index < (int)_imageLayers.size() )
        return _imageLayers[index].get();
    else
        return 0L;
}

Revision
Map::getElevationLayers( ElevationLayerVector& out_list ) const
{
    out_list.reserve( _elevationLayers.size() );

    Threading::ScopedReadLock lock( const_cast<Map*>(this)->_mapDataMutex );
    for( ElevationLayerVector::const_iterator i = _elevationLayers.begin(); i != _elevationLayers.end(); ++i )
        out_list.push_back( i->get() );

    return _dataModelRevision;
}

int
Map::getNumElevationLayers() const
{
    Threading::ScopedReadLock lock( const_cast<Map*>(this)->_mapDataMutex );
    return _elevationLayers.size();
}

ElevationLayer*
Map::getElevationLayerByName( const std::string& name ) const
{
    Threading::ScopedReadLock( const_cast<Map*>(this)->_mapDataMutex );
    for( ElevationLayerVector::const_iterator i = _elevationLayers.begin(); i != _elevationLayers.end(); ++i )
        if ( i->get()->getName() == name )
            return i->get();
    return 0L;
}

ElevationLayer*
Map::getElevationLayerByUID( UID layerUID ) const
{
    Threading::ScopedReadLock( const_cast<Map*>(this)->_mapDataMutex );
    for( ElevationLayerVector::const_iterator i = _elevationLayers.begin(); i != _elevationLayers.end(); ++i )
        if ( i->get()->getUID() == layerUID )
            return i->get();
    return 0L;
}

ElevationLayer*
Map::getElevationLayerAt( int index ) const
{
    Threading::ScopedReadLock( const_cast<Map*>(this)->_mapDataMutex );
    if ( index >= 0 && index < (int)_elevationLayers.size() )
        return _elevationLayers[index].get();
    else
        return 0L;
}

Revision
Map::getModelLayers( ModelLayerVector& out_list ) const
{
    out_list.reserve( _modelLayers.size() );

    Threading::ScopedReadLock lock( const_cast<Map*>(this)->_mapDataMutex );
    for( ModelLayerVector::const_iterator i = _modelLayers.begin(); i != _modelLayers.end(); ++i )
        out_list.push_back( i->get() );

    return _dataModelRevision;
}

ModelLayer*
Map::getModelLayerByName( const std::string& name ) const
{
    Threading::ScopedReadLock( const_cast<Map*>(this)->_mapDataMutex );
    for( ModelLayerVector::const_iterator i = _modelLayers.begin(); i != _modelLayers.end(); ++i )
        if ( i->get()->getName() == name )
            return i->get();
    return 0L;
}

ModelLayer*
Map::getModelLayerByUID( UID layerUID ) const
{
    Threading::ScopedReadLock( const_cast<Map*>(this)->_mapDataMutex );
    for( ModelLayerVector::const_iterator i = _modelLayers.begin(); i != _modelLayers.end(); ++i )
        if ( i->get()->getUID() == layerUID )
            return i->get();
    return 0L;
}


ModelLayer*
Map::getModelLayerAt( int index ) const
{
    Threading::ScopedReadLock( const_cast<Map*>(this)->_mapDataMutex );
    if ( index >= 0 && index < (int)_modelLayers.size() )
        return _modelLayers[index].get();
    else
        return 0L;
}

int
Map::getNumModelLayers() const
{
    Threading::ScopedReadLock lock( const_cast<Map*>(this)->_mapDataMutex );
    return _modelLayers.size();
}

int
Map::getTerrainMaskLayers( MaskLayerVector& out_list ) const
{
    out_list.reserve( _terrainMaskLayers.size() );

    Threading::ScopedReadLock lock( const_cast<Map*>(this)->_mapDataMutex );
    for( MaskLayerVector::const_iterator i = _terrainMaskLayers.begin(); i != _terrainMaskLayers.end(); ++i )
        out_list.push_back( i->get() );

    return _dataModelRevision;
}

void
Map::setName( const std::string& name ) {
    _name = name;
}

Revision
Map::getDataModelRevision() const
{
    Threading::ScopedReadLock lock( const_cast<Map*>(this)->_mapDataMutex );
    return _dataModelRevision;
}

const Profile*
Map::getProfile() const
{
    if ( !_profile.valid() )
        const_cast<Map*>(this)->calculateProfile();
    return _profile.get();
}

Cache*
Map::getCache() const
{
    if ( !_cache.valid() )
    {
        Cache* cache = 0L;
        
        // if a cache is defined in the options, use that.
        if ( _mapOptions.cache().isSet() )
        {
            cache = CacheFactory::create( _mapOptions.cache().get() );
        }

        // or, if there's a cache in the registry, install it now.
        else if ( Registry::instance()->getCache() )
        {
            cache = Registry::instance()->getCache();
        }

        if ( cache )
        {
            const_cast<Map*>(this)->setCache( cache );
        }
    }
	return _cache.get();
}

void
Map::setCache( Cache* cache )
{
    if (_cache.get() != cache)
    {
        _cache = cache;

        if ( _cache.valid() )
        {
            _cache->store( _dbOptions.get() );
        }

        // Propagate the cache to any of our layers
        for (ImageLayerVector::iterator i = _imageLayers.begin(); i != _imageLayers.end(); ++i)
        {
            i->get()->setDBOptions( _dbOptions.get() );
            //i->get()->setCache( _cache.get() );
        }

        for (ElevationLayerVector::iterator i = _elevationLayers.begin(); i != _elevationLayers.end(); ++i)
        {
            i->get()->setDBOptions( _dbOptions.get() );
            //i->get()->setCache( _cache.get() );
        }
    }
}

void 
Map::addMapCallback( MapCallback* cb ) const
{
    if ( cb )
        const_cast<Map*>(this)->_mapCallbacks.push_back( cb );
}

void 
Map::removeMapCallback( MapCallback* cb )
{
    MapCallbackList::iterator i = std::find( _mapCallbacks.begin(), _mapCallbacks.end(), cb);
    if (i != _mapCallbacks.end())
    {
        _mapCallbacks.erase( i );
    }    
}

void
Map::addImageLayer( ImageLayer* layer )
{
    osgEarth::Registry::instance()->clearBlacklist();
    unsigned int index = -1;
    if ( layer )
    {
	    // Set the DB options for the map from the layer
        layer->setDBOptions( _dbOptions.get() );

        // propagate the cache to the layer:
        if (_mapOptions.cachePolicy().isSet())
		{
            layer->overrideCachePolicy( _mapOptions.cachePolicy().value() );
		}

        // propagate the cache to the layer:
        layer->setCache( this->getCache() );

        // Tell the layer the map profile, if possible:
        if ( _profile.valid() )
        {
            layer->setTargetProfileHint( _profile.get() );
        }

        int newRevision;

        // Add the layer to our stack.
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );

            _imageLayers.push_back( layer );
            index = _imageLayers.size() - 1;
            newRevision = ++_dataModelRevision;
        }

        // a separate block b/c we don't need the mutex   
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::ADD_IMAGE_LAYER, newRevision, layer, index) );
        }	
    }	
}


void
Map::insertImageLayer( ImageLayer* layer, unsigned int index )
{
    osgEarth::Registry::instance()->clearBlacklist();
    if ( layer )
    {
        //Set options for the map from the layer
        layer->setDBOptions( _dbOptions.get() );

        //propagate the cache to the layer:
        if (_mapOptions.cachePolicy().isSet() )
        {
            layer->overrideCachePolicy( *_mapOptions.cachePolicy() );
        }

        //Set the Cache for the MapLayer to our cache.
        layer->setCache( this->getCache() );

        // Tell the layer the map profile, if possible:
        if ( _profile.valid() )
            layer->setTargetProfileHint( _profile.get() );

        int newRevision;

        // Add the layer to our stack.
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );

            if (index >= _imageLayers.size())
                _imageLayers.push_back(layer);
            else
                _imageLayers.insert( _imageLayers.begin() + index, layer );

            newRevision = ++_dataModelRevision;
        }

        // a separate block b/c we don't need the mutex   
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::ADD_IMAGE_LAYER, newRevision, layer, index) );
        }	
    }	
}

void
Map::addElevationLayer( ElevationLayer* layer )
{
    osgEarth::Registry::instance()->clearBlacklist();
    unsigned int index = -1;
    if ( layer )
    {
	    //Set options for the map from the layer
		layer->setDBOptions( _dbOptions.get() );

        //propagate the cache to the layer:
        if ( _mapOptions.cachePolicy().isSet() )
        {
            layer->overrideCachePolicy( *_mapOptions.cachePolicy() );
		}

		//Set the Cache for the MapLayer to our cache.
		layer->setCache( this->getCache() );
        
        // Tell the layer the map profile, if possible:
        if ( _profile.valid() )
            layer->setTargetProfileHint( _profile.get() );

        int newRevision;

        // Add the layer to our stack.
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );

            _elevationLayers.push_back( layer );
            index = _elevationLayers.size() - 1;
            newRevision = ++_dataModelRevision;
        }

        // a separate block b/c we don't need the mutex   
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::ADD_ELEVATION_LAYER, newRevision, layer, index) );
        }	
    }	
}

void 
Map::removeImageLayer( ImageLayer* layer )
{
    osgEarth::Registry::instance()->clearBlacklist();
    unsigned int index = -1;

    osg::ref_ptr<ImageLayer> layerToRemove = layer;
    Revision newRevision;

    if ( layerToRemove.get() )
    {
        Threading::ScopedWriteLock lock( _mapDataMutex );
        index = 0;
        for( ImageLayerVector::iterator i = _imageLayers.begin(); i != _imageLayers.end(); i++, index++ )
        {
            if ( i->get() == layerToRemove.get() )
            {
                _imageLayers.erase( i );
                newRevision = ++_dataModelRevision;
                break;
            }
        }
    }

    // a separate block b/c we don't need the mutex
    if ( newRevision >= 0 ) // layerToRemove.get() )
    {
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::REMOVE_IMAGE_LAYER, newRevision, layerToRemove.get(), index) );
            //i->get()->onImageLayerRemoved( layerToRemove.get(), index, newRevision );
        }
    }
}

void 
Map::removeElevationLayer( ElevationLayer* layer )
{
    osgEarth::Registry::instance()->clearBlacklist();
    unsigned int index = -1;

    osg::ref_ptr<ElevationLayer> layerToRemove = layer;
    Revision newRevision;

    if ( layerToRemove.get() )
    {
        Threading::ScopedWriteLock lock( _mapDataMutex );
        index = 0;
        for( ElevationLayerVector::iterator i = _elevationLayers.begin(); i != _elevationLayers.end(); i++, index++ )
        {
            if ( i->get() == layerToRemove.get() )
            {
                _elevationLayers.erase( i );
                newRevision = ++_dataModelRevision;
                break;
            }
        }
    }

    // a separate block b/c we don't need the mutex
    if ( newRevision >= 0 ) //layerToRemove.get() )
    {
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::REMOVE_ELEVATION_LAYER, newRevision, layerToRemove.get(), index) );
        }
    }
}

void
Map::moveImageLayer( ImageLayer* layer, unsigned int newIndex )
{
    unsigned int oldIndex = 0;
    unsigned int actualIndex = 0;
    Revision newRevision;

    if ( layer )
    {
        Threading::ScopedWriteLock lock( _mapDataMutex );

        // preserve the layer with a ref:
        osg::ref_ptr<ImageLayer> layerToMove = layer;

        // find it:
        ImageLayerVector::iterator i_oldIndex = _imageLayers.end();
        for( ImageLayerVector::iterator i = _imageLayers.begin(); i != _imageLayers.end(); i++, actualIndex++ )
        {
            if ( i->get() == layer )
            {
                i_oldIndex = i;
                oldIndex = actualIndex;
                break;
            }
        }

        if ( i_oldIndex == _imageLayers.end() )
            return; // layer not found in list

        // erase the old one and insert the new one.
        _imageLayers.erase( i_oldIndex );
        _imageLayers.insert( _imageLayers.begin() + newIndex, layerToMove.get() );

        newRevision = ++_dataModelRevision;
    }

    // a separate block b/c we don't need the mutex
    if ( layer )
    {
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::MOVE_IMAGE_LAYER, newRevision, layer, oldIndex, newIndex) );
        }
    }
}

void
Map::moveElevationLayer( ElevationLayer* layer, unsigned int newIndex )
{
    unsigned int oldIndex = 0;
    unsigned int actualIndex = 0;
    Revision newRevision;

    if ( layer )
    {
        Threading::ScopedWriteLock lock( _mapDataMutex );

        // preserve the layer with a ref:
        osg::ref_ptr<ElevationLayer> layerToMove = layer;

        // find it:
        ElevationLayerVector::iterator i_oldIndex = _elevationLayers.end();
        for( ElevationLayerVector::iterator i = _elevationLayers.begin(); i != _elevationLayers.end(); i++, actualIndex++ )
        {
            if ( i->get() == layer )
            {
                i_oldIndex = i;
                oldIndex = actualIndex;
                break;
            }
        }

        if ( i_oldIndex == _elevationLayers.end() )
            return; // layer not found in list

        // erase the old one and insert the new one.
        _elevationLayers.erase( i_oldIndex );
        _elevationLayers.insert( _elevationLayers.begin() + newIndex, layerToMove.get() );

        newRevision = ++_dataModelRevision;
    }

    // a separate block b/c we don't need the mutex
    if ( layer )
    {
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::MOVE_ELEVATION_LAYER, newRevision, layer, oldIndex, newIndex) );
        }
    }
}

void
Map::addModelLayer( ModelLayer* layer )
{
    if ( layer )
    {
        unsigned int index = -1;

        Revision newRevision;
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );
            _modelLayers.push_back( layer );
            index = _modelLayers.size() - 1;
            newRevision = ++_dataModelRevision;
        }

        //TODO: deprecate this in favor of URIContext..
        layer->initialize( _dbOptions.get(), this ); //getReferenceURI(), this );        

        // a seprate block b/c we don't need the mutex
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::ADD_MODEL_LAYER, newRevision, layer, index ) );
        }
    }
}

void
Map::insertModelLayer( ModelLayer* layer, unsigned int index )
{
    if ( layer )
    {
        Revision newRevision;
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );
            _modelLayers.insert( _modelLayers.begin() + index, layer );
            newRevision = ++_dataModelRevision;
        }

        //TODO: deprecate this in favor of URIContext..
        layer->initialize( _dbOptions.get(), this ); //getReferenceURI(), this );        

        // a seprate block b/c we don't need the mutex
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::ADD_MODEL_LAYER, newRevision, layer, index) );
        }
    }
}

void
Map::removeModelLayer( ModelLayer* layer )
{
    if ( layer )
    {
        //Take a reference to the layer since we will be deleting it
        osg::ref_ptr< ModelLayer > layerRef = layer;

        Revision newRevision;
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );
            for( ModelLayerVector::iterator i = _modelLayers.begin(); i != _modelLayers.end(); ++i )
            {
                if ( i->get() == layer )
                {
                    _modelLayers.erase( i );
                    newRevision = ++_dataModelRevision;
                    break;
                }
            }
        }

        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); ++i )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::REMOVE_MODEL_LAYER, newRevision, layerRef.get()) );
        }
    }
}

void
Map::moveModelLayer( ModelLayer* layer, unsigned int newIndex )
{
    unsigned int oldIndex = 0;
    unsigned int actualIndex = 0;
    Revision newRevision;

    if ( layer )
    {
        Threading::ScopedWriteLock lock( _mapDataMutex );

        // preserve the layer with a ref:
        osg::ref_ptr<ModelLayer> layerToMove = layer;

        // find it:
        ModelLayerVector::iterator i_oldIndex = _modelLayers.end();
        for( ModelLayerVector::iterator i = _modelLayers.begin(); i != _modelLayers.end(); i++, actualIndex++ )
        {
            if ( i->get() == layer )
            {
                i_oldIndex = i;
                oldIndex = actualIndex;
                break;
            }
        }

        if ( i_oldIndex == _modelLayers.end() )
            return; // layer not found in list

        // erase the old one and insert the new one.
        _modelLayers.erase( i_oldIndex );
        _modelLayers.insert( _modelLayers.begin() + newIndex, layerToMove.get() );

        newRevision = ++_dataModelRevision;
    }

    // a separate block b/c we don't need the mutex
    if ( layer )
    {
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::MOVE_MODEL_LAYER, newRevision, layer, oldIndex, newIndex) );
        }
    }
}

void
Map::addTerrainMaskLayer( MaskLayer* layer )
{
    if ( layer )
    {
        Revision newRevision;
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );
            _terrainMaskLayers.push_back(layer);
            newRevision = ++_dataModelRevision;
        }

        layer->initialize( _dbOptions.get(), this );

        // a separate block b/c we don't need the mutex   
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::ADD_MASK_LAYER, newRevision, layer) );
        }
    }
}

void
Map::removeTerrainMaskLayer( MaskLayer* layer )
{
    if ( layer )
    {
        //Take a reference to the layer since we will be deleting it
        osg::ref_ptr< MaskLayer > layerRef = layer;
        Revision newRevision;
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );
            for( MaskLayerVector::iterator i = _terrainMaskLayers.begin(); i != _terrainMaskLayers.end(); ++i )
            {
                if ( i->get() == layer )
                {
                    _terrainMaskLayers.erase( i );
                    newRevision = ++_dataModelRevision;
                    break;
                }
            }
        }
        
        // a separate block b/c we don't need the mutex   
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::REMOVE_MASK_LAYER, newRevision, layerRef.get()) );
        }	
    }
}

void
Map::calculateProfile()
{
    if ( !_profile.valid() )
    {
        osg::ref_ptr<const Profile> userProfile;
        if ( _mapOptions.profile().isSet() )
        {
            userProfile = Profile::create( _mapOptions.profile().value() );
        }

        if ( _mapOptions.coordSysType() == MapOptions::CSTYPE_GEOCENTRIC )
        {
            if ( userProfile.valid() )
            {
                if ( userProfile->isOK() && userProfile->getSRS()->isGeographic() )
                {
                    _profile = userProfile.get();
                }
                else
                {
                    OE_WARN << LC 
                        << "Map is geocentric, but the configured profile does not "
                        << "have a geographic SRS. Falling back on default.."
                        << std::endl;
                }
            }

            if ( !_profile.valid() )
            {
                // by default, set a geocentric map to use global-geodetic WGS84.
                _profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
            }
        }

        else if ( _mapOptions.coordSysType() == MapOptions::CSTYPE_GEOCENTRIC_CUBE )
        {
            //If the map type is a Geocentric Cube, set the profile to the cube profile.
            _profile = osgEarth::Registry::instance()->getCubeProfile();
        }

        else // CSTYPE_PROJECTED
        {
            if ( userProfile.valid() )
            {
                _profile = userProfile.get();
            }
        }

        // At this point, if we don't have a profile we need to search tile sources until we find one.
        if ( !_profile.valid() )
        {
            Threading::ScopedReadLock lock( _mapDataMutex );

            for( ImageLayerVector::iterator i = _imageLayers.begin(); i != _imageLayers.end() && !_profile.valid(); i++ )
            {
                ImageLayer* layer = i->get();
                if ( layer->getTileSource() )
                {
                    _profile = layer->getTileSource()->getProfile();
                }
            }

            for( ElevationLayerVector::iterator i = _elevationLayers.begin(); i != _elevationLayers.end() && !_profile.valid(); i++ )
            {
                ElevationLayer* layer = i->get();
                if ( layer->getTileSource() )
                {
                    _profile = layer->getTileSource()->getProfile();
                }
            }
        }

        // finally, fire an event if the profile has been set.
        if ( _profile.valid() )
        {
            OE_INFO << LC << "Map profile is: " << _profile->toString() << std::endl;

            for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
            {
                i->get()->onMapInfoEstablished( MapInfo(this) );
            }
        }

        else
        {
            OE_WARN << LC << "Warning, not yet able to establish a map profile!" << std::endl;
        }
    }

    if ( _profile.valid() )
    {
        // tell all the loaded layers what the profile is, as a hint
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );

            for( ImageLayerVector::iterator i = _imageLayers.begin(); i != _imageLayers.end(); i++ )
            {
                ImageLayer* layer = i->get();
                layer->setTargetProfileHint( _profile.get() );
            }

            for( ElevationLayerVector::iterator i = _elevationLayers.begin(); i != _elevationLayers.end(); i++ )
            {
                ElevationLayer* layer = i->get();
                layer->setTargetProfileHint( _profile.get() );
            }
        }

        // create a "proxy" profile to use when querying elevation layers with a vertical datum
        if ( _profile->getSRS()->getVerticalDatum() != 0L )
        {
            ProfileOptions po = _profile->toProfileOptions();
            po.vsrsString().unset();
            _profileNoVDatum = Profile::create(po);
        }
        else
        {
            _profileNoVDatum = _profile;
        }

        // finally, if the map is flat but the SRS is geographic, mark it as "plate carre"
        if (_profile->getSRS()->isGeographic() && 
            getMapOptions().coordSysType() == MapOptions::CSTYPE_PROJECTED)
        {
            OE_INFO << LC << "Projected display with geographic SRS; activating Plate Carre mode" << std::endl;
            const_cast<Profile*>(_profile.get())->overrideSRS(
                _profile->getSRS()->createPlateCarreGeographicSRS() );
        }
    }
}

namespace
{
    typedef std::pair<ElevationLayer*, GeoHeightField> GeoHFPair;

    /**
     * Returns a heightfield corresponding to the input key by compositing
     * elevation data for a vector of elevation layers. The resulting 
     * heightfield's height values will be expressed relative to the vertical
     * datum in the requesting key (which is usually that of the Map itself).
     */
    bool
    s_getHeightField(const TileKey&                  key,
                     const ElevationLayerVector&     elevLayers,
                     bool                            fallback,
                     const Profile*                  haeProfile,
                     ElevationInterpolation          interpolation,
                     ElevationSamplePolicy           samplePolicy,
                     osg::ref_ptr<osg::HeightField>& out_result,
                     bool*                           out_isFallback,
                     ProgressCallback*               progress ) 
    {
        unsigned lowestLOD = key.getLevelOfDetail();
        bool hfInitialized = false;

        //Get a HeightField for each of the enabled layers
	    GeoHeightFieldVector heightFields;

        unsigned int numValidHeightFields = 0;

        if ( out_isFallback )
        {
            *out_isFallback = false;
        }

        // if the caller provided an "HAE map profile", he wants an HAE elevation grid even if
        // the map profile has a vertical datum. This is the usual case when building the 3D
        // terrain, for example. Construct a temporary key that doesn't have the vertical
        // datum info and use that to query the elevation data.
        TileKey keyToUse = key;
        if ( haeProfile )
        {
            keyToUse = TileKey(key.getLevelOfDetail(), key.getTileX(), key.getTileY(), haeProfile );
        }

        // Generate a heightfield for each elevation layer.
        for( ElevationLayerVector::const_iterator i = elevLayers.begin(); i != elevLayers.end(); i++ )
        {
            ElevationLayer* layer = i->get();
            if ( layer->getEnabled() )
            {
                GeoHeightField geoHF = layer->createHeightField( keyToUse, progress );

                // if "fallback" is set, try to fall back on lower LODs.
                if ( !geoHF.valid() && fallback )
                {
                    TileKey hf_key = keyToUse;

                    while ( hf_key.valid() && !geoHF.valid() )
                    {
                        geoHF = layer->createHeightField( hf_key, progress );
                        if ( !geoHF.valid() )
                            hf_key = hf_key.createParentKey();
                    }

                    if ( geoHF.valid() )
                    {
                        if ( hf_key.getLevelOfDetail() < lowestLOD )
                            lowestLOD = hf_key.getLevelOfDetail();

                        if ( out_isFallback )
                            *out_isFallback = true;
                    }
                }

                if ( geoHF.valid() )
                {
                    heightFields.push_back( geoHF );
                }
            }
        }

        // If we didn't get any heightfields and weren't requested to fallback, just return NULL
        if ( heightFields.size() == 0 )
        {
            return false;
        }

	    else if (heightFields.size() == 1)
	    {
            if ( lowestLOD == key.getLevelOfDetail() )
            {
		        //If we only have on heightfield, just return it.
		        out_result = heightFields[0].takeHeightField();
            }
            else
            {
                GeoHeightField geoHF = heightFields[0].createSubSample( key.getExtent(), interpolation);
                out_result = geoHF.takeHeightField();
                hfInitialized = true;
            }
	    }

	    else
	    {
		    //If we have multiple heightfields, we need to composite them together.
		    unsigned int width = 0;
		    unsigned int height = 0;

		    for (GeoHeightFieldVector::const_iterator i = heightFields.begin(); i < heightFields.end(); ++i)
		    {
			    if (i->getHeightField()->getNumColumns() > width) 
                    width = i->getHeightField()->getNumColumns();
			    if (i->getHeightField()->getNumRows() > height) 
                    height = i->getHeightField()->getNumRows();
		    }
		    out_result = new osg::HeightField();
		    out_result->allocate( width, height );

		    //Go ahead and set up the heightfield so we don't have to worry about it later
            double minx, miny, maxx, maxy;
            key.getExtent().getBounds(minx, miny, maxx, maxy);
            double dx = (maxx - minx)/(double)(out_result->getNumColumns()-1);
            double dy = (maxy - miny)/(double)(out_result->getNumRows()-1);

            const SpatialReference* keySRS = keyToUse.getProfile()->getSRS();
            
		    //Create the new heightfield by sampling all of them.
            for (unsigned int c = 0; c < width; ++c)
            {
                double x = minx + (dx * (double)c);
                for (unsigned r = 0; r < height; ++r)
                {
                    double y = miny + (dy * (double)r);

                    //Collect elevations from all of the layers. Iterate BACKWARDS because the last layer
                    // is the highest priority.
                    std::vector<float> elevations;
                    for( GeoHeightFieldVector::reverse_iterator itr = heightFields.rbegin(); itr != heightFields.rend(); ++itr )
                    {
                        const GeoHeightField& geoHF = *itr;

                        float elevation = 0.0f;
                        if ( geoHF.getElevation(keySRS, x, y, interpolation, keySRS, elevation) )
                        {
                            if (elevation != NO_DATA_VALUE)
                            {
                                elevations.push_back(elevation);
                            }
                        }
                    }

                    float elevation = NO_DATA_VALUE;

                    //The list of elevations only contains valid values
                    if (elevations.size() > 0)
                    {
                        if (samplePolicy == SAMPLE_FIRST_VALID)
                        {
                            elevation = elevations[0];
                        }
                        else if (samplePolicy == SAMPLE_HIGHEST)
                        {
                            elevation = -FLT_MAX;
                            for (unsigned int i = 0; i < elevations.size(); ++i)
                            {
                                if (elevation < elevations[i]) elevation = elevations[i];
                            }
                        }
                        else if (samplePolicy == SAMPLE_LOWEST)
                        {
                            elevation = FLT_MAX;
                            for (unsigned i = 0; i < elevations.size(); ++i)
                            {
                                if (elevation > elevations[i]) elevation = elevations[i];
                            }
                        }
                        else if (samplePolicy == SAMPLE_AVERAGE)
                        {
                            elevation = 0.0;
                            for (unsigned i = 0; i < elevations.size(); ++i)
                            {
                                elevation += elevations[i];
                            }
                            elevation /= (float)elevations.size();
                        }
                    }
                    out_result->setHeight(c, r, elevation);
                }
            }
	    }

        // Replace any NoData areas with the reference value. This is zero for HAE datums,
        // and some geoid height for orthometric datums.
	    if (out_result.valid())
	    {
            const Geoid*         geoid = 0L;
            const VerticalDatum* vdatum = key.getProfile()->getSRS()->getVerticalDatum();

            if ( haeProfile && vdatum )
            {
                geoid = vdatum->getGeoid();
            }

            HeightFieldUtils::resolveInvalidHeights(
                out_result.get(),
                key.getExtent(),
                NO_DATA_VALUE,
                geoid );

		    //ReplaceInvalidDataOperator o;
		    //o.setValidDataOperator(new osgTerrain::NoDataValue(NO_DATA_VALUE));
		    //o( out_result.get() );
	    }

	    //Initialize the HF values for osgTerrain
	    if (out_result.valid() && !hfInitialized )
	    {	
		    //Go ahead and set up the heightfield so we don't have to worry about it later
		    double minx, miny, maxx, maxy;
		    key.getExtent().getBounds(minx, miny, maxx, maxy);
		    out_result->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
		    double dx = (maxx - minx)/(double)(out_result->getNumColumns()-1);
		    double dy = (maxy - miny)/(double)(out_result->getNumRows()-1);
		    out_result->setXInterval( dx );
		    out_result->setYInterval( dy );
		    out_result->setBorderWidth( 0 );
	    }

	    return out_result.valid();
    }
}


bool
Map::getHeightField(const TileKey&                  key,
                    bool                            fallback,
                    osg::ref_ptr<osg::HeightField>& out_result,
                    bool*                           out_isFallback,
                    bool                            convertToHAE,
                    ElevationSamplePolicy           samplePolicy,
                    ProgressCallback*               progress) const
{
    Threading::ScopedReadLock lock( const_cast<Map*>(this)->_mapDataMutex );

    ElevationInterpolation interp = getMapOptions().elevationInterpolation().get();    

    return s_getHeightField(
        key, 
        _elevationLayers, 
        fallback, 
        convertToHAE ? _profileNoVDatum.get() : 0L,
        interp, 
        samplePolicy, 
        out_result,  
        out_isFallback,
        progress );
}

bool
Map::sync( MapFrame& frame ) const
{
    bool result = false;

    if ( frame._mapDataModelRevision != _dataModelRevision || !frame._initialized )
    {
        // hold the read lock while copying the layer lists.
        Threading::ScopedReadLock lock( const_cast<Map*>(this)->_mapDataMutex );

        if ( frame._parts & IMAGE_LAYERS )
        {
            if ( !frame._initialized )
                frame._imageLayers.reserve( _imageLayers.size() );
            frame._imageLayers.clear();
            std::copy( _imageLayers.begin(), _imageLayers.end(), std::back_inserter(frame._imageLayers) );
        }

        if ( frame._parts & ELEVATION_LAYERS )
        {
            if ( !frame._initialized )
                frame._elevationLayers.reserve( _elevationLayers.size() );
            frame._elevationLayers.clear();
            std::copy( _elevationLayers.begin(), _elevationLayers.end(), std::back_inserter(frame._elevationLayers) );
        }

        if ( frame._parts & MODEL_LAYERS )
        {
            if ( !frame._initialized )
                frame._modelLayers.reserve( _modelLayers.size() );
            frame._modelLayers.clear();
            std::copy( _modelLayers.begin(), _modelLayers.end(), std::back_inserter(frame._modelLayers) );
        }

        if ( frame._parts & MASK_LAYERS )
        {
          if ( !frame._initialized )
              frame._maskLayers.reserve( _terrainMaskLayers.size() );
          frame._maskLayers.clear();
          std::copy( _terrainMaskLayers.begin(), _terrainMaskLayers.end(), std::back_inserter(frame._maskLayers) );
        }

        // sync the revision numbers.
        frame._initialized = true;
        frame._mapDataModelRevision = _dataModelRevision;
            
        result = true;
    }    
    return result;
}

bool
Map::toMapPoint( const GeoPoint& input, GeoPoint& output ) const
{
    return MapInfo(this).toMapPoint(input, output);
}

bool
Map::toWorldPoint( const GeoPoint& input, osg::Vec3d& output ) const
{
    return MapInfo(this).toWorldPoint(input, output);
}

bool
Map::worldPointToMapPoint( const osg::Vec3d& input, GeoPoint& output ) const
{
    return MapInfo(this).worldPointToMapPoint(input, output);
}

//------------------------------------------------------------------------

bool
MapInfo::toMapPoint( const GeoPoint& input, GeoPoint& output ) const
{
    return input.isValid() ? input.transform(_profile->getSRS(), output) : false;
}

bool
MapInfo::toWorldPoint( const GeoPoint& input, osg::Vec3d& output ) const
{
    return input.isValid() ?
        input.getSRS()->transformToWorld(input.vec3d(), output) :
        false;
}

bool
MapInfo::worldPointToMapPoint( const osg::Vec3d& input, GeoPoint& output ) const
{
    osg::Vec3d temp;
    bool ok = _profile->getSRS()->transformFromWorld(input, temp);
    if ( ok )
        output.set(_profile->getSRS(), temp);
    return ok;
}

//------------------------------------------------------------------------

MapFrame::MapFrame( const Map* map, Map::ModelParts parts, const std::string& name ) :
_initialized( false ),
_map        ( map ),
_name       ( name ),
_mapInfo    ( map ),
_parts      ( parts )
{
    sync();
}

MapFrame::MapFrame( const MapFrame& src, const std::string& name ) :
_initialized         ( src._initialized ),
_map                 ( src._map.get() ),
_name                ( name ),
_mapInfo             ( src._mapInfo ),
_parts               ( src._parts ),
_mapDataModelRevision( src._mapDataModelRevision ),
_imageLayers         ( src._imageLayers ),
_elevationLayers     ( src._elevationLayers ),
_modelLayers         ( src._modelLayers ),
_maskLayers          ( src._maskLayers )
{
    //no sync required here; we copied the arrays etc
}

bool
MapFrame::sync()
{
    return _map->sync( *this );
}

bool
MapFrame::getHeightField(const TileKey&                  key,
                         bool                            fallback,
                         osg::ref_ptr<osg::HeightField>& out_hf,
                         bool*                           out_isFallback,    
                         bool                            convertToHAE,
                         ElevationSamplePolicy           samplePolicy,
                         ProgressCallback*               progress) const
{
    return s_getHeightField( 
        key, 
        _elevationLayers,
        fallback, 
        convertToHAE ? _map->getProfileNoVDatum() : 0L,
        _mapInfo.getElevationInterpolation(), 
        samplePolicy, 
        out_hf, 
        out_isFallback,
        progress );
}

int
MapFrame::indexOf( ImageLayer* layer ) const
{
    ImageLayerVector::const_iterator i = std::find( _imageLayers.begin(), _imageLayers.end(), layer );
    return i != _imageLayers.end() ? i - _imageLayers.begin() : -1;
}

int
MapFrame::indexOf( ElevationLayer* layer ) const
{
    ElevationLayerVector::const_iterator i = std::find( _elevationLayers.begin(), _elevationLayers.end(), layer );
    return i != _elevationLayers.end() ? i - _elevationLayers.begin() : -1;
}

int
MapFrame::indexOf( ModelLayer* layer ) const
{
    ModelLayerVector::const_iterator i = std::find( _modelLayers.begin(), _modelLayers.end(), layer );
    return i != _modelLayers.end() ? i - _modelLayers.begin() : -1;
}

ImageLayer*
MapFrame::getImageLayerByUID( UID uid ) const
{
    for(ImageLayerVector::const_iterator i = _imageLayers.begin(); i != _imageLayers.end(); ++i )
        if ( i->get()->getUID() == uid )
            return i->get();
    return 0L;
}

ImageLayer*
MapFrame::getImageLayerByName( const std::string& name ) const
{
    for(ImageLayerVector::const_iterator i = _imageLayers.begin(); i != _imageLayers.end(); ++i )
        if ( i->get()->getName() == name )
            return i->get();
    return 0L;
}

bool
MapFrame::isCached( const TileKey& key ) const
{
    //Check to see if the tile will load fast
    // Check the imagery layers
    for( ImageLayerVector::const_iterator i = imageLayers().begin(); i != imageLayers().end(); i++ )
    {   
        //If we're cache only we should be fast
        if (i->get()->isCacheOnly()) continue;

        osg::ref_ptr< TileSource > source = i->get()->getTileSource();
        if (!source.valid()) continue;

        //If the tile is blacklisted, it should also be fast.
        if ( source->getBlacklist()->contains( key.getTileId() ) ) continue;
        //If no data is available on this tile, we'll be fast
        if ( !source->hasData( key ) ) continue;

        if ( !i->get()->isCached( key ) ) return false;
    }

    for( ElevationLayerVector::const_iterator i = elevationLayers().begin(); i != elevationLayers().end(); ++i )
    {
        //If we're cache only we should be fast
        if (i->get()->isCacheOnly()) continue;

        osg::ref_ptr< TileSource > source = i->get()->getTileSource();
        if (!source.valid()) continue;

        //If the tile is blacklisted, it should also be fast.
        if ( source->getBlacklist()->contains( key.getTileId() ) ) continue;
        if ( !source->hasData( key ) ) continue;
        if ( !i->get()->isCached( key ) )
        {
            return false;
        }
    }

    return true;        
}
