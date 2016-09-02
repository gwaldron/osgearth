/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/MapFrame>
#include <osgEarth/MapModelChange>
#include <osgEarth/Registry>
#include <osgEarth/TileSource>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/URI>
#include <iterator>

using namespace osgEarth;

#define LC "[Map] "

//------------------------------------------------------------------------

Map::ElevationLayerCB::ElevationLayerCB(Map* map) :
_map(map)
{
    //nop
}

void
Map::ElevationLayerCB::onVisibleChanged(TerrainLayer* layer)
{
    osg::ref_ptr<Map> map;
    if ( _map.lock(map) )
    {
        _map->notifyElevationLayerVisibleChanged(layer);
    }
}

//------------------------------------------------------------------------

Map::Map( const MapOptions& options ) :
osg::Referenced      ( true ),
_mapOptions          ( options ),
_initMapOptions      ( options ),
_dataModelRevision   ( 0 )
{
    // Generate a UID.
    _uid = Registry::instance()->createUID();

    // If the registry doesn't have a default cache policy, but the
    // map options has one, make the map policy the default.
    if (_mapOptions.cachePolicy().isSet() &&
        !Registry::instance()->defaultCachePolicy().isSet())
    {
        Registry::instance()->setDefaultCachePolicy( _mapOptions.cachePolicy().get() );
        OE_INFO << LC 
            << "Setting default cache policy from map ("
            << _mapOptions.cachePolicy()->usageString() << ")" << std::endl;
    }

    // the map-side dbOptions object holds I/O information for all components.
    _readOptions = osg::clone( Registry::instance()->getDefaultOptions() );

    // put the CacheSettings object in there. We will propogate this throughout
    // the data model and the renderer.
    CacheSettings* cacheSettings = new CacheSettings();

    // Set up a cache if there's one in the options:
    if (_mapOptions.cache().isSet())
        cacheSettings->setCache(CacheFactory::create(_mapOptions.cache().get()));

    // Otherwise use the registry default cache if there is one:
    if (cacheSettings->getCache() == 0L)
        cacheSettings->setCache(Registry::instance()->getDefaultCache());

    // Integrate local cache policy (which can be overridden by the environment)
    cacheSettings->integrateCachePolicy(_mapOptions.cachePolicy());

    // store in the options so we can propagate it to layers, etc.
    cacheSettings->store(_readOptions.get());

    OE_INFO << LC << cacheSettings->toString() << "\n";


    // remember the referrer for relative-path resolution:
    URIContext( _mapOptions.referrer() ).store( _readOptions.get() );

    // we do our own caching
    _readOptions->setObjectCacheHint( osgDB::Options::CACHE_NONE );

    // set up a callback that the Map will use to detect Elevation Layer
    // visibility changes
    _elevationLayerCB = new ElevationLayerCB(this);
}

Map::~Map()
{
    OE_DEBUG << "~Map" << std::endl;
}

void
Map::notifyElevationLayerVisibleChanged(TerrainLayer* layer)
{
    // bump the revision safely:
    Revision newRevision;
    {
        Threading::ScopedWriteLock lock( const_cast<Map*>(this)->_mapDataMutex );
        newRevision = ++_dataModelRevision;
    }

    // a separate block b/c we don't need the mutex   
    for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
    {
        i->get()->onMapModelChanged( MapModelChange(
            MapModelChange::TOGGLE_ELEVATION_LAYER, newRevision, layer) );
    }
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
    CacheSettings* cacheSettings = CacheSettings::get(_readOptions.get());
    return cacheSettings ? cacheSettings->getCache() : 0L;
}

void
Map::setCache(Cache* cache)
{
    // note- probably unsafe to do this after initializing the terrain. so don't.
    CacheSettings* cacheSettings = CacheSettings::get(_readOptions.get());
    if (cacheSettings && cacheSettings->getCache() != cache)
        cacheSettings->setCache(cache);
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
Map::beginUpdate()
{
    MapModelChange msg( MapModelChange::BEGIN_BATCH_UPDATE, _dataModelRevision );

    for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
    {
        i->get()->onMapModelChanged( msg );
    }
}

void
Map::endUpdate()
{
    MapModelChange msg( MapModelChange::END_BATCH_UPDATE, _dataModelRevision );
 
    for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
    {
        i->get()->onMapModelChanged( msg );
    }
}

void
Map::addImageLayer( ImageLayer* layer )
{
    osgEarth::Registry::instance()->clearBlacklist();
    unsigned int index = -1;
    if ( layer )
    {
        // Set the DB options for the map from the layer, including the cache policy.
        layer->setReadOptions( _readOptions.get() );

        // Tell the layer the map profile, if possible:
        if ( _profile.valid() )
        {
            layer->setTargetProfileHint( _profile.get() );
        }

        // open the layer:
        layer->open();

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
        layer->setReadOptions( _readOptions.get() );

        // Tell the layer the map profile, if possible:
        if ( _profile.valid() )
        {
            layer->setTargetProfileHint( _profile.get() );
        }

        // open the layer:
        layer->open();

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
        layer->setReadOptions( _readOptions.get() );

        // Tell the layer the map profile, if possible:
        if ( _profile.valid() )
            layer->setTargetProfileHint( _profile.get() );

        layer->open();

        int newRevision;

        // Add the layer to our stack.
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );

            _elevationLayers.push_back( layer );
            index = _elevationLayers.size() - 1;
            newRevision = ++_dataModelRevision;
        }

        // listen for changes in the layer.
        layer->addCallback( _elevationLayerCB.get() );

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

        layerToRemove->removeCallback( _elevationLayerCB.get() );
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

        // initialize the model layer
        layer->setReadOptions(_readOptions.get());

        // open it and check the status
        layer->open();

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

        // initialize the model layer
        layer->setReadOptions(_readOptions.get());

        // open and get the status
        layer->open();

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

        layer->setReadOptions(_readOptions.get());

        layer->open();

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
Map::clear()
{
    ImageLayerVector     imageLayersRemoved;
    ElevationLayerVector elevLayersRemoved;
    ModelLayerVector     modelLayersRemoved;
    MaskLayerVector      maskLayersRemoved;

    Revision newRevision;
    {
        Threading::ScopedWriteLock lock( _mapDataMutex );

        imageLayersRemoved.swap( _imageLayers );
        elevLayersRemoved.swap ( _elevationLayers );
        modelLayersRemoved.swap( _modelLayers );

        // calculate a new revision.
        newRevision = ++_dataModelRevision;
    }
    
    // a separate block b/c we don't need the mutex   
    for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
    {
        for( ImageLayerVector::iterator k = imageLayersRemoved.begin(); k != imageLayersRemoved.end(); ++k )
            i->get()->onMapModelChanged( MapModelChange(MapModelChange::REMOVE_IMAGE_LAYER, newRevision, k->get()) );
        for( ElevationLayerVector::iterator k = elevLayersRemoved.begin(); k != elevLayersRemoved.end(); ++k )
            i->get()->onMapModelChanged( MapModelChange(MapModelChange::REMOVE_ELEVATION_LAYER, newRevision, k->get()) );
        for( ModelLayerVector::iterator k = modelLayersRemoved.begin(); k != modelLayersRemoved.end(); ++k )
            i->get()->onMapModelChanged( MapModelChange(MapModelChange::REMOVE_MODEL_LAYER, newRevision, k->get()) );
    }
}


void
Map::setLayersFromMap( const Map* map )
{
    this->clear();

    if ( map )
    {
        ImageLayerVector newImages;
        map->getImageLayers( newImages );
        for( ImageLayerVector::iterator i = newImages.begin(); i != newImages.end(); ++i )
            addImageLayer( i->get() );

        ElevationLayerVector newElev;
        map->getElevationLayers( newElev );
        for( ElevationLayerVector::iterator i = newElev.begin(); i != newElev.end(); ++i )
            addElevationLayer( i->get() );

        ModelLayerVector newModels;
        map->getModelLayers( newModels );
        for( ModelLayerVector::iterator i = newModels.begin(); i != newModels.end(); ++i )
            addModelLayer( i->get() );
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
                        << "Map is geocentric, but the configured profile SRS ("
                        << userProfile->getSRS()->getName() << ") is not geographic; "
                        << "it will be ignored."
                        << std::endl;
                }
            }
        }
        else if ( _mapOptions.coordSysType() == MapOptions::CSTYPE_GEOCENTRIC_CUBE )
        {
            if ( userProfile.valid() )
            {
                if ( userProfile->isOK() && userProfile->getSRS()->isCube() )
                {
                    _profile = userProfile.get();
                }
                else
                {
                    OE_WARN << LC 
                        << "Map is geocentric cube, but the configured profile SRS ("
                        << userProfile->getSRS()->getName() << ") is not geocentric cube; "
                        << "it will be ignored."
                        << std::endl;
                }
            }
        }
        else // CSTYPE_PROJECTED
        {
            if ( userProfile.valid() )
            {
                if ( userProfile->isOK() && userProfile->getSRS()->isProjected() )
                {
                    _profile = userProfile.get();
                }
                else
                {
                    OE_WARN << LC 
                        << "Map is projected, but the configured profile SRS ("
                        << userProfile->getSRS()->getName() << ") is not projected; "
                        << "it will be ignored."
                        << std::endl;
                }
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

        // ensure that the profile we found is the correct kind
        // convert a geographic profile to Plate Carre if necessary
        if ( _mapOptions.coordSysType() == MapOptions::CSTYPE_GEOCENTRIC && !( _profile.valid() && _profile->getSRS()->isGeographic() ) )
        {
            // by default, set a geocentric map to use global-geodetic WGS84.
            _profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
        }
        else if ( _mapOptions.coordSysType() == MapOptions::CSTYPE_GEOCENTRIC_CUBE && !( _profile.valid() && _profile->getSRS()->isCube() ) )
        {
            //If the map type is a Geocentric Cube, set the profile to the cube profile.
            _profile = osgEarth::Registry::instance()->getCubeProfile();
        }
        else if ( _mapOptions.coordSysType() == MapOptions::CSTYPE_PROJECTED && _profile.valid() && _profile->getSRS()->isGeographic() )
        {
            OE_INFO << LC << "Projected map with geographic SRS; activating EQC profile" << std::endl;            
            unsigned u, v;
            _profile->getNumTiles(0, u, v);
            const osgEarth::SpatialReference* eqc = _profile->getSRS()->createEquirectangularSRS();
            osgEarth::GeoExtent e = _profile->getExtent().transform( eqc );
            _profile = osgEarth::Profile::create( eqc, e.xMin(), e.yMin(), e.xMax(), e.yMax(), u, v);
        }
        else if ( _mapOptions.coordSysType() == MapOptions::CSTYPE_PROJECTED && !( _profile.valid() && _profile->getSRS()->isProjected() ) )
        {
            // TODO: should there be a default projected profile?
            _profile = 0;
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
                if ( layer->getEnabled() == true )
                {
                    layer->setTargetProfileHint( _profile.get() );
                }
            }

            for( ElevationLayerVector::iterator i = _elevationLayers.begin(); i != _elevationLayers.end(); i++ )
            {
                ElevationLayer* layer = i->get();
                if ( layer->getEnabled() )
                {
                    layer->setTargetProfileHint( _profile.get() );
                }
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
    }
}

const SpatialReference*
Map::getWorldSRS() const
{
    return isGeocentric() ? getSRS()->getECEF() : getSRS();
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
            frame._elevationLayers = _elevationLayers;
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
