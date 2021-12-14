/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/MapModelChange>
#include <osgEarth/Registry>

using namespace osgEarth;

#define LC "[Map] "

//...................................................................

Map::LayerCB::LayerCB(Map* map) : _map(map) { }

void Map::LayerCB::onOpen(Layer* layer)
{
    osg::ref_ptr<Map> map;
    if (_map.lock(map))
        map->notifyOnLayerOpenOrClose(layer);
}

void Map::LayerCB::onClose(Layer* layer)
{
    osg::ref_ptr<Map> map;
    if (_map.lock(map))
        map->notifyOnLayerOpenOrClose(layer);
}

//...................................................................

Config
Map::Options::getConfig() const
{
    Config conf = ConfigOptions::getConfig();

    conf.set( "name",         name() );
    conf.set( "profile",      profile() );
    conf.set( "cache",        cache() );
    conf.set( "cache_policy", cachePolicy() );

    conf.set( "elevation_interpolation", "nearest",     elevationInterpolation(), INTERP_NEAREST);
    conf.set( "elevation_interpolation", "average",     elevationInterpolation(), INTERP_AVERAGE);
    conf.set( "elevation_interpolation", "bilinear",    elevationInterpolation(), INTERP_BILINEAR);
    conf.set( "elevation_interpolation", "triangulate", elevationInterpolation(), INTERP_TRIANGULATE);

    conf.set( "profile_layer", profileLayer() );

    return conf;
}

void
Map::Options::fromConfig(const Config& conf)
{
    elevationInterpolation().init(INTERP_BILINEAR);
    
    conf.get( "name",         name() );
    conf.get( "profile",      profile() );
    conf.get( "cache",        cache() );  
    conf.get( "cache_policy", cachePolicy() );

    // legacy support:
    if ( conf.value<bool>( "cache_only", false ) == true )
        cachePolicy()->usage() = CachePolicy::USAGE_CACHE_ONLY;

    if ( conf.value<bool>( "cache_enabled", true ) == false )
        cachePolicy()->usage() = CachePolicy::USAGE_NO_CACHE;

    conf.get( "elevation_interpolation", "nearest",     elevationInterpolation(), INTERP_NEAREST);
    conf.get( "elevation_interpolation", "average",     elevationInterpolation(), INTERP_AVERAGE);
    conf.get( "elevation_interpolation", "bilinear",    elevationInterpolation(), INTERP_BILINEAR);
    conf.get( "elevation_interpolation", "triangulate", elevationInterpolation(), INTERP_TRIANGULATE);

    conf.get( "profile_layer", profileLayer() );
}

//...................................................................

Map::Map() :
osg::Object()
{
    init();
}

Map::Map(const osgDB::Options* readOptions) :
    osg::Object(),
    _readOptions(readOptions ? osg::clone(readOptions) : nullptr)
{
    init();
}

Map::Map(const Map::Options& options, const osgDB::Options* readOptions) :
    osg::Object(),
    _optionsConcrete(options),
    _readOptions(readOptions ? osg::clone(readOptions) : nullptr)
{
    init();
}

void
Map::init()
{
    // reset the revision:
    _dataModelRevision = 0;

    _mapDataMutex.setName("Map dataMutex(OE)");

    // set the object name from the options:
    if (options().name().isSet())
        osg::Object::setName(options().name().get());
    else
        osg::Object::setName("osgEarth.Map");

    // Generate a UID.
    _uid = Registry::instance()->createUID();

    // Set up the map's profile
    if (options().profile().isSet())
        setProfile(Profile::create(options().profile().get()));

    if (getProfile() == nullptr)
        setProfile(Profile::create(Profile::GLOBAL_GEODETIC));

    // If the registry doesn't have a default cache policy, but the
    // map options has one, make the map policy the default.
    if (options().cachePolicy().isSet() &&
        !Registry::instance()->defaultCachePolicy().isSet())
    {
        Registry::instance()->setDefaultCachePolicy( options().cachePolicy().get() );
        OE_INFO << LC
            << "Setting default cache policy from map ("
            << options().cachePolicy()->usageString() << ")" << std::endl;
    }

    // the map-side dbOptions object holds I/O information for all components.
    if (!_readOptions.valid())
    {
        _readOptions = new osgDB::Options();
    }

    // put the CacheSettings object in there. We will propogate this throughout
    // the data model and the renderer. These will be stored in the readOptions
    // (and ONLY there)
    _cacheSettings = new CacheSettings();

    // Set up a cache if there's one in the options:
    if (options().cache().isSet())
        _cacheSettings->setCache(CacheFactory::create(options().cache().get()));

    // Otherwise use the registry default cache if there is one:
    if (_cacheSettings->getCache() == nullptr)
        _cacheSettings->setCache(Registry::instance()->getDefaultCache());

    // Integrate local cache policy (which can be overridden by the environment)
    _cacheSettings->integrateCachePolicy(options().cachePolicy());

    // store in the options so we can propagate it to layers, etc.
    _cacheSettings->store(_readOptions.get());
    OE_INFO << LC << _cacheSettings->toString() << "\n";

    // remember the referrer for relative-path resolution:
    URIContext( options().referrer() ).store( _readOptions.get() );

    // create a callback that the Map will use to detect setEnabled calls
    _layerCB = new LayerCB(this);

    // elevation sampling
    _elevationPool = new ElevationPool();
    _elevationPool->setMap( this );

    _numTerrainPatchLayers = 0;
}

Map::~Map()
{
    OE_DEBUG << LC << "~Map" << std::endl;
}

ElevationPool*
Map::getElevationPool() const
{
    return _elevationPool.get();
}

void
Map::notifyOnLayerOpenOrClose(Layer* layer)
{
    // bump the revision safely:
    Revision newRevision;
    {
        Threading::ScopedWriteLock lock(_mapDataMutex);
        newRevision = ++_dataModelRevision;
    }

    // reinitialize the elevation pool:
    if (dynamic_cast<ElevationLayer*>(layer) ||
        dynamic_cast<TerrainConstraintLayer*>(layer))
    {
        _elevationPool->clear();
    }

    if (layer->isOpen())
    {
        if (getProfile())
        {
            layer->addedToMap(this);
        }
    }
    else
    {
        layer->removedFromMap(this);
    }

    MapModelChange change(
        layer->isOpen() ? MapModelChange::OPEN_LAYER : MapModelChange::CLOSE_LAYER,
        newRevision,
        layer);

    for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
    {
        i->get()->onMapModelChanged(change);
    }
}

void
Map::setMapName( const std::string& name )
{
    _name = name;
}

Revision
Map::getDataModelRevision() const
{
    return _dataModelRevision;
}

void
Map::setProfile(const Profile* value)
{
    bool notifyLayers = !_profile.valid();

    if (value)
    {
        _profile = value;

        // create a "proxy" profile to use when querying elevation layers with a vertical datum
        if (_profile.valid() && _profile->getSRS()->getVerticalDatum() != 0L )
        {
            ProfileOptions po = _profile->toProfileOptions();
            po.vsrsString().unset();
            _profileNoVDatum = Profile::create(po);
        }
        else
        {
            _profileNoVDatum = _profile;
        }

        // finally, fire an event if the profile has been set.
        OE_INFO << LC << "Map profile is: " << _profile->toString() << std::endl;
    }

    // If we just set the profile, tell all our layers they are now added
    // to a valid map.
    if (_profile.valid() && notifyLayers)
    {
        for(LayerVector::iterator i = _layers.begin(); i != _layers.end(); ++i)
        {
            Layer* layer = i->get();
            if (layer->isOpen())
            {
                layer->addedToMap(this);
            }
        }
    }
}

const Profile*
Map::getProfile() const
{
    return _profile.get();
}

void
Map::setCachePolicy(const CachePolicy& value)
{
    options().cachePolicy() = value;

    CacheSettings* cacheSettings = CacheSettings::get(_readOptions.get());
    if (cacheSettings)
    {
        cacheSettings->integrateCachePolicy(value);
    }
}

const CachePolicy&
Map::getCachePolicy() const
{
    return options().cachePolicy().get();
}

void
Map::setElevationInterpolation(const RasterInterpolation& value)
{
    options().elevationInterpolation() = value;
}

const RasterInterpolation&
Map::getElevationInterpolation() const
{
    return options().elevationInterpolation().get();
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
Map::getAttributions(StringSet& attributions) const
{
    LayerVector layers;
    getLayers(layers);

    for (LayerVector::const_iterator itr = layers.begin(); itr != layers.end(); ++itr)
    {
        if (itr->get()->isOpen())
        {
            VisibleLayer* visibleLayer = dynamic_cast<VisibleLayer*>(itr->get());
            if (!visibleLayer || visibleLayer->getVisible())
            {
                std::string attribution = itr->get()->getAttribution();
                if (!attribution.empty())
                {
                    attributions.insert(attribution);
                }
            }
        }
    }
}

MapCallback*
Map::addMapCallback(MapCallback* cb) const
{
    if ( cb )
        _mapCallbacks.push_back( cb );
    return cb;
}

void
Map::removeMapCallback(MapCallback* cb) const
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
Map::addLayer(Layer* layer)
{
    if (layer == NULL)
        return;

    // ensure it's not already in the map
    if (getIndexOfLayer(layer) != getNumLayers())
        return;

    // Store in a ref_ptr for scope to ensure callbacks don't accidentally delete while adding
    osg::ref_ptr<Layer> layerRef( layer );

    //osgEarth::Registry::instance()->clearBlacklist();

    layer->setReadOptions(getReadOptions());

    if (layer->getEnabled())
    {
        layer->open();
    }

    // do we need this? Won't the callback to this?
    if (layer->isOpen() && getProfile() != NULL)
    {
        layer->addedToMap(this);
    }

    // Set up callbacks. Do this *after* calling addedToMap (since the callback invokes addedToMap)
    installLayerCallbacks(layer);

    // Add the layer to our stack.
    int newRevision;
    unsigned index = -1;
    {
        Threading::ScopedWriteLock lock( _mapDataMutex );

        _layers.push_back( layer );
        index = _layers.size() - 1;
        newRevision = ++_dataModelRevision;

        if (layer->options().terrainPatch() == true)
            ++_numTerrainPatchLayers;
    }

    // a separate block b/c we don't need the mutex
    for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
    {
        i->get()->onMapModelChanged(MapModelChange(
            MapModelChange::ADD_LAYER, newRevision, layer, index));
    }
}

void
Map::insertLayer(Layer* layer, unsigned index)
{
    if (layer == NULL)
        return;

    // ensure it's not already in the map
    if (getIndexOfLayer(layer) != getNumLayers())
        return;

    // Store in a ref_ptr for scope to ensure callbacks don't accidentally delete while adding
    osg::ref_ptr<Layer> layerRef( layer );

    //osgEarth::Registry::instance()->clearBlacklist();

    layer->setReadOptions(getReadOptions());

    if (layer->getEnabled())
    {
        layer->open();
    }

    if (layer->isOpen() && getProfile() != NULL)
    {
        layer->addedToMap(this);
    }

    // Set up callbacks. Do this *after* calling addedToMap (since the callback invokes addedToMap)
    installLayerCallbacks(layer);

    // Add the layer to our stack.
    int newRevision;
    {
        Threading::ScopedWriteLock lock(_mapDataMutex);

        if (index >= _layers.size())
            _layers.push_back(layer);
        else
            _layers.insert(_layers.begin() + index, layer);

        newRevision = ++_dataModelRevision;

        if (layer->options().terrainPatch() == true)
            ++_numTerrainPatchLayers;
    }

    // a separate block b/c we don't need the mutex
    for (MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++)
    {
        i->get()->onMapModelChanged(MapModelChange(
            MapModelChange::ADD_LAYER, newRevision, layer, index));
    }
}

void
Map::removeLayer(Layer* layer)
{
    if (layer == NULL)
        return;

    // ensure it's in the map
    if (getIndexOfLayer(layer) == getNumLayers())
        return;

    //osgEarth::Registry::instance()->clearBlacklist();
    unsigned int index = -1;

    osg::ref_ptr<Layer> layerToRemove(layer);
    Revision newRevision;

    uninstallLayerCallbacks(layerToRemove.get());

    layer->removedFromMap(this);

    layer->close();

    if ( layerToRemove.get() )
    {
        Threading::ScopedWriteLock lock( _mapDataMutex );
        index = 0;
        for( LayerVector::iterator i = _layers.begin(); i != _layers.end(); i++, index++ )
        {
            if ( i->get() == layerToRemove.get() )
            {
                _layers.erase( i );
                newRevision = ++_dataModelRevision;

                if (layer->options().terrainPatch() == true)
                    --_numTerrainPatchLayers;

                break;
            }
        }
    }

    // a separate block b/c we don't need the mutex
    if ( newRevision >= 0 )
    {
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::REMOVE_LAYER, newRevision, layerToRemove.get(), index) );
        }
    }
}

void
Map::moveLayer(Layer* layer, unsigned newIndex)
{
    unsigned int oldIndex = 0;
    unsigned int actualIndex = 0;
    Revision newRevision;

    if ( layer )
    {
        Threading::ScopedWriteLock lock( _mapDataMutex );

        // preserve the layer with a ref:
        osg::ref_ptr<Layer> layerToMove( layer );

        // find it:
        LayerVector::iterator i_oldIndex = _layers.end();
        for( LayerVector::iterator i = _layers.begin(); i != _layers.end(); i++, actualIndex++ )
        {
            if ( i->get() == layer )
            {
                i_oldIndex = i;
                oldIndex = actualIndex;
                break;
            }
        }

        if ( i_oldIndex == _layers.end() )
            return; // layer not found in list

        // erase the old one and insert the new one.
        _layers.erase( i_oldIndex );
        _layers.insert( _layers.begin() + newIndex, layerToMove.get() );

        newRevision = ++_dataModelRevision;
    }

    // if this is an elevation layer, invalidate the elevation pool
    if (dynamic_cast<ElevationLayer*>(layer) ||
        dynamic_cast<TerrainConstraintLayer*>(layer))
    {
        getElevationPool()->clear();
    }

    // a separate block b/c we don't need the mutex
    if ( layer )
    {
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged( MapModelChange(
                MapModelChange::MOVE_LAYER, newRevision, layer, oldIndex, newIndex) );
        }
    }
}

void
Map::addLayers(const LayerVector& layers)
{
    // This differs from addLayer() in a loop because it will
    // (a) call addedToMap only after all the layers are added, and
    // (b) invoke all the MapModelChange callbacks with the same 
    // new revision number.

    //osgEarth::Registry::instance()->clearBlacklist();

    for(LayerVector::const_iterator layerRef = layers.begin();
        layerRef != layers.end();
        ++layerRef)
    {
        Layer* layer = layerRef->get();
        if ( !layer )
            continue;

        layer->setReadOptions(getReadOptions());

        // open, but don't call addedToMap(layer) yet.
        if (layer->getEnabled())
        {
            layer->open();
        }
    }

    unsigned firstIndex;
    unsigned count = 0;
    int newRevision;

    // Add the layers to the map.
    {
        Threading::ScopedWriteLock lock( _mapDataMutex );

        firstIndex = _layers.size();
        newRevision = ++_dataModelRevision;

        for(LayerVector::const_iterator layerRef = layers.begin();
            layerRef != layers.end();
            ++layerRef)
        {
            Layer* layer = layerRef->get();
            if ( !layer )
                continue;

            _layers.push_back( layer );
        }
    }

    // call addedToMap on each new layer in turn:
    unsigned index = firstIndex;

    for(LayerVector::const_iterator layerRef = layers.begin();
        layerRef != layers.end();
        ++layerRef)
    {
        Layer* layer = layerRef->get();
        if ( !layer )
            continue;

        if (layer->isOpen() && getProfile() != NULL)
        {
            layer->addedToMap(this);
        }

        // Set up callbacks.
        installLayerCallbacks(layer);

        // a separate block b/c we don't need the mutex
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged(MapModelChange(
                MapModelChange::ADD_LAYER, newRevision, layer, index++));
        }
    }
}

void
Map::installLayerCallbacks(Layer* layer)
{
    // If this is an elevation layer, install a callback so we know when
    // it's enabled state changes:
    ElevationLayer* elevationLayer = dynamic_cast<ElevationLayer*>(layer);
    if (elevationLayer)
    {
        // invalidate the elevation pool
        getElevationPool()->clear();
    }

    // Callback to detect changes in "enabled"
    layer->addCallback(_layerCB.get());
}

void
Map::uninstallLayerCallbacks(Layer* layer)
{
    // undo the things we did in prepareLayer:
    ElevationLayer* elevationLayer = dynamic_cast<ElevationLayer*>(layer);
    if (elevationLayer)
    {
        // invalidate the pool
        getElevationPool()->clear();
    }

    layer->removeCallback(_layerCB.get());
}

Revision
Map::getLayers(LayerVector& out_list) const
{
    out_list.reserve( _layers.size() );

    Threading::ScopedReadLock lock(_mapDataMutex);
    for( LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i )
        out_list.push_back( i->get() );

    return _dataModelRevision;
}

unsigned
Map::getNumLayers() const
{
    Threading::ScopedReadLock lock( _mapDataMutex );
    return _layers.size();
}

Layer*
Map::getLayerByName(const std::string& name) const
{
    Threading::ScopedReadLock lock( _mapDataMutex );
    for(LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
        if ( i->get()->getName() == name )
            return i->get();
    return 0L;
}

Layer*
Map::getLayerByUID(UID layerUID) const
{
    Threading::ScopedReadLock lock( _mapDataMutex );
    for( LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i )
        if ( i->get()->getUID() == layerUID )
            return i->get();
    return 0L;
}

Layer*
Map::getLayerAt(unsigned index) const
{
    Threading::ScopedReadLock lock( _mapDataMutex );
    if ( index >= 0 && index < (int)_layers.size() )
        return _layers[index].get();
    else
        return 0L;
}

unsigned
Map::getIndexOfLayer(const Layer* layer) const
{
    Threading::ScopedReadLock lock( _mapDataMutex );
    unsigned index = 0;
    for (; index < _layers.size(); ++index)
    {
        if (_layers[index] == layer)
            break;
    }
    return index;
}


void
Map::clear()
{
    LayerVector layersRemoved;
    Revision newRevision;
    {
        Threading::ScopedWriteLock lock( _mapDataMutex );

        layersRemoved.swap( _layers );

        // calculate a new revision.
        newRevision = ++_dataModelRevision;
    }

    // a separate block b/c we don't need the mutex
    for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
    {
        i->get()->onBeginUpdate();

        for(LayerVector::iterator layer = layersRemoved.begin();
            layer != layersRemoved.end();
            ++layer)
        {
            i->get()->onMapModelChanged(MapModelChange(MapModelChange::REMOVE_LAYER, newRevision, layer->get()));
        }

        i->get()->onEndUpdate();
    }

    // Invalidate the elevation pool.
    getElevationPool()->clear();
}

const SpatialReference*
Map::getSRS() const
{
    return _profile.valid() ? _profile->getSRS() : 0L;
}

const SpatialReference*
Map::getWorldSRS() const
{
    return getSRS() && getSRS()->isGeographic() ? getSRS()->getGeocentricSRS() : getSRS();
}

bool
Map::isFast(const TileKey& key, const LayerVector& layers) const
{
    if (getCache() == NULL)
        return false;

    for (LayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i)
    {
        Layer* layer = i->get();
        if (!layer)
            continue;

        if (!layer->isOpen())
            continue;

        TileLayer* tilelayer = dynamic_cast<TileLayer*>(layer);
        if (tilelayer)
        {
            if (tilelayer->getCacheSettings()->cachePolicy()->isCacheDisabled())
              return false;

            //If no data is available on this tile, we'll be fast
            if (!tilelayer->mayHaveData(key))
                continue;

            if (!tilelayer->isCached(key))
                return false;
        }
    }
    return true;
}

int 
Map::getNumTerrainPatchLayers() const
{
    return _numTerrainPatchLayers;
}
