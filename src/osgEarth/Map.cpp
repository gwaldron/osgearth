/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarth/Utils>

using namespace osgEarth;

#define LC "[Map] "

//------------------------------------------------------------------------

Map::VisibleLayerCB::VisibleLayerCB(Map* map) : _map(map) { }

void Map::VisibleLayerCB::onVisibleChanged(VisibleLayer* layer) {
    osg::ref_ptr<Map> map;
    if ( _map.lock(map) )
        _map->notifyLayerVisibleChanged(layer);
}

Map::LayerCB::LayerCB(Map* map) : _map(map) { }

void Map::LayerCB::onEnabledChanged(Layer* layer) {
    osg::ref_ptr<Map> map;
    if (_map.lock(map))
        map->notifyOnLayerEnabledChanged(layer);
}

//------------------------------------------------------------------------

Map::Map() :
osg::Object(),
_dataModelRevision(0)
{
    ctor();
}

Map::Map( const MapOptions& options ) :
osg::Object(),
_mapOptions          ( options ),
_initMapOptions      ( options ),
_dataModelRevision   ( 0 )
{
    ctor();
}

void
Map::ctor()
{
    // Set the object name.
    osg::Object::setName("osgEarth.Map");

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
    // the data model and the renderer. These will be stored in the readOptions
    // (and ONLY there)
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

    // encode this map in the read options.
    OptionsData<const Map>::set(_readOptions.get(), "osgEarth.Map", this);

    // set up a callback that the Map will use to detect Layer visibility changes
    _visibleLayerCB = new VisibleLayerCB(this);

    // create a callback that the Map will use to detect setEnabled calls
    _layerCB = new LayerCB(this);

    // elevation sampling
    _elevationPool = new ElevationPool();
    _elevationPool->setMap( this );
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
Map::notifyLayerVisibleChanged(VisibleLayer* layer)
{
    // bump the revision safely:
    Revision newRevision;
    {
        Threading::ScopedWriteLock lock(_mapDataMutex);
        newRevision = ++_dataModelRevision;
    }

    ElevationLayer* elevationLayer = dynamic_cast<ElevationLayer*>(layer);
    if (elevationLayer)
    {
        // reinitialize the elevation pool:
        _elevationPool->clear();
    }

    MapModelChange change(
        MapModelChange::TOGGLE_LAYER,
        newRevision,
        layer);

    for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
    {
        i->get()->onMapModelChanged(change);
    }
}

void
Map::notifyOnLayerEnabledChanged(Layer* layer)
{
    // bump the revision safely:
    Revision newRevision;
    {
        Threading::ScopedWriteLock lock(_mapDataMutex);
        newRevision = ++_dataModelRevision;
    }

    // reinitialize the elevation pool:
    if (dynamic_cast<ElevationLayer*>(layer))
    {
        _elevationPool->clear();
    }

    if (layer->getEnabled())
    {
        openLayer(layer);
    }
    else
    {
        closeLayer(layer);
    }

    MapModelChange change(
        layer->getEnabled() ? MapModelChange::ENABLE_LAYER : MapModelChange::DISABLE_LAYER,
        newRevision,
        layer);

    for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
    {
        i->get()->onMapModelChanged(change);
    }
}

bool
Map::isGeocentric() const
{
    return
        _mapOptions.coordSysType().isSet() ? _mapOptions.coordSysType() == MapOptions::CSTYPE_GEOCENTRIC :
        getSRS() ? getSRS()->isGeographic() :
        true;
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

void
Map::setMapName( const std::string& name ) {
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
Map::getAttributions(StringSet& attributions) const
{
    LayerVector layers;
    getLayers(layers);

    for (LayerVector::const_iterator itr = layers.begin(); itr != layers.end(); ++itr)
    {
        if (itr->get()->getEnabled())
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
    // Store in a ref_ptr for scope to ensure callbacks don't accidentally delete while adding
    osg::ref_ptr<Layer> layerRef( layer );
    osgEarth::Registry::instance()->clearBlacklist();
    if ( layer )
    {
        // Set up callbacks
        installLayerCallbacks(layer);

        // Open the layer if it's enabled:
        if (layer->getEnabled())
        {
            openLayer(layer);
        }

        // Add the layer to our stack.
        int newRevision;
        unsigned index = -1;
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );

            _layers.push_back( layer );
            index = _layers.size() - 1;
            newRevision = ++_dataModelRevision;
        }

        // tell the layer it was just added.
        //layer->addedToMap(this);

        // a separate block b/c we don't need the mutex
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapModelChanged(MapModelChange(
                MapModelChange::ADD_LAYER, newRevision, layer, index));
        }
    }
}

void
Map::insertLayer(Layer* layer, unsigned index)
{
    osgEarth::Registry::instance()->clearBlacklist();
    if ( layer )
    {
        // Set up callbacks
        installLayerCallbacks(layer);

        // Open the layer if it's enabled:
        if (layer->getEnabled())
        {
            openLayer(layer);
        }

        // Add the layer to our stack.
        int newRevision;
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );

            if (index >= _layers.size())
                _layers.push_back(layer);
            else
                _layers.insert( _layers.begin() + index, layer );

            newRevision = ++_dataModelRevision;
        }

        // tell the layer it was just added.
        //layer->addedToMap(this);

        // a separate block b/c we don't need the mutex
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            //i->get()->onMapModelChanged( MapModelChange(
            //    MapModelChange::ADD_IMAGE_LAYER, newRevision, layer, index) );

            i->get()->onMapModelChanged(MapModelChange(
                MapModelChange::ADD_LAYER, newRevision, layer, index));
        }
    }
}

void
Map::removeLayer(Layer* layer)
{
    osgEarth::Registry::instance()->clearBlacklist();
    unsigned int index = -1;

    osg::ref_ptr<Layer> layerToRemove = layer;
    Revision newRevision;

    closeLayer(layer);

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
                break;
            }
        }

        // tell the layer it was just removed.
        //layerToRemove->removedFromMap(this);
    }

    uninstallLayerCallbacks(layerToRemove.get());

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
        osg::ref_ptr<Layer> layerToMove = layer;

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
    if (dynamic_cast<ElevationLayer*>(layer))
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
Map::installLayerCallbacks(Layer* layer)
{
    VisibleLayer* visibleLayer = dynamic_cast<VisibleLayer*>(layer);
    if (visibleLayer)
    {
        visibleLayer->addCallback(_visibleLayerCB.get());
    }

    // If this is an elevation layer, install a callback so we know when
    // it's visibility changes:
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
    VisibleLayer* visibleLayer = dynamic_cast<VisibleLayer*>(layer);
    if (visibleLayer)
    {
        visibleLayer->removeCallback(_visibleLayerCB.get());
    }

    // undo the things we did in prepareLayer:
    ElevationLayer* elevationLayer = dynamic_cast<ElevationLayer*>(layer);
    if (elevationLayer)
    {
        // invalidate the pool
        getElevationPool()->clear();
    }

    layer->removeCallback(_layerCB.get());
}

void
Map::openLayer(Layer* layer)
{
    // Pass along the Read Options (including the cache settings, etc.) to the layer:
    layer->setReadOptions(_readOptions.get());

    // If this is a terrain layer, tell it about the Map profile.
    TerrainLayer* terrainLayer = dynamic_cast<TerrainLayer*>(layer);
    if (terrainLayer && _profile.valid())
    {
        terrainLayer->setTargetProfileHint(_profile.get());
    }

    // Attempt to open the layer. Don't check the status here.
    if (layer->open().isOK())
    {
        layer->addedToMap(this);
    }
}

void
Map::closeLayer(Layer* layer)
{
    if (layer)
    {
        layer->removedFromMap(this);
    }
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


void
Map::setLayersFromMap(const Map* map)
{
    this->clear();

    if ( map )
    {
        LayerVector layers;
        map->getLayers(layers);
        for (LayerVector::iterator i = layers.begin(); i != layers.end(); ++i)
            addLayer(i->get());
    }
}


void
Map::calculateProfile()
{
    // collect the terrain layers; we will need them later
    TerrainLayerVector layers;
    getLayers(layers);

    // Figure out the map profile:
    if ( !_profile.valid() )
    {
        osg::ref_ptr<const Profile> profile;

        // Do the map options contain a profile? If so, try to use it:
        if ( _mapOptions.profile().isSet() )
        {
            profile = Profile::create( _mapOptions.profile().value() );
        }

        // Do the map options contain an override coordinate system type?
        // If so, attempt to apply that next:
        if (_mapOptions.coordSysType().isSetTo(MapOptions::CSTYPE_GEOCENTRIC))
        {
            if (profile.valid() && profile->getSRS()->isProjected())
            {
                OE_WARN << LC << "Geocentric map type conflicts with the projected SRS profile; ignoring your profile\n";
                profile = Registry::instance()->getGlobalGeodeticProfile();
            }
        }

        // Do the map options ask for a projected map?
        else if (_mapOptions.coordSysType().isSetTo(MapOptions::CSTYPE_PROJECTED))
        {
            // Is there a conflict in the MapOptions?
            if (profile.valid() && profile->getSRS()->isGeographic())
            {
                OE_WARN << LC << "Projected map type conflicts with the geographic SRS profile; converting to Equirectangular projection\n";
                unsigned u, v;
                profile->getNumTiles(0, u, v);
                const osgEarth::SpatialReference* eqc = profile->getSRS()->createEquirectangularSRS();
                osgEarth::GeoExtent e = profile->getExtent().transform( eqc );
                profile = osgEarth::Profile::create( eqc, e.xMin(), e.yMin(), e.xMax(), e.yMax(), u, v);
            }

            // Is there no profile set? Try to derive it from the Map layers:
            if (!profile.valid())
            {
                for (TerrainLayerVector::iterator i = layers.begin(); !profile.valid() && i != layers.end(); ++i)
                {
                    profile = i->get()->getProfile();
                }
            }

            if (!profile.valid())
            {
                OE_WARN << LC << "No profile information available; defaulting to Spherical Mercator projection\n";
                profile = Registry::instance()->getSphericalMercatorProfile();
            }
        }

        // Finally, if there is still no profile, default to global geodetic.
        if (!profile.valid())
        {
            profile = Registry::instance()->getGlobalGeodeticProfile();
        }


        // Set the map's profile!
        _profile = profile.release();

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

        // finally, fire an event if the profile has been set.
        OE_INFO << LC << "Map profile is: " << _profile->toString() << std::endl;

        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            //i->get()->onMapInfoEstablished( MapInfo(this) );
        }
    }

    // Tell all the layers about the profile
    for (TerrainLayerVector::iterator i = layers.begin(); i != layers.end(); ++i)
    {
        if (i->get()->getEnabled())
        {
            i->get()->setTargetProfileHint(_profile.get());
        }
    }
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

        if (!layer->getEnabled())
            continue;

        TerrainLayer* terrainlayer = dynamic_cast<TerrainLayer*>(layer);
        if (terrainlayer)
        {
            if (terrainlayer->getCacheSettings()->cachePolicy()->isCacheDisabled())
              return false;

            //If no data is available on this tile, we'll be fast
            if (!terrainlayer->mayHaveData(key))
                continue;

            // No tile source? skip it
            osg::ref_ptr< TileSource > source = terrainlayer->getTileSource();
            if (!source.valid())
                continue;

            //If the tile is blacklisted, it should also be fast.
            if (source->getBlacklist()->contains(key))
                continue;

            if (!terrainlayer->isCached(key))
                return false;
        }
    }
    return true;
}
