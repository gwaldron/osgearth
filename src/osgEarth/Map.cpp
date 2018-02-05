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
#include <osgEarth/ElevationPool>
#include <osgEarth/Utils>
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
Map::ElevationLayerCB::onVisibleChanged(VisibleLayer* layer)
{
    osg::ref_ptr<Map> map;
    if ( _map.lock(map) )
    {
        _map->notifyElevationLayerVisibleChanged(layer);
    }
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

    // set up a callback that the Map will use to detect Elevation Layer
    // visibility changes
    _elevationLayerCB = new ElevationLayerCB(this);

    // elevation sampling
    _elevationPool = new ElevationPool();
    _elevationPool->setMap( this );
}

Map::~Map()
{
    OE_DEBUG << LC << "~Map" << std::endl;
    clear();
}

ElevationPool*
Map::getElevationPool() const
{
    return _elevationPool.get();
}

void
Map::notifyElevationLayerVisibleChanged(VisibleLayer* layer)
{
    // bump the revision safely:
    Revision newRevision;
    {
        Threading::ScopedWriteLock lock( const_cast<Map*>(this)->_mapDataMutex );
        newRevision = ++_dataModelRevision;
    }

    // reinitialize the elevation pool:
    _elevationPool->clear();

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
    osgEarth::Registry::instance()->clearBlacklist();
    if ( layer )
    {
        if (layer->getEnabled())
        {
            // Pass along the Read Options (including the cache settings, etc.) to the layer:
            layer->setReadOptions(_readOptions.get());
            
            // If this is a terrain layer, tell it about the Map profile.
            TerrainLayer* terrainLayer = dynamic_cast<TerrainLayer*>(layer);
            if (terrainLayer && _profile.valid())
            {
                terrainLayer->setTargetProfileHint( _profile.get() );
            }            

            // Attempt to open the layer. Don't check the status here.
            layer->open();

            // If this is an elevation layer, install a callback so we know when
            // it's visibility changes:
            ElevationLayer* elevationLayer = dynamic_cast<ElevationLayer*>(layer);
            if (elevationLayer)
            {
                elevationLayer->addCallback(_elevationLayerCB.get());

                // invalidate the elevation pool
                getElevationPool()->clear();
            }
        }

        int newRevision;
        unsigned index = -1;

        // Add the layer to our stack.
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );

            _layers.push_back( layer );
            index = _layers.size() - 1;
            newRevision = ++_dataModelRevision;
        }

        // tell the layer it was just added.
        layer->addedToMap(this);

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
        if (layer->getEnabled())
        {
            // Pass along the Read Options (including the cache settings, etc.) to the layer:
            layer->setReadOptions(_readOptions.get());
            
            // If this is a terrain layer, tell it about the Map profile.
            TerrainLayer* terrainLayer = dynamic_cast<TerrainLayer*>(layer);
            if (terrainLayer && _profile.valid())
            {
                terrainLayer->setTargetProfileHint( _profile.get() );
            }            

            // Attempt to open the layer. Don't check the status here.
            layer->open();

            // If this is an elevation layer, install a callback so we know when
            // it's visibility changes:
            ElevationLayer* elevationLayer = dynamic_cast<ElevationLayer*>(layer);
            if (elevationLayer)
            {
                elevationLayer->addCallback(_elevationLayerCB.get());

                // invalidate the elevation pool
                getElevationPool()->clear();
            }
        }

        int newRevision;

        // Add the layer to our stack.
        {
            Threading::ScopedWriteLock lock( _mapDataMutex );

            if (index >= _layers.size())
                _layers.push_back(layer);
            else
                _layers.insert( _layers.begin() + index, layer );

            newRevision = ++_dataModelRevision;
        }

        // tell the layer it was just added.
        layer->addedToMap(this);

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
        layerToRemove->removedFromMap(this);
    }

    ElevationLayer* elevationLayer = dynamic_cast<ElevationLayer*>(layerToRemove.get());
    if (elevationLayer)
    {
        elevationLayer->removeCallback(_elevationLayerCB.get());

        // invalidate the pool
        getElevationPool()->clear();
    }

    // a separate block b/c we don't need the mutex
    if ( newRevision >= 0 ) // layerToRemove.get() )
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

Revision
Map::getLayers(LayerVector& out_list) const
{
    out_list.reserve( _layers.size() );

    Threading::ScopedReadLock lock( const_cast<Map*>(this)->_mapDataMutex );
    for( LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i )
        out_list.push_back( i->get() );

    return _dataModelRevision;
}

unsigned
Map::getNumLayers() const
{
    Threading::ScopedReadLock lock( const_cast<Map*>(this)->_mapDataMutex );
    return _layers.size();
}

Layer*
Map::getLayerByName(const std::string& name) const
{
    Threading::ScopedReadLock( const_cast<Map*>(this)->_mapDataMutex );
    for(LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i)
        if ( i->get()->getName() == name )
            return i->get();
    return 0L;
}

Layer*
Map::getLayerByUID(UID layerUID) const
{
    Threading::ScopedReadLock( const_cast<Map*>(this)->_mapDataMutex );
    for( LayerVector::const_iterator i = _layers.begin(); i != _layers.end(); ++i )
        if ( i->get()->getUID() == layerUID )
            return i->get();
    return 0L;
}

Layer*
Map::getLayerAt(unsigned index) const
{
    Threading::ScopedReadLock( const_cast<Map*>(this)->_mapDataMutex );
    if ( index >= 0 && index < (int)_layers.size() )
        return _layers[index].get();
    else
        return 0L;
}

unsigned
Map::getIndexOfLayer(const Layer* layer) const
{
    Threading::ScopedReadLock( const_cast<Map*>(this)->_mapDataMutex );
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
        for(LayerVector::iterator layer = layersRemoved.begin();
            layer != layersRemoved.end();
            ++layer)
        {
            i->get()->onMapModelChanged(MapModelChange(MapModelChange::REMOVE_LAYER, newRevision, layer->get()));
        }
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
            i->get()->onMapInfoEstablished( MapInfo(this) );
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
    return getSRS() && getSRS()->isGeographic() ? getSRS()->getECEF() : getSRS();
}

bool
Map::sync(MapFrame& frame) const
{
    bool result = false;

    if ( frame._mapDataModelRevision != _dataModelRevision || !frame._initialized )
    {
        // hold the read lock while copying the layer lists.
        Threading::ScopedReadLock lock( const_cast<Map*>(this)->_mapDataMutex );

        if (!frame._initialized)
            frame._layers.reserve(_layers.size());

        frame._layers.clear();

        std::copy(_layers.begin(), _layers.end(), std::back_inserter(frame._layers));

        // sync the revision numbers.
        frame._initialized = true;
        frame._mapDataModelRevision = _dataModelRevision;

        result = true;
    }
    return result;
}


//......................................................................
// @deprecated functions

#include <osgEarth/ImageLayer>
#include <osgEarth/ElevationLayer>
#include <osgEarth/ModelLayer>
#include <osgEarth/MaskLayer>

void 
Map::addImageLayer(class ImageLayer* layer) {
    OE_DEPRECATED(Map::addImageLayer, Map::addLayer);
    addLayer(layer);
}

void 
Map::insertImageLayer(class ImageLayer* layer, unsigned index) {
    OE_DEPRECATED(Map::insertImageLayer, Map::insertLayer);
    insertLayer(layer, index);
}

void 
Map::removeImageLayer(class ImageLayer* layer) {
    OE_DEPRECATED(Map::removeImageLayer, Map::removeLayer);
    removeLayer(layer);
}

void 
Map::moveImageLayer(class ImageLayer* layer, unsigned newIndex) {
    OE_DEPRECATED(Map::moveImageLayer, Map::moveLayer);
    moveLayer(layer, newIndex);
}

void 
Map::addElevationLayer(class ElevationLayer* layer) {
    OE_DEPRECATED(Map::addElevationLayer, Map::addLayer);
    addLayer(layer);
}

void 
Map::removeElevationLayer(class ElevationLayer* layer) {
    OE_DEPRECATED(Map::removeElevationLayer, Map::removeLayer);
    removeLayer(layer);
}

void 
Map::moveElevationLayer(class ElevationLayer* layer, unsigned newIndex) {
    OE_DEPRECATED(Map::moveElevationLayer, Map::moveLayer);
    moveLayer(layer, newIndex);
}

void 
Map::addModelLayer(class ModelLayer* layer) {
    OE_DEPRECATED(Map::addModelLayer, Map::addLayer);
    addLayer(layer);
}

void 
Map::insertModelLayer(class ModelLayer* layer, unsigned index) {
    OE_DEPRECATED(Map::insertModelLayer, Map::insertLayer);
    insertLayer(layer, index);
}

void 
Map::removeModelLayer(class ModelLayer* layer) {
    OE_DEPRECATED(Map::removeModelLayer, Map::removeLayer);
    removeLayer(layer);
}

void 
Map::moveModelLayer(class ModelLayer* layer, unsigned newIndex) {
    OE_DEPRECATED(Map::moveModelLayer, Map::moveLayer);
    moveLayer(layer, newIndex);
}

void 
Map::addTerrainMaskLayer(class MaskLayer* layer) {
    OE_DEPRECATED(Map::addTerrainMaskLayer, Map::addLayer);
    addLayer(layer);
}

void 
Map::removeTerrainMaskLayer(class MaskLayer* layer) {
    OE_DEPRECATED(Map::removeTerrainMaskLayer, Map::removeLayer);
    removeLayer(layer);
}
