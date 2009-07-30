/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarth/TileSourceFactory>
#include <osgEarth/GeocentricMap>
#include <osgEarth/ProjectedMap>
#include <OpenThreads/ScopedLock>

using namespace osgEarth;
using namespace OpenThreads;

Map::Map(const CoordinateSystemType& cstype) :
_cstype( cstype ),
_id(-1)
{
    //NOP
}

void
Map::setId( unsigned int id ) {
    _id = id;
}

unsigned int
Map::getId() const {
    return _id;
}

OpenThreads::ReadWriteMutex&
Map::getMapDataMutex() {
    return _mapDataMutex;
}

const Map::CoordinateSystemType&
Map::getCoordinateSystemType() const {
    return _cstype;
}

const osgDB::ReaderWriter::Options*
Map::getGlobalOptions() const {
    return _globalOptions.get();
}

void
Map::setGlobalOptions( const osgDB::ReaderWriter::Options* options ) {
    _globalOptions = options;
}

const std::string&
Map::getReferenceURI() const { 
    return _referenceURI;
}

void
Map::setReferenceURI( const std::string& uri ) {
    _referenceURI = uri;
}

optional<CacheConfig>&
Map::cacheConfig() {
    return _cacheConf;
}
const optional<CacheConfig>&
Map::cacheConfig() const {
    return _cacheConf;
}

optional<ProfileConfig>&
Map::profileConfig() {
    return _profileConf;
}
const optional<ProfileConfig>&
Map::profileConfig() const {
    return _profileConf;
}

const MapLayerList& 
Map::getImageMapLayers() const {
    return _imageMapLayers;
}

const MapLayerList& 
Map::getHeightFieldMapLayers() const {
    return _heightFieldMapLayers;
}

void
Map::setName( const std::string& name ) {
    _name = name;
}

const std::string&
Map::getName() const {
    return _name;
}

const Profile*
Map::getProfile() const
{
    if ( !_profile.valid() )
        const_cast<Map*>(this)->calculateProfile();
    return _profile.get();
}

MapEngine*
Map::createMapEngine( const MapEngineProperties& engineProps )
{
    if ( _cstype == CSTYPE_GEOCENTRIC || _cstype == CSTYPE_GEOCENTRIC_CUBE )
        return new GeocentricMapEngine( engineProps );
    else
        return new ProjectedMapEngine( engineProps );
}


void 
Map::addMapCallback( MapCallback* cb )
{
    if ( cb )
        _mapCallbacks.push_back( cb );
}

void 
Map::addMapLayer( MapLayer* layer )
{
    unsigned int index = -1;
    if ( layer )
    {
        // first, install the layer's tile source.
        TileSourceFactory factory;
        osg::ref_ptr<TileSource> tileSource = factory.createMapTileSource( layer, this );

        if ( tileSource.valid() )
        {
            const Profile* layerProfile = tileSource->initProfile( getProfile(), getReferenceURI() );
            if ( !layerProfile )
            {
                osg::notify(osg::NOTICE) << "[osgEarth::Map] Could not initialize profile for layer " << layer->getName() << std::endl;
            }
            else
            {
                layer->setTileSource( tileSource.get() );
                {
                    ScopedWriteLock lock( getMapDataMutex() );

                    MapLayerList& list = 
                        layer->getType() == MapLayer::TYPE_IMAGE? _imageMapLayers :
                        _heightFieldMapLayers;
                    
                    list.push_back( layer );
                    index = list.size()-1;
                }

                for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
                {
                    i->get()->onMapLayerAdded( layer, index );
                }
            }
        }
        else
        {
            osg::notify(osg::NOTICE) << "[osgEarth::Map] Could not initialize TileSource for layer " << layer->getName() << std::endl;
        }
    }
}

void 
Map::removeMapLayer( MapLayer* layer )
{
    unsigned int index = -1;

    osg::ref_ptr<MapLayer> layerToRemove = layer;

    if ( layerToRemove.get() )
    {
        ScopedWriteLock lock( getMapDataMutex() );

        MapLayerList& list = 
            layerToRemove->getType() == MapLayer::TYPE_IMAGE? _imageMapLayers :
            _heightFieldMapLayers;

        index = 0;
        for( MapLayerList::iterator i = list.begin(); i != list.end(); i++, index++ )
        {
            if ( i->get() == layerToRemove.get() )
            {
                list.erase( i );
                break;
            }
        }
    }

    // a separate block b/c we don't need the mutex
    if ( layerToRemove.get() )
    {
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapLayerRemoved( layerToRemove.get(), index );
        }
    }
}

void
Map::moveMapLayer( MapLayer* layer, unsigned int newIndex )
{
    unsigned int oldIndex = 0;
    unsigned int actualIndex = 0;

    if ( layer )
    {
        ScopedWriteLock lock( getMapDataMutex() );

        MapLayerList& list = 
            layer->getType() == MapLayer::TYPE_IMAGE? _imageMapLayers :
            _heightFieldMapLayers;

        // preserve the layer with a ref:
        osg::ref_ptr<MapLayer> layerToMove = layer;

        // find it:
        MapLayerList::iterator i_oldIndex = list.end();
        for( MapLayerList::iterator i = list.begin(); i != list.end(); i++, actualIndex++ )
        {
            if ( i->get() == layer )
            {
                i_oldIndex = i;
                oldIndex = actualIndex;
                break;
            }
        }

        if ( i_oldIndex == list.end() )
            return; // layer not found in list

        // erase the old one:
        list.erase( i_oldIndex );

        list.insert( list.begin() + newIndex, layerToMove.get() );
    }

    // a separate block b/c we don't need the mutex
    if ( layer )
    {
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapLayerMoved( layer, oldIndex, newIndex );
        }
    }
}


static const Profile*
getSuitableMapProfileFor( const Profile* candidate )
{
    if ( candidate->getProfileType() == Profile::TYPE_GEODETIC )
        return osgEarth::Registry::instance()->getGlobalGeodeticProfile();
    else if ( candidate->getProfileType() == Profile::TYPE_MERCATOR )
        return osgEarth::Registry::instance()->getGlobalMercatorProfile();
    else
        return candidate;
}

void
Map::calculateProfile()
{
    if ( _profile.valid() )
        return;

    if ( getCoordinateSystemType() == CSTYPE_GEOCENTRIC )
    {
        //If the map type if Geocentric, set the profile to global-geodetic
        _profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
        osg::notify(osg::INFO) << "[osgEarth::MapNode] Setting Profile to global-geodetic for geocentric scene" << std::endl;
    }
    else if ( getCoordinateSystemType() == CSTYPE_GEOCENTRIC_CUBE )
    {
        //If the map type is a Geocentric Cube, set the profile to the cube profile.
        _profile = osgEarth::Registry::instance()->getCubeProfile();
        osg::notify(osg::INFO) << "[osgEarth::MapNode] Using cube profile for geocentric scene" << std::endl;
    }
    else // CSTYPE_PROJECTED
    {
        if ( _profileConf.isSet() )
        //if ( getProfileConfig().defined() )
        {
            const ProfileConfig& conf = _profileConf.get();

            // Check for a "well known named" profile:
            std::string namedProfile = conf.getNamedProfile();
            if ( !namedProfile.empty() )
            {
                _profile = osgEarth::Registry::instance()->getNamedProfile( namedProfile );
                if ( _profile.valid() )
                {
                    osg::notify(osg::INFO) << "[osgEarth::MapConfig] Set map profile to " << namedProfile << std::endl;
                }
                else
                {
                    osg::notify(osg::WARN) << "[osgEarth::MapConfig] " << namedProfile << " is not a known profile name" << std::endl;
                    //TODO: continue on? or fail here?
                }
            }

            // Next check for a user-defined profile:
            else if ( conf.areExtentsValid() )
            {
                double minx, miny, maxx, maxy;
                conf.getExtents( minx, miny, maxx, maxy );
                _profile = Profile::create( conf.getSRS(), minx, miny, maxx, maxy );

                if ( _profile.valid() )
                {
                    osg::notify( osg::INFO ) << "[[osgEarth::MapEngine] Set map profile from SRS: " 
                        << _profile->getSRS()->getName() << std::endl;
                }
            }
        }
    }


    // At this point, if we don't have a profile we need to search tile sources until we find one.
    {
        ScopedWriteLock lock( getMapDataMutex() );

        for( MapLayerList::iterator i = _imageMapLayers.begin(); i != _imageMapLayers.end() && !_profile.valid(); i++ )
        {
            MapLayer* layer = i->get();
            if ( layer->getTileSource() )
            {
                _profile = layer->getTileSource()->getProfile();
            }
        }

        for( MapLayerList::iterator i = _heightFieldMapLayers.begin(); i != _heightFieldMapLayers.end() && !_profile.valid(); i++ )
        {
            if ( i->get()->getTileSource() )
            {
                _profile = i->get()->getTileSource()->getProfile();
            }
        }
    }

    // finally, fire an event if the profile has been set.
    if ( _profile.valid() )
    {
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapProfileEstablished( _profile.get() );
        }
    }
}
