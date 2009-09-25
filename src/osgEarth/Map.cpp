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
#include <OpenThreads/ScopedLock>

using namespace osgEarth;
using namespace OpenThreads;

Map::Map(const CoordinateSystemType& cstype) :
_cstype( cstype ),
_id(-1),
_use_mercator_locator(true)
{
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

void
Map::setUseMercatorLocator(bool value)
{
    _use_mercator_locator = value;
}

bool
Map::getUseMercatorLocator() const
{
    return _use_mercator_locator;
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
	return _cache.get();
}

void
Map::setCache( Cache* cache)
{
	_cache = cache;

	//Propagate the cache to any of our layers
	for (MapLayerList::iterator i = _imageMapLayers.begin(); i != _imageMapLayers.end(); ++i)
	{
		i->get()->setCache( _cache.get() );
	}

	for (MapLayerList::iterator i = _heightFieldMapLayers.begin(); i != _heightFieldMapLayers.end(); ++i)
	{
		i->get()->setCache( _cache.get() );
	}
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
	    //Set options for the map from the layer
		layer->setReferenceURI( getReferenceURI() );
		layer->setGlobalOptions( getGlobalOptions() );
		if ( _cacheConf.isSet() && _cacheConf->runOffCacheOnly().isSet() && _cacheConf->runOffCacheOnly().get())
		{
			layer->setCacheOnly( true );
		}
		layer->setUseMercatorFastPath( getUseMercatorLocator() );

		//Set the Cache for the MapLayer to our cache.
		layer->setCache( this->getCache() );

		ScopedWriteLock lock( getMapDataMutex() );

		MapLayerList& list = 
			layer->getType() == MapLayer::TYPE_IMAGE? _imageMapLayers : _heightFieldMapLayers;
		list.push_back( layer );
		index = list.size()-1;

		for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
		{
			i->get()->onMapLayerAdded( layer, index );
		}

		/*//TODO:  This needs to be moved somewhere else for sure.  You should be able to do most of this programtically.
		//First, determine if we need to run off of the cache only.
		bool cacheOnlyEnv = false;
		// If the OSGEARTH_CACHE_ONLY environment variable is set, override whateve is in the map config
		if (getenv("OSGEARTH_CACHE_ONLY") != 0)
		{
			osg::notify(osg::NOTICE) << "[osgEarth::TileSourceFactory] Setting osgEarth to cache only mode due to OSGEARTH_CACHE_ONLY environment variable " << std::endl;
			cacheOnlyEnv = true;
		}

		optional<CacheConfig> layerCacheConf = layer->cacheConfig();
		const optional<CacheConfig>& mapCacheConf = this->cacheConfig();

		// Load the tile source ... unless we are running in "offline" mode (from the cache only)
		bool runOffCacheOnly =
			cacheOnlyEnv ||
			( mapCacheConf.isSet() && mapCacheConf->runOffCacheOnly() == true ) ||
			( layerCacheConf.isSet() && layerCacheConf->runOffCacheOnly() == true );

		//Create the cache.
	    Cache* cache = NULL;
		if ( layerCacheConf.isSet() && layerCacheConf->getType() != CacheConfig::TYPE_DISABLED )
		{
			CacheFactory cacheFactory;
			cache = cacheFactory.create( layerCacheConf.get() );
			//Set the cache on the MapLayer;
			layer->setCache( cache );
		}

		//TODO:  If CacheOnly, then set up a NullTileSource with the cache properties.

     	//Create the TileSource itself.
        TileSourceFactory tileSourceFactory;
		osg::ref_ptr< TileSource > tileSource = tileSourceFactory.create( this,
			                                                              layer->getDriver(),
																          layer->getDriverProperties());
		if ( tileSource.valid() )
		{
			//const Profile* layerProfile = tileSource->initProfile( getProfile(), getReferenceURI() );

			const ProfileConfig& pconf = layer->profileConfig().get();

			osg::ref_ptr<const Profile> override_profile;

			if ( !pconf.getNamedProfile().empty() )
			{
				override_profile = osgEarth::Registry::instance()->getNamedProfile( pconf.getNamedProfile() );
			}

			if ( !override_profile.valid() && !pconf.getSRS().empty() )
			{
				double xmin, ymin, xmax, ymax;
				pconf.getExtents( xmin, ymin, xmax, ymax );
				override_profile = Profile::create( pconf.getSRS(), xmin, ymin, xmax, ymax );
			}

			tileSource->initialize( getReferenceURI(), override_profile.get() );

            if ( !tileSource->isOK() )
            {
                osg::notify(osg::NOTICE) << "[osgEarth::Map] Could not initialize TileSource for layer " << layer->getName() << std::endl;
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
        }*/
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

    osg::ref_ptr<const Profile> userProfile;
    if ( _profileConf.isSet() )
    {
        userProfile = Profile::create( _profileConf.get() );
    }

    if ( getCoordinateSystemType() == CSTYPE_GEOCENTRIC )
    {
        if ( userProfile.valid() )
        {
            if ( userProfile->isOK() && userProfile->getSRS()->isGeographic() )
            {
                _profile = userProfile.get();
            }
            else
            {
                osg::notify(osg::WARN) << "[osgEarth::Map] Map is geocentric, but the configured profile does not "
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

    else if ( getCoordinateSystemType() == CSTYPE_GEOCENTRIC_CUBE )
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
        osg::notify(osg::INFO) << "[osgEarth::Map] Map profile is: " << _profile->toString() << std::endl;

        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapProfileEstablished( _profile.get() );
        }
    }

    else
    {
        osg::notify(osg::WARN) << "[osgEarth::Map] Bad news, unable to establish a map profile!" << std::endl;
    }
}

osg::HeightField*
Map::createHeightField( const TileKey* key, bool fallback, SamplePolicy samplePolicy)
{
	osg::notify(osg::INFO) << "[osgEarth::Map::createHeightField]" << std::endl;
    OpenThreads::ScopedReadLock lock( _mapDataMutex );

	osg::HeightField *result = NULL;

	//Get a HeightField for each of the enabled layers
	GeoHeightFieldList heightFields;
	for( MapLayerList::const_iterator i = getHeightFieldMapLayers().begin(); i != getHeightFieldMapLayers().end(); i++ )
	{
		if (i->get()->getEnabled())
		{
			osg::ref_ptr< const TileKey > hf_key = key;
			osg::HeightField* hf = NULL;
			while (hf_key.valid())
			{
				hf = i->get()->createHeightField( key );
				if (hf || !fallback) break;
				hf_key = hf_key->createParentKey();
			}

			if (hf)
			{
				heightFields.push_back( new GeoHeightField( hf, key->getGeoExtent() ) );
			}
		}
	}

	if (heightFields.size() == 0)
	{
	    //If we got no heightfields, return NULL
		return NULL;
	}
	else if (heightFields.size() == 1)
	{
		//If we only have on heightfield, just return it.
		result = heightFields[0]->takeHeightField();
	}
	else
	{
		//If we have multiple heightfields, we need to composite them together.
		unsigned int width = 0;
		unsigned int height = 0;

		for (GeoHeightFieldList::const_iterator i = heightFields.begin(); i < heightFields.end(); ++i)
		{
			if (i->get()->getHeightField()->getNumColumns() > width) width = i->get()->getHeightField()->getNumColumns();
			if (i->get()->getHeightField()->getNumRows() > height) height = i->get()->getHeightField()->getNumRows();
		}
		result = new osg::HeightField();
		result->allocate( width, height );

		//Go ahead and set up the heightfield so we don't have to worry about it later
        double minx, miny, maxx, maxy;
        key->getGeoExtent().getBounds(minx, miny, maxx, maxy);
        double dx = (maxx - minx)/(double)(result->getNumColumns()-1);
        double dy = (maxy - miny)/(double)(result->getNumRows()-1);
        
		//Create the new heightfield by sampling all of them.
        for (unsigned int c = 0; c < width; ++c)
        {
            double geoX = minx + (dx * (double)c);
            for (unsigned r = 0; r < height; ++r)
            {
                double geoY = miny + (dy * (double)r);

                bool hasValidData = false;

                //Collect elevations from all of the layers
                std::vector<float> elevations;
                for (GeoHeightFieldList::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
                {
                    float elevation = 0.0f;
                    if (itr->get()->getElevation(key->getGeoExtent().getSRS(), geoX, geoY, BILINEAR, elevation))
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
                    if (samplePolicy == FIRST_VALID)
                    {
                        elevation = elevations[0];
                    }
                    else if (samplePolicy == HIGHEST)
                    {
                        elevation = -FLT_MAX;
                        for (unsigned int i = 0; i < elevations.size(); ++i)
                        {
                            if (elevation < elevations[i]) elevation = elevations[i];
                        }
                    }
                    else if (samplePolicy == LOWEST)
                    {
                        elevation = FLT_MAX;
                        for (unsigned i = 0; i < elevations.size(); ++i)
                        {
                            if (elevation > elevations[i]) elevation = elevations[i];
                        }
                    }
                    else if (samplePolicy = AVERAGE)
                    {
                        elevation = 0.0;
                        for (unsigned i = 0; i < elevations.size(); ++i)
                        {
                            elevation += elevations[i];
                        }
                        elevation /= (float)elevations.size();
                    }
                }
                result->setHeight(c, r, elevation);
            }
        }
	}

	//Replace any NoData areas with 0
	if (result)
	{
		ReplaceInvalidDataOperator o;
		o.setValidDataOperator(new osgTerrain::NoDataValue(NO_DATA_VALUE));
		o(result);
	}

	if (!result)
	{
	}

	//Initialize the HF values for osgTerrain
	if (result)
	{	
		//Go ahead and set up the heightfield so we don't have to worry about it later
		double minx, miny, maxx, maxy;
		key->getGeoExtent().getBounds(minx, miny, maxx, maxy);
		result->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
		double dx = (maxx - minx)/(double)(result->getNumColumns()-1);
		double dy = (maxy - miny)/(double)(result->getNumRows()-1);
		result->setXInterval( dx );
		result->setYInterval( dy );
		result->setBorderWidth( 0 );
	}

	return result;

	/*
    osg::ref_ptr< ElevationManager > em = new ElevationManager;
    for( MapLayerList::const_iterator i = getHeightFieldMapLayers().begin(); i != getHeightFieldMapLayers().end(); i++ )
    {
        em->getElevationLayers().push_back( i->get() );
    }
    return em->createHeightField( key, 0, 0, fallback );*/
}