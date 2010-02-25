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
osg::Referenced(true),
_cstype( cstype ),
_id(-1),
_dataModelRevision(0),
_cacheConf( CacheConfig() ),
_profileConf( ProfileConfig() )
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

bool
Map::isGeocentric() const
{
    return _cstype == CSTYPE_GEOCENTRIC || _cstype == CSTYPE_GEOCENTRIC_CUBE;
}

const osgDB::ReaderWriter::Options*
Map::getGlobalOptions() const {
    return _globalOptions.get();
}

void
Map::setGlobalOptions( const osgDB::ReaderWriter::Options* options ) {
    _globalOptions = options;

    //Set the global options for all of the existing map layers as well
    for (MapLayerList::iterator i = _imageMapLayers.begin(); i < _imageMapLayers.end(); ++i)
    {
        i->get()->setGlobalOptions( _globalOptions.get() );
    }

    for (MapLayerList::iterator i = _heightFieldMapLayers.begin(); i < _heightFieldMapLayers.end(); ++i)
    {
        i->get()->setGlobalOptions( _globalOptions.get() );
    }
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

int
Map::getImageMapLayers( MapLayerList& out_list ) const {
    ScopedReadLock lock( const_cast<Map*>(this)->getMapDataMutex() );
    for( MapLayerList::const_iterator i = _imageMapLayers.begin(); i != _imageMapLayers.end(); ++i )
        out_list.push_back( i->get() );
    return _dataModelRevision;
}

const MapLayerList& 
Map::getHeightFieldMapLayers() const {
    return _heightFieldMapLayers;
}

const ModelLayerList&
Map::getModelLayers() const {
    return _modelLayers;
}

ModelLayer*
Map::getTerrainMaskLayer() const {
    return _terrainMaskLayer.get();
}

void
Map::setName( const std::string& name ) {
    _name = name;
}

const std::string&
Map::getName() const {
    return _name;
}

int
Map::getDataModelRevision() const {
    ScopedReadLock lock( const_cast<Map*>(this)->getMapDataMutex() );
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
	return _cache.get();
}

void
Map::setCache( Cache* cache)
{
    if (_cache != cache)
    {
        _cache = cache;
        _cache->setMapConfigFilename( _referenceURI );

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
			layer->cacheOnly() = true;
		}
		//layer->setUseMercatorFastPath( getUseMercatorLocator() );

		//Set the Cache for the MapLayer to our cache.
		layer->setCache( this->getCache() );

        {
            ScopedWriteLock lock( getMapDataMutex() );
            MapLayerList& list = 
                layer->getType() == MapLayer::TYPE_IMAGE? _imageMapLayers : _heightFieldMapLayers;
            list.push_back( layer );
            index = list.size()-1;
            _dataModelRevision++;
        }

        // a separate block b/c we don't need the mutex   
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMapLayerAdded( layer, index );
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
                _dataModelRevision++;
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

        _dataModelRevision++;
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


void
Map::addModelLayer( ModelLayer* layer )
{
    if ( layer )
    {
        {
            ScopedWriteLock lock( getMapDataMutex() );
            _modelLayers.push_back( layer );
            _dataModelRevision++;
        }

        layer->initialize( getReferenceURI(), this );        

        // a seprate block b/c we don't need the mutex
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onModelLayerAdded( layer );
        }
    }
}

void
Map::removeModelLayer( ModelLayer* layer )
{
    if ( layer )
    {
        {
            ScopedWriteLock lock( getMapDataMutex() );
            for( ModelLayerList::iterator i = _modelLayers.begin(); i != _modelLayers.end(); ++i )
            {
                if ( i->get() == layer )
                {
                    _modelLayers.erase( i );
                    _dataModelRevision++;
                    break;
                }
            }
        }

        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); ++i )
        {
            i->get()->onModelLayerRemoved( layer );
        }
    }
}

void
Map::setTerrainMaskLayer( ModelLayer* layer )
{
    if ( layer )
    {
        {
            ScopedWriteLock lock( getMapDataMutex() );
            _terrainMaskLayer = layer;
        }

        layer->initialize( getReferenceURI(), this );

        // a separate block b/c we don't need the mutex   
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onTerrainMaskLayerAdded( layer );
        }	
    }
    else
    {
        removeTerrainMaskLayer();
    }
}

void
Map::removeTerrainMaskLayer()
{
    if ( _terrainMaskLayer.valid() )
    {
        osg::ref_ptr<ModelLayer> layer = _terrainMaskLayer.get();
        {
            ScopedWriteLock lock( getMapDataMutex() );
            _terrainMaskLayer = 0L;
        }
        
        // a separate block b/c we don't need the mutex   
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onTerrainMaskLayerRemoved( layer );
        }	
    }
}


//static const Profile*
//getSuitableMapProfileFor( const Profile* candidate )
//{
//    if ( candidate->getProfileType() == Profile::TYPE_GEODETIC )
//        return osgEarth::Registry::instance()->getGlobalGeodeticProfile();
//    else if ( candidate->getProfileType() == Profile::TYPE_MERCATOR )
//        return osgEarth::Registry::instance()->getGlobalMercatorProfile();
//    else
//        return candidate;
//}

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
        ScopedReadLock lock( getMapDataMutex() );

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
        osg::notify(osg::WARN) << "[osgEarth::Map] Warning, not yet able to establish a map profile!" << std::endl;
    }
}


osg::HeightField*
Map::createHeightField( const TileKey* key,
                        bool fallback,
                        SamplePolicy samplePolicy,
                        ProgressCallback* progress)
{
	//osg::notify(osg::INFO) << "[osgEarth::Map::createHeightField]" << std::endl;\
       //Note:  Assumes that the map data mutex is locked before calling.  Avoids reentrantcy issue on Linux.
//     OpenThreads::ScopedReadLock lock( _mapDataMutex );

	osg::HeightField *result = NULL;
    int lowestLOD = key->getLevelOfDetail();
    bool hfInitialized = false;

    typedef std::map< MapLayer*, bool> LayerValidMap;
    LayerValidMap layerValidMap;

	//Get a HeightField for each of the enabled layers
	GeoHeightFieldList heightFields;

    unsigned int numValidHeightFields = 0;

    
    //First pass:  Try to get the exact LOD requested for each enabled heightfield
    for( MapLayerList::const_iterator i = getHeightFieldMapLayers().begin(); i != getHeightFieldMapLayers().end(); i++ )
    {
        MapLayer* layer = i->get();
        if (layer->enabled() == true)
        {
            osg::ref_ptr< osg::HeightField > hf = layer->createHeightField( key, progress );
            layerValidMap[ layer ] = hf.valid();
            if (hf.valid())
            {
                numValidHeightFields++;
                heightFields.push_back( new GeoHeightField( hf.get(), key->getGeoExtent() ) );
            }
        }
    }

    //If we didn't get any heightfields and weren't requested to fallback, just return NULL
    if (numValidHeightFields == 0 && !fallback)
    {
        return NULL;
    }

    //Second pass:  We were either asked to fallback or we might have some heightfields at the requested LOD and some that are NULL
    //              Fall back on parent tiles to fill in the missing data if possible.
    for( MapLayerList::const_iterator i = getHeightFieldMapLayers().begin(); i != getHeightFieldMapLayers().end(); i++ )
    {
        MapLayer* layer = i->get();

        if (layer->enabled() == true)
        {
            if (!layerValidMap[ layer ])
            {
                osg::ref_ptr< const TileKey > hf_key = key;
                osg::ref_ptr< osg::HeightField > hf;
                while (hf_key.valid())
                {
                    hf = layer->createHeightField( hf_key.get(), progress );
                    if (hf.valid()) break;
                    hf_key = hf_key->createParentKey();
                }

                if (hf.valid())
                {
                    if ( hf_key->getLevelOfDetail() < lowestLOD )
                        lowestLOD = hf_key->getLevelOfDetail();
                    heightFields.push_back( new GeoHeightField( hf.get(), hf_key->getGeoExtent() ) );
                }
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
        if ( lowestLOD == key->getLevelOfDetail() )
        {
		    //If we only have on heightfield, just return it.
		    result = heightFields[0]->takeHeightField();
        }
        else
        {
            osg::ref_ptr<GeoHeightField> geoHF = heightFields[0]->createSubSample( key->getGeoExtent() );
            result = geoHF->takeHeightField();
            hfInitialized = true;
        }
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
                    if (itr->get()->getElevation(key->getGeoExtent().getSRS(), geoX, geoY, INTERP_BILINEAR, elevation))
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
                    else if (samplePolicy == AVERAGE)
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

	//Initialize the HF values for osgTerrain
	if (result && !hfInitialized )
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
