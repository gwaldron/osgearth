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
#include <OpenThreads/ScopedLock>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[Map] "

Map::Map( const MapOptions& options ) :
osg::Referenced( true ),
_mapOptions( options ),
_dataModelRevision(0)
{
    //NOP
}

Threading::ReadWriteMutex&
Map::getMapDataMutex() {
    return _mapDataMutex;
}

bool
Map::isGeocentric() const
{
    return 
        _mapOptions.coordSysType() == MapOptions::CSTYPE_GEOCENTRIC ||
        _mapOptions.coordSysType() == MapOptions::CSTYPE_GEOCENTRIC_CUBE;
}

const osgDB::ReaderWriter::Options*
Map::getGlobalOptions() const {
    return _globalOptions.get();
}

void
Map::setGlobalOptions( const osgDB::ReaderWriter::Options* options ) {
    _globalOptions = options;
}

//const MapLayerList& 
//Map::getImageLayers() const {
//    return _imageLayers;
//}

int
Map::getImageLayers( ImageLayerVector& out_list, bool validLayersOnly ) const
{
    out_list.reserve( _imageLayers.size() );

    Threading::ScopedReadLock lock( const_cast<Map*>(this)->getMapDataMutex() );
    for( ImageLayerVector::const_iterator i = _imageLayers.begin(); i != _imageLayers.end(); ++i )
        if ( !validLayersOnly || i->get()->getProfile() )
            out_list.push_back( i->get() );

    return _dataModelRevision;
}

//const MapLayerList& 
//Map::getElevationLayers() const {
//    return _elevationLayers;
//}

int
Map::getElevationLayers( ElevationLayerVector& out_list, bool validLayersOnly ) const
{
    out_list.reserve( _elevationLayers.size() );

    Threading::ScopedReadLock lock( const_cast<Map*>(this)->getMapDataMutex() );
    for( ElevationLayerVector::const_iterator i = _elevationLayers.begin(); i != _elevationLayers.end(); ++i )
        if ( !validLayersOnly || i->get()->getProfile() )
            out_list.push_back( i->get() );

    return _dataModelRevision;
}

//const ModelLayerList&
//Map::getModelLayers() const {
//    return _modelLayers;
//}

int
Map::getModelLayers( ModelLayerVector& out_list, bool validLayersOnly ) const
{
    out_list.reserve( _modelLayers.size() );

    Threading::ScopedReadLock lock( const_cast<Map*>(this)->getMapDataMutex() );
    for( ModelLayerVector::const_iterator i = _modelLayers.begin(); i != _modelLayers.end(); ++i )
        //if ( !validLayersOnly || i->get()->i->get()->getProfile() )
            out_list.push_back( i->get() );

    return _dataModelRevision;
}

MaskLayer*
Map::getTerrainMaskLayer() const {
    return _terrainMaskLayer.get();
}

void
Map::setName( const std::string& name ) {
    _name = name;
}

int
Map::getDataModelRevision() const
{
    Threading::ScopedReadLock lock( const_cast<Map*>(this)->getMapDataMutex() );
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
Map::getCache()
{
    if ( !_cache.valid() && _mapOptions.cache().isSet() ) //_cacheConf.isSet() )
    {
        Cache* cache = CacheFactory::create( _mapOptions.cache().get() );
        if ( cache ) {
            const_cast<Map*>(this)->setCache( cache );
        }
    }
	return _cache.get();
}

void
Map::setCache( Cache* cache)
{
    if (_cache.get() != cache)
    {
        _cache = cache;
        _cache->setReferenceURI( _mapOptions.referenceURI().value() ); //_referenceURI );

        //Propagate the cache to any of our layers
        for (ImageLayerVector::iterator i = _imageLayers.begin(); i != _imageLayers.end(); ++i)
        {
            i->get()->setCache( _cache.get() );
        }

        for (ElevationLayerVector::iterator i = _elevationLayers.begin(); i != _elevationLayers.end(); ++i)
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
Map::addImageLayer( ImageLayer* layer )
{
    unsigned int index = -1;
    if ( layer )
    {
	    //Set options for the map from the layer
		layer->setReferenceURI( _mapOptions.referenceURI().value() );

        //propagate the cache to the layer:
        if ( _mapOptions.cache().isSet() && _mapOptions.cache()->cacheOnly().isSetTo( true ) )
		{
			layer->setCacheOnly( true );
		}

		//Set the Cache for the MapLayer to our cache.
		layer->setCache( this->getCache() );

        // Add the layer to our stack.
        {
            Threading::ScopedWriteLock lock( getMapDataMutex() );

            _imageLayers.push_back( layer );
            index = _imageLayers.size() - 1;
            ++_dataModelRevision;
        }

        // a separate block b/c we don't need the mutex   
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onImageLayerAdded( layer, index );
        }	
    }	
}

void
Map::addElevationLayer( ElevationLayer* layer )
{
    unsigned int index = -1;
    if ( layer )
    {
	    //Set options for the map from the layer
		layer->setReferenceURI( _mapOptions.referenceURI().value() );

        //propagate the cache to the layer:
        if ( _mapOptions.cache().isSet() && _mapOptions.cache()->cacheOnly().isSetTo( true ) )
		{
			layer->setCacheOnly( true );
		}

		//Set the Cache for the MapLayer to our cache.
		layer->setCache( this->getCache() );

        // Add the layer to our stack.
        {
            Threading::ScopedWriteLock lock( getMapDataMutex() );

            _elevationLayers.push_back( layer );
            index = _elevationLayers.size() - 1;
            ++_dataModelRevision;
        }

        // a separate block b/c we don't need the mutex   
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onElevationLayerAdded( layer, index );
        }	
    }	
}

void 
Map::removeImageLayer( ImageLayer* layer )
{
    unsigned int index = -1;

    osg::ref_ptr<ImageLayer> layerToRemove = layer;

    if ( layerToRemove.get() )
    {
        Threading::ScopedWriteLock lock( getMapDataMutex() );
        index = 0;
        for( ImageLayerVector::iterator i = _imageLayers.begin(); i != _imageLayers.end(); i++, index++ )
        {
            if ( i->get() == layerToRemove.get() )
            {
                _imageLayers.erase( i );
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
            i->get()->onImageLayerRemoved( layerToRemove.get(), index );
        }
    }
}

void 
Map::removeElevationLayer( ElevationLayer* layer )
{
    unsigned int index = -1;

    osg::ref_ptr<ElevationLayer> layerToRemove = layer;

    if ( layerToRemove.get() )
    {
        Threading::ScopedWriteLock lock( getMapDataMutex() );
        index = 0;
        for( ElevationLayerVector::iterator i = _elevationLayers.begin(); i != _elevationLayers.end(); i++, index++ )
        {
            if ( i->get() == layerToRemove.get() )
            {
                _elevationLayers.erase( i );
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
            i->get()->onElevationLayerRemoved( layerToRemove.get(), index );
        }
    }
}

void
Map::moveImageLayer( ImageLayer* layer, unsigned int newIndex )
{
    unsigned int oldIndex = 0;
    unsigned int actualIndex = 0;

    if ( layer )
    {
        Threading::ScopedWriteLock lock( getMapDataMutex() );

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

        _dataModelRevision++;
    }

    // a separate block b/c we don't need the mutex
    if ( layer )
    {
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onImageLayerMoved( layer, oldIndex, newIndex );
        }
    }
}

void
Map::moveElevationLayer( ElevationLayer* layer, unsigned int newIndex )
{
    unsigned int oldIndex = 0;
    unsigned int actualIndex = 0;

    if ( layer )
    {
        Threading::ScopedWriteLock lock( getMapDataMutex() );

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

        _dataModelRevision++;
    }

    // a separate block b/c we don't need the mutex
    if ( layer )
    {
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onElevationLayerMoved( layer, oldIndex, newIndex );
        }
    }
}

void
Map::addModelLayer( ModelLayer* layer )
{
    if ( layer )
    {
        {
            Threading::ScopedWriteLock lock( getMapDataMutex() );
            _modelLayers.push_back( layer );
            _dataModelRevision++;
        }

        layer->initialize( _mapOptions.referenceURI().get(), this ); //getReferenceURI(), this );        

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
            Threading::ScopedWriteLock lock( getMapDataMutex() );
            for( ModelLayerVector::iterator i = _modelLayers.begin(); i != _modelLayers.end(); ++i )
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
Map::setTerrainMaskLayer( MaskLayer* layer )
{
    if ( layer )
    {
        {
            Threading::ScopedWriteLock lock( getMapDataMutex() );
            _terrainMaskLayer = layer;
        }

        layer->initialize( _mapOptions.referenceURI().value(), this ); //getReferenceURI(), this );

        // a separate block b/c we don't need the mutex   
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMaskLayerAdded( layer );
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
        osg::ref_ptr<MaskLayer> layer = _terrainMaskLayer.get();
        {
            Threading::ScopedWriteLock lock( getMapDataMutex() );
            _terrainMaskLayer = 0L;
        }
        
        // a separate block b/c we don't need the mutex   
        for( MapCallbackList::iterator i = _mapCallbacks.begin(); i != _mapCallbacks.end(); i++ )
        {
            i->get()->onMaskLayerRemoved( layer );
        }	
    }
}

osg::TransferFunction1D* Map::getContourTransferFunction(void) const
{
  return _contourTransferFunction;
}
        
void Map::setContourTransferFunction(osg::TransferFunction1D* transferFunction)
{
  _contourTransferFunction = transferFunction;
}

void
Map::calculateProfile()
{
    if ( _profile.valid() )
        return;

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
        Threading::ScopedReadLock lock( getMapDataMutex() );

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
            i->get()->onMapProfileEstablished( _profile.get() );
        }
    }

    else
    {
        OE_WARN << LC << "Warning, not yet able to establish a map profile!" << std::endl;
    }
}


static osg::HeightField*
s_createHeightField(const TileKey& key,
                    const ElevationLayerVector& elevLayers,
                    const Profile* mapProfile,
                    bool fallback,
                    ElevationInterpolation interpolation,
                    Map::SamplePolicy samplePolicy,
                    ProgressCallback* progress) 
{
	osg::HeightField *result = NULL;
    int lowestLOD = key.getLevelOfDetail();
    bool hfInitialized = false;

    typedef std::map< TerrainLayer*, bool > LayerValidMap;
    LayerValidMap layerValidMap;

	//Get a HeightField for each of the enabled layers
	GeoHeightFieldVector heightFields;

    unsigned int numValidHeightFields = 0;

    
    //First pass:  Try to get the exact LOD requested for each enabled heightfield
    for( ElevationLayerVector::const_iterator i = elevLayers.begin(); i != elevLayers.end(); i++ )
    {
        ElevationLayer* layer = i->get();
        if (layer->getProfile() && layer->getEnabled() )
        {
            osg::ref_ptr< osg::HeightField > hf = layer->createHeightField( key, progress );
            layerValidMap[ layer ] = hf.valid();
            if (hf.valid())
            {
                numValidHeightFields++;
                GeoHeightField ghf( hf.get(), key.getExtent(), layer->getProfile()->getVerticalSRS() );
                heightFields.push_back( ghf );
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
    for( ElevationLayerVector::const_iterator i = elevLayers.begin(); i != elevLayers.end(); i++ )
    {
        ElevationLayer* layer = i->get();

        if (layer->getProfile() && layer->getEnabled() )
        {
            if (!layerValidMap[ layer ])
            {
                TileKey hf_key = key;
                osg::ref_ptr< osg::HeightField > hf;
                while (hf_key.valid())
                {
                    hf = layer->createHeightField( hf_key, progress );
                    if (hf.valid()) break;
                    hf_key = hf_key.createParentKey();
                }

                if (hf.valid())
                {
                    if ( hf_key.getLevelOfDetail() < lowestLOD )
                        lowestLOD = hf_key.getLevelOfDetail();

                    heightFields.push_back( GeoHeightField(
                        hf.get(), hf_key.getExtent(), layer->getProfile()->getVerticalSRS() ) );
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
        if ( lowestLOD == key.getLevelOfDetail() )
        {
		    //If we only have on heightfield, just return it.
		    result = heightFields[0].takeHeightField();
        }
        else
        {
            GeoHeightField geoHF = heightFields[0].createSubSample( key.getExtent(), interpolation);
            result = geoHF.takeHeightField();
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
		result = new osg::HeightField();
		result->allocate( width, height );

		//Go ahead and set up the heightfield so we don't have to worry about it later
        double minx, miny, maxx, maxy;
        key.getExtent().getBounds(minx, miny, maxx, maxy);
        double dx = (maxx - minx)/(double)(result->getNumColumns()-1);
        double dy = (maxy - miny)/(double)(result->getNumRows()-1);

        const VerticalSpatialReference* vsrs = mapProfile->getVerticalSRS();
        
		//Create the new heightfield by sampling all of them.
        for (unsigned int c = 0; c < width; ++c)
        {
            double geoX = minx + (dx * (double)c);
            for (unsigned r = 0; r < height; ++r)
            {
                double geoY = miny + (dy * (double)r);

                //Collect elevations from all of the layers
                std::vector<float> elevations;
                for (GeoHeightFieldVector::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
                {
                    const GeoHeightField& geoHF = *itr;

                    float elevation = 0.0f;
                    if ( geoHF.getElevation(key.getExtent().getSRS(), geoX, geoY, interpolation, vsrs, elevation) )
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
                    if (samplePolicy == Map::FIRST_VALID)
                    {
                        elevation = elevations[0];
                    }
                    else if (samplePolicy == Map::HIGHEST)
                    {
                        elevation = -FLT_MAX;
                        for (unsigned int i = 0; i < elevations.size(); ++i)
                        {
                            if (elevation < elevations[i]) elevation = elevations[i];
                        }
                    }
                    else if (samplePolicy == Map::LOWEST)
                    {
                        elevation = FLT_MAX;
                        for (unsigned i = 0; i < elevations.size(); ++i)
                        {
                            if (elevation > elevations[i]) elevation = elevations[i];
                        }
                    }
                    else if (samplePolicy == Map::AVERAGE)
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
		key.getExtent().getBounds(minx, miny, maxx, maxy);
		result->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
		double dx = (maxx - minx)/(double)(result->getNumColumns()-1);
		double dy = (maxy - miny)/(double)(result->getNumRows()-1);
		result->setXInterval( dx );
		result->setYInterval( dy );
		result->setBorderWidth( 0 );
	}

	return result;
}


osg::HeightField*
Map::createHeightField( const TileKey& key,
                        bool fallback,
                        ElevationInterpolation interpolation,
                        SamplePolicy samplePolicy,
                        ProgressCallback* progress)
{
    Threading::ScopedReadLock lock( this->getMapDataMutex() );
    return s_createHeightField( key, _elevationLayers, getProfile(), fallback, interpolation, samplePolicy, progress );
}

#if 0
    //Note:  Assumes that the map data mutex is locked before calling.  Avoids reentrantcy issue on Linux.
    //TODO: reevaluate the need for this assumption ...
    //TODO: consider just taking a reference instead ...

	//OE_INFO << "[osgEarth::Map::createHeightField]" << std::endl;\
//     OpenThreads::ScopedReadLock lock( _mapDataMutex );

	osg::HeightField *result = NULL;
    int lowestLOD = key.getLevelOfDetail();
    bool hfInitialized = false;

    typedef std::map< TerrainLayer*, bool > LayerValidMap;
    LayerValidMap layerValidMap;

	//Get a HeightField for each of the enabled layers
	GeoHeightFieldVector heightFields;

    unsigned int numValidHeightFields = 0;

    
    //First pass:  Try to get the exact LOD requested for each enabled heightfield
    for( ElevationLayerVector::const_iterator i = _elevationLayers.begin(); i != _elevationLayers.end(); i++ )
    {
        ElevationLayer* layer = i->get();
        if (layer->getProfile() && layer->getEnabled() )
        {
            osg::ref_ptr< osg::HeightField > hf = layer->createHeightField( key, progress );
            layerValidMap[ layer ] = hf.valid();
            if (hf.valid())
            {
                numValidHeightFields++;
                GeoHeightField ghf( hf.get(), key.getExtent(), layer->getProfile()->getVerticalSRS() );
                heightFields.push_back( ghf );
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
    for( ElevationLayerVector::const_iterator i = _elevationLayers.begin(); i != _elevationLayers.end(); i++ )
    {
        ElevationLayer* layer = i->get();

        if (layer->getProfile() && layer->getEnabled() )
        {
            if (!layerValidMap[ layer ])
            {
                TileKey hf_key = key;
                osg::ref_ptr< osg::HeightField > hf;
                while (hf_key.valid())
                {
                    hf = layer->createHeightField( hf_key, progress );
                    if (hf.valid()) break;
                    hf_key = hf_key.createParentKey();
                }

                if (hf.valid())
                {
                    if ( hf_key.getLevelOfDetail() < lowestLOD )
                        lowestLOD = hf_key.getLevelOfDetail();

                    heightFields.push_back( GeoHeightField(
                        hf.get(), hf_key.getExtent(), layer->getProfile()->getVerticalSRS() ) );
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
        if ( lowestLOD == key.getLevelOfDetail() )
        {
		    //If we only have on heightfield, just return it.
		    result = heightFields[0].takeHeightField();
        }
        else
        {
            GeoHeightField geoHF = heightFields[0].createSubSample( key.getExtent(), interpolation);
            result = geoHF.takeHeightField();
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
		result = new osg::HeightField();
		result->allocate( width, height );

		//Go ahead and set up the heightfield so we don't have to worry about it later
        double minx, miny, maxx, maxy;
        key.getExtent().getBounds(minx, miny, maxx, maxy);
        double dx = (maxx - minx)/(double)(result->getNumColumns()-1);
        double dy = (maxy - miny)/(double)(result->getNumRows()-1);

        const VerticalSpatialReference* vsrs = getProfile()->getVerticalSRS();
        
		//Create the new heightfield by sampling all of them.
        for (unsigned int c = 0; c < width; ++c)
        {
            double geoX = minx + (dx * (double)c);
            for (unsigned r = 0; r < height; ++r)
            {
                double geoY = miny + (dy * (double)r);

                //Collect elevations from all of the layers
                std::vector<float> elevations;
                for (GeoHeightFieldVector::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
                {
                    const GeoHeightField& geoHF = *itr;

                    float elevation = 0.0f;
                    if ( geoHF.getElevation(key.getExtent().getSRS(), geoX, geoY, interpolation, vsrs, elevation) )
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
		key.getExtent().getBounds(minx, miny, maxx, maxy);
		result->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
		double dx = (maxx - minx)/(double)(result->getNumColumns()-1);
		double dy = (maxy - miny)/(double)(result->getNumRows()-1);
		result->setXInterval( dx );
		result->setYInterval( dy );
		result->setBorderWidth( 0 );
	}

	return result;
}
#endif

//------------------------------------------------------------------------

MapWorkingSet::MapWorkingSet( Map* map ) :
_map( map )
{
    Threading::ScopedReadLock lock( map->_mapDataMutex );

    _imageLayers.reserve( map->_imageLayers.size() );
    std::copy( map->_imageLayers.begin(), map->_imageLayers.end(), _imageLayers.begin() );

    _elevationLayers.reserve( map->_elevationLayers.size() );
    std::copy( map->_elevationLayers.begin(), map->_elevationLayers.end(), _elevationLayers.begin() );
}

osg::HeightField*
MapWorkingSet::createHeightField(const TileKey& key,
                                 bool fallback,
                                 ElevationInterpolation interpolation,
                                 Map::SamplePolicy samplePolicy,
                                 ProgressCallback* progress)
{
    return s_createHeightField( key, _elevationLayers, _map->getProfile(), fallback, interpolation, samplePolicy, progress );
}
