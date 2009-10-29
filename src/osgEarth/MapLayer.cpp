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
#include <osgEarth/MapLayer>
#include <osgEarth/Compositing>
#include <osgEarth/TileSourceFactory>
#include <osgEarth/ImageUtils>

using namespace osgEarth;

static unsigned int s_mapLayerID = 0;

MapLayer::MapLayer(const std::string& name, Type type, const std::string& driver, const Config& driverConf ) :
_name( name ),
_type( type ),
_driver( driver ),
_driverConf( driverConf ),
_opacity(1.0f),
_enabled(true),
_exactCropping(false),
_useMercatorFastPath(true),
_reprojected_tile_size(256),
_cacheOnly( false ),
_cacheOnlyEnv( false ),
_loadWeight( 1.0f )
{
	readEnvironmentalVariables();
    _id = s_mapLayerID++;
}

MapLayer::MapLayer(const std::string& name, Type type, TileSource* source ) :
_name( name ),
_type( type ),
_tileSource( source ),
_opacity(1.0f),
_enabled(true),
_exactCropping(false),
_useMercatorFastPath(true),
_reprojected_tile_size(256),
_cacheOnly( false ),
_cacheOnlyEnv( false ),
_loadWeight( 1.0f )
{
	readEnvironmentalVariables();
    _id = s_mapLayerID++;
}

optional<int>&
MapLayer::minLevel() {
    return _minLevel;
}
const optional<int>&
MapLayer::minLevel() const {
    return _minLevel;
}

optional<int>&
MapLayer::maxLevel() {
    return _maxLevel;
}
const optional<int>&
MapLayer::maxLevel() const {
    return _maxLevel;
}

const std::string&
MapLayer::getName() const {
    return _name; 
}

const MapLayer::Type&
MapLayer::getType() const {
    return _type;
}

unsigned int
MapLayer::getId() const {
    return _id;
}

const std::string& 
MapLayer::getDriver() const {
    return _driver;
}

const Config&
MapLayer::getDriverConfig() const {
    return _driverConf;
};

bool MapLayer::getCacheOnly() const
{
	return _cacheOnly || _cacheOnlyEnv;
}

void MapLayer::setCacheOnly( bool cacheOnly )
{
	_cacheOnly = cacheOnly;
}

TileSource* 
MapLayer::getTileSource() const {
	//Only load the TileSource if it hasn't been loaded previously and we aren't running strictly off the cache.
	if (!_tileSource.valid() && !getCacheOnly())
	{
		const_cast<MapLayer*>(this)->initTileSource();
	}
    return _tileSource.get();
}

const Profile*
MapLayer::getProfile() const
{
	if (!_profile.valid())
	{
		//Try first to get the profile from the TileSource
		if (getTileSource())
		{
			const_cast<MapLayer*>(this)->_profile = getTileSource()->getProfile();
		}	
		else if (_cache.valid())
		{
			//Get it from the Cache if possible.
			std::string format;
			unsigned int tile_size;
			const_cast<MapLayer*>(this)->_profile = _cache->loadLayerProperties(_name, format, tile_size);
			//Initialize the cache format if it hasn't been set already.
			if (_profile.valid() && _cacheFormat.empty())
			{
				const_cast<MapLayer*>(this)->_cacheFormat = format;
			}
		}
	}
	return _profile.get();
}

optional<ProfileConfig>&
MapLayer::profileConfig() {
    return _profileConf;
}
const optional<ProfileConfig>&
MapLayer::profileConfig() const {
    return _profileConf;
}

optional<std::string>& 
MapLayer::noDataImageFilename()
{
	return _nodata_image_filename;
}
const optional<std::string>&
MapLayer::noDataImageFilename() const
{
	return _nodata_image_filename;
}

const std::string&
MapLayer::getCacheFormat() const
{
	return _cacheFormat;
}

void
MapLayer::setCacheFormat(const std::string& cacheFormat)
{
	_cacheFormat = cacheFormat;
}

float
MapLayer::getLoadWeight() const
{
    return _loadWeight;
}

void
MapLayer::setLoadWeight(float loadWeight)
{
    _loadWeight = loadWeight;
}

void 
MapLayer::readEnvironmentalVariables()
{
	//See if cache only is turned on in an env var
	if (getenv("OSGEARTH_CACHE_ONLY") != 0)
	{
		_cacheOnlyEnv = true;
	}
}

void
MapLayer::initTileSource()
{	
	osg::notify(osg::INFO) << "[osgEarth::MapLayer::initTileSource()]" << std::endl;
	//Create the TileSource
	TileSourceFactory tileSourceFactory;
	osg::ref_ptr< TileSource > tileSource = tileSourceFactory.create( getDriver(),
        getDriverConfig(),
																	  getGlobalOptions());

	//Get the override profile if it is set.
	osg::ref_ptr<const Profile> override_profile;
	if (profileConfig().isSet())
	{
		override_profile = Profile::create( profileConfig().get() );
	}

	if ( tileSource.valid() )
	{
		//Initialize the TileSource
		tileSource->initialize( getReferenceURI(), override_profile.get() );

		if ( tileSource->isOK() )
		{
			if ( _nodata_image_filename.isSet() && _nodata_image_filename.get().length() > 0 )
			{
				osg::notify(osg::NOTICE) << "Setting nodata image to " << _nodata_image_filename.get() << std::endl;
				_nodata_image = osgDB::readImageFile( _nodata_image_filename.get());
				if (!_nodata_image.valid())
				{
					osg::notify(osg::NOTICE) << "Warning:  Could not read nodata image from " << _nodata_image_filename.get() << std::endl;
				}
			}
		}
		else
		{
	      osg::notify(osg::NOTICE) << "[osgEarth::MapLayer]  Could not initialize TileSource for layer " << getName() << std::endl;
		}
	}
	_tileSource = tileSource;

	//Set the cache format to the native format of the TileSource if it isn't already set.
	if (_tileSource.valid() && _cacheFormat.empty())
	{
		_cacheFormat = _tileSource->getExtension();
	}

	if (_tileSource.valid() && _cache.valid())
	{
		_cache->storeLayerProperties( _name, _tileSource->getProfile(), _cacheFormat, _tileSource->getPixelsPerTile() );
	}
}

float
MapLayer::getOpacity() const
{
	return _opacity;
}

void
MapLayer::setOpacity(float opacity)
{
	_opacity = osg::clampBetween(opacity, 0.0f, 1.0f);
}

bool
MapLayer::getEnabled() const
{
	return _enabled;
}

void
MapLayer::setEnabled(bool enabled)
{
	_enabled = enabled;
}

const optional<osg::Vec4ub>& 
MapLayer::transparentColor() const
{
	return _transparentColor;
}

optional<osg::Vec4ub>&
MapLayer::transparentColor()
{
	return _transparentColor;
}

const osgDB::ReaderWriter::Options*
MapLayer::getGlobalOptions() const {
    return _globalOptions.get();
}

void
MapLayer::setGlobalOptions( const osgDB::ReaderWriter::Options* options ) {
    _globalOptions = options;
}


bool
MapLayer::isKeyValid(const TileKey* key) const
{
	if (!key) return false;

	if (_minLevel.isSet() && key->getLevelOfDetail() < _minLevel.get()) return false;
	if (_maxLevel.isSet() && key->getLevelOfDetail() > _maxLevel.get()) return false;
	return true;
}

const std::string&
MapLayer::getReferenceURI() const { 
    return _referenceURI;
}

void
MapLayer::setReferenceURI( const std::string& uri ) {
    _referenceURI = uri;
}

GeoImage*
MapLayer::createImage( const TileKey* key,
                       ProgressCallback* progress)
{
	GeoImage* result = NULL;
    const Profile* mapProfile = key->getProfile();
	const Profile* layerProfile = getProfile();

	if (!layerProfile)
	{
		osg::notify(osg::NOTICE) << "Could not get a valid profile for Layer " << _name << std::endl;
		return NULL;
	}



	//osg::notify(osg::NOTICE) << "[osgEarth::MapLayer::createImage] " << key->str() << std::endl;
	if (!getTileSource() && !getCacheOnly())
	{
		osg::notify(osg::NOTICE) << "Error:  MapLayer does not have a valid TileSource, cannot create image " << std::endl;
		return NULL;
	}

	//Determine whether we should cache in the Map profile or the Layer profile.
	
	bool cacheInMapProfile = true;
	if (mapProfile->isEquivalentTo( layerProfile) )
	{
		osg::notify(osg::INFO) << "Layer " << _name << ": Map and Layer profiles are equivalent " << std::endl;
	}
	//If the map profile and layer profile are in the same SRS but with different tiling scemes and exact cropping is not required, cache in the layer profile.
	else if (mapProfile->getSRS()->isEquivalentTo( layerProfile->getSRS()) && !_exactCropping)
	{
		osg::notify(osg::INFO) << "Layer " << _name << ": Map and Layer profiles are in the same SRS and non-exact cropping is allowed, caching in layer profile." << std::endl;
		cacheInMapProfile = false;
	}
	//If the map profile is geographic and the layer is mercator and we are allowed to use the mercator fast path, cache in the layer profile.
	else if (mapProfile->getSRS()->isGeographic() && layerProfile->getSRS()->isMercator() && _useMercatorFastPath)
	{
		osg::notify(osg::INFO) << "Layer " << _name << ": Map profile is geographic and Layer profile is mercator and mercator fast path is allowed, caching in layer profile" << std::endl;
		cacheInMapProfile = false;
	}

	bool cacheInLayerProfile = !cacheInMapProfile;

	if (cacheInMapProfile)
	{
		osg::notify(osg::INFO) << "Layer " << _name << " caching in Map profile " << std::endl;
	}

	//If we are caching in the map profile, try to get the image immediately.
	if (cacheInMapProfile && _cache.valid())
	{
		osg::Image* image = _cache->getImage( key, _name, _cacheFormat);
		if (image)
		{
			osg::notify(osg::INFO) << "Layer " << _name << " got tile " << key->str() << " from map cache " << std::endl;
			return new GeoImage( image, key->getGeoExtent() );
		}
	}

	//If the key profile and the source profile exactly match, simply request the image from the source
    if ( mapProfile->isEquivalentTo( layerProfile ) )
    {
		osg::notify(osg::INFO) << "  Key and source profiles are equivalent, requesting single tile" << std::endl;
        osg::Image* image = createImageWrapper( key, cacheInLayerProfile, progress );
        if ( image )
        {
            result = new GeoImage( image, key->getGeoExtent() );
        }
    }
    // Otherwise, we need to process the tiles.
    else
    {
		osg::notify(osg::INFO) << "  Key and source profiles are different, creating mosaic" << std::endl;
		osg::ref_ptr<GeoImage> mosaic;
		osg::Image* image = NULL;

		// Determine the intersecting keys and create and extract an appropriate image from the tiles
		std::vector< osg::ref_ptr<const TileKey> > intersectingTiles;
		layerProfile->getIntersectingTiles(key, intersectingTiles);

		if (intersectingTiles.size() > 0)
		{
			double dst_minx, dst_miny, dst_maxx, dst_maxy;
			key->getGeoExtent().getBounds(dst_minx, dst_miny, dst_maxx, dst_maxy);

			osg::ref_ptr<MultiImage> mi = new MultiImage;
			for (unsigned int j = 0; j < intersectingTiles.size(); ++j)
			{
				double minX, minY, maxX, maxY;
				intersectingTiles[j]->getGeoExtent().getBounds(minX, minY, maxX, maxY);

				//osg::notify(osg::NOTICE) << "\t Intersecting Tile " << j << ": " << minX << ", " << minY << ", " << maxX << ", " << maxY << std::endl;

				osg::ref_ptr<osg::Image> img = createImageWrapper(intersectingTiles[j].get(), cacheInLayerProfile, progress);
				if (img.valid())
				{
					mi->getImages().push_back(TileImage(img.get(), intersectingTiles[j].get()));
				}
				else
				{
					//If we couldn't create an image that is needed to composite, return NULL
					osg::notify(osg::INFO) << "Couldn't create image for MultiImage " << std::endl;
					return 0;
				}
			}

			double rxmin, rymin, rxmax, rymax;
			mi->getExtents( rxmin, rymin, rxmax, rymax );

			mosaic = new GeoImage(
				mi->createImage(),
				GeoExtent( layerProfile->getSRS(), rxmin, rymin, rxmax, rymax ) );
		}

		if ( mosaic.valid() )
        {
            // whether to use the fast-path mercator locator. If so, DO NOT reproject the imagery here.
            bool useMercatorFastPath =
                mosaic->getSRS()->isMercator() &&
                key->getProfile()->getSRS()->isGeographic() &&
                _useMercatorFastPath;

            // the imagery must be reprojected iff:
            //  * we're not using the mercator fast path (see above);
            //  * the SRS of the image is different from the SRS of the key;
            //  * UNLESS they are both geographic SRS's (in which case we can skip reprojection)
            bool needsReprojection =
                !useMercatorFastPath &&
                !mosaic->getSRS()->isEquivalentTo( key->getProfile()->getSRS()) &&
                !(mosaic->getSRS()->isGeographic() && key->getProfile()->getSRS()->isGeographic());

            if ( needsReprojection )
            {
				osg::notify(osg::INFO) << "  Reprojecting image" << std::endl;
                //We actually need to reproject the image.  Note:  The GeoImage::reprojection function will automatically
                //crop the image to the correct extents, so there is no need to crop after reprojection.
                result = mosaic->reproject( key->getProfile()->getSRS(), &key->getGeoExtent(), _reprojected_tile_size, _reprojected_tile_size);
            }
            else
            {
				osg::notify(osg::INFO) << "  Cropping image" << std::endl;
                // crop to fit the map key extents
                GeoExtent clampedMapExt = layerProfile->clampAndTransformExtent( key->getGeoExtent() );
                if ( clampedMapExt.width() * clampedMapExt.height() > 0 )
				{
					int size = _exactCropping ? _reprojected_tile_size : 0;
                    result = mosaic->crop(clampedMapExt, _exactCropping, size, size);
				}
                else
                    result = NULL;
            }
        }
    }

	//If we got a result, the cache is valid and we are caching in the map profile, write to the map cache.
	if (result && _cache.valid() && cacheInMapProfile)
	{
		osg::notify(osg::INFO) << "Layer " << _name << " writing tile " << key->str() << " to cache " << std::endl;
		_cache->setImage( key, _name, _cacheFormat, result->getImage());
	}

    return result;
}

osg::Image*
MapLayer::createImageWrapper( const TileKey* key,
                              bool cacheInLayerProfile,
                              ProgressCallback* progress)
{
	TileSource* source = getTileSource();

	osg::ref_ptr<osg::Image> image;

	if (_cache.valid() && cacheInLayerProfile)
		image = _cache->getImage( key, _name, _cacheFormat );

	if (image.valid())
	{
		osg::notify(osg::INFO) << " Layer " << _name << " got " << key->str() << " from cache " << std::endl;
	}

	if (source && !image.valid() && !getCacheOnly())
	{
		image = source->getImage( key, progress );

		//Check to see if the image is the nodata image
		if (image.valid() && _nodata_image.valid())
		{
			if (ImageUtils::areEquivalent(image.get(), _nodata_image.get()))
			{
				osg::notify(osg::INFO) << "[osgEarth::MapLayer::createImage] Found nodata for " << key->str() << std::endl;
				image = 0;
			}
		}

		//Apply a transparent color mask if one is specified
		if (image.valid() && _transparentColor.isSet())
		{
			for (unsigned int row = 0; row < image->t(); ++row)
			{
				for (unsigned int col = 0; col < image->s(); ++col)
				{
					unsigned char r = image->data(col, row)[0];
					unsigned char g = image->data(col, row)[1];
					unsigned char b = image->data(col, row)[2];

					if (r == _transparentColor->r() &&
						g == _transparentColor->g() &&
						b == _transparentColor->b())
					{
						//osg::notify(osg::NOTICE) << "Transparent..." << std::endl;
						image->data(col,row)[3] = 0;
					}
				}
			}
		}

		if (image.valid() && _cache.valid() && cacheInLayerProfile)
		{
			_cache->setImage( key, _name, _cacheFormat, image);
		}
	}
	return image.release();
}

GeoHeightField*
MapLayer::createGeoHeightField(const TileKey* key,
                               ProgressCallback * progress)
{
	osg::HeightField* hf = getTileSource()->getHeightField( key, progress );
	if (hf)
	{
		//Modify the heightfield data so that is contains a standard value for NO_DATA
		osg::ref_ptr<CompositeValidValueOperator> ops = new CompositeValidValueOperator;
		ops->getOperators().push_back(new osgTerrain::NoDataValue(getTileSource()->getNoDataValue()));
		ops->getOperators().push_back(new osgTerrain::ValidRange(getTileSource()->getNoDataMinValue(), getTileSource()->getNoDataMaxValue()));

		ReplaceInvalidDataOperator o;
		o.setReplaceWith(NO_DATA_VALUE);
		o.setValidDataOperator(ops.get());
		o(hf);

		return new GeoHeightField(hf, key->getGeoExtent());
	}
	return NULL;
}

osg::HeightField*
MapLayer::createHeightField(const osgEarth::TileKey *key,
                            ProgressCallback* progress)
{
	osg::ref_ptr<osg::HeightField> result;

	//See if we can get it from the cache.
	if (_cache.valid())
	{
		result = _cache->getHeightField( key, _name, _cacheFormat );
		if (result.valid())
		{
			osg::notify(osg::INFO) << "MapLayer::createHeightField got tile " << key->str() << " from layer " << _name << " from cache " << std::endl;
		}
	}

	if (!result.valid() && getTileSource())
	{
		//If the profiles are equivalent, get the HF from the TileSource.
		if (key->getProfile()->isEquivalentTo( getProfile() ))
		{
			if (isKeyValid( key ) )
			{
				osg::ref_ptr<GeoHeightField> hf = createGeoHeightField( key, progress );
				if (hf.valid())
				{
					result = hf->takeHeightField();
				}
			}
		}
		else
		{
			//Collect the heightfields for each of the intersecting tiles.
			typedef std::vector< osg::ref_ptr<GeoHeightField> > HeightFields;
			HeightFields heightFields;

			//Determine the intersecting keys
			std::vector< osg::ref_ptr<const TileKey> > intersectingTiles;
			getProfile()->getIntersectingTiles(key, intersectingTiles);
			if (intersectingTiles.size() > 0)
			{
				for (unsigned int i = 0; i < intersectingTiles.size(); ++i)
				{
					if (isKeyValid( intersectingTiles[i].get() ) )
					{
						GeoHeightField *hf = createGeoHeightField( intersectingTiles[i].get(), progress );
						if (hf)
						{
							heightFields.push_back(hf);
						}
					}
				}
			}

			//If we actually got a HeightField, resample/reproject it to match the incoming TileKey's extents.
			if (heightFields.size() > 0)
			{		
				unsigned int width = 0;
				unsigned int height = 0;

				for (HeightFields::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
				{
					if (itr->get()->getHeightField()->getNumColumns() > width) width = itr->get()->getHeightField()->getNumColumns();
					if (itr->get()->getHeightField()->getNumRows() > height) height = itr->get()->getHeightField()->getNumRows();
				}

				result = new osg::HeightField;
				result->allocate(width, height);

				//Go ahead and set up the heightfield so we don't have to worry about it later
				double minx, miny, maxx, maxy;
				key->getGeoExtent().getBounds(minx, miny, maxx, maxy);
				double dx = (maxx - minx)/(double)(width-1);
				double dy = (maxy - miny)/(double)(height-1);

				//Create the new heightfield by sampling all of them.
				for (unsigned int c = 0; c < width; ++c)
				{
					double geoX = minx + (dx * (double)c);
					for (unsigned r = 0; r < height; ++r)
					{
						double geoY = miny + (dy * (double)r);

						//For each sample point, try each heightfield.  The first one with a valid elevation wins.
						float elevation = NO_DATA_VALUE;
						for (HeightFields::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
						{
							float e = 0.0;
							if (itr->get()->getElevation(key->getGeoExtent().getSRS(), geoX, geoY, BILINEAR, e))
							{
								elevation = e;
								break;
							}
						}
						result->setHeight( c, r, elevation );                
					}
				}
			}
		}
	}

	//Initialize the HF values for osgTerrain
	if (result.valid())
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

	//Write the result to the cache.
	if (result.valid() && _cache.valid())
	{
		_cache->setHeightField( key, _name, _cacheFormat, result.get() );
	}

	return result.release();
}

