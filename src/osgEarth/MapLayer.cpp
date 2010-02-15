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
#include <osgEarth/TaskService>
#include <osgEarth/Registry>
#include <osgDB/WriteFile>
#include <osg/Version>
#include <OpenThreads/ScopedLock>

using namespace osgEarth;
using namespace OpenThreads;

static unsigned int s_mapLayerID = 0;

MapLayer::MapLayer(const std::string& name, Type type, const std::string& driver, const Config& driverConf ) :
osg::Referenced( true ),
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
_cacheEnabled(true),
_loadWeight( 1.0f ),
_profileConf( ProfileConfig() ),
_minLevel(0),
_maxLevel(99),
_nodata_image_filename(""),
_transparentColor(osg::Vec4ub(0,0,0,0))
{
	readEnvironmentalVariables();
    _id = s_mapLayerID++;
}

// this ctor is for backwards compat only
MapLayer::MapLayer(const std::string& name, Type type, const std::string& driver, const Properties& driverProps ) :
osg::Referenced( true ),
_name( name ),
_type( type ),
_driver( driver ),
_opacity(1.0f),
_enabled(true),
_exactCropping(false),
_useMercatorFastPath(true),
_reprojected_tile_size(256),
_cacheOnly( false ),
_cacheOnlyEnv( false ),
_cacheEnabled(true),
_loadWeight( 1.0f ),
_profileConf( ProfileConfig() ),
_minLevel(0),
_maxLevel(99),
_nodata_image_filename(""),
_transparentColor(osg::Vec4ub(0,0,0,0))
{
    for( Properties::const_iterator i = driverProps.begin(); i != driverProps.end(); i++ )
        _driverConf.add( i->first, i->second );

	readEnvironmentalVariables();
    _id = s_mapLayerID++;
    _cacheFormat = suggestCacheFormat();
}

MapLayer::MapLayer(const std::string& name, Type type, TileSource* source ) :
osg::Referenced( true ),
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
_loadWeight( 1.0f ),
_profileConf( ProfileConfig() ),
_minLevel(0),
_maxLevel(99),
_nodata_image_filename(""),
_transparentColor(osg::Vec4ub(0,0,0,0))
{
	readEnvironmentalVariables();
    _id = s_mapLayerID++;
    _cacheFormat = suggestCacheFormat();

    //Try to get the profile from the TileSource
    if (_tileSource)
    {
        _profile = _tileSource->getProfile();
    }
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
}

bool MapLayer::getCacheOnly() const
{
	return _cacheOnly || _cacheOnlyEnv;
}

void MapLayer::setCacheOnly( bool cacheOnly )
{
	_cacheOnly = cacheOnly;
}

bool MapLayer::getCacheEnabled() const
{
    return _cacheEnabled;
}

void MapLayer::setCacheEnabled( bool cacheEnabled)
{
    _cacheEnabled = cacheEnabled;
}

Cache*
MapLayer::getCache() const
{
    return _cache.get();
}

void
MapLayer::setCache(Cache* cache)
{
    if (_cache.get() != cache)
    {
        _cache = cache;        

        //Read properties from the cache if not already set
        if (_cache.valid() && _cacheEnabled)
        {
            std::string format;
            unsigned int tile_size;
            osg::ref_ptr< const Profile > profile = _cache->loadLayerProperties(_name, format, tile_size);

            //Set the profile if it hasn't already been set
            if (!_profile.valid() && profile.valid())
            {
                _profile = profile.get();
            }

            //Set the cache format if it hasn't already been set
            if (_cacheFormat.empty())
            {
                _cacheFormat = format;
            }
        }
    }
}

TileSource* 
MapLayer::getTileSource() const {
	//Only load the TileSource if it hasn't been loaded previously and we aren't running strictly off the cache.
	if (!_tileSource.valid() && !getCacheOnly())
	{
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock(const_cast<MapLayer*>(this)->_initMutex );
        //Double check
        if (!_tileSource.valid() && !getCacheOnly())
        {
            const_cast<MapLayer*>(this)->initTileSource();
        }
	}
    return _tileSource.get();
}

const Profile*
MapLayer::getProfile() const
{
	if (!_profile.valid())
	{
		//Make sure the tileSource is initialized, the profile will be set when the tilesource is initialized.
		getTileSource();
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

std::string
MapLayer::suggestCacheFormat() const
{
    if (_type == MapLayer::TYPE_HEIGHTFIELD)
    {
#if OSG_MIN_VERSION_REQUIRED(2,8,0)
        //OSG 2.8 onwards should use TIF for heightfields
        return "tif";
#else
        //OSG 2.8 and below should use DDS
        return "dds";
#endif
    }

    //Use the format of the TileSource if it is valid.
    if (_tileSource.valid())
    {
        return _tileSource->getExtension();
    }

    //Default to PNG
    return "png";
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
	osg::ref_ptr< TileSource > tileSource = tileSourceFactory.create(
        getDriver(),
        getDriverConfig(),
        getGlobalOptions() );

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
    if (_cacheFormat.empty())
    {
        _cacheFormat = suggestCacheFormat();
    }

    if (_tileSource.valid())
    {
      _profile = _tileSource->getProfile();
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
    osg::ref_ptr<GeoImage> result = NULL;
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

    //Write the cache TMS file if it hasn't been written yet.
    if (!_cacheProfile.valid() && _cache.valid() && _cacheEnabled && _tileSource.valid())
    {
        _cacheProfile = cacheInMapProfile ? mapProfile : _profile.get();
        _cache->storeLayerProperties( _name, _cacheProfile, _cacheFormat, _tileSource->getPixelsPerTile() );
    }

	if (cacheInMapProfile)
	{
		osg::notify(osg::INFO) << "Layer " << _name << " caching in Map profile " << std::endl;
	}

	//If we are caching in the map profile, try to get the image immediately.
	if (cacheInMapProfile && _cache.valid() && _cacheEnabled)
	{
        osg::ref_ptr<osg::Image> image = _cache->getImage( key, _name, _cacheFormat);
		if (image)
		{
			osg::notify(osg::INFO) << "Layer " << _name << " got tile " << key->str() << " from map cache " << std::endl;
			return new GeoImage( image.get(), key->getGeoExtent() );
		}
	}

	//If the key profile and the source profile exactly match, simply request the image from the source
    if ( mapProfile->isEquivalentTo( layerProfile ) )
    {
		osg::notify(osg::INFO) << "  Key and source profiles are equivalent, requesting single tile" << std::endl;
        osg::ref_ptr<osg::Image> image = createImageWrapper( key, cacheInLayerProfile, progress );
        if ( image )
        {
            result = new GeoImage( image.get(), key->getGeoExtent() );
        }
    }
    // Otherwise, we need to process the tiles.
    else
    {
		osg::notify(osg::INFO) << "  Key and source profiles are different, creating mosaic" << std::endl;
		osg::ref_ptr<GeoImage> mosaic;

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

				osg::notify(osg::INFO) << "\t Intersecting Tile " << j << ": " << minX << ", " << minY << ", " << maxX << ", " << maxY << std::endl;

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

            bool needsLeftBorder = false;
            bool needsRightBorder = false;
            bool needsTopBorder = false;
            bool needsBottomBorder = false;

            //If we don't need to reproject the data, we had to composite the data, so check to see if we need to add an extra, transparent pixel on the sides
            //because the data doesn't encompass the entire map.
            if (!needsReprojection)
            {
                GeoExtent keyExtent = key->getGeoExtent();
                //If the key is geographic and the mosaic is mercator, we need to get the mercator extents to determine if we need
                //to add the border or not
                if (key->getGeoExtent().getSRS()->isGeographic() && mosaic->getSRS()->isMercator())
                {
                    keyExtent = osgEarth::Registry::instance()->getGlobalMercatorProfile()->clampAndTransformExtent( key->getGeoExtent( ));
                }


                //Use an epsilon to only add the border if it is significant enough.
                double eps = 1e-6;
                
                double leftDiff = mosaic->getExtent().xMin() - keyExtent.xMin();
                if (leftDiff > eps)
                {
                    needsLeftBorder = true;
                }

                double rightDiff = keyExtent.xMax() - mosaic->getExtent().xMax();
                if (rightDiff > eps)
                {
                    needsRightBorder = true;
                }

                double bottomDiff = mosaic->getExtent().yMin() - keyExtent.yMin();
                if (bottomDiff > eps)
                {
                    needsBottomBorder = true;
                }

                double topDiff = keyExtent.yMax() - mosaic->getExtent().yMax();
                if (topDiff > eps)
                {
                    needsTopBorder = true;
                }

               
                //needsLeftBorder  = mosaic->getExtent().xMin() > keyExtent.xMin();
                //needsBottomBorder = mosaic->getExtent().yMin() > keyExtent.yMin();
                //needsRightBorder = mosaic->getExtent().xMax() < keyExtent.xMax();
                //needsTopBorder = mosaic->getExtent().yMax() < keyExtent.yMax();
            }

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

            //Add the transparent pixel AFTER the crop so that it doesn't get cropped out
            if (result && (needsLeftBorder || needsRightBorder || needsBottomBorder || needsTopBorder))
            {
                result = result->addTransparentBorder(needsLeftBorder, needsRightBorder, needsBottomBorder, needsTopBorder);
            }
        }
    }

	//If we got a result, the cache is valid and we are caching in the map profile, write to the map cache.
	if (result && _cache.valid() && _cacheEnabled && cacheInMapProfile)
	{
		osg::notify(osg::INFO) << "Layer " << _name << " writing tile " << key->str() << " to cache " << std::endl;
		_cache->setImage( key, _name, _cacheFormat, result->getImage());
	}

    return result.release();
}

osg::Image*
MapLayer::createImageWrapper( const TileKey* key,
                              bool cacheInLayerProfile,
                              ProgressCallback* progress)
{
	TileSource* source = getTileSource();

	osg::ref_ptr<osg::Image> image;

	if (_cache.valid() && cacheInLayerProfile && _cacheEnabled)
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

		if (image.valid() && _cache.valid() && cacheInLayerProfile && _cacheEnabled)
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
    osg::ref_ptr<osg::HeightField> hf = getTileSource()->getHeightField( key, progress );
	if (hf.valid())
	{
		//Modify the heightfield data so that is contains a standard value for NO_DATA
		osg::ref_ptr<CompositeValidValueOperator> ops = new CompositeValidValueOperator;
		ops->getOperators().push_back(new osgTerrain::NoDataValue(getTileSource()->getNoDataValue()));
		ops->getOperators().push_back(new osgTerrain::ValidRange(getTileSource()->getNoDataMinValue(), getTileSource()->getNoDataMaxValue()));

		ReplaceInvalidDataOperator o;
		o.setReplaceWith(NO_DATA_VALUE);
		o.setValidDataOperator(ops.get());
		o(hf);

		return new GeoHeightField(hf.get(), key->getGeoExtent());
	}
	return NULL;
}

osg::HeightField*
MapLayer::createHeightField(const osgEarth::TileKey *key,
                            ProgressCallback* progress)
{
    const Profile* mapProfile = getProfile();

	osg::ref_ptr<osg::HeightField> result;

    //Write the layer properties if they haven't been written yet.  Heightfields are always stored in the map profile.
    if (!_cacheProfile.valid() && _cache.valid() && _cacheEnabled && _tileSource.valid())
    {
        _cacheProfile = mapProfile;
        if ( _tileSource->isOK() )
        {
            _cache->storeLayerProperties( _name, _cacheProfile, _cacheFormat, _tileSource->getPixelsPerTile() );
        }
    }

	//See if we can get it from the cache.
	if (_cache.valid() && _cacheEnabled)
	{
		result = _cache->getHeightField( key, _name, _cacheFormat );
		if (result.valid())
		{
			osg::notify(osg::INFO) << "MapLayer::createHeightField got tile " << key->str() << " from layer " << _name << " from cache " << std::endl;
		}
	}

	if (!result.valid() && getTileSource() && getTileSource()->isOK() )
	{
		//If the profiles are equivalent, get the HF from the TileSource.
		if (key->getProfile()->isEquivalentTo( mapProfile ))
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
                        osg::ref_ptr<GeoHeightField> hf = createGeoHeightField( intersectingTiles[i].get(), progress );
						if (hf.valid())
						{
							heightFields.push_back(hf.get());
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
	if (result.valid() && _cache.valid() && _cacheEnabled)
	{
		_cache->setHeightField( key, _name, _cacheFormat, result.get() );
	}

	return result.release();
}

//----------------------------------------------------------------------------

struct FetchTask : public TaskRequest 
{
    FetchTask( MapLayer* layer, const TileKey* key, L2Cache::TileCache& cache )
        : _layer(layer), _key(key), _cache(cache) { }

    void operator() ( ProgressCallback* progress )
    {
        //osg::notify(osg::NOTICE) << "Task: fetching " << _key->str() << std::endl;

        if ( _layer->getType() == MapLayer::TYPE_IMAGE ) {
            GeoImage* img = _layer->createImage( _key );
            if ( img ) _cache.put( L2Cache::TileTag(_layer.get(),_key->str()), img );
        }
        else { // MapLayer::TYPE_HEIGHTFIELD
            osg::HeightField* hf = _layer->createHeightField( _key.get() );
            if ( hf ) _cache.put( L2Cache::TileTag(_layer.get(),_key->str()), hf );
        }
    }
    osg::ref_ptr<MapLayer> _layer;
    osg::ref_ptr<const TileKey> _key;
    L2Cache::TileCache& _cache;
};

void
L2Cache::TileCache::put( const L2Cache::TileTag& tag, osg::Referenced* obj )
{
    ScopedLock<Mutex> lock(_mutex);
    _map[tag] = obj;
    _fifo.push(tag);
    if ( _fifo.size() > 1024 ) {
        _map.erase( _fifo.front() );
        _fifo.pop();
    }
}

osg::Referenced*
L2Cache::TileCache::get( const L2Cache::TileTag& tag )
{
    ScopedLock<Mutex> lock(_mutex);
    L2Cache::TileMap::iterator i = _map.find(tag);
    osg::ref_ptr<osg::Referenced> result = i != _map.end() ? i->second.get() : 0L;
    if ( result.valid() ) {
        _map.erase( i );
    }
    if ( result.valid() )
        _hits++;
    else
        _misses++;
    if ( result.valid() )
        osg::notify(osg::NOTICE) << "hit, ratio = " << (float)_hits/((float)_hits+(float)_misses) << "%, cache size=" << _map.size() << std::endl;
    else
        osg::notify(osg::NOTICE) << "miss, ratio = " << (float)_hits/((float)_hits+(float)_misses) << "%, cache size=" << _map.size() << std::endl;
    return result.release();
}

L2Cache::L2Cache()
{
    _service = new TaskService(16);
}

void
L2Cache::scheduleSubKeys(MapLayer* layer, const TileKey* key)
{
    for( int i=0; i<4; i++ )
    {
        osg::ref_ptr<TileKey> subkey = key->createSubkey( i );
        if ( layer->isKeyValid( subkey.get() ) )
            _service->add( new FetchTask( layer, subkey.get(), _cache ) );
    }
}

GeoImage*
L2Cache::createImage(MapLayer* layer, const TileKey* key)
{
    scheduleSubKeys(layer,key);
    osg::ref_ptr<GeoImage> result;
    result = dynamic_cast<GeoImage*>( _cache.get( TileTag(layer,key->str()) ) );
    if ( !result.valid() )
        result = layer->createImage( key );
    return result.release();
}

osg::HeightField*
L2Cache::createHeightField(MapLayer* layer, const TileKey* key)
{
    scheduleSubKeys(layer,key);
    osg::ref_ptr<osg::HeightField> result;
    result = dynamic_cast<osg::HeightField*>( _cache.get( TileTag(layer,key->str()) ) );
    if ( !result.valid() )
        result = layer->createHeightField( key );
    return result.release();
}
