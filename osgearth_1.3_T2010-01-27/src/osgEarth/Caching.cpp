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
#include <limits.h>
#include <iomanip>

#include <osgEarth/Caching>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/FileUtils>
#include <osgEarth/ImageUtils>

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <OpenThreads/ScopedLock>

using namespace osgEarth;



/************************************************************************/


CacheConfig::CacheConfig() :
_type( CacheConfig::TYPE_DEFAULT ),
_runOffCacheOnly( false )
{
    //NOP
}

CacheConfig::CacheConfig( const Config& conf ) :
_runOffCacheOnly( false )
{
    _type =
        conf.attr( "type" ) == "tilecache" ? TYPE_TILECACHE :
        conf.attr( "type" ) == "tms" ? TYPE_TMS :
        TYPE_DEFAULT;

    if ( !conf.attr("cache_only").empty() )
        _runOffCacheOnly = conf.attr("cache_only") == "true";



    for( ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); i++ )
        if ( !i->value().empty() )
            _properties[ i->name() ] = i->value();
}

Config
CacheConfig::toConfig( const std::string& name ) const
{
    Config conf( name.empty()? "cache" : name );
    
    conf.attr( "type" ) =
        _type == TYPE_TILECACHE ? "tilecache" :
        _type == TYPE_TMS ? "tms" :
        "default";

    if ( _runOffCacheOnly.isSet() )
        conf.attr("cache_only") = _runOffCacheOnly.get() ? "true" : "false";

    for( Properties::const_iterator p = _properties.begin(); p != _properties.end(); p++ )
        conf.add( p->first, p->second );

    return conf;
}

const CacheConfig::CacheType&
CacheConfig::getType() const
{
    return _type;
}

void
CacheConfig::setType(const CacheConfig::CacheType& type)
{
    _type = type;
}

optional<bool>&
CacheConfig::runOffCacheOnly() {
    return _runOffCacheOnly;
}
const optional<bool>&
CacheConfig::runOffCacheOnly() const {
    return _runOffCacheOnly;
}

/**
* Gets the collection of name/value pairs for the cache.
*/
Properties&
CacheConfig::getProperties()
{
    return _properties;
}

const Properties& CacheConfig::getProperties() const
{
    return _properties;
}

std::string
CacheConfig::toString() const
{
    std::stringstream buf;
    buf << "type=" << getType();    
    buf << ", reproject=";
    buf << ", caching=";
    if (runOffCacheOnly().isSet()) buf << runOffCacheOnly().get(); else buf << "unset";

    for (Properties::const_iterator i = _properties.begin(); i != _properties.end(); i++ )
        buf << ", " << i->first << "=" << i->second;   

	std::string bufStr;
	bufStr = buf.str();
    return bufStr;
}


/*****************************************************************************/

Cache::Cache():
osg::Referenced(true)
{
}

void Cache::storeLayerProperties( const std::string& layerName,
		                          const Profile* profile,
			                      const std::string& format,
			                      unsigned int tile_size)
{
	//NOP
}

const Profile* Cache::loadLayerProperties( const std::string& layerName,
	                                       std::string& format,
			                               unsigned int &tile_size )
{
	//NOP
	return NULL;
}

osg::HeightField*
Cache::getHeightField( const TileKey* key,
			           const std::string& layerName,
					   const std::string& format)
{
	osg::HeightField* hf = 0;

	//Try to get an image from the cache
	osg::ref_ptr<osg::Image> image = getImage( key, layerName, format );
	if (image.valid())
	{
		osg::notify(osg::INFO) << "[osgEarth::Cache] Read cached heightfield " << std::endl;
		ImageToHeightFieldConverter conv;
		hf = conv.convert(image.get());
	}
	return hf;
}

void
Cache::setHeightField( const TileKey* key,
					   const std::string& layerName,
					   const std::string& format,
					   osg::HeightField* hf)
{
	ImageToHeightFieldConverter conv;
	//Take a reference to the returned image so it gets deleted properly
    osg::ref_ptr<osg::Image> image = conv.convert(hf);
	setImage( key, layerName, format, image.get() );
}


/*****************************************************************************/

static OpenThreads::ReadWriteMutex s_mutex;

DiskCache::DiskCache():
_writeWorldFiles(false)
{
}

DiskCache::DiskCache(const std::string& path):
_path(path),
_writeWorldFiles(false)
{
}

bool
DiskCache::isCached(const osgEarth::TileKey *key,
					const std::string& layerName,
					const std::string& format) const
{
	//Check to see if the file for this key exists
	std::string filename = getFilename(key,layerName,format);
    return osgDB::fileExists(filename);
}

std::string
DiskCache::getPath() const
{
	//Return early if the cache path is empty
    if (_path.empty() || _mapConfigFilename.empty() ) return _path;
    else return getFullPath(_mapConfigFilename, _path);
}

std::string
DiskCache::getFilename(const osgEarth::TileKey *key,
					   const std::string& layerName,
					   const std::string& format) const
{
	unsigned int level, x, y;
    level = key->getLevelOfDetail() +1;
    key->getTileXY( x, y );

    unsigned int numCols, numRows;
    key->getProfile()->getNumTiles(level, numCols, numRows);

    // need to invert the y-tile index
    y = numRows - y - 1;


    char buf[2048];
    sprintf( buf, "%s/%s/%02d/%03d/%03d/%03d/%03d/%03d/%03d.%s",
        getPath().c_str(),
        layerName.c_str(),
        level,
        (x / 1000000),
        (x / 1000) % 1000,
        (x % 1000),
        (y / 1000000),
        (y / 1000) % 1000,
        (y % 1000),
        format.c_str());
    return buf;
}

osg::Image*
DiskCache::getImage( const TileKey* key,
					 const std::string& layerName,
					 const std::string& format)
{
    OpenThreads::ScopedReadLock lock(s_mutex);
	std::string filename = getFilename(key,layerName,format);

    //If the path doesn't contain a zip file, check to see that it actually exists on disk
    if (!osgEarth::isZipPath(filename))
    {
        if (!osgDB::fileExists(filename)) return 0;
    }

    return osgDB::readImageFile( filename );
}

/**
* Sets the cached image for the given TileKey
*/
void 
DiskCache::setImage( const TileKey* key,
					 const std::string& layerName,
					 const std::string& format,
					 osg::Image* image )
{
	std::string filename = getFilename( key, layerName, format );
    std::string path = osgDB::getFilePath(filename);
	std::string extension = format;

	//If the format is empty, see if we can glean an extension from the image filename
	if (extension.empty())
	{
		if (!image->getFileName().empty())
		{
			extension = osgDB::getFileExtension( image->getFileName() );
		}
	}

	//If the format is STILL empty, just use PNG
	if (extension.empty())
	{
		extension = "png";
	}

    // serialize cache writes.
    OpenThreads::ScopedWriteLock lock(s_mutex);

    //If the path doesn't currently exist or we can't create the path, don't cache the file
    if (!osgDB::fileExists(path) && !osgEarth::isZipPath(path) && !osgDB::makeDirectory(path))
    {
        osg::notify(osg::WARN) << "[osgEarth::Cache] Couldn't create path " << path << std::endl;
    }

    std::string ext = osgDB::getFileExtension(filename);

    if (_writeWorldFiles || (getenv("OSGEARTH_WRITE_WORLD_FILES") != 0))
    {
        //Write out the world file along side the image
        double minx, miny, maxx, maxy;
        key->getGeoExtent().getBounds(minx, miny, maxx, maxy);

        std::string baseFilename = osgDB::getNameLessExtension(filename);

        /*
        Determine the correct extension for the file type.  Typically, world file extensions
        consist of the first letter of the extension, followed by the third, then the letter "w".
        For instance a jpg file's world file would be a jgw file.
        */
        std::string worldFileExt = "wld";
        if (ext.size() >= 3)
        {
            worldFileExt[0] = ext[0];
            worldFileExt[1] = ext[2];
            worldFileExt[2] = 'w';
        }
        std::string worldFileName = baseFilename + std::string(".") + worldFileExt;
        std::ofstream worldFile;
        worldFile.open(worldFileName.c_str());

        double x_units_per_pixel = (maxx - minx) / (double)image->s();
        double y_units_per_pixel = -(maxy - miny) / (double)image->t();
        worldFile << std::fixed << std::setprecision(10)
            //X direction units per pixel
            << x_units_per_pixel << std::endl
            //Rotation about the y axis, in our case 0
            << "0" << std::endl
            //Rotation about the x axis, in our case 0
            << "0" << std::endl
            //Y direction units per pixel, typically negative
            << y_units_per_pixel << std::endl
            //X coordinate of the center of the upper left pixel
            << minx + 0.5 * x_units_per_pixel << std::endl
            //Y coordinate of the upper left pixel
            << maxy + 0.5 * y_units_per_pixel;
        worldFile.close();
    }

    bool writingJpeg = (ext == "jpg" || ext == "jpeg");

	//If we are trying to write a non RGB image to JPEG, convert it to RGB before we write it
    if ((image->getPixelFormat() != GL_RGB) && writingJpeg)
    {
		//Take a reference so the converted image will be deleted
		osg::ref_ptr<osg::Image> rgb = ImageUtils::convertToRGB( image );
		if (rgb.valid())
		{
			osgDB::writeImageFile(*rgb.get(), filename);
		}
    }
    else
    {
        osgDB::writeImageFile(*image, filename);
    }
}

std::string
DiskCache::getTMSPath(const std::string &layerName)
{
    return getPath() + std::string("/") + layerName + std::string("/tms.xml");
}

void DiskCache::storeLayerProperties( const std::string& layerName,
		                          const Profile* profile,
			                      const std::string& format,
			                      unsigned int tile_size)
{
	osg::ref_ptr<TileMap> tileMap = TileMap::create("", profile, format, tile_size, tile_size);
	std::string path = getTMSPath( layerName );
    osg::notify(osg::INFO) << "[osgEarth::DiskCache] Writing TMS file to  " << path << std::endl;
	TileMapReaderWriter::write( tileMap.get(), path );
}

const Profile* DiskCache::loadLayerProperties( const std::string& layerName,
											  std::string& format,
											  unsigned int &tile_size )
{
	//Try to get it from the cache.
	LayerPropertiesCache::const_iterator i = _layerPropertiesCache.find(layerName);
	if ( i != _layerPropertiesCache.end())
	{
		format = i->second._format;
		tile_size = i->second._tile_size;
		return i->second._profile.get();
	}

	osg::ref_ptr<TileMap> tileMap;

	std::string path = getTMSPath( layerName );
	osg::notify(osg::NOTICE) << "[osgEarth::DiskCache] TileMap file is " << path << std::endl;

	if (osgDB::fileExists( path ) )
	{
		osg::ref_ptr<TileMap> tileMap = TileMapReaderWriter::read(path, NULL);
		if (tileMap.valid())
		{
			LayerProperties props;
			props._format = tileMap->getFormat().getExtension();
			props._tile_size = tileMap->getFormat().getWidth();
			props._profile = tileMap->createProfile();

			_layerPropertiesCache[layerName] = props;

			tile_size = props._tile_size;
			format = props._format;
			return props._profile.get();
		}
	}
	return NULL;
}

/*****************************************************************************/
MemCache::MemCache():
_maxNumTilesInCache(16)
{
}

unsigned int
MemCache::getMaxNumTilesInCache() const
{
	return _maxNumTilesInCache;
}

void
MemCache::setMaxNumTilesInCache(unsigned int max)
{
	_maxNumTilesInCache = max;
}

osg::Image*
MemCache::getImage(const osgEarth::TileKey *key,
				   const std::string& layerName,
				   const std::string& format)
{
	osg::Timer_t now = osg::Timer::instance()->tick();

    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);

    //osg::notify(osg::NOTICE) << "List contains: " << _images.size() << std::endl;

    std::string id = key->str();
    //Find the image in the cache
    //ImageCache::iterator itr = _images.find(id);
    KeyToIteratorMap::iterator itr = _keyToIterMap.find(id);
    if (itr != _keyToIterMap.end())
    {
        CachedImage entry = *itr->second;
        _images.erase(itr->second);
        _images.push_front(entry);
        itr->second = _images.begin();
        //osg::notify(osg::NOTICE)<<"Getting from memcache "<< key->str() <<std::endl;
        return const_cast<osg::Image*>(itr->second->_image.get());
    }
    return 0;
}

void
MemCache::setImage(const osgEarth::TileKey *key,
				   const std::string& layerName,
				   const std::string& format,
				   osg::Image *image)
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);

    std::string id = key->str();

    CachedImage entry;
    entry._image = const_cast<osg::Image*>(image);
    entry._key = id;
    _images.push_front(entry);

    _keyToIterMap[id] = _images.begin();

    if (_images.size() > _maxNumTilesInCache)
    {
        CachedImage toRemove = _images.back();
        _images.pop_back();
        _keyToIterMap.erase(toRemove._key);
    }
}

bool
MemCache::isCached(const osgEarth::TileKey *key,
				   const std::string& layerName,
				   const std::string& format)
{
	osg::ref_ptr<osg::Image> image = getImage(key,layerName,format);
	return image.valid();
}


/*****************************************************************************/

TMSCache::TMSCache(const std::string &path):
DiskCache(path),
_invertY(false)
{
}

std::string
TMSCache::getFilename( const TileKey* key,
					   const std::string& layerName,
					   const std::string& format) const
{
	unsigned int x,y;
    key->getTileXY(x, y);

    unsigned int lod = key->getLevelOfDetail();
    
    unsigned int numCols, numRows;
    key->getProfile()->getNumTiles(lod, numCols, numRows);
    if (!_invertY)
    {
        y = numRows - y - 1;
    }

    std::stringstream buf;
    buf << getPath() << "/" << layerName << "/" << lod << "/" << x << "/" << y << "." << format;
	std::string bufStr;
	bufStr = buf.str();
    return bufStr;
}

static bool getProp(const std::map<std::string,std::string> &map, const std::string &key, std::string &value)
{
    std::map<std::string,std::string>::const_iterator i = map.find(key);
    if (i != map.end())
    {
        value = i->second;
        return true;
    }
    return false;
}

/*****************************************************************************/

Cache* CacheFactory::create(const CacheConfig &cacheConfig)
{
	CacheConfig::CacheType type = cacheConfig.getType();
    //Default to TMS if the default is selected.
    if (type == CacheConfig::TYPE_DEFAULT) type = CacheConfig::TYPE_TMS;

    if (type == CacheConfig::TYPE_TMS || type == CacheConfig::TYPE_TILECACHE )
    {
        std::string path;

        if ((!getProp(cacheConfig.getProperties(), "path", path)) || (path.empty()))
        {
            osg::notify(osg::NOTICE) << "[osgEarth::Cache] No path specified for " << type << " cache " << std::endl;
            return 0;
        }      

        DiskCache* cache = NULL;
        if (type == CacheConfig::TYPE_TMS ) //"tms" || type.empty())
        {
			TMSCache* tms_cache = new TMSCache(path);
            std::string tms_type; 
            getProp(cacheConfig.getProperties(), "tms_type", tms_type);
            if (tms_type == "google")
            {
                osg::notify(osg::INFO) << "[osgEarth::Cache] Inverting Y in TMS cache " << std::endl;
                tms_cache->setInvertY(true);
            }
            osg::notify(osg::INFO) << "[osgEarth::Cache] Returning TMS cache " << std::endl;
            cache = tms_cache;
        }

        if (type == CacheConfig::TYPE_TILECACHE ) //"tilecache")
        {
            osg::notify(osg::INFO) << "[osgEarth::Cache] Returning disk cache " << std::endl;
            cache = new DiskCache(path);
        }

        if (cache)
        {
            std::string write_world_files;
            getProp(cacheConfig.getProperties(), "write_world_files", write_world_files);
            if (write_world_files == "true")
            {
                cache->setWriteWorldFiles( true );
            }
            else
            {
                cache->setWriteWorldFiles( false );
            }
            return cache;
        }
    }
    else
    {
        osg::notify(osg::NOTICE) << "[osgEarth::Cache] Unknown cache type " << type << std::endl;
    }
    return 0;
}