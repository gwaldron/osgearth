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
#include <limits.h>
#include <iomanip>

#include <osgEarth/Caching>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/FileUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/ThreadingUtils>

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <OpenThreads/ScopedLock>

using namespace osgEarth;

#define LC "[Cache] "

//------------------------------------------------------------------------

Cache::Cache( const CacheOptions& options ) :
osg::Object( true )
{
    //NOP
}

Cache::Cache(const Cache& rhs, const osg::CopyOp& op) :
osg::Object(rhs),
_refURI( rhs._refURI )
{
    //NOP
}

osg::HeightField*
Cache::getHeightField( const TileKey& key, const CacheSpec& spec )
{
	osg::HeightField* hf = 0;

	//Try to get an image from the cache
	osg::ref_ptr<osg::Image> image = getImage( key, spec );
	if (image.valid())
	{
		OE_DEBUG << LC << "Read cached heightfield " << std::endl;
		ImageToHeightFieldConverter conv;
		hf = conv.convert(image.get());
	}
	return hf;
}

void
Cache::setHeightField( const TileKey& key, const CacheSpec& spec, osg::HeightField* hf)
{
	ImageToHeightFieldConverter conv;
	//Take a reference to the returned image so it gets deleted properly
    osg::ref_ptr<osg::Image> image = conv.convert(hf);
	setImage( key, spec, image.get() );
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[DiskCache] "

static Threading::ReadWriteMutex s_mutex;

DiskCache::DiskCache( const DiskCacheOptions& options ) :
_options( options )
{
    _writeWorldFilesOverride = getenv("OSGEARTH_WRITE_WORLD_FILES") != 0L;
}

DiskCache::DiskCache( const DiskCache& rhs, const osg::CopyOp& op ) :
Cache( rhs, op ),
_options( rhs._options ),
_layerPropertiesCache( rhs._layerPropertiesCache ),
_writeWorldFilesOverride( rhs._writeWorldFilesOverride )
{
    //NOP
}

bool
DiskCache::isCached(const osgEarth::TileKey& key, const CacheSpec& spec ) const
{
	//Check to see if the file for this key exists
	std::string filename = getFilename( key, spec );
    return osgDB::fileExists(filename);
}

std::string
DiskCache::getPath() const
{
    if ( _options.path().empty() || _refURI.empty() )
        return _options.path();
    else
        return osgEarth::getFullPath( _refURI, _options.path() );
}

std::string
DiskCache::getFilename(const osgEarth::TileKey& key, const CacheSpec& spec ) const
{
	unsigned int level, x, y;
    level = key.getLevelOfDetail() +1;
    key.getTileXY( x, y );

    unsigned int numCols, numRows;
    key.getProfile()->getNumTiles(level, numCols, numRows);

    // need to invert the y-tile index
    y = numRows - y - 1;

    char buf[2048];
    sprintf( buf, "%s/%s/%02d/%03d/%03d/%03d/%03d/%03d/%03d.%s",
        getPath().c_str(),
        spec.cacheId().c_str(),
        level,
        (x / 1000000),
        (x / 1000) % 1000,
        (x % 1000),
        (y / 1000000),
        (y / 1000) % 1000,
        (y % 1000),
        spec.format().c_str());

    return buf;
}

osg::Image*
DiskCache::getImage( const TileKey& key, const CacheSpec& spec )
{
	std::string filename = getFilename(key, spec);

    //If the path doesn't contain a zip file, check to see that it actually exists on disk
    if (!osgEarth::isZipPath(filename))
    {
        if (!osgDB::fileExists(filename)) return 0;
    }

    {
        Threading::ScopedReadLock lock(s_mutex);
        return osgDB::readImageFile( filename );
    }
}

/**
* Sets the cached image for the given TileKey
*/
void 
DiskCache::setImage( const TileKey& key, const CacheSpec& spec, osg::Image* image)
{
	std::string filename = getFilename( key, spec );
    std::string path = osgDB::getFilePath(filename);
	std::string extension = spec.format();

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
    Threading::ScopedWriteLock lock(s_mutex);

    //If the path doesn't currently exist or we can't create the path, don't cache the file
    if (!osgDB::fileExists(path) && !osgEarth::isZipPath(path) && !osgDB::makeDirectory(path))
    {
        OE_WARN << LC << "Couldn't create path " << path << std::endl;
    }

    std::string ext = osgDB::getFileExtension(filename);

    if ( _options.writeWorldFiles() == true || _writeWorldFilesOverride )
    {
        //Write out the world file along side the image
        double minx, miny, maxx, maxy;
        key.getExtent().getBounds(minx, miny, maxx, maxy);

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
		osg::ref_ptr<osg::Image> rgb = ImageUtils::convertToRGB8( image );
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
DiskCache::getTMSPath(const std::string& cacheId) const
{
    return getPath() + std::string("/") + cacheId + std::string("/tms.xml");
}

void DiskCache::storeProperties(const CacheSpec& spec,
                                const Profile* profile,
                                unsigned int tile_size)
{
	osg::ref_ptr<TileMap> tileMap = TileMap::create("", profile, spec.format(), tile_size, tile_size);
    tileMap->setTitle( spec.name() );
	std::string path = getTMSPath( spec.cacheId() );
    OE_DEBUG << LC << "Writing TMS file to  " << path << std::endl;
	TileMapReaderWriter::write( tileMap.get(), path );
}

bool
DiskCache::loadProperties(const std::string&           cacheId,
                          CacheSpec&                   out_spec,
                          osg::ref_ptr<const Profile>& out_profile,
                          unsigned int&                out_tileSize)
{
	//Try to get it from the cache.
	LayerPropertiesCache::const_iterator i = _layerPropertiesCache.find(cacheId);
	if ( i != _layerPropertiesCache.end())
	{
        out_spec = CacheSpec(cacheId, i->second._format);
		out_tileSize = i->second._tile_size;
        out_profile = i->second._profile.get();
        return true;
	}

	osg::ref_ptr<TileMap> tileMap;

	std::string path = getTMSPath( cacheId );
	OE_INFO << LC << "Metadata file is " << path << std::endl;

	if (osgDB::fileExists( path ) )
	{
		osg::ref_ptr<TileMap> tileMap = TileMapReaderWriter::read(path, NULL);
		if (tileMap.valid())
		{
			LayerProperties props;
			props._format = tileMap->getFormat().getExtension();
			props._tile_size = tileMap->getFormat().getWidth();
			props._profile = tileMap->createProfile();

			_layerPropertiesCache[cacheId] = props;

            out_spec = CacheSpec( cacheId, props._format );
            out_profile = props._profile.get();
            out_tileSize = props._tile_size;
            return true;
		}
	}
	return false;
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[MemCache] "

MemCache::MemCache():
_maxNumTilesInCache(16)
{
}

MemCache::MemCache( const MemCache& rhs, const osg::CopyOp& op ) :
_maxNumTilesInCache( rhs._maxNumTilesInCache )
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
MemCache::getImage(const osgEarth::TileKey& key, const CacheSpec& spec )
{
  return dynamic_cast<osg::Image*>( getObject( key, spec ) );
}

void
MemCache::setImage(const osgEarth::TileKey& key, const CacheSpec& spec, osg::Image* image)
{
  setObject( key, spec, image );
}

osg::HeightField* MemCache::getHeightField( const TileKey& key,const CacheSpec& spec )
{
  return dynamic_cast<osg::HeightField*>( getObject( key, spec ) );
}

void MemCache::setHeightField( const TileKey& key, const CacheSpec& spec, osg::HeightField* hf)
{
    setObject( key, spec, hf );
}

bool
MemCache::purge( const std::string& cacheId, int olderThan, bool async )
{
    //TODO: can this be a ReadWriteLock instead?


    // MemCache does not support timestamps or async, so just clear it out altogether.
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);

    // MemCache does not support cacheId...
    _keyToIterMap.clear();
    _objects.clear();

    return true;
}

osg::Referenced* MemCache::getObject( const TileKey& key, const CacheSpec& spec )
{
  osg::Timer_t now = osg::Timer::instance()->tick();

  OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);

  //OE_NOTICE << "List contains: " << _images.size() << std::endl;

  std::string id = key.str() + spec.cacheId();
  //Find the image in the cache
  //ImageCache::iterator itr = _images.find(id);
  KeyToIteratorMap::iterator itr = _keyToIterMap.find(id);
  if (itr != _keyToIterMap.end())
  {
    CachedObject entry = *itr->second;
    _objects.erase(itr->second);
    _objects.push_front(entry);
    itr->second = _objects.begin();
    //OE_NOTICE<<"Getting from memcache "<< key.str() <<std::endl;
    return itr->second->_object.get();
  }
  return 0;
}

void MemCache::setObject( const TileKey& key, const CacheSpec& spec, osg::Referenced* referenced )
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);

  std::string id = key.str() + spec.cacheId();

  CachedObject entry;
  entry._object = referenced;
  entry._key = id;
  _objects.push_front(entry);

  _keyToIterMap[id] = _objects.begin();

  if (_objects.size() > _maxNumTilesInCache)
  {
    CachedObject toRemove = _objects.back();
    _objects.pop_back();
    _keyToIterMap.erase(toRemove._key);
  }
}

bool
MemCache::isCached(const osgEarth::TileKey& key, const CacheSpec& spec) const
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock( const_cast<MemCache*>(this)->_mutex);
    std::string id = key.str() + spec.cacheId();
    return _keyToIterMap.find(id) != _keyToIterMap.end();
	//osg::ref_ptr<osg::Image> image = getImage(key,spec);
	//return image.valid();
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[TMSCache] "

TMSCache::TMSCache( const TMSCacheOptions& options ) :
DiskCache( options ),
_options( options )
{
    //nop
}

TMSCache::TMSCache( const TMSCache& rhs, const osg::CopyOp& op ) :
DiskCache( rhs, op ),
_options( rhs._options )
{
    //nop
}

std::string
TMSCache::getFilename( const TileKey& key,const CacheSpec& spec ) const
{
	unsigned int x,y;
    key.getTileXY(x, y);

    unsigned int lod = key.getLevelOfDetail();
    
    unsigned int numCols, numRows;
    key.getProfile()->getNumTiles(lod, numCols, numRows);
    if ( _options.invertY() == false )
    {
        y = numRows - y - 1;
    }

    std::stringstream buf;
    buf << getPath() << "/" << spec.cacheId() << "/" << lod << "/" << x << "/" << y << "." << spec.format();
	std::string bufStr;
	bufStr = buf.str();
    return bufStr;
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[CacheFactory] "
#define CACHE_OPTIONS_TAG "__osgEarth::CacheOptions"

Cache*
CacheFactory::create( const CacheOptions& options )
{
    osg::ref_ptr<Cache> result =0L;
    OE_INFO << LC << "Initializing cache of type \"" << options.getDriver() << "\"" << std::endl;

    if ( options.getDriver().empty() )
    {
        OE_WARN << LC << "ILLEGAL: no driver set in cache options" << std::endl;
    }
    else if ( options.getDriver() == "tms" )
    {
        result = new TMSCache( options );
    }
    else if ( options.getDriver() == "tilecache" )
    {
        result = new DiskCache( options );
    }
    else // try to load from a plugin
    {
        osg::ref_ptr<osgDB::ReaderWriter::Options> rwopt = new osgDB::ReaderWriter::Options();
        rwopt->setPluginData( CACHE_OPTIONS_TAG, (void*)&options );

        std::string driverExt = ".osgearth_cache_" + options.getDriver();
        osgDB::ReaderWriter::ReadResult rr = osgDB::readObjectFile( driverExt, rwopt.get() );
        result = dynamic_cast<Cache*>( rr.getObject() );
        if ( !result )
        {
            OE_WARN << LC << "Failed to load cache plugin for type \"" << options.getDriver() << "\"" << std::endl;
        }
    }
    return result.release();
}

//------------------------------------------------------------------------

const CacheOptions&
CacheDriver::getCacheOptions( const osgDB::ReaderWriter::Options* rwopt ) const 
{
    return *static_cast<const CacheOptions*>( rwopt->getPluginData( CACHE_OPTIONS_TAG ) );
}

