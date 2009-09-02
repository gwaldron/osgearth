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
_type( CacheConfig::TYPE_DEFAULT )
{
    //NOP
}

CacheConfig::CacheConfig( const CacheConfig& rhs ) :
_type( rhs._type ),
_properties( rhs._properties ),
_runOffCacheOnly( rhs._runOffCacheOnly ),
_reprojectBeforeCaching( rhs._reprojectBeforeCaching )
{
    //NOP
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

optional<bool>& 
CacheConfig::reprojectBeforeCaching() {
    return _reprojectBeforeCaching;
}
const optional<bool>&
CacheConfig::reprojectBeforeCaching() const {
    return _reprojectBeforeCaching;
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

void
CacheConfig::inheritFrom(const CacheConfig& parent)
{
    if ( _type != TYPE_DISABLED )
    {
        if ( parent.getType() == TYPE_DISABLED ) // disabling overrides everything.
            setType( TYPE_DISABLED );

        else if ( _type == TYPE_DEFAULT ) // if "DEFAULT", inherit from the parent.
            setType( parent.getType() );

        // Inherit any properites that are not defined locally
        for (Properties::const_iterator itr = parent.getProperties().begin(); itr != parent.getProperties().end(); ++itr)
        {
            if ( _properties.find( itr->first ) == _properties.end() )
                _properties[itr->first] = itr->second;
        }

        if ( !reprojectBeforeCaching().isSet() )
            reprojectBeforeCaching() = parent.reprojectBeforeCaching();

        if ( !runOffCacheOnly().isSet() )
            runOffCacheOnly() = parent.runOffCacheOnly();
    }
}

std::string
CacheConfig::toString() const
{
    if ( getType() == TYPE_DISABLED ) return "disabled";

    std::stringstream buf;
    buf << "type=" << getType();    
    buf << ", reproject=";
    if (reprojectBeforeCaching().isSet()) buf << reprojectBeforeCaching().get(); else buf << "unset";
    buf << ", caching=";
    if (runOffCacheOnly().isSet()) buf << runOffCacheOnly().get(); else buf << "unset";

    for (Properties::const_iterator i = _properties.begin(); i != _properties.end(); i++ )
        buf << ", " << i->first << "=" << i->second;   

    return buf.str();
}


/*****************************************************************************/


CachedTileSource::CachedTileSource(TileSource* tileSource, const osgDB::ReaderWriter::Options* options) :
ChainedTileSource( tileSource, options )
{
    //NOP
}


const Profile* 
CachedTileSource::createProfile( const Profile* mapProfile, const std::string& configPath )
{
    const Profile* ts_profile = nextTileSource()? nextTileSource()->initProfile( mapProfile, configPath ) : 0;
    const Profile* loaded_profile = loadTileMap();

    if ( ts_profile && loaded_profile && !ts_profile->isEquivalentTo( loaded_profile ) )
    {
        osg::notify(osg::WARN) 
            << "[osgEarth::Cache] The cache does not match the datasource for \"" 
            << this->getName() << "\"" << std::endl;
        return NULL;
    }

    // if we are "live", store the profile to the tilemap file.
    if ( ts_profile )
        storeTileMap( ts_profile );

    // return the profile from the cache if we have one.
    return loaded_profile? loaded_profile : ts_profile;
}


osg::Image*
CachedTileSource::createImageImplementation( const TileKey* key )
{
    osg::Image* image = 0L;

    //Try to get the image from the cache.
    image = getCachedImage( key);
    if (image)
    {
        osg::notify(osg::INFO) << "[osgEarth::Cache] Read cached image " << std::endl;
        return image;
    }

    //Get the image from the TileSource if it exists
    if ( nextTileSource() )
    {
        image = nextTileSource()->createImage( key );
        if (image)
        {
            writeCachedImage(key, image);
        }
    }

    return image;
}

osg::HeightField*
CachedTileSource::createHeightField( const TileKey* key )
{
    osg::HeightField* hf = 0;

    //Try to get an image from the cache
    osg::ref_ptr<osg::Image> image = getCachedImage( key );
    if (image.valid())
    {
        osg::notify(osg::INFO) << "[osgEarth::Cache] Read cached heightfield " << std::endl;
        ImageToHeightFieldConverter conv;
        hf = conv.convert(image.get());
    }

    //Create the heightfield and write it to the cache
    if ( !hf && nextTileSource() )
    {
        hf = nextTileSource()->createHeightField( key );
        if (hf)
        {
            ImageToHeightFieldConverter conv;
            //Take a reference to the returned image so it gets deleted properly
            image = conv.convert(hf);
            writeCachedImage( key, image.get() );
        }
    }
    return hf;
}

osg::Image*
CachedTileSource::getCachedImage( const TileKey* key)
{
    //NOP
    return 0;
}

void CachedTileSource::writeCachedImage(const TileKey* key, const osg::Image* image)
{
    //NOP
}

const Profile*
CachedTileSource::loadTileMap()
{
    return NULL;
}

void CachedTileSource::storeTileMap( const Profile* profile )
{
    //NOP
}


int CachedTileSource::getPixelsPerTile() const
{
    //Try the tile source first
    if ( nextTileSource() ) return nextTileSource()->getPixelsPerTile();
    
    //Assumes the width and height are the same
    if (_tileMap.valid()) return _tileMap->getFormat().getWidth();

    //Return the default implementation.
    return TileSource::getPixelsPerTile();
}

/************************************************************************/
DiskCachedTileSource::DiskCachedTileSource(TileSource* tileSource,
                                           const std::string &path,
                                           const std::string format,
                                           const osgDB::ReaderWriter::Options* options) :
CachedTileSource(tileSource, options),
_writeWorldFiles(false)
{
    _path = path;
    _format = format;
}

bool
DiskCachedTileSource::hasPersistentCache() const
{
    return true;
}

bool DiskCachedTileSource::isCached( const TileKey* key)
{
    std::string filename = getFileName(key);
    return osgDB::fileExists(filename);
}

osg::Image* DiskCachedTileSource::getCachedImage( const TileKey* key)
{
     std::string filename = getFileName(key);

    //If the path doesn't contain a zip file, check to see that it actually exists on disk
    if (!osgEarth::isZipPath(filename))
    {
        if (!osgDB::fileExists(filename)) return 0;
    }
    return osgDB::readImageFile( filename );
}

void DiskCachedTileSource::writeCachedImage(const TileKey* key, const osg::Image* image)
{
    std::string filename = getFileName( key );
    std::string path = osgDB::getFilePath(filename);

    // serialize cache writes.
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock( _fs_mutex );

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
        std::string worldFileName = baseFilename + "." + worldFileExt;
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
DiskCachedTileSource::getExtension() const
{
    //If the format is empty, use the format of the TileSource.  If the TileSource doesn't report a format, default to png 
    std::string format = _format;
    if (format.empty() && nextTileSource() ) format = nextTileSource()->getExtension();
    if (format.empty() && _tileMap.valid()) format = _tileMap->getFormat().getExtension();
    if (format.empty()) format = "png";
    return format;
}

std::string DiskCachedTileSource::getPath()
{
    //Return early if the cache path is empty
    if (_path.empty() || _mapConfigFilename.empty() ) return _path;
    else return getFullPath(_mapConfigFilename, _path);
}

std::string DiskCachedTileSource::getFileName(const TileKey* key )
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
        getName().c_str(),
        level,
        (x / 1000000),
        (x / 1000) % 1000,
        (x % 1000),
        (y / 1000000),
        (y / 1000) % 1000,
        (y % 1000),
        getExtension().c_str());
    return buf;
}

std::string
DiskCachedTileSource::getTMSPath()
{
    return getPath() + "/" + getName() + "/tms.xml";
}


const Profile*
DiskCachedTileSource::loadTileMap()
{
    const Profile* existing_cache_profile = NULL;

    std::string tmsFile = getTMSPath();

    osg::notify(osg::INFO) << "[osgEarth::DiskCache] TileMap file is " << tmsFile << std::endl;

    if (osgDB::fileExists( tmsFile ) )
    {
        //Load the tile map if it exists
        if (!_tileMap.valid())
        {
            _tileMap = TileMapReaderWriter::read(getTMSPath());
            if (_tileMap.valid())
            {
                osg::notify(osg::INFO) << "[osgEarth::DiskCache] Loaded TMS file from " << getTMSPath() << std::endl;
                existing_cache_profile = _tileMap->createProfile();
            }
        }
    }
    return existing_cache_profile;
}

void
DiskCachedTileSource::storeTileMap( const Profile* profile )
{
    //Write the TMS file to disk if it doesn't exist
    if ( !osgDB::fileExists( getTMSPath() ) && nextTileSource() )
    {
        osg::notify(osg::INFO) << "[osgEarth::DiskCache] Writing TMS file to  " << getTMSPath() << std::endl;
            //_tileMap = TileMap::create(_tileSource.get());
        _tileMap = TileMap::create( this, profile );
        TileMapReaderWriter::write(_tileMap.get(), getTMSPath() );
    }
}


/****************************************************************************/

TMSCacheTileSource::TMSCacheTileSource(osgEarth::TileSource *tileSource, 
                                       const std::string &path, 
                                       const std::string format,
                                       const osgDB::ReaderWriter::Options* options) :
DiskCachedTileSource(tileSource, path, format, options),
_invertY(false)
{
}

std::string TMSCacheTileSource::getFileName(const osgEarth::TileKey *key)
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
    buf << getPath() << "/" << getName() << "/" << lod << "/" << x << "/" << y << "." << getExtension();

    //osg::notify(osg::NOTICE) << "URL for key " << key->str() << " = " << buf.str() << std::endl;
    return buf.str();

}

/****************************************************************************/

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

CachedTileSource* CachedTileSourceFactory::create(TileSource* tileSource,
                                                  const CacheConfig::CacheType& type,
                                                  std::map<std::string,std::string> properties,
                                                  const osgDB::ReaderWriter::Options* options)
{
    if (type == CacheConfig::TYPE_TMS || type == CacheConfig::TYPE_TILECACHE ) //"tilecache" || type.empty())
    {
        std::string path;
        std::string format;

        if ((!getProp(properties, "path", path)) || (path.empty()))
        {
            osg::notify(osg::NOTICE) << "[osgEarth::Cache] No path specified for " << type << " cache " << std::endl;
            return 0;
        }

        getProp(properties, "format", format);
        
        
        DiskCachedTileSource* cache = NULL;
        if (type == CacheConfig::TYPE_TMS ) //"tms" || type.empty())
        {
            TMSCacheTileSource *tms_cache = new TMSCacheTileSource(tileSource, path, format, options);
            std::string tms_type; 
            getProp(properties, "tms_type", tms_type);
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
            cache = new DiskCachedTileSource(tileSource, path, format, options);
        }

        if (cache)
        {
            std::string write_world_files;
            getProp(properties, "write_world_files", write_world_files);
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
    else if (type == CacheConfig::TYPE_DISABLED)
    {
        return 0;
    }
    else
    {
        osg::notify(osg::NOTICE) << "[osgEarth::Cache] Unknown cache type " << type << std::endl;
    }
    return 0;
}

/****************************************************************************/

MemCachedTileSource::MemCachedTileSource(osgEarth::TileSource *tileSource, const osgDB::ReaderWriter::Options* options):
CachedTileSource(tileSource,options),
_maxNumTilesInCache(16)
{
}

MemCachedTileSource::~MemCachedTileSource()
{
}

bool
MemCachedTileSource::isCached(const osgEarth::TileKey* key)
{
    //Check our own cache first
    osg::ref_ptr<osg::Image> cachedImage = getCachedImage(key);
    if (cachedImage.valid()) return true;

    //Check the underlying TileSource to see if it is cached
    bool cached = false;
    if ( nextTileSource() )
    {
        cached = nextTileSource()->isCached(key);
    }
    return cached;
}

const Profile*
MemCachedTileSource::createProfile( const Profile* mapProfile, const std::string& configPath )
{
    if ( nextTileSource() ) return nextTileSource()->createProfile( mapProfile, configPath );
    return 0;
}

osg::Image*
MemCachedTileSource::getCachedImage( const TileKey* key )
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
MemCachedTileSource::writeCachedImage(const TileKey* key, const osg::Image* image)
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


unsigned int
MemCachedTileSource::getMaxNumTilesInCache() const
{
    return _maxNumTilesInCache;
}

void
MemCachedTileSource::setMaxNumTilesInCache(unsigned int max)
{
    _maxNumTilesInCache = max;
}