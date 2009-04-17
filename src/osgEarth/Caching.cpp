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
#include <osgEarth/Mercator>

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <OpenThreads/ScopedLock>

using namespace osgEarth;



CachedTileSource::CachedTileSource(TileSource* tileSource, const osgDB::ReaderWriter::Options* options) :
TileSource(options),
_tileSource(tileSource)
{
    //NOP
}


const Profile* 
CachedTileSource::createProfile( const Profile* mapProfile, const std::string& configPath )
{
    const Profile* ts_profile = _tileSource.valid()? _tileSource->initProfile( mapProfile, configPath ) : 0;
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
CachedTileSource::createImage( const TileKey* key )
{
    //Try to get the image from the cache.
    osg::Image *image = getCachedImage( key);
    if (image)
    {
        osg::notify(osg::INFO) << "Read cached image " << std::endl;
        return image;
    }

    //Get the image from the TileSource if it exists
    if (_tileSource.valid())
    {
        image = _tileSource->createImage( key );
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
        osg::notify(osg::INFO) << "Read cached heightfield " << std::endl;
        ImageToHeightFieldConverter conv;
        hf = conv.convert(image.get());
    }

    //Create the heightfield and write it to the cache
    if (!hf && _tileSource.valid())
    {
        hf = _tileSource->createHeightField( key );
        if (hf)
        {
            ImageToHeightFieldConverter conv;
            writeCachedImage( key, conv.convert(hf) );
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
    if (_tileSource.valid()) return _tileSource->getPixelsPerTile();
    
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
CachedTileSource(tileSource, options)
{
    _path = path;
    _format = format;
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
        osg::notify(osg::WARN) << "Couldn't create path " << path << std::endl;
    }

    //Write out the world file along side the image
    double minx, miny, maxx, maxy;
    key->getGeoExtent().getBounds(minx, miny, maxx, maxy);

    std::string baseFilename = osgDB::getNameLessExtension(filename);
    std::string ext = osgDB::getFileExtension(filename);

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

    osgDB::writeImageFile(*image, filename);
}

std::string DiskCachedTileSource::getExtension() const
{
    //If the format is empty, use the format of the TileSource.  If the TileSource doesn't report a format, default to png 
    std::string format = _format;
    if (format.empty() && _tileSource.valid()) format = _tileSource->getExtension();
    if (format.empty() && _tileMap.valid()) format = _tileMap->getFormat().getExtension();
    if (format.empty()) format = "png";
    return format;
}

std::string DiskCachedTileSource::getPath()
{
    //Return early if the cache path is empty
    if (_path.empty()) return _path;
    return getFullPath(_mapConfigFilename, _path);
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

std::string DiskCachedTileSource::getTMSPath()
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
    if ( !osgDB::fileExists( getTMSPath() ) && _tileSource.valid() )
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
                                                  const std::string &type,
                                                  std::map<std::string,std::string> properties,
                                                  const osgDB::ReaderWriter::Options* options)
{
    if (type == "tms" || type == "tilecache" || type.empty())
    {
        std::string path;
        std::string format;

        if ((!getProp(properties, "path", path)) || (path.empty()))
        {
            osg::notify(osg::NOTICE) << "No path specified for " << type << " cache " << std::endl;
            return 0;
        }

        getProp(properties, "format", format);
        
        
        if (type == "tms" || type.empty())
        {
            TMSCacheTileSource *cache = new TMSCacheTileSource(tileSource, path, format, options);
            std::string tms_type; 
            getProp(properties, "tms_type", tms_type);
            if (tms_type == "google")
            {
                osg::notify(osg::INFO) << "Inverting Y in TMS cache " << std::endl;
                cache->setInvertY(true);
            }
            osg::notify(osg::INFO) << "Returning TMS cache " << std::endl;
            return cache;
        }

        if (type == "tilecache")
        {
            osg::notify(osg::INFO) << "Returning disk cache " << std::endl;
            return new DiskCachedTileSource(tileSource, path, format, options);
        }
    }
    else if (type == "none")
    {
        return 0;
    }
    else
    {
        osg::notify(osg::NOTICE) << "Unknown cache type " << type << std::endl;
    }
    return 0;
}

/****************************************************************************/

MemCachedTileSource::MemCachedTileSource(osgEarth::TileSource *tileSource):
CachedTileSource(tileSource),
_maxNumTilesInCache(50)
{
}

MemCachedTileSource::~MemCachedTileSource()
{
}

const Profile* MemCachedTileSource::createProfile( const Profile* mapProfile, const std::string& configPath )
{
    if (_tileSource.valid()) return _tileSource->createProfile(mapProfile, configPath);
    return 0;
}

osg::Image* MemCachedTileSource::getCachedImage( const TileKey* key)
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

void MemCachedTileSource::writeCachedImage(const TileKey* key, const osg::Image* image)
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


unsigned int MemCachedTileSource::getMaxNumTilesInCache() const
{
    return _maxNumTilesInCache;
}

void MemCachedTileSource::setMaxNumTilesInCache(unsigned int max)
{
    _maxNumTilesInCache = max;
}