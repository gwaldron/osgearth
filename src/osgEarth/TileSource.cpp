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

#include <osgEarth/TileSource>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/FileUtils>
#include <osgEarth/Mercator>

#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <OpenThreads/ScopedLock>

using namespace osgEarth;

TileSource::TileSource():
_minLevel(0),
_maxLevel(25)
{
}

TileSource::~TileSource()
{
}

//const osgEarth::TileGridProfile&
//TileSource::getProfile() const
//{
//    return _profile;
//}

#define PROPERTY_MIN_LEVEL "min_level"
#define PROPERTY_MAX_LEVEL "max_level"

void
TileSource::init(const osgDB::ReaderWriter::Options* options)
{
    if ( options->getPluginData( PROPERTY_MIN_LEVEL ) )
        _minLevel = as<int>( (const char*)options->getPluginData( PROPERTY_MIN_LEVEL ), 0 );

    if ( options->getPluginData( PROPERTY_MAX_LEVEL ) )
        _maxLevel = as<int>( (const char*)options->getPluginData( PROPERTY_MAX_LEVEL ), INT_MAX );
}

osg::HeightField* TileSource::createHeightField( const TileKey* key )
{
    osg::ref_ptr<osg::Image> image = createImage(key);
    osg::HeightField* hf = 0;
    if (image.valid())
    {
        hf = ImageToHeightFieldConverter::convert(image.get());
    }      
    return hf;
}

/************************************************************************/

CachedTileSource::CachedTileSource(TileSource* tileSource):
_tileSource(tileSource)
{
    //Initialize the profile if the TileSource is valid
    if (_tileSource.valid())
    {
        _profile = _tileSource->getProfile();
    }
}

const osgEarth::TileGridProfile&
CachedTileSource::getProfile() const
{
    return _profile;
}

osg::Image* CachedTileSource::createImage( const TileKey* key )
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

osg::HeightField* CachedTileSource::createHeightField( const TileKey* key )
{
    osg::HeightField* hf = 0;

    //Try to get an image from the cache
    osg::ref_ptr<osg::Image> image = getCachedImage( key );
    if (image.valid())
    {
        osg::notify(osg::INFO) << "Read cached heightfield " << std::endl;
        hf = ImageToHeightFieldConverter::convert(image.get());
    }

    //Create the heightfield and write it to the cache
    if (!hf && _tileSource.valid())
    {
        hf = _tileSource->createHeightField( key );
        if (hf)
        {
            writeCachedImage( key, ImageToHeightFieldConverter::convert(hf) );
        }
    }
    return hf;
}

osg::Image* CachedTileSource::getCachedImage( const TileKey* key)
{
    //NOP
    return 0;
}

void CachedTileSource::writeCachedImage(const TileKey* key, const osg::Image* image)
{
    //NOP
}

void CachedTileSource::initTileMap()
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
DiskCachedTileSource::DiskCachedTileSource( TileSource* tileSource, const std::string &path, const std::string format):
CachedTileSource(tileSource)
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
    level = key->getLevelOfDetail();
    key->getTileXY( x, y );

    // need to invert the y-tile index
    y = key->getMapSizeTiles() - y - 1;

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

void DiskCachedTileSource::initTileMap()
{
    std::string tmsFile = getTMSPath();

    osg::notify(osg::INFO) << "TileMap file is " << tmsFile << std::endl;

    if (osgDB::fileExists( tmsFile ) )
    {
        //Load the tile map if it exists
        if (!_tileMap.valid())
        {
            _tileMap = TileMapReaderWriter::read(getTMSPath());
            if (_tileMap.valid())
            {
                osg::notify(osg::INFO) << "Loaded TMS file from " << getTMSPath() << std::endl;
                if (_profile.getProfileType() == TileGridProfile::UNKNOWN)
                {
                    _profile = TileGridProfile(_tileMap->getProfile());
                }
            }
        }
    }

    //Write the TMS file to disk if it doesn't exist
    if (!osgDB::fileExists(tmsFile))
    {
        if (_tileSource.valid())
        {
            osg::notify(osg::INFO) << "Writing TMS file to  " << tmsFile << std::endl;
            //_tileMap = TileMap::create(_tileSource.get());
            _tileMap = TileMap::create( this );
            TileMapReaderWriter::write(_tileMap.get(), tmsFile);
        }
    }
}

/****************************************************************************/

TMSCacheTileSource::TMSCacheTileSource(osgEarth::TileSource *tileSource, const std::string &path, const std::string format):
DiskCachedTileSource(tileSource, path, format),
_invertY(false)
{
}

std::string TMSCacheTileSource::getFileName(const osgEarth::TileKey *key)
{
    unsigned int x,y;
    key->getTileXY(x, y);

    unsigned int lod = key->getLevelOfDetail();
    int totalTiles = TileKey::getMapSizeTiles(lod);

    if ( key->isGeodetic() )
    {
        //In global-geodetic TMS, level 0 is two tiles that cover the entire earth.
        //Level 0 in osgEarth is a single tile that covers the entire earth and extends down to -270,
        //so osgEarth level 1 is more like TMS level 0.
        lod--;

        //Only half the vertical tiles are used in TMS global-geodetic
        totalTiles /= 2;
    }

    if ((!_invertY) && lod > 0)
    {
        y = totalTiles - y - 1;
    }

    std::stringstream buf;
    buf << getPath() << "/" << getName() << "/" << lod << "/" << x << "/" << y << "." << getExtension();
    return buf.str();

}

/****************************************************************************/
QuadKeyCachedTileSource::QuadKeyCachedTileSource(osgEarth::TileSource *tileSource, const std::string &path, const std::string format):
DiskCachedTileSource(tileSource, path, format)
{
}

std::string QuadKeyCachedTileSource::getFileName(const osgEarth::TileKey *key)
{
    std::stringstream buf;
    buf << getPath() << "/" << getName() << "/" << key->str() << "." << getExtension();
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

CachedTileSource* CachedTileSourceFactory::create(TileSource* tileSource, const std::string &type, std::map<std::string,std::string> properties)
{
    if (type == "tms" || type == "tilecache" || type == "quadkey" || type.empty())
    {
        std::string path;
        std::string format;

        if ((!getProp(properties, "path", path)) || (path.empty()))
        {
            osg::notify(osg::NOTICE) << "No path specified for " << type << " cache " << std::endl;
        }

        getProp(properties, "format", format);
        
        
        if (type == "tms" || type.empty())
        {
            TMSCacheTileSource *cache = new TMSCacheTileSource(tileSource, path, format);
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
            return new DiskCachedTileSource(tileSource, path, format);
        }

        
        if (type == "quadkey")
        {
            osg::notify(osg::INFO) << "Returning quadkey cache " << std::endl;
            return new QuadKeyCachedTileSource(tileSource, path, format);
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
