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

#include <osgEarth/TileSource>
#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/ImageUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/ThreadingUtils>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

using namespace osgEarth;

//------------------------------------------------------------------------

TileBlacklist::TileBlacklist()
{
    //NOP
}

void
TileBlacklist::add(const osgTerrain::TileID &tile)
{
    Threading::ScopedWriteLock lock(_mutex);
    _tiles.insert(tile);
    OE_DEBUG << "Added " << tile.level << " (" << tile.x << ", " << tile.y << ") to blacklist" << std::endl;
}

void
TileBlacklist::remove(const osgTerrain::TileID &tile)
{
    Threading::ScopedWriteLock lock(_mutex);
    _tiles.erase(tile);
    OE_DEBUG << "Removed " << tile.level << " (" << tile.x << ", " << tile.y << ") from blacklist" << std::endl;
}

void
TileBlacklist::clear()
{
    Threading::ScopedWriteLock lock(_mutex);
    _tiles.clear();
    OE_DEBUG << "Cleared blacklist" << std::endl;
}

bool
TileBlacklist::contains(const osgTerrain::TileID &tile) const
{
    Threading::ScopedReadLock lock(const_cast<TileBlacklist*>(this)->_mutex);
    return _tiles.find(tile) != _tiles.end();
}

unsigned int
TileBlacklist::size() const
{
    Threading::ScopedReadLock lock(const_cast<TileBlacklist*>(this)->_mutex);
    return _tiles.size();
}

TileBlacklist*
TileBlacklist::read(std::istream &in)
{
    osg::ref_ptr< TileBlacklist > result = new TileBlacklist();


    while (!in.eof())
    {
        std::string line;
        std::getline(in, line);
        if (!line.empty())
        {
            int z, x, y;
            if (sscanf(line.c_str(), "%d %d %d", &z, &x, &y) == 3)
            {
                result->add(osgTerrain::TileID(z, x, y ));
            }

        }
    }

    return result.release();
}

TileBlacklist*
TileBlacklist::read(const std::string &filename)
{
    if (osgDB::fileExists(filename) && (osgDB::fileType(filename) == osgDB::REGULAR_FILE))
    {
        std::ifstream in( filename.c_str() );
        return read( in );
    }
    return NULL;
}

void
TileBlacklist::write(const std::string &filename) const
{ 
    std::string path = osgDB::getFilePath(filename);
    if (!path.empty() && !osgDB::fileExists(path) && !osgDB::makeDirectory(path))
    {
        OE_NOTICE << "Couldn't create path " << path << std::endl;
        return;
    }
    std::ofstream out(filename.c_str());
    write(out);
}

void
TileBlacklist::write(std::ostream &output) const
{
    Threading::ScopedReadLock lock(const_cast<TileBlacklist*>(this)->_mutex);
    for (BlacklistedTiles::const_iterator itr = _tiles.begin(); itr != _tiles.end(); ++itr)
    {
        output << itr->level << " " << itr->x << " " << itr->y << std::endl;
    }
}

//------------------------------------------------------------------------

TileSource::TileSource( const TileSourceOptions& options ) :
_options( options )
{
    this->setThreadSafeRefUnref( true );

    _memCache = new MemCache();

    if (_options.blacklistFilename().isSet())
    {
        _blacklistFilename = _options.blacklistFilename().value();
    }

    
    if (!_blacklistFilename.empty() && osgDB::fileExists(_blacklistFilename))
    {
        _blacklist = TileBlacklist::read(_blacklistFilename);
        if (_blacklist.valid())
        {
            OE_INFO << "Read blacklist from file" << _blacklistFilename << std::endl;
        }
    }

    if (!_blacklist.valid())
    {
        //Initialize the blacklist if we couldn't read it.
        _blacklist = new TileBlacklist();
    }
}

TileSource::~TileSource()
{
    if (_blacklist.valid() && !_blacklistFilename.empty())
    {
        _blacklist->write(_blacklistFilename);
    }
}

int
TileSource::getPixelsPerTile() const
{
    return _options.tileSize().value();
}

bool
TileSource::getImage( const TileKey& key, osg::ref_ptr<osg::Image>& out_image, ProgressCallback* progress )
{
	// Try to get it from the memcache fist
    if (_memCache.valid())
	{
		if ( _memCache->getImage( key, CacheSpec(), out_image ) )
            return true;
	}

    out_image = createImage( key, progress );

    if ( out_image.valid() && _memCache.valid() )
    {
        // cache it to the memory cache.
        _memCache->setImage( key, CacheSpec(), out_image.get() );
    }

	return out_image.valid();
}

bool
TileSource::getHeightField( const TileKey& key, osg::ref_ptr<osg::HeightField>& out_hf, ProgressCallback* progress )
{
    // Try to get it from the memcache first:
	if (_memCache.valid())
	{
		if ( _memCache->getHeightField( key, CacheSpec(), out_hf ) )
            return true;
	}

    out_hf = createHeightField( key, progress );

    if ( out_hf.valid() && _memCache.valid() )
    {
        _memCache->setHeightField( key, CacheSpec(), out_hf.get() );
    }

    return out_hf.valid();
}

osg::HeightField*
TileSource::createHeightField( const TileKey& key,
                               ProgressCallback* progress)
{
    osg::ref_ptr<osg::Image> image = createImage(key, progress);
    osg::HeightField* hf = 0;
    if (image.valid())
    {
        ImageToHeightFieldConverter conv;
        hf = conv.convert( image.get() );
    }      
    return hf;
}

bool
TileSource::isOK() const 
{
    return getProfile() != NULL;
}

void
TileSource::setProfile( const Profile* profile )
{
    _profile = profile;
}

const Profile*
TileSource::getProfile() const
{
    return _profile.get();
}

unsigned int
TileSource::getMaxDataLevel() const
{
    //If we have no data extents, just use a reasonably high number
    if (_dataExtents.size() == 0) return 35;

    unsigned int maxDataLevel = 0;
    for (DataExtentList::const_iterator itr = _dataExtents.begin(); itr != _dataExtents.end(); ++itr)
    {
        if (itr->getMaxLevel() > maxDataLevel) maxDataLevel = itr->getMaxLevel();
    }
    return maxDataLevel;
}

unsigned int
TileSource::getMinDataLevel() const
{
    //If we have no data extents, just use 0
    if (_dataExtents.size() == 0) return 0;

    unsigned int minDataLevel = INT_MAX;
    for (DataExtentList::const_iterator itr = _dataExtents.begin(); itr != _dataExtents.end(); ++itr)
    {
        if (itr->getMinLevel() < minDataLevel) minDataLevel = itr->getMinLevel();
    }
    return minDataLevel;
}

bool
TileSource::hasData(const osgEarth::TileKey& key)
{
    //If no data extents are provided, just return true
    if (_dataExtents.size() == 0) return true;

    const osgEarth::GeoExtent& keyExtent = key.getExtent();
    bool intersectsData = false;
    
    //osg::Timer_t loopStart = osg::Timer::instance()->tick();

    for (DataExtentList::const_iterator itr = _dataExtents.begin(); itr != _dataExtents.end(); ++itr)
    {
        if (keyExtent.intersects( *itr ) && key.getLevelOfDetail() >= itr->getMinLevel() && key.getLevelOfDetail() <= itr->getMaxLevel())
        {
            intersectsData = true;
            break;
        }
    }
    //osg::Timer_t loopEnd = osg::Timer::instance()->tick();

    /*
    osg::Timer_t rtreeStart = osg::Timer::instance()->tick();

    std::list<unsigned int> validExtents = _dataExtentsIndex->find(keyExtent);
    //OE_NOTICE << "Found " << validExtents.size() << " intersecting areas " << std::endl;
    for (std::list<unsigned int>::const_iterator itr = validExtents.begin(); itr != validExtents.end(); ++itr)
    {
        unsigned int index = *itr;
        //OE_NOTICE << "index " << index << std::endl;
        const DataExtent& e = _dataExtents[index];
        if (keyExtent.intersects( e ) && key.getLevelOfDetail() >= e.getMinLevel() && key.getLevelOfDetail() <= e.getMaxLevel())
        {
            intersectsData = true;
            break;
        }
    }
    osg::Timer_t rtreeEnd = osg::Timer::instance()->tick();

    double loopTime = osg::Timer::instance()->delta_m(loopStart, loopEnd);
    double rtreeTime = osg::Timer::instance()->delta_m(rtreeStart, rtreeEnd);

    OE_NOTICE << "Loop = " << loopTime << "ms   rtree=" << rtreeTime << "ms  diff=" << rtreeTime - loopTime << std::endl;
    */

    return intersectsData;
}

bool
TileSource::supportsPersistentCaching() const
{
    return true;
}

TileBlacklist*
TileSource::getBlacklist()
{
    return _blacklist.get();
}

const TileBlacklist*
TileSource::getBlacklist() const
{
    return _blacklist.get();
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[TileSourceFactory] "
#define TILESOURCEOPTIONS_TAG "__osgEarth::TileSourceOptions"

TileSource*
TileSourceFactory::create( const TileSourceOptions& options )
{
    TileSource* result = 0L;

    std::string driver = options.getDriver();
    if ( driver.empty() )
    {
        OE_WARN << "ILLEGAL- no driver set for tile source" << std::endl;
        return 0L;
    }

    osg::ref_ptr<osgDB::ReaderWriter::Options> rwopt = new osgDB::ReaderWriter::Options();
    rwopt->setPluginData( TILESOURCEOPTIONS_TAG, (void*)&options );

    std::string driverExt = std::string( ".osgearth_" ) + driver;
    result = dynamic_cast<TileSource*>( osgDB::readObjectFile( driverExt, rwopt.get() ) );
    if ( !result )
    {
        OE_WARN << "WARNING: Failed to load terrain engine driver for \"" << driver << "\"" << std::endl;
    }

    return result;
}

//------------------------------------------------------------------------

const TileSourceOptions&
TileSourceDriver::getTileSourceOptions( const osgDB::ReaderWriter::Options* rwopt ) const
{
    return *static_cast<const TileSourceOptions*>( rwopt->getPluginData( TILESOURCEOPTIONS_TAG ) );
}
