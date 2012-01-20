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

#define LC "[TileSource] "

using namespace osgEarth;

//#undef OE_DEBUG
//#define OE_DEBUG OE_INFO

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

    if ( *options.L2CacheSize() > 0 )
    {
        _memCache = new MemCache( *options.L2CacheSize() );
    }
    else
    {
        OE_INFO << LC << "L2 Cache disabled" << std::endl;
    }

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

osg::Image*
TileSource::createImage(const TileKey&        key,
                        ImageOperation*       prepOp, 
                        ProgressCallback*     progress )
{
    // Try to get it from the memcache fist
    if (_memCache.valid())
    {
        ReadResult r = _memCache->getOrCreateDefaultBin()->readImage( key.str() );
        if ( r.succeeded() )
            return r.releaseImage();
    }

    osg::ref_ptr<osg::Image> newImage = createImage(key, progress);

    if ( prepOp )
        (*prepOp)( newImage );

    if ( newImage.valid() && _memCache.valid() )
    {
        // cache it to the memory cache.
        _memCache->getOrCreateDefaultBin()->write( key.str(), newImage.get() );
    }

    return newImage.release();
}

osg::HeightField*
TileSource::createHeightField(const TileKey&        key,
                              HeightFieldOperation* prepOp, 
                              ProgressCallback*     progress )
{
    // Try to get it from the memcache first:
	if (_memCache.valid())
	{
        ReadResult r = _memCache->getOrCreateDefaultBin()->readObject( key.str() );
        if ( r.succeeded() )
            return r.release<osg::HeightField>();
	}

    osg::ref_ptr<osg::HeightField> newHF = createHeightField( key, progress );

    if ( prepOp )
        (*prepOp)( newHF );

    if ( newHF.valid() && _memCache.valid() )
    {
        _memCache->getOrCreateDefaultBin()->write( key.str(), newHF.get() );
    }

    //TODO: why not just newHF.release()? -gw
    return newHF.valid() ? new osg::HeightField( *newHF.get() ) : 0L;
}

osg::HeightField*
TileSource::createHeightField(const TileKey&        key,
                              ProgressCallback*     progress)
{
    //osg::ref_ptr<osg::Image> image = createImage(key, dbOptions, progress);
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
    if (_dataExtents.size() == 0) return 23;

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
TileSource::hasDataAtLOD( unsigned lod ) const
{
    //If no data extents are provided, just return true
    if ( _dataExtents.size() == 0 )
        return true;

    bool intersects = false;

    for (DataExtentList::const_iterator itr = _dataExtents.begin(); itr != _dataExtents.end(); ++itr)
    {
        if ( itr->getMinLevel() <= lod && lod <= itr->getMaxLevel() )
        {
            intersects = true;
            break;
        }
    }
    return intersects;
}

bool
TileSource::hasDataInExtent( const GeoExtent& extent ) const
{
    //If no data extents are provided, just return true
    if ( _dataExtents.size() == 0 )
        return true;

    bool intersects = false;

    for (DataExtentList::const_iterator itr = _dataExtents.begin(); itr != _dataExtents.end(); ++itr)
    {
        if ( extent.intersects( *itr ) )
        {
            intersects = true;
            break;
        }
    }
    return intersects;
}


bool
TileSource::hasData(const osgEarth::TileKey& key) const
{
    //If no data extents are provided, just return true
    if (_dataExtents.size() == 0) return true;

    const osgEarth::GeoExtent& keyExtent = key.getExtent();
    bool intersectsData = false;

    for (DataExtentList::const_iterator itr = _dataExtents.begin(); itr != _dataExtents.end(); ++itr)
    {
        if (keyExtent.intersects( *itr ) && key.getLevelOfDetail() >= itr->getMinLevel() && key.getLevelOfDetail() <= itr->getMaxLevel())
        {
            intersectsData = true;
            break;
        }
    }

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
        OE_WARN << "WARNING: Failed to load TileSource driver for \"" << driver << "\"" << std::endl;
    }

    return result;
}

//------------------------------------------------------------------------

const TileSourceOptions&
TileSourceDriver::getTileSourceOptions( const osgDB::ReaderWriter::Options* rwopt ) const
{
    return *static_cast<const TileSourceOptions*>( rwopt->getPluginData( TILESOURCEOPTIONS_TAG ) );
}
