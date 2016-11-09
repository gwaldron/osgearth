/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include "LevelDBCache"
#include "LevelDBCacheBin"
#include <osgEarth/URI>
#include <osgEarth/ThreadingUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ObjectWrapper>

#include <sys/stat.h>
#ifndef _WIN32
#   include <unistd.h>
#endif

#define LC "[LevelDBCache] "

#define OSGEARTH_ENV_CACHE_MAX_SIZE_MB "OSGEARTH_CACHE_MAX_SIZE_MB"

#define LEVELDB_CACHE_VERSION 1

using namespace osgEarth;
using namespace osgEarth::Drivers::LevelDBCache;


LevelDBCacheImpl::LevelDBCacheImpl( const CacheOptions& options ) :
osgEarth::Cache( options ),
_options       ( options ),
_active        ( true )
{
    // Force OSG to initialize the image wrapper. Failure to do this can result
    // in a race condition within OSG when the cache is accessed from multiple threads.
    osgDB::ObjectWrapperManager* owm = osgDB::Registry::instance()->getObjectWrapperManager();
    owm->findWrapper("osg::Image");
    owm->findWrapper("osg::HeightField");

    if ( _options.rootPath().isSet() )
    {
        _rootPath = URI( *_options.rootPath(), options.referrer() ).full();
    }
    else
    {
        // read the root path from ENV is necessary:
        const char* cachePath = ::getenv(OSGEARTH_ENV_CACHE_PATH);
        if ( cachePath )
        {
            _rootPath = cachePath;           
            OE_INFO << LC << "Cache location set from environment: \"" 
                << cachePath << "\"" << std::endl;
        }
    }

    const char* maxsize = ::getenv(OSGEARTH_ENV_CACHE_MAX_SIZE_MB);
    if ( maxsize )
    {
        unsigned mb = as<unsigned>(std::string(maxsize), 0u);
        if ( mb > 0 )
        {
            _options.maxSizeMB() = mb;

            OE_INFO << LC << "Set max cache size from environment: "
                << (_options.maxSizeMB().value()) << " MB"
                << std::endl;
        }
        else
        {
            OE_WARN << LC 
                << "Env var \"" OSGEARTH_ENV_CACHE_MAX_SIZE_MB "\" set to an invalid value"
                << std::endl;
        }
    }

    _tracker = new Tracker(_options, _rootPath);
    
    if ( !_rootPath.empty() )
    {
        init();
    }
    else
    {
        _active = false;
        OE_WARN << LC << "Illegal: no root path set for cache!" << std::endl;
    }
}

LevelDBCacheImpl::~LevelDBCacheImpl()
{
    if ( _db )
    {
        // problem. This destructor causes a lockup sometimes. Perhaps try
        // upgrading to a newer version of LDB. In the meantime, that's why it
        // is commented out
        //delete _db;
        _db = 0L;
    }
}

void
LevelDBCacheImpl::init()
{
    // ensure there's a folder for the cache.
    if ( !osgDB::fileExists(_rootPath) )
    {
        if (osgDB::makeDirectory(_rootPath) == false)
        {
            OE_WARN << LC << "Oh no, failed to create root cache folder \"" << _rootPath << "\""
                << std::endl;
            return;
        }
    }

    open();

    // Do an initial size check.
    if ( _db )
    {
        _tracker->calcSize();
    }

    if ( _active )
    {
        OE_INFO << LC << "Opened a cache at \"" << _rootPath << "\"" << std::endl;
    }
}

void
LevelDBCacheImpl::open()
{
    leveldb::Options options;
    options.create_if_missing = true;
    options.block_size        = _options.blockSize().value();

    leveldb::Status status;
        
    status = leveldb::DB::Open(options, _rootPath, &_db);
    if ( status.ok() )
        return;

    OE_WARN << LC << "Database problem...attempting to repair..." << std::endl;
    status = leveldb::RepairDB(_rootPath, options);
    if ( status.ok() )
    {
        status = leveldb::DB::Open(options, _rootPath, &_db);
        if ( status.ok() )
        {
            OE_WARN << LC << "...repair complete!" << std::endl;
            return;
        }
    }

    OE_WARN << LC << "Failed to open or create cache bin at " << _rootPath << std::endl;
    if ( _db )
    {
        delete _db;
        _db = 0L;
        _active = false;
    }
}

CacheBin*
LevelDBCacheImpl::addBin( const std::string& name )
{
    return _db ?
        _bins.getOrCreate(name, new LevelDBCacheBin(name, _db, _tracker.get())) :
        0L;
}

CacheBin*
LevelDBCacheImpl::getOrCreateDefaultBin()
{    
    if ( !_db )
        return 0L;

    static Threading::Mutex s_defaultBinMutex;
    if ( !_defaultBin.valid() )
    {
        Threading::ScopedMutexLock lock( s_defaultBinMutex );
        if ( !_defaultBin.valid() ) // double-check
        {
            _defaultBin = new LevelDBCacheBin("_default", _db, _tracker.get());
        }
    }
    return _defaultBin.get();
}

off_t
LevelDBCacheImpl::getApproximateSize() const
{
    return _tracker->calcSize();
}

bool
LevelDBCacheImpl::compact()
{
    if ( !_db )
        return false;

    _db->CompactRange(0L, 0L);

    return true;
}

bool
LevelDBCacheImpl::clear()
{
    if ( !_db )
        return false;

    // No WriteBatch because it doesn't seem to allow compaction to occur
    // -- need to figure out why someday.

    leveldb::Iterator* it = _db->NewIterator(leveldb::ReadOptions());
    for(it->SeekToFirst(); it->Valid(); it->Next())
    {
        _db->Delete(leveldb::WriteOptions(), it->key());
    }

    return true;
}
