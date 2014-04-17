/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

#define LC "[LevelDBCache] "

#define LEVELDB_CACHE_VERSION 1

using namespace osgEarth;
using namespace osgEarth::Drivers::LevelDBCache;


LevelDBCacheImpl::LevelDBCacheImpl( const CacheOptions& options ) :
osgEarth::Cache( options ),
_active        ( true )
{
    LevelDBCacheOptions local( options );
    if ( !local.rootPath().isSet() )
    {
        // read the root path from ENV is necessary:
        const char* cachePath = ::getenv(OSGEARTH_ENV_CACHE_PATH);
        if ( cachePath )
        {
            local.rootPath() = cachePath;
        }
    }

    if ( local.rootPath().isSet() )
    {
        _rootPath = URI( *local.rootPath(), options.referrer() ).full();
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
        delete _db;
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
}

void
LevelDBCacheImpl::open()
{
    leveldb::Options options;
    options.create_if_missing = true;
    leveldb::Status status = leveldb::DB::Open(options, _rootPath, &_db);
    if ( !status.ok() )
    {
        OE_WARN << LC << "Failed to open or create cache bin at " << _rootPath << std::endl;
        if ( _db )
        {
            delete _db;
            _db = 0L;
        }
    }
}

CacheBin*
LevelDBCacheImpl::addBin( const std::string& name )
{
    return _db ?
        _bins.getOrCreate(name, new LevelDBCacheBin(name, _db)) :
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
            _defaultBin = new LevelDBCacheBin("_default", _db);
        }
    }
    return _defaultBin.get();
}
