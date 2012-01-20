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
#include <osgEarth/MemCache>
#include <osgEarth/StringUtils>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Containers>

using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    typedef std::pair<osg::ref_ptr<const osg::Object>, Config> MemCacheEntry;
    typedef LRUCache<std::string, MemCacheEntry> MemCacheLRU;

    struct MemCacheBin : public CacheBin
    {
        MemCacheBin( const std::string& id, unsigned maxSize )
            : CacheBin( id ),
              _lru    ( maxSize )
        {
            //nop
        }

        ReadResult readObject(const std::string& key,
                              double             maxAge )
        {
            // sadly we must write-lock even to read, since get() alters the LRU list
            Threading::ScopedWriteLock exclusiveLock( _mutex );
            MemCacheLRU::Record rec = _lru.get(key);
            // clone required since the cache is in memory

            if ( rec.valid() )
            {
                return ReadResult( 
                   osg::clone(rec.value().first.get(), osg::CopyOp::DEEP_COPY_ALL),
                   rec.value().second );
            }
            else
                return ReadResult();
        }

        ReadResult readImage(const std::string& key,
                             double             maxAge )
        {
            return readObject( key, maxAge );
        }

        ReadResult readString(const std::string& key,
                              double             maxAge )
        {
            return readObject( key, maxAge );
        }

        bool write( const std::string& key, const osg::Object* object, const Config& meta )
        {
            if ( object ) 
            {
                Threading::ScopedWriteLock exclusiveLock( _mutex );
                _lru.insert( key, std::make_pair(object, meta) );
                return true;
            }
            else
                return false;
        }

        bool isCached( const std::string& key, double maxAge ) 
        {
            Threading::ScopedReadLock sharedLock( _mutex );
            return _lru.has(key);
        }

        bool purge()
        {
            Threading::ScopedWriteLock exclusiveLock( _mutex );
            _lru.clear();
            return true;
        }

    private:
        MemCacheLRU               _lru;
        Threading::ReadWriteMutex _mutex;
    };
}

//------------------------------------------------------------------------

MemCache::MemCache( unsigned maxBinSize ) :
_maxBinSize( std::max(maxBinSize, 1u) )
{
    //nop
}

CacheBin*
MemCache::addBin( const std::string& binID )
{
    return _bins.getOrCreate( binID, new MemCacheBin(binID, _maxBinSize) );
}

CacheBin*
MemCache::getOrCreateDefaultBin()
{
    static Threading::Mutex s_defaultBinMutex;

    if ( !_defaultBin.valid() )
    {
        Threading::ScopedMutexLock lock( s_defaultBinMutex );
        // double check
        if ( !_defaultBin.valid() )
        {
            _defaultBin = new MemCacheBin("__default", _maxBinSize);
        }
    }

    return _defaultBin.get();
}
