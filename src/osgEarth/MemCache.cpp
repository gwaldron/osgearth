/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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

#define LC "[MemCacheBin] "

//------------------------------------------------------------------------

namespace
{
    typedef std::pair<osg::ref_ptr<const osg::Object>, Config> MemCacheEntry;
    typedef LRUCache<std::string, MemCacheEntry> MemCacheLRU;

    struct MemCacheBin : public CacheBin
    {
        MemCacheBin( const std::string& id, unsigned maxSize )
            : CacheBin( id ),
              _lru    ( true /* MT-safe */, maxSize )
        {
            //nop
        }

        ReadResult readObject(const std::string& key )
        {
            MemCacheLRU::Record rec;
            _lru.get(key, rec);

            // clone required since the cache is in memory

            if ( rec.valid() )
            {
                //OE_INFO << LC << "hits: " << _lru.getStats()._hitRatio*100.0f << "%" << std::endl;

                return ReadResult( 
                   osg::clone(rec.value().first.get(), osg::CopyOp::DEEP_COPY_ALL),
                   rec.value().second );
            }
            else
            {
                //OE_INFO << LC << "hits: " << _lru.getStats()._hitRatio*100.0f << "%" << std::endl;
                return ReadResult();
            }
        }

        ReadResult readImage(const std::string& key)
        {
            return readObject( key );
        }

        ReadResult readString(const std::string& key)
        {
            return readObject( key );
        }

        bool write( const std::string& key, const osg::Object* object, const Config& meta )
        {
            if ( object ) 
            {
                _lru.insert( key, std::make_pair(object, meta) );
                return true;
            }
            else
                return false;
        }

        bool remove(const std::string& key)
        {
            _lru.erase(key);
            return true;
        }

        bool touch(const std::string& key)
        {
            // just doing a get will put it at the front of the LRU list
            MemCacheLRU::Record dummy;
            return _lru.get(key, dummy);
        }

        RecordStatus getRecordStatus( const std::string& key )
        {
            // ignore minTime; MemCache does not support expiration
            return _lru.has(key) ? STATUS_OK : STATUS_NOT_FOUND;
        }

        bool purge()
        {
            _lru.clear();
            return true;
        }

        MemCacheLRU _lru;
    };
    

    static Threading::Mutex s_defaultBinMutex;
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
MemCache::getOrCreateBin(const std::string& binID)
{
    CacheBin* bin = getBin(binID);
    if ( !bin )
        bin = addBin(binID);
    return bin;
}

CacheBin*
MemCache::getOrCreateDefaultBin()
{
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


void
MemCache::dumpStats(const std::string& binID)
{
    MemCacheBin* bin = static_cast<MemCacheBin*>(getBin(binID));
    CacheStats stats = bin->_lru.getStats();
    OE_INFO << LC << "hit ratio = " << stats._hitRatio << std::endl;
}
