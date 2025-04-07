/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/MemCache>

using namespace osgEarth;

#define LC "[MemCacheBin] "

//#define CLONE_DATA

//------------------------------------------------------------------------

namespace
{
    typedef std::pair<osg::ref_ptr<const osg::Object>, Config> MemCacheEntry;
    typedef LRUCache<std::string, MemCacheEntry> MemCacheLRU;

    struct MemCacheBin : public CacheBin
    {
        MemCacheBin( const std::string& id, unsigned maxSize )
            : CacheBin( id, true ),
              _lru    ( true /* MT-safe */, maxSize )
        {
            //nop
        }

        ReadResult readObject(const std::string& key, const osgDB::Options*)
        {
            MemCacheLRU::Record rec;
            _lru.get(key, rec);

            // clone required since the cache is in memory

            if ( rec.valid() )
            {
#ifdef CLONE_DATA
                return ReadResult( 
                   osg::clone(rec.value().first.get(), osg::CopyOp::DEEP_COPY_ALL),
                   rec.value().second );
#else
                return ReadResult(const_cast<osg::Object*>(rec.value().first.get()), rec.value().second);
#endif
            }
            else
            {
                return ReadResult();
            }
        }

        ReadResult readImage(const std::string& key, const osgDB::Options* readOptions)
        {
            return readObject(key, readOptions);
        }

        ReadResult readString(const std::string& key, const osgDB::Options* readOptions)
        {
            return readObject(key, readOptions);
        }

        bool write( const std::string& key, const osg::Object* object, const Config& meta, const osgDB::Options* writeOptions)
        {
            if ( object ) 
            {
#ifdef CLONE_DATA
                osg::ref_ptr<const osg::Object> cloned = osg::clone(object, osg::CopyOp::DEEP_COPY_ALL);
                _lru.insert( key, std::make_pair(cloned.get(), meta) );
#else
                _lru.insert( key, std::make_pair(object, meta) );
#endif
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

        std::string getHashedKey(const std::string& key) const
        {
            return key;
        }

        MemCacheLRU _lru;
    };
    

    static std::mutex s_defaultBinMutex;
}

//------------------------------------------------------------------------

MemCache::MemCache( unsigned maxBinSize ) :
_maxBinSize( osg::maximum(maxBinSize, 1u) )
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
        std::lock_guard<std::mutex> lock( s_defaultBinMutex );
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
