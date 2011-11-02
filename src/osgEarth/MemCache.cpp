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
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Utils>

using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    typedef LRUCache<std::string, osg::ref_ptr<const osg::Object> > MemCacheLRU;
    
    struct RefString : public osg::Object, public std::string
    {
        RefString() { }
        RefString( const RefString& rhs, const osg::CopyOp& op ) { }
        RefString( const std::string& in ) : std::string(in) { }
        META_Object( osgEarth, RefString );
    };

    struct MemCacheBin : public CacheBin
    {
        MemCacheBin( const std::string& id, unsigned maxSize )
            : CacheBin( id ),
              _lru    ( maxSize )
        {
            //nop
        }

        bool readObject(osg::ref_ptr<osg::Object>& output,
                        const std::string&         key,
                        double                     maxAge )
        {
            Threading::ScopedReadLock sharedLock( _mutex );
            MemCacheLRU::Record rec = _lru.get(key);
            // clone required since the cache is in memory
            output = rec.valid() ? osg::clone(rec.value().get(), osg::CopyOp::DEEP_COPY_ALL) : 0L;
            return output.valid();
        }

        bool readImage(osg::ref_ptr<osg::Image>& output,
                       const std::string&        key,
                       double                    maxAge )
        {
            osg::ref_ptr<osg::Object> obj;
            readObject( obj, key, maxAge );
            output = dynamic_cast<osg::Image*>(obj.get());
            return output.valid();
        }

        bool readString(std::string&        buffer,
                        const std::string&  key,
                        double              maxAge )
        {
            osg::ref_ptr<osg::Object> obj;
            readObject( obj, key, maxAge );
            RefString* r = dynamic_cast<RefString*>(obj.get());
            if ( r ) buffer = *r;
            return r != 0L;
        }

        bool write( const std::string& key, const osg::Object* object )
        {
            Threading::ScopedWriteLock exclusiveLock( _mutex );
            if ( object ) 
            {
                _lru.insert( key, object );
                return true;
            }
            else
                return false;
        }

        bool write( const std::string& key, const std::string& buffer )
        {
            Threading::ScopedWriteLock exclusiveLock( _mutex );
            _lru.insert( key, new RefString(buffer) );
            return true;
        }

        bool isCached( const std::string& key, double maxAge ) 
        {
            Threading::ScopedReadLock sharedLock( _mutex );
            MemCacheLRU::Record rec = _lru.get(key);
            return rec.valid();
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
