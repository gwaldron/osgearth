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

    struct MemCacheBin : public CacheBin
    {
        MemCacheBin( const std::string& id, unsigned maxSize )
            : CacheBin( id ),
              _lru    ( maxSize )
        {
            //nop
        }

        const osg::Object* readObject( const std::string& key, double maxAge )
        {
            Threading::ScopedReadLock sharedLock( _mutex );
            MemCacheLRU::Record rec = _lru.get(key);
            return rec.valid() ? rec.value().get() : 0L;
        }

        const osg::Image* readImage( const std::string& key, double maxAge )
        {
            return dynamic_cast<const osg::Image*>( readObject(key, maxAge) );
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
