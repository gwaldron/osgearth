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
#include <osgEarthSymbology/ResourceCache>

using namespace osgEarth;
using namespace osgEarth::Symbology;

ResourceCache::ResourceCache(const osgDB::Options* dbOptions,
                             bool                  threadSafe ) :
_dbOptions    ( dbOptions ),
_threadSafe   ( threadSafe ),
_skinCache    ( false ),
_markerCache  ( false ),
_instanceCache( false )
{
    //nop
}

bool
ResourceCache::getStateSet(SkinResource*                skin,
                           osg::ref_ptr<osg::StateSet>& output)
{
    output = 0L;
    std::string key = skin->getConfig().toJSON(false);

    if ( _threadSafe )
    {
        // first check if it exists
        {
            Threading::ScopedReadLock shared( _skinMutex );

            SkinCache::Record rec;
            if ( _skinCache.get(key, rec) )
            {
                output = rec.value().get();
            }
        }

        // no? exclusive lock and create it.
        if ( !output.valid() )
        {
            Threading::ScopedWriteLock exclusive( _skinMutex );
            
            // double check to avoid race condition
            SkinCache::Record rec;
            if ( _skinCache.get(key, rec) )
            {
                output = rec.value().get();
            }
            else
            {
                // still not there, make it.
                output = skin->createStateSet( _dbOptions.get() );
                if ( output.valid() )
                    _skinCache.insert( key, output.get() );
            }
        }
    }

    else
    {
        SkinCache::Record rec;
        if ( _skinCache.get(key, rec) )
        {
            output = rec.value();
        }
        else
        {
            output = skin->createStateSet( _dbOptions.get() );
            if ( output.valid() )
                _skinCache.insert( key, output.get() );
        }
    }

    return output.valid();
}


bool
ResourceCache::getInstanceNode(InstanceResource*        res,
                               osg::ref_ptr<osg::Node>& output)
{
    output = 0L;
    std::string key = res->getConfig().toJSON(false);

    if ( _threadSafe )
    {
        // first check if it exists
        {
            Threading::ScopedReadLock shared( _instanceMutex );

            InstanceCache::Record rec;
            if ( _instanceCache.get(key, rec) )
            {
                output = rec.value().get();
            }
        }

        // no? exclusive lock and create it.
        if ( !output.valid() )
        {
            Threading::ScopedWriteLock exclusive( _instanceMutex );
            
            // double check to avoid race condition
            InstanceCache::Record rec;
            if ( _instanceCache.get(key, rec) )
            {
                output = rec.value().get();
            }
            else
            {
                // still not there, make it.
                output = res->createNode( _dbOptions.get() );
                if ( output.valid() )
                    _instanceCache.insert( key, output.get() );
            }
        }
    }

    else
    {
        InstanceCache::Record rec;
        if ( _instanceCache.get(key, rec) )
        {
            output = rec.value().get();
        }
        else
        {
            output = res->createNode( _dbOptions.get() );
            if ( output.valid() )
                _instanceCache.insert( key, output.get() );
        }
    }

    return output.valid();
}


bool
ResourceCache::getMarkerNode(MarkerResource*          marker,
                             osg::ref_ptr<osg::Node>& output)
{
    output = 0L;
    std::string key = marker->getConfig().toJSON(false);

    if ( _threadSafe )
    {
        // first check if it exists
        {
            Threading::ScopedReadLock shared( _markerMutex );

            MarkerCache::Record rec;
            if ( _markerCache.get(key, rec) )
            {
                output = rec.value().get();
            }
        }

        // no? exclusive lock and create it.
        if ( !output.valid() )
        {
            Threading::ScopedWriteLock exclusive( _markerMutex );
            
            // double check to avoid race condition
            MarkerCache::Record rec;
            if ( _markerCache.get( key, rec ) )
            {
                output = rec.value().get();
            }
            else
            {
                // still not there, make it.
                output = marker->createNode( _dbOptions.get() );
                if ( output.valid() )
                    _markerCache.insert( key, output.get() );
            }
        }
    }

    else
    {
        MarkerCache::Record rec;
        if ( _markerCache.get( key, rec ) )
        {
            output = rec.value().get();
        }
        else
        {
            output = marker->createNode( _dbOptions.get() );
            if ( output.valid() )
                _markerCache.insert( key, output.get() );
        }
    }

    return output.valid();
}
