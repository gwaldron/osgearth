/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
_dbOptions ( dbOptions ),
_threadSafe( threadSafe )
{
    //nop
}

osg::StateSet*
ResourceCache::getStateSet( SkinResource* skin )
{
    osg::StateSet* result = 0L;

    if ( _threadSafe )
    {
        // first check if it exists
        {
            Threading::ScopedReadLock shared( _mutex );

            SkinCache::Record rec = _skinCache.get( skin );
            if ( rec.valid() )
            {
                result = rec.value();
            }
        }

        // no? exclusive lock and create it.
        if ( !result )
        {
            Threading::ScopedWriteLock exclusive( _mutex );
            
            // double check to avoid race condition
            SkinCache::Record rec = _skinCache.get( skin );
            if ( rec.valid() )
            {
                result = rec.value();
            }
            else
            {
                // still not there, make it.
                result = skin->createStateSet( _dbOptions.get() );
                if ( result )
                    _skinCache.insert( skin, result );
            }
        }
    }

    else
    {
        SkinCache::Record rec = _skinCache.get( skin );
        if ( rec.valid() )
        {
            result = rec.value();
        }
        else
        {
            result = skin->createStateSet( _dbOptions.get() );
            if ( result )
                _skinCache.insert( skin, result );
        }
    }

    return result;
}

osg::Node*
ResourceCache::getInstanceNode( InstanceResource* res )
{
    osg::Node* result = 0L;

    if ( _threadSafe )
    {
        // first check if it exists
        {
            Threading::ScopedReadLock shared( _mutex );

            InstanceCache::Record rec = _instanceCache.get( res );
            if ( rec.valid() )
            {
                result = rec.value();
            }
        }

        // no? exclusive lock and create it.
        if ( !result )
        {
            Threading::ScopedWriteLock exclusive( _mutex );
            
            // double check to avoid race condition
            InstanceCache::Record rec = _instanceCache.get( res );
            if ( rec.valid() )
            {
                result = rec.value();
            }
            else
            {
                // still not there, make it.
                result = res->createNode( _dbOptions.get() );
                if ( result )
                    _instanceCache.insert( res, result );
            }
        }
    }

    else
    {
        InstanceCache::Record rec = _instanceCache.get( res );
        if ( rec.valid() )
        {
            result = rec.value();
        }
        else
        {
            result = res->createNode( _dbOptions.get() );
            if ( result )
                _instanceCache.insert( res, result );
        }
    }

    return result;
}


osg::Node*
ResourceCache::getMarkerNode( MarkerResource* marker )
{
    osg::Node* result = 0L;

    if ( _threadSafe )
    {
        // first check if it exists
        {
            Threading::ScopedReadLock shared( _mutex );

            MarkerCache::Record rec = _markerCache.get( marker );
            if ( rec.valid() )
            {
                result = rec.value();
            }
        }

        // no? exclusive lock and create it.
        if ( !result )
        {
            Threading::ScopedWriteLock exclusive( _mutex );
            
            // double check to avoid race condition
            MarkerCache::Record rec = _markerCache.get( marker );
            if ( rec.valid() )
            {
                result = rec.value();
            }
            else
            {
                // still not there, make it.
                result = marker->createNode( _dbOptions.get() );
                if ( result )
                    _markerCache.insert( marker, result );
            }
        }
    }

    else
    {
        MarkerCache::Record rec = _markerCache.get( marker );
        if ( rec.valid() )
        {
            result = rec.value();
        }
        else
        {
            result = marker->createNode( _dbOptions.get() );
            if ( result )
                _markerCache.insert( marker, result );
        }
    }

    return result;
}
