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
#include <osgEarthSymbology/ResourceCache>

using namespace osgEarth;
using namespace osgEarth::Symbology;

osg::StateSet*
ResourceCache::getStateSet( SkinResource* skin )
{
    // first check if it exists
    {
        Threading::ScopedReadLock shared( _skinCacheMutex );
        SkinCache::Record rec = _skinCache.get( skin );
        if ( rec.valid() )
            return rec.value();
    }

    // no? exclusive lock and create it.
    {
        Threading::ScopedWriteLock exclusive( _skinCacheMutex );
        
        // double check to avoid race condition
        SkinCache::Record rec = _skinCache.get( skin );
        if ( rec.valid() )
            return rec.value();

        // still not there, make it.
        osg::StateSet* stateSet = skin->createStateSet();
        if ( stateSet )
            _skinCache.insert( skin, stateSet );

        return stateSet;
    }
}
