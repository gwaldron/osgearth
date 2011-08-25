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
#include <osgEarthSymbology/ResourceLibrary>
#include <osgEarth/ThreadingUtils>
#include <iterator>
#include <algorithm>

#define LC "[ResourceLibrary] "

using namespace osgEarth;
using namespace osgEarth::Symbology;
using namespace OpenThreads;

ResourceLibrary::ResourceLibrary( Threading::ReadWriteMutex& mutex ) :
_mutex( mutex )
{
    //nop
}

void
ResourceLibrary::addResource( Resource* resource )
{
    if ( dynamic_cast<SkinResource*>( resource ) )
    {
        Threading::ScopedWriteLock exclusive(_mutex);
        _skins[resource->name()] = static_cast<SkinResource*>(resource);
    }
    else
    {
        OE_WARN << LC << "Added a resource type that is not supported; ignoring." << std::endl;
    }
}

void
ResourceLibrary::removeResource( Resource* resource )
{
    if ( dynamic_cast<SkinResource*>( resource ) )
    {
        Threading::ScopedWriteLock exclusive(_mutex);
        _skins.erase( resource->name() );
    }
}

SkinResource*
ResourceLibrary::getSkin( const std::string& name ) const
{
    Threading::ScopedReadLock shared(_mutex);
    SkinResourceMap::const_iterator i = _skins.find( name );
    return i != _skins.end() ? i->second.get() : 0L;
}

void
ResourceLibrary::getSkins( SkinResourceVector& output ) const
{
    Threading::ScopedReadLock shared(_mutex);
    output.reserve( _skins.size() );
    for( SkinResourceMap::const_iterator i = _skins.begin(); i != _skins.end(); ++i )
        output.push_back( i->second.get() );
}

void
ResourceLibrary::getSkins( const SkinSymbol* q, SkinResourceVector& output ) const
{
    Threading::ScopedReadLock shared(_mutex);

    for( SkinResourceMap::const_iterator i = _skins.begin(); i != _skins.end(); ++i )
    {
        SkinResource* s = i->second.get();
        
        if (q->objectHeight().isSet())
        {
            if (s->minObjectHeight().isSet() && 
                q->objectHeight().value() < s->minObjectHeight().value() )
            {
                continue;
            }
            if (s->maxObjectHeight().isSet() && 
                q->objectHeight().value() > s->maxObjectHeight().value() )
            {
                continue;
            }
        }
             
        if (q->minObjectHeight().isSet() && 
            s->maxObjectHeight().isSet() && 
            q->minObjectHeight().value() > s->maxObjectHeight().value() )
        {
            continue;
        }

        if (q->maxObjectHeight().isSet() && 
            s->minObjectHeight().isSet() &&
            q->maxObjectHeight().value() < s->minObjectHeight().value() )
        {
            continue;
        }

        if (q->repeatsVertically().isSet() && 
            q->repeatsVertically().value() != s->repeatsVertically().value() )
        {
            continue;
        }

        if (q->tags().size() > 0 && !s->containsTags(q->tags()) )
        {
            continue;
        }

        // it's a good one.
        output.push_back( s );
    }
}
