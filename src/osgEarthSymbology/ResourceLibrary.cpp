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
#include <osgEarth/XmlUtils>
#include <osgEarth/HTTPClient>
#include <osgEarth/Random>
#include <iterator>
#include <algorithm>
#include <fstream>

#define LC "[ResourceLibrary] "

using namespace osgEarth;
using namespace osgEarth::Symbology;
using namespace OpenThreads;

//------------------------------------------------------------------------

ResourceLibrary*
ResourceLibrary::create( const URI& uri )
{
    osg::ref_ptr<XmlDocument> xml = XmlDocument::load( uri ); // buf, uri.full() );
    if ( !xml.valid() )
    {
        OE_WARN << LC << "Failed to parse XML for resource library \"" << uri.full() << "\"" << std::endl;
        return 0L;
    }

    Config conf = xml->getConfig();

    if ( conf.key() == "resources" )
    {
        return new ResourceLibrary( conf );
    }
    else
    {
        const Config& child = conf.child("resources");
        if ( !child.empty() )
            return new ResourceLibrary( child );
    }

    OE_WARN << LC << "Could not find top level 'resources' entry in resource library \""
        << uri.full() << "\"; load failed." << std::endl;
    return 0L;
}

//------------------------------------------------------------------------

ResourceLibrary::ResourceLibrary( const Config& conf )
{
    mergeConfig( conf );
}

void
ResourceLibrary::mergeConfig( const Config& conf )
{
    // read skins
    const ConfigSet skins = conf.children( "skin" );
    for( ConfigSet::const_iterator i = skins.begin(); i != skins.end(); ++i )
    {
        addResource( new SkinResource(*i) );
    }

    //todo: other types later..
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
    Threading::ScopedReadLock shared( const_cast<ResourceLibrary*>(this)->_mutex );
    SkinResourceMap::const_iterator i = _skins.find( name );
    return i != _skins.end() ? i->second.get() : 0L;
}

void
ResourceLibrary::getSkins( SkinResourceVector& output ) const
{
    Threading::ScopedReadLock shared( const_cast<ResourceLibrary*>(this)->_mutex );
    output.reserve( _skins.size() );
    for( SkinResourceMap::const_iterator i = _skins.begin(); i != _skins.end(); ++i )
        output.push_back( i->second.get() );
}

void
ResourceLibrary::getSkins( const SkinSymbol* symbol, SkinResourceVector& output ) const
{
    Threading::ScopedReadLock shared( const_cast<ResourceLibrary*>(this)->_mutex );

    for( SkinResourceMap::const_iterator i = _skins.begin(); i != _skins.end(); ++i )
    {
        SkinResource* skin = i->second.get();
        if ( matches(symbol, skin) )
        {
            output.push_back( skin );
        }
    }
}

SkinResource*
ResourceLibrary::getSkin( const SkinSymbol* symbol, Random& prng ) const
{
    SkinResourceVector candidates;
    getSkins( symbol, candidates );
    unsigned size = candidates.size();
    if ( size == 0 )
    {
        return 0L;
    }
    else if ( size == 1 )
    {
        return candidates[0].get();
    }
    else
    {
        return candidates[ prng.next(size) ].get();
    }
}

bool
ResourceLibrary::matches( const SkinSymbol* q, SkinResource* s ) const
{
    if (q->objectHeight().isSet())
    {
        if (s->minObjectHeight().isSet() && 
            q->objectHeight().value() < s->minObjectHeight().value() )
        {
            return false;
        }
        if (s->maxObjectHeight().isSet() && 
            q->objectHeight().value() > s->maxObjectHeight().value() )
        {
            return false;
        }
    }

    if (q->minObjectHeight().isSet() && 
        s->maxObjectHeight().isSet() && 
        q->minObjectHeight().value() > s->maxObjectHeight().value() )
    {
        return false;
    }

    if (q->maxObjectHeight().isSet() && 
        s->minObjectHeight().isSet() &&
        q->maxObjectHeight().value() < s->minObjectHeight().value() )
    {
        return false;
    }

    if (q->isTiled().isSet() && 
        q->isTiled().value() != s->isTiled().value() )
    {
        return false;
    }

    if (q->tags().size() > 0 && !s->containsTags(q->tags()) )
    {
        return false;
    }

    return true;
}
