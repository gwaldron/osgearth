/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarthSymbology/ModelResource>
#include <osgEarthSymbology/IconResource>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/XmlUtils>
#include <osgEarth/Random>
#include <iterator>
#include <algorithm>
#include <fstream>

#define LC "[ResourceLibrary] "

using namespace osgEarth;
using namespace osgEarth::Symbology;
using namespace OpenThreads;

//------------------------------------------------------------------------

ResourceLibrary::ResourceLibrary(const Config& conf) :
_initialized( false )
{
    mergeConfig( conf );
}

ResourceLibrary::ResourceLibrary(const std::string&    name,
                                 const URI&            uri) :
_name       ( name ),
_uri        ( uri, uri ),
_initialized( false )
{
    //nop
}

void
ResourceLibrary::mergeConfig( const Config& conf )
{
    _name = conf.value( "name" );

    conf.getIfSet( "url", _uri );

    for( ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i )
    {
        const Config& child = *i;
        if ( child.key() == "skin" )
        {
            addResource( new SkinResource(child) );
        }
        else if ( child.key() == "marker" )
        {
            // to be decrepated
            addResource( new MarkerResource(child) );
        }
        else if ( child.key() == "model" )
        {
            addResource( new ModelResource(child) );
        }
        else if ( child.key() == "icon" )
        {
            addResource( new IconResource(child) );
        }
    }
}

Config
ResourceLibrary::getConfig() const
{
    Config conf("resources");
    {
        Threading::ScopedReadLock shared( const_cast<ResourceLibrary*>(this)->_mutex );

        if ( !_name.empty() )
        {
            conf.set( "name", _name );
        }

        if ( _uri.isSet() )
        {
            conf.addIfSet( "url", _uri );
        }
        else
        {
            for( ResourceMap<SkinResource>::const_iterator i = _skins.begin(); i != _skins.end(); ++i )
            {
                SkinResource* res = i->second.get();
                conf.add( res->getConfig() );
            }

            for( ResourceMap<MarkerResource>::const_iterator i = _markers.begin(); i != _markers.end(); ++i )
            {
                MarkerResource* res = i->second.get();
                conf.add( res->getConfig() );
            }

            for( ResourceMap<InstanceResource>::const_iterator i = _instances.begin(); i != _instances.end(); ++i )
            {
                InstanceResource* res = i->second.get();
                conf.add( res->getConfig() );
            }
        }
    }
    return conf;
}

void
ResourceLibrary::addResource( Resource* resource )
{
    if ( dynamic_cast<SkinResource*>( resource ) )
    {
        Threading::ScopedWriteLock exclusive(_mutex);
        _skins[resource->name()] = static_cast<SkinResource*>(resource);
    }
    else if ( dynamic_cast<MarkerResource*>(resource) )
    {
        Threading::ScopedWriteLock exclusive(_mutex);
        _markers[resource->name()] = static_cast<MarkerResource*>(resource);
    }
    else if ( dynamic_cast<InstanceResource*>(resource) )
    {
        Threading::ScopedWriteLock exclusive(_mutex);
        _instances[resource->name()] = static_cast<InstanceResource*>(resource);
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
    else if ( dynamic_cast<MarkerResource*>( resource ) )
    {
        Threading::ScopedWriteLock exclusive(_mutex);
        _markers.erase( resource->name() );
    }
    else if ( dynamic_cast<InstanceResource*>( resource ) )
    {
        Threading::ScopedWriteLock exclusive(_mutex);
        _instances.erase( resource->name() );
    }
}


static Threading::Mutex s_initMutex;

bool
ResourceLibrary::initialize( const osgDB::Options* dbOptions )
{
    bool ok = true;

    if ( !_initialized )
    {
        Threading::ScopedMutexLock exclusive(s_initMutex);
        if ( !_initialized )
        {
            ok = false;

            if ( _uri.isSet() )
            {
                osg::ref_ptr<XmlDocument> xml = XmlDocument::load( *_uri, dbOptions );
                if ( xml.valid() )
                {
                    ok = true;

                    Config conf = xml->getConfig();
                    if ( conf.key() == "resources" )
                    {
                        mergeConfig( conf );
                    }
                    else
                    {
                        const Config& child = conf.child("resources");
                        if ( !child.empty() )
                            mergeConfig( child );
                    }
                }
            }
            _initialized = true;
        }
    }

    return ok;
}

SkinResource*
ResourceLibrary::getSkin( const std::string& name, const osgDB::Options* dbOptions ) const
{
    const_cast<ResourceLibrary*>(this)->initialize( dbOptions );
    Threading::ScopedReadLock shared( _mutex );
    ResourceMap<SkinResource>::const_iterator i = _skins.find( name );
    return i != _skins.end() ? i->second.get() : 0L;
}

void
ResourceLibrary::getSkins( SkinResourceVector& output, const osgDB::Options* dbOptions ) const
{
    const_cast<ResourceLibrary*>(this)->initialize( dbOptions );
    Threading::ScopedReadLock shared( _mutex );
    output.reserve( _skins.size() );
    for( ResourceMap<SkinResource>::const_iterator i = _skins.begin(); i != _skins.end(); ++i )
        output.push_back( i->second.get() );
}

void
ResourceLibrary::getSkins( const SkinSymbol* symbol, SkinResourceVector& output, const osgDB::Options* dbOptions ) const
{
    const_cast<ResourceLibrary*>(this)->initialize( dbOptions );
    Threading::ScopedReadLock shared( _mutex );

    for( ResourceMap<SkinResource>::const_iterator i = _skins.begin(); i != _skins.end(); ++i )
    {
        SkinResource* skin = i->second.get();
        if ( matches(symbol, skin) )
        {
            output.push_back( skin );
        }
    }
}

SkinResource*
ResourceLibrary::getSkin( const SkinSymbol* symbol, Random& prng, const osgDB::Options* dbOptions ) const
{
    const_cast<ResourceLibrary*>(this)->initialize( dbOptions );

    if (symbol->name().isSet())
    {
        return getSkin(symbol->name()->eval(), dbOptions);
    }

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
    if ( q->name().isSet() )
    {
        return osgEarth::ciEquals(q->name()->eval(), s->name());
    }

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

MarkerResource*
ResourceLibrary::getMarker( const std::string& name, const osgDB::Options* dbOptions ) const
{
    const_cast<ResourceLibrary*>(this)->initialize( dbOptions );
    Threading::ScopedReadLock shared( _mutex );
    ResourceMap<MarkerResource>::const_iterator i = _markers.find( name );
    return i != _markers.end() ? i->second.get() : 0L;
}

void
ResourceLibrary::getMarkers( MarkerResourceVector& output, const osgDB::Options* dbOptions ) const
{
    const_cast<ResourceLibrary*>(this)->initialize( dbOptions );
    Threading::ScopedReadLock shared( _mutex );

    output.clear();
    output.reserve( _markers.size() );

    for( ResourceMap<MarkerResource>::const_iterator i = _markers.begin(); i != _markers.end(); ++i )
        output.push_back( i->second.get() );
}

InstanceResource*
ResourceLibrary::getInstance( const std::string& name, const osgDB::Options* dbOptions ) const
{
    const_cast<ResourceLibrary*>(this)->initialize( dbOptions );
    Threading::ScopedReadLock shared( _mutex );
    ResourceMap<InstanceResource>::const_iterator i = _instances.find( name );
    return i != _instances.end() ? i->second.get() : 0L;
}

ModelResource*
ResourceLibrary::getModel( const ModelSymbol* ms, const osgDB::Options* dbOptions ) const
{
    const_cast<ResourceLibrary*>(this)->initialize( dbOptions );
    Threading::ScopedReadLock shared( _mutex );
    ResourceMap<InstanceResource>::const_iterator i = _instances.find( ms->name()->eval() );
    return i != _instances.end() ? dynamic_cast<ModelResource*>(i->second.get()) : 0L;
}

void
ResourceLibrary::getModels( ModelResourceVector& output, const osgDB::Options* dbOptions ) const
{
    const_cast<ResourceLibrary*>(this)->initialize( dbOptions );
    Threading::ScopedReadLock shared( _mutex );
    output.reserve( _instances.size() );
    for( ResourceMap<InstanceResource>::const_iterator i = _instances.begin(); i != _instances.end(); ++i ) {
        ModelResource* m = dynamic_cast<ModelResource*>(i->second.get());
        if ( m ) output.push_back( m );
    }
}

void
ResourceLibrary::getModels(const ModelSymbol* ms, ModelResourceVector& output, const osgDB::Options* dbOptions) const
{
    if ( !ms ) return;

    // if a name is set, use the other getter.
    if ( ms->name().isSet() )
    {
        ModelResource* mr = getModel(ms, dbOptions);
        if ( mr )
            output.push_back( mr );
    }

    else
    {
        const_cast<ResourceLibrary*>(this)->initialize( dbOptions );

        Threading::ScopedReadLock shared( _mutex );

        output.reserve( _instances.size() );
        for( ResourceMap<InstanceResource>::const_iterator i = _instances.begin(); i != _instances.end(); ++i )
        {
            ModelResource* m = dynamic_cast<ModelResource*>(i->second.get());
            if ( m )
            {
                if (ms && ms->tags().size() > 0 && !m->containsTags(ms->tags()) )
                    continue;

                // if there's a size restriction, pre-load the model so we can know its size.
                if (ms && ms->maxSizeX().isSet() && ms->maxSizeY().isSet())
                {
                    const osg::BoundingBox& bbox = m->getBoundingBox( dbOptions );
                    if ( ms->maxSizeX().get() < (bbox.xMax() - bbox.xMin()) )
                        continue;
                    if ( ms->maxSizeY().get() < (bbox.yMax() - bbox.yMin()) )
                        continue;
                }

                // good, return it
                output.push_back( m );
            }
        }
    }
}