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
#include "TileNodeRegistry"

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;

#define LC "[TileNodeRegistry] "

#define OE_TEST OE_NULL
//#define OE_TEST OE_INFO


//----------------------------------------------------------------------------

TileNodeRegistry::TileNodeRegistry(const std::string& name) :
_revisioningEnabled( false ),
_name              ( name ),
_frameNumber       ( 0u )
{
    //nop
}


void
TileNodeRegistry::setRevisioningEnabled(bool value)
{
    _revisioningEnabled = value;
}


void
TileNodeRegistry::setMapRevision(const Revision& rev,
                                 bool            setToDirty)
{
    if ( _revisioningEnabled )
    {
        if ( _maprev != rev || setToDirty )
        {
            Threading::ScopedWriteLock exclusive( _tilesMutex );

            if ( _maprev != rev || setToDirty )
            {
                _maprev = rev;

                for( TileNodeMap::iterator i = _tiles.begin(); i != _tiles.end(); ++i )
                {
                    i->second->setMapRevision( _maprev );
                    if ( setToDirty )
                    {
                        i->second->setDirty( true );
                    }
                }
            }
        }
    }
}


//NOTE: this method assumes the input extent is the same SRS as
// the terrain profile SRS.
void
TileNodeRegistry::setDirty(const GeoExtent& extent,
                           unsigned         minLevel,
                           unsigned         maxLevel)
{
    Threading::ScopedWriteLock exclusive( _tilesMutex );
    
    bool checkSRS = false;
    for( TileNodeMap::iterator i = _tiles.begin(); i != _tiles.end(); ++i )
    {
        const TileKey& key = i->first;
        if (minLevel <= key.getLOD() && 
            maxLevel >= key.getLOD() &&
            extent.intersects(i->first.getExtent(), checkSRS) )
        {
            i->second->setDirty( true );
        }
    }
}

void
TileNodeRegistry::addSafely(TileNode* tile)
{
    _tiles[ tile->getTileKey() ] = tile;
    if ( _revisioningEnabled )
        tile->setMapRevision( _maprev );

    // check for tiles that are waiting on this tile, and notify them!
    TileKeyOneToMany::iterator notifier = _notifiers.find( tile->getTileKey() );
    if ( notifier != _notifiers.end() )
    {
        TileKeySet& listeners = notifier->second;

        for(TileKeySet::iterator listener = listeners.begin(); listener != listeners.end(); ++listener)
        {
            TileNodeMap::iterator listenerTile = _tiles.find( *listener );
            if ( listenerTile != _tiles.end() )
            {
                listenerTile->second->notifyOfArrival( tile );
            }
        }
        _notifiers.erase( notifier );
    }

    OE_TEST << LC << _name 
        << ": tiles=" << _tiles.size()
        << ", notifiers=" << _notifiers.size()
        << std::endl;
}

void
TileNodeRegistry::removeSafely(const TileKey& key)
{
    _tiles.erase( key );

    for(TileKeyOneToMany::iterator i = _notifiers.begin(); i != _notifiers.end(); )
    {
        i->second.erase( key );

        if ( i->second.size() == 0 )
            _notifiers.erase( i++ ); // http://stackoverflow.com/a/8234813/4218920
        else
            ++i;
    }
}

void
TileNodeRegistry::add( TileNode* tile )
{
    if ( tile )
    {
        Threading::ScopedWriteLock exclusive( _tilesMutex );
        addSafely( tile );
    }
}

void
TileNodeRegistry::add( const TileNodeVector& tiles )
{
    if ( tiles.size() > 0 )
    {
        Threading::ScopedWriteLock exclusive( _tilesMutex );
        for( TileNodeVector::const_iterator i = tiles.begin(); i != tiles.end(); ++i )
        {
            if ( i->valid() )
                addSafely( i->get() );
        }
        OE_TEST << LC << _name << ": tiles=" << _tiles.size() << std::endl;
    }
}

void
TileNodeRegistry::remove( TileNode* tile )
{
    if ( tile )
    {
        Threading::ScopedWriteLock exclusive( _tilesMutex );
        removeSafely( tile->getTileKey() );
    }
}

void
TileNodeRegistry::clear()
{
    Threading::ScopedWriteLock exclusive( _tilesMutex );
    _tiles.clear();
}

void
TileNodeRegistry::move(TileNode* tile, TileNodeRegistry* destination)
{
    if ( tile )
    {
        // ref just in case remove() is the last reference
        osg::ref_ptr<TileNode> tileSafe = tile;
        remove( tile );
        if ( destination )
            destination->add( tileSafe.get() );
    }
}

void
TileNodeRegistry::moveAll(TileNodeRegistry* destination)
{
    Threading::ScopedWriteLock exclusive( _tilesMutex );

    if ( destination )
    {
        for( TileNodeMap::iterator i = _tiles.begin(); i != _tiles.end(); ++i )
        {
            if ( i->second.valid() )
                destination->add( i->second.get() );
        }
    }

    _tiles.clear();
}
    

bool
TileNodeRegistry::get( const TileKey& key, osg::ref_ptr<TileNode>& out_tile )
{
    Threading::ScopedReadLock shared( _tilesMutex );

    TileNodeMap::iterator i = _tiles.find(key);
    if ( i != _tiles.end() )
    {
        out_tile = i->second.get();
        return true;
    }
    return false;
}


bool
TileNodeRegistry::take( const TileKey& key, osg::ref_ptr<TileNode>& out_tile )
{
    Threading::ScopedWriteLock exclusive( _tilesMutex );

    TileNodeMap::iterator i = _tiles.find(key);
    if ( i != _tiles.end() )
    {
        out_tile = i->second.get();
        removeSafely( key );
        return true;
    }
    return false;
}


void
TileNodeRegistry::run( TileNodeRegistry::Operation& op )
{
    Threading::ScopedWriteLock lock( _tilesMutex );
    unsigned size = _tiles.size();
    op.operator()( _tiles );
    if ( size != _tiles.size() )
        OE_TEST << LC << _name << ": tiles=" << _tiles.size() << std::endl;
}


void
TileNodeRegistry::run( const TileNodeRegistry::ConstOperation& op ) const
{
    Threading::ScopedReadLock lock( _tilesMutex );
    op.operator()( _tiles );
    OE_TEST << LC << _name << ": tiles=" << _tiles.size() << std::endl;
}


bool
TileNodeRegistry::empty() const
{
    // don't bother mutex-protecteding this.
    return _tiles.empty();
}

void
TileNodeRegistry::listenFor(const TileKey& tileToWaitFor, TileNode* waiter)
{
    Threading::ScopedWriteLock lock( _tilesMutex );
    TileNodeMap::iterator i = _tiles.find( tileToWaitFor );
    if ( i != _tiles.end() )
    {
        OE_DEBUG << LC << waiter->getTileKey().str() << " listened for " << tileToWaitFor.str()
            << ", but it was already in the repo.\n";

        waiter->notifyOfArrival( i->second.get() );
    }
    else
    {
        OE_DEBUG << LC << waiter->getTileKey().str() << " listened for " << tileToWaitFor.str() << ".\n";
        //_notifications[tileToWaitFor].push_back( waiter->getKey() );
        _notifiers[tileToWaitFor].insert( waiter->getTileKey() );
    }
}
        
TileNode*
TileNodeRegistry::takeAny()
{
    Threading::ScopedWriteLock exclusive( _tilesMutex );
    osg::ref_ptr<TileNode> tile = _tiles.begin()->second.get();
    removeSafely( tile->getTileKey() );
    return tile.release();
}
