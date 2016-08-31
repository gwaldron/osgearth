/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "TileNodeRegistry"

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;

#define LC "[TileNodeRegistry] "

#define OE_TEST OE_NULL
//#define OE_TEST OE_INFO


//----------------------------------------------------------------------------

TileNodeRegistry::TileNodeRegistry(const std::string& name) :
_name              ( name ),
_revisioningEnabled( false ),
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
                        i->second->setDirty();
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
            i->second->setDirty();
        }
    }
}


void
TileNodeRegistry::add( TileNode* tile )
{
    if ( tile )
    {
        Threading::ScopedWriteLock exclusive( _tilesMutex );
        _tiles[ tile->getKey() ] = tile;
        if ( _revisioningEnabled )
            tile->setMapRevision( _maprev );

        OE_TEST << LC << _name << ": tiles=" << _tiles.size() << std::endl;

        // check for waiters.
        Notifications::iterator i = _notifications.find(tile->getKey());
        if ( i != _notifications.end() )
        {
            TileKeySet& waiters = i->second;

            for (TileKeySet::iterator j = waiters.begin(); j != waiters.end(); ++j)
            {
                const TileKey& waiter = *j;
                TileNodeMap::iterator k = _tiles.find(waiter);
                if ( k != _tiles.end() )
                {
                    // notify the listener:
                    k->second->notifyOfArrival( tile );
                }
            }

            // clear the wait list for this tile.
            // if there were waiters whose keys weren't found in the registry,
            // they should not have been there anyway!
            _notifications.erase(i);
                    
            //for(unsigned j=0; j<waiters.size(); )
            //{
            //    TileKey& waiter = waiters[j];
            //    TileNodeMap::iterator k = _tiles.find(waiter);
            //    if ( k != _tiles.end() )
            //    {
            //        k->second->notifyOfArrival( tile );

            //        // erase the waiter by swapping the back element into the 
            //        // current position and resizing the vector:
            //        waiter = waiters.back();
            //        waiters.resize( waiters.size()-1 );
            //    }
            //    else
            //    {
            //        ++j;
            //    }
            //}

            //// when the waiters list goes empty, remove the entire list
            //if ( waiters.size() == 0 )
            //{
            //    _notifications.erase( i );
            //}
        }

        // Listen for east and south neighbors of the new tile:
        const TileKey& key = tile->getTileNode()->getKey();
        startListeningFor( key.createNeighborKey(1, 0), tile->getTileNode() );
        startListeningFor( key.createNeighborKey(0, 1), tile->getTileNode() );
    }
}

void
TileNodeRegistry::remove( TileNode* tile )
{
    if ( tile )
    {
        Threading::ScopedWriteLock exclusive( _tilesMutex );
        _tiles.erase( tile->getKey() );
        OE_TEST << LC << _name << ": tiles=" << _tiles.size() << std::endl;
        
        // remove neighbor listeners:
        const TileKey& key = tile->getTileNode()->getKey();
        stopListeningFor( key.createNeighborKey(1, 0), tile->getTileNode() );
        stopListeningFor( key.createNeighborKey(0, 1), tile->getTileNode() );
    }
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


bool
TileNodeRegistry::get( const TileKey& key, osg::ref_ptr<TileNode>& out_tile )
{
    Threading::ScopedReadLock shared( _tilesMutex );

    TileNodeMap::iterator i = _tiles.find(key);
    if ( i != _tiles.end() && i->second.valid() )
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
        _tiles.erase( i );
        OE_TEST << LC << _name << ": tiles=" << _tiles.size() << std::endl;
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
        OE_TEST << LC << _name << ": tiles=" << _tiles.size() << ", notifications=" << _notifications.size() << std::endl;
}


void
TileNodeRegistry::run( const TileNodeRegistry::ConstOperation& op ) const
{
    Threading::ScopedReadLock lock( _tilesMutex );
    op.operator()( _tiles );
    OE_TEST << LC << _name << ": tiles=" << _tiles.size() << ", notifications=" << _notifications.size() << std::endl;
}


bool
TileNodeRegistry::empty() const
{
    // don't bother mutex-protecteding this.
    return _tiles.empty();
}

void
TileNodeRegistry::startListeningFor(const TileKey& tileToWaitFor, TileNode* waiter)
{
    //Threading::ScopedWriteLock lock( _tilesMutex );
    // ASSUME EXCLUSIVE LOCK

    TileNodeMap::iterator i = _tiles.find( tileToWaitFor );
    if ( i != _tiles.end() )
    {
        OE_DEBUG << LC << waiter->getKey().str() << " listened for " << tileToWaitFor.str()
            << ", but it was already in the repo.\n";

        waiter->notifyOfArrival( i->second.get() );
    }
    else
    {
        OE_DEBUG << LC << waiter->getKey().str() << " listened for " << tileToWaitFor.str() << ".\n";
        _notifications[tileToWaitFor].insert( waiter->getKey() );
    }
}

void
TileNodeRegistry::stopListeningFor(const TileKey& tileToWaitFor, TileNode* waiter)
{
    //Threading::ScopedWriteLock lock( _tilesMutex );
    // ASSUME EXCLUSIVE LOCK

    Notifications::iterator i = _notifications.find(tileToWaitFor);
    if (i != _notifications.end())
    {
        // remove the waiter from this set:
        i->second.erase(waiter->getKey());

        // if the set is now empty, remove the set entirely
        if (i->second.empty())
        {
            _notifications.erase(i);
        }
    }
}
