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
            TileKeyVector& waiters = i->second;
            for(unsigned j=0; j<waiters.size(); )
            {
                TileKey& waiter = waiters[j];
                TileNodeMap::iterator k = _tiles.find(waiter);
                if ( k != _tiles.end() )
                {
                    k->second->notifyOfArrival( tile );
                    waiter = waiters.back();
                    waiters.resize( waiters.size()-1 );
                }
                else
                {
                    ++j;
                }
            }
            if ( waiters.size() == 0 )
            {
                _notifications.erase( i );
            }
        }
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
        OE_DEBUG << LC << waiter->getKey().str() << " listened for " << tileToWaitFor.str()
            << ", but it was already in the repo.\n";

        waiter->notifyOfArrival( i->second.get() );
    }
    else
    {
        OE_DEBUG << LC << waiter->getKey().str() << " listened for " << tileToWaitFor.str() << ".\n";
        _notifications[tileToWaitFor].push_back( waiter->getKey() );
    }
}
