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
#include <osgEarth/TileVisitor>
#include <osgEarth/CacheEstimator>

using namespace osgEarth;

TileVisitor::TileVisitor():
_total(0),
_processed(0)
{
}


TileVisitor::TileVisitor(TileHandler* handler):
_tileHandler( handler ),
_total(0),
_processed(0)
{
}

void TileVisitor::addExtent( const GeoExtent& extent )
{
    _extents.push_back( extent );
}

bool TileVisitor::intersects( const GeoExtent& extent )
{    
    if ( _extents.empty()) return true;
    else
    {
        for (unsigned int i = 0; i < _extents.size(); ++i)
        {
            if (_extents[i].intersects( extent ))                
            {
                return true;
            }

        }
    }
    return false;
}

void TileVisitor::setTileHandler( TileHandler* handler )
{
    _tileHandler = handler;
}

void TileVisitor::setProgressCallback( ProgressCallback* progress )
{
    _progress = progress;
}

void TileVisitor::run( const Profile* mapProfile )
{
    _profile = mapProfile;

    estimate();

    std::vector<TileKey> keys;
    mapProfile->getRootKeys(keys);

    for (unsigned int i = 0; i < keys.size(); ++i)
    {
        processKey( keys[i] );
    }
}

void TileVisitor::estimate()
{
    //Estimate the number of tiles
    _total = 0;    
    CacheEstimator est;
    est.setMinLevel( _minLevel );
    est.setMaxLevel( _maxLevel );
    est.setProfile( _profile ); 
    for (unsigned int i = 0; i < _extents.size(); i++)
    {                
        est.addExtent( _extents[ i ] );
    } 
    _total = est.getNumTiles();
}

void TileVisitor::processKey( const TileKey& key )
{
    unsigned int x, y, lod;
    key.getTileXY(x, y);
    lod = key.getLevelOfDetail();

    bool traverseChildren = true;

    // Check to see if this key is within valid range.
    if ( _minLevel <= lod && _maxLevel >= lod && intersects( key.getExtent() ) )
    {        
        // Process the key
        traverseChildren = handleTile( key );        
    }

    // Traverse the children
    if (traverseChildren && lod < _maxLevel)
    {
        for (unsigned int i = 0; i < 4; i++)
        {
            TileKey k = key.createChildKey(i);
            processKey( k );
        }                                
    }       
}

void TileVisitor::incrementProgress(unsigned int amount)
{
    {
        OpenThreads::ScopedLock< OpenThreads::Mutex > lk(_progressMutex );
        _processed += amount;
    }
    _progress->reportProgress( _processed, _total );
}

bool TileVisitor::handleTile( const TileKey& key )
{
    bool result = false;
    if (_tileHandler.valid() )
    {
        result = _tileHandler->handleTile( key );
    }

    incrementProgress(1);
    
    return result;
}



/*****************************************************************************************/
/**
 * A TaskRequest that runs a TileHandler in a background thread.
 */
class HandleTileTask : public TaskRequest
{
public:
    HandleTileTask( TileHandler* handler, const TileKey& key ):      
      _handler( handler ),
          _key( key )
      {

      }

      virtual void operator()(ProgressCallback* progress )
      {         
          if (_handler.valid())
          {             
              _handler->handleTile( _key );
          }
      }

      osg::ref_ptr < TileHandler > _handler;
      TileKey _key;
};

MultithreadedTileVisitor::MultithreadedTileVisitor():
_numThreads( OpenThreads::GetNumberOfProcessors() )
{
    // We must do this to avoid an error message in OpenSceneGraph b/c the findWrapper method doesn't appear to be threadsafe.
    // This really isn't a big deal b/c this only effects data that is already cached.
    osgDB::ObjectWrapper* wrapper = osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper( "osg::Image" );
}

MultithreadedTileVisitor::MultithreadedTileVisitor( TileHandler* handler ):
TileVisitor( handler ),
    _numThreads( OpenThreads::GetNumberOfProcessors() )
{
}

unsigned int MultithreadedTileVisitor::getNumThreads() const
{
    return _numThreads; 
}

void MultithreadedTileVisitor::setNumThreads( unsigned int numThreads)
{
    _numThreads = numThreads; 
}

void MultithreadedTileVisitor::run(const Profile* mapProfile)
{                   
    // Start up the task service
    OE_NOTICE << "Starting " << _numThreads << std::endl;
    _taskService = new TaskService( "MTTileHandler", _numThreads, 5000 );

    // Produce the tiles
    TileVisitor::run( mapProfile );

    // Send a poison pill to kill all the threads
    _taskService->add( new PoisonPill() );

    // Wait for everything to finish
    _taskService->waitforThreadsToComplete();

    OE_NOTICE << "All threads have completed" << std::endl;
}

bool MultithreadedTileVisitor::handleTile( const TileKey& key )        
{
    _taskService->add( new HandleTileTask(_tileHandler, key ) );
    return true;
}