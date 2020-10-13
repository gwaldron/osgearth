/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/FileUtils>
#include <thread>

#if OSG_VERSION_GREATER_OR_EQUAL(3,5,10)
#include <osg/os_utils>
#define OS_SYSTEM osg_system
#else
#define OS_SYSTEM system
#endif

using namespace osgEarth;
using namespace osgEarth::Util;

TileVisitor::TileVisitor():
_total(0),
_processed(0),
_minLevel(0),
_maxLevel(99)
{
}


TileVisitor::TileVisitor(TileHandler* handler):
_tileHandler( handler ),
_total(0),
_processed(0),
_minLevel(0),
_maxLevel(99)
{
}

void TileVisitor::resetProgress()
{
    _total = 0;
    _processed = 0;
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
    
    // Reset the progress in case this visitor has been ran before.
    resetProgress();
    
    estimate();

    // Get all the root keys and process them.
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
    CacheEstimator est;
    est.setMinLevel( _minLevel );
    est.setMaxLevel( _maxLevel );
    est.setProfile( _profile.get() ); 
    for (unsigned int i = 0; i < _extents.size(); i++)
    {                
        est.addExtent( _extents[ i ] );
    } 
    _total = est.getNumTiles();
}

void TileVisitor::processKey( const TileKey& key )
{        
    // If we've been cancelled then just return.
    if (_progress && _progress->isCanceled())
    {        
        return;
    }    

    unsigned int x, y, lod;
    key.getTileXY(x, y);
    lod = key.getLevelOfDetail();    

    // Only process this key if it has a chance of succeeding.
    if (_tileHandler && key.getLOD() >= _minLevel && !_tileHandler->hasData(key))
    {                
        return;
    }    

    bool traverseChildren = false;

    // If the key intersects the extent attempt to traverse
    if (intersects( key.getExtent() ))
    {
        // If the lod is less than the min level don't do anything but do traverse the children.
        if (lod < _minLevel)
        {
            traverseChildren = true;
        }
        else
        {         
            // Process the key
            traverseChildren = handleTile( key );
        }
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
    if (_progress.valid())
    {
        // If report progress returns true then mark the task as being cancelled.
        if (_progress->reportProgress( _processed, _total ))
        {
            _progress->cancel();
        }
    }    
}

bool TileVisitor::handleTile( const TileKey& key )
{    
    bool result = false;
    if (_tileHandler.valid() )
    {
        result = _tileHandler->handleTile( key, *this );
    }

    incrementProgress(1);    
    
    return result;
}



/*****************************************************************************************/

namespace
{
    /**
     * A TaskRequest that runs a TileHandler in a background thread.
     */
    class HandleTileTask : public osg::Operation
    {
    public:
        HandleTileTask(TileHandler* handler, TileVisitor* visitor, const TileKey& key, ProgressCallback* progress) :
            _handler(handler),
            _visitor(visitor),
            _key(key),
            _progress(progress)
        {

        }

        virtual void operator()(osg::Object*)
        {
            if (_progress.valid() && _progress->isCanceled())
                return;

            if (_handler.valid())
            {
                _handler->handleTile(_key, *_visitor.get());
                _visitor->incrementProgress(1);
            }
        }

        osg::ref_ptr<TileHandler> _handler;
        TileKey _key;
        osg::ref_ptr<TileVisitor> _visitor;
        osg::ref_ptr<ProgressCallback> _progress;
    };
}

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
    OE_INFO << "Starting " << _numThreads << " threads " << std::endl;

    _threadPool = new ThreadPool("osgEarth.TileVisitor", _numThreads);

    // Produce the tiles
    TileVisitor::run( mapProfile );

    OE_INFO << _threadPool->getNumOperationsInQueue() << " tasks in the queue." << std::endl;

    // Wait for everything to finish, checking for cancellation while we wait so we can kill all the existing tasks.
    while(_threadPool->getNumOperationsInQueue() > 0)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

bool MultithreadedTileVisitor::handleTile(const TileKey& key)
{    
    // Add the tile to the task queue.
    _threadPool->run(new HandleTileTask(_tileHandler.get(), this, key, getProgressCallback()));
    return true;
}

/*****************************************************************************************/

TaskList::TaskList(const Profile* profile):
_profile( profile )
{
}

bool TaskList::load( const std::string &filename)
{          
    std::ifstream in( filename.c_str(), std::ios::in );

    std::string line;
    while( getline(in, line) )
    {            
        std::vector< std::string > parts;
        StringTokenizer(line, parts, "," );

        if (parts.size() >= 3)
        {
            _keys.push_back( TileKey(
                as<unsigned int>(parts[0], 0u), 
                as<unsigned int>(parts[1], 0u), 
                as<unsigned int>(parts[2], 0u),
                _profile.get() ) );
        }
    }


    return true;
}

void TaskList::save( const std::string& filename )
{        
    std::ofstream out( filename.c_str() );
    for (TileKeyList::iterator itr = _keys.begin(); itr != _keys.end(); ++itr)
    {
        out << (*itr).getLevelOfDetail() << ", " << (*itr).getTileX() << ", " << (*itr).getTileY() << std::endl;
    }
}

TileKeyList& TaskList::getKeys()
{
    return _keys;
}

const TileKeyList& TaskList::getKeys() const
{
    return _keys;
}
