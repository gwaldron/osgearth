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
#include <osgEarth/TileVisitor>
#include <osgEarth/CacheEstimator>
#include <osgEarth/FileUtils>

using namespace osgEarth;

TileVisitor::TileVisitor():
_total(0),
_processed(0),
_minLevel(0),
_maxLevel(5)
{
}


TileVisitor::TileVisitor(TileHandler* handler):
_tileHandler( handler ),
_total(0),
_processed(0),
_minLevel(0),
_maxLevel(5)
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
    est.setProfile( _profile ); 
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
    if (_tileHandler && !_tileHandler->hasData(key))
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
/**
 * A TaskRequest that runs a TileHandler in a background thread.
 */
class HandleTileTask : public TaskRequest
{
public:
    HandleTileTask( TileHandler* handler, TileVisitor* visitor, const TileKey& key ):      
      _handler( handler ),
          _visitor(visitor),
          _key( key )
      {

      }

      virtual void operator()(ProgressCallback* progress )
      {         
          if (_handler.valid())
          {                           
              _handler->handleTile( _key, *_visitor.get() );
              _visitor->incrementProgress(1);
          }
      }

      osg::ref_ptr<TileHandler> _handler;
      TileKey _key;
      osg::ref_ptr<TileVisitor> _visitor;
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
    OE_INFO << "Starting " << _numThreads << std::endl;
    _taskService = new TaskService( "MTTileHandler", _numThreads, 100000 );

    // Produce the tiles
    TileVisitor::run( mapProfile );

    // Send a poison pill to kill all the threads
    _taskService->add( new PoisonPill() );

    OE_INFO << "Waiting on threads to complete" << _taskService->getNumRequests() << " tasks remaining" << std::endl;

    // Wait for everything to finish, checking for cancellation while we wait so we can kill all the existing tasks.
    while (_taskService->areThreadsRunning())
    {
        OpenThreads::Thread::microSleep(10000);
        if (_progress && _progress->isCanceled())
        {            
            _taskService->cancelAll();
        }
    }
    OE_INFO << "All threads have completed" << std::endl;
}

bool MultithreadedTileVisitor::handleTile( const TileKey& key )        
{    
    // Add the tile to the task queue.
    _taskService->add( new HandleTileTask(_tileHandler, this, key ) );
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


        _keys.push_back( TileKey(as<unsigned int>(parts[0], 0), 
            as<unsigned int>(parts[1], 0), 
            as<unsigned int>(parts[2], 0),
            _profile ) );
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

/*****************************************************************************************/
MultiprocessTileVisitor::MultiprocessTileVisitor():
    _numProcesses( OpenThreads::GetNumberOfProcessors() ),
    _batchSize(100)
{
    osgDB::ObjectWrapper* wrapper = osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper( "osg::Image" );
}

MultiprocessTileVisitor::MultiprocessTileVisitor( TileHandler* handler ):
TileVisitor( handler ),
    _numProcesses( OpenThreads::GetNumberOfProcessors() ),
    _batchSize(100)
{
}

unsigned int MultiprocessTileVisitor::getNumProcesses() const
{
    return _numProcesses; 
}

void MultiprocessTileVisitor::setNumProcesses( unsigned int numProcesses)
{
    _numProcesses = numProcesses; 
}

unsigned int MultiprocessTileVisitor::getBatchSize() const
{
    return _batchSize;
}

void MultiprocessTileVisitor::setBatchSize( unsigned int batchSize )
{
    _batchSize = batchSize;
}


void MultiprocessTileVisitor::run(const Profile* mapProfile)
{                             
    // Start up the task service          
    _taskService = new TaskService( "MPTileHandler", _numProcesses, 100000 );
    
    // Produce the tiles
    TileVisitor::run( mapProfile );

    // Process any remaining tasks in the final batch
    processBatch();

    // Send a poison pill to kill all the threads
    _taskService->add( new PoisonPill() );
   
    OE_INFO << "Waiting on threads to complete" << _taskService->getNumRequests() << " tasks remaining" << std::endl;

    // Wait for everything to finish, checking for cancellation while we wait so we can kill all the existing tasks.
    while (_taskService->areThreadsRunning())
    {
        OpenThreads::Thread::microSleep(10000);
        if (_progress && _progress->isCanceled())
        {            
            _taskService->cancelAll();
        }
    }
    OE_INFO << "All threads have completed" << std::endl;
}

bool MultiprocessTileVisitor::handleTile( const TileKey& key )        
{        
    _batch.push_back( key );

    if (_batch.size() == _batchSize)
    {
        processBatch();
    }         
    return true;
}

const std::string& MultiprocessTileVisitor::getEarthFile() const
{
    return _earthFile;
}

void MultiprocessTileVisitor::setEarthFile( const std::string& earthFile )
{
    _earthFile = earthFile;
}

/**
* Executes a command in an external process
*/
class ExecuteTask : public TaskRequest
{
public:
    ExecuteTask(const std::string& command, TileVisitor* visitor, unsigned int count):            
      _command( command ),
      _visitor( visitor ),
      _count( count )
      {
      }

      virtual void operator()(ProgressCallback* progress )
      {         
          system(_command.c_str());     

          // Cleanup the temp files and increment the progress on the visitor.
          cleanupTempFiles();
          _visitor->incrementProgress( _count );
      }

      void addTempFile( const std::string& filename )
      {
          _tempFiles.push_back(filename);
      }

      void cleanupTempFiles()
      {
          for (unsigned int i = 0; i < _tempFiles.size(); i++)
          {
              remove( _tempFiles[i].c_str() );
          }
      }


      std::vector< std::string > _tempFiles;
      std::string _command;
      TileVisitor* _visitor;
      unsigned int _count;
};

void MultiprocessTileVisitor::processBatch()
{       
    TaskList tasks( 0 );
    for (unsigned int i = 0; i < _batch.size(); i++)
    {
        tasks.getKeys().push_back( _batch[i] );
    }
    // Save the task file out.
    std::string tmpPath = getTempPath();
    std::string filename = getTempName(tmpPath, "batch.tiles");        
    tasks.save( filename );        

    std::stringstream command;        
    command << _tileHandler->getProcessString() << " --tiles " << filename << " " << _earthFile;
    OE_INFO << "Running command " << command.str() << std::endl;
    osg::ref_ptr< ExecuteTask > task = new ExecuteTask( command.str(), this, tasks.getKeys().size() );
    // Add the task file as a temp file to the task to make sure it gets deleted
    task->addTempFile( filename );

    _taskService->add(task);
    _batch.clear();
}


/*****************************************************************************************/
TileKeyListVisitor::TileKeyListVisitor()
{
}

void TileKeyListVisitor::setKeys(const TileKeyList& keys)
{
    _keys = keys;
}

void TileKeyListVisitor::run(const Profile* mapProfile)
{
    resetProgress();        

    for (TileKeyList::iterator itr = _keys.begin(); itr != _keys.end(); ++itr)
    {
        if (_tileHandler)
        {
            _tileHandler->handleTile( *itr, *this );
            incrementProgress(1);
        }
    }
}
