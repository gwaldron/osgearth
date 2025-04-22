/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/TileVisitor>
#include <thread>

#include <osg/os_utils>
#define OS_SYSTEM osg_system

using namespace osgEarth;
using namespace osgEarth::Util;

TileVisitor::TileVisitor() :
    _total(0),
    _processed(0),
    _minLevel(0),
    _maxLevel(99),
    _lastProgressUpdate(std::chrono::steady_clock::now())
{
}


TileVisitor::TileVisitor(TileHandler* handler) :
    _tileHandler(handler),
    _total(0),
    _processed(0),
    _minLevel(0),
    _maxLevel(99),
    _lastProgressUpdate(std::chrono::steady_clock::now())
{
}

void TileVisitor::resetProgress()
{
    _total = 0;
    _processed = 0;
}

void
TileVisitor::addExtentToVisit( const GeoExtent& extent )
{
    _extentsToVisit.push_back( extent );
}

void
TileVisitor::addExtentToDataIndex(const GeoExtent& extent)
{
    double min[2] = { extent.xMin(), extent.yMin() };
    double max[2] = { extent.xMax(), extent.yMax() };
    _dataExtentIndex.Insert(min, max, _dataExtentIndex.Count());
}

bool TileVisitor::intersects( const GeoExtent& extent )
{
    for (auto& e : _extentsToVisit)
    {
        if (e.intersects(extent))
        {
            return true;
        }
    }

    return false;
}

bool TileVisitor::hasData(const TileKey& key)
{
    GeoExtent extent = key.getExtent();

    // Check the data extents index to see if we might have data in this area.
    if (_dataExtentIndex.Count() > 0)
    {
        double min[2] = { extent.xMin(), extent.yMin() };
        double max[2] = { extent.xMax(), extent.yMax() };
        std::vector< unsigned int > hits;
        auto stop_on_any_hit = [](const unsigned&) { return RTREE_STOP_SEARCHING; }; // stop on any hit
        if (_dataExtentIndex.Search(min, max, stop_on_any_hit) == 0)
        {
            return false;
        }
    }

    // Check the tile handler
    if (_tileHandler.valid())
    {
        return _tileHandler->hasData(key);
    }

    return true;
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
    if (_tileHandler.valid())
    {
        _total = _tileHandler->getEstimatedTileCount(
            _extentsToVisit,
            _minLevel,
            _maxLevel);
    }
    else
    {
        _total = 0;
    }
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
    if (lod >= getMinLevel() && !hasData(key))
    {
        return;
    }

    bool traverseChildren = false;

    // If the key intersects the extent attempt to traverse
    if (intersects(key.getExtent()))
    {
        // If the lod is less than the min level don't do anything but do traverse the children.
        if (lod < _minLevel)
        {
            traverseChildren = true;
        }
        else
        {
            // Process the key
            traverseChildren = handleTile(key);
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
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - _lastProgressUpdate).count();    
    bool shouldReportProgress = false;

    {
        std::lock_guard<std::mutex> lk(_progressMutex );
        _processed += amount;
        if (elapsed >= 5 || _processed >= _total)
        {           
            _lastProgressUpdate = now;
            shouldReportProgress = true;
        }
    }

    if (_progress.valid())
    {        
        if (shouldReportProgress)
        {   
            // If report progress returns true then mark the task as being cancelled.
            if (_progress->reportProgress( _processed, _total ))
            {
                _progress->cancel();
            }
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

MultithreadedTileVisitor::MultithreadedTileVisitor() :
    _numThreads(std::max(1u, std::thread::hardware_concurrency()))
{
    // We must do this to avoid an error message in OpenSceneGraph b/c the findWrapper method doesn't appear to be threadsafe.
    // This really isn't a big deal b/c this only effects data that is already cached.
    osgDB::ObjectWrapper* wrapper = osgDB::Registry::instance()->getObjectWrapperManager()->findWrapper("osg::Image");

    _group = jobs::jobgroup::create();
}

MultithreadedTileVisitor::MultithreadedTileVisitor(TileHandler* handler) :
    TileVisitor(handler),
    _numThreads(std::max(1u, std::thread::hardware_concurrency()))
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

#define MTTV "oe.mttilevisitor"

void MultithreadedTileVisitor::run(const Profile* mapProfile)
{
    // Start up the task service
    OE_DEBUG << "Starting " << _numThreads << " threads " << std::endl;

    jobs::get_pool(MTTV)->set_concurrency(_numThreads);

    // Produce the tiles
    TileVisitor::run( mapProfile );

    _group->join();
}

bool MultithreadedTileVisitor::handleTile(const TileKey& key)
{
    // atomically increment the task count
    //_numTiles++;

    // don't let the task queue get too large...?    
    while(jobs::get_metrics()->total_pending() > 1000)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Add the tile to the task queue.
    auto task = [this, key]()
    {
        if ((_tileHandler.valid()) && (!_progress.valid() || !_progress->isCanceled()))
        {
            _tileHandler->handleTile(key, *this);
            this->incrementProgress(1);
        }
    };

    jobs::context job;
    job.name = "handleTile";
    job.pool = jobs::get_pool(MTTV);
    job.group = _group;

    jobs::dispatch(task, job);

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
        auto parts = StringTokenizer()
            .delim(",")
            .standardQuotes()
            .tokenize(line);

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
