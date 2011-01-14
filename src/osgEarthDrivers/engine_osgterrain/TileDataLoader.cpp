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
#include "TileDataLoader"
#include "CustomTerrain"
#include <osg/Timer>

#define LC "[TileDataLoader] "

//#define TEST_RUN_SYNC 1
#define TEST_BLOCK_ON_REQUESTS 1
//#define TEST_SPIN_WAIT 1

using namespace osgEarth;

//++++++++++++++ TODO +++++++++++++++++

//  -- get rid of expired jobs ..
//  -- fix cancelation (stamp should be updated every frame)

//------------------------------------------------------------------------

struct SignalingProgressCallback : public ProgressCallback
{
    SignalingProgressCallback( Threading::Event* ev ) : _ev(ev) { }

    void onCompleted() {
        _ev->set();
    }

    Threading::Event* _ev;
};

//------------------------------------------------------------------------

namespace
{
    struct TileImageRequest : public TaskRequest
    {
        TileImageRequest( const TileKey& key, const MapFrame& mapf, UID layerUID, OSGTileFactory* tf )
            : _key(key), _mapf(mapf), _layerUID(layerUID), _factory(tf) { }
        
        void operator()( ProgressCallback* progress )
        {
            _result = _factory->createImageLayer(
                _mapf.getMapInfo(), 
                _mapf.getImageLayerByUID( _layerUID ),
                _key, 
                progress );
        }

        TileKey _key;
        const MapFrame& _mapf;
        UID _layerUID;
        OSGTileFactory* _factory;
    };

    struct TileHeightFieldRequest : public TaskRequest
    {
        TileHeightFieldRequest( const TileKey& key, const MapFrame& mapf, UID layerUID, OSGTileFactory* tf )
            : _key(key), _mapf(mapf), _layerUID(layerUID), _factory(tf) { }

        void operator()( ProgressCallback* progress )
        {
            _result = _factory->createHeightFieldLayer( _mapf, _key, true );
        }

        TileKey _key;
        const MapFrame& _mapf;
        UID _layerUID;
        OSGTileFactory* _factory;
    };
}

//------------------------------------------------------------------------

TileJob::TileJob(const TileKey& key, const MapFrame& mapf,
                 OSGTileFactory* factory, TaskService* service ) :
_key(key),
_mapf(mapf),
_factory(factory),
_service(service)
{
    //NOP    
}

void
TileJob::start( ProgressCallback* progress )
{
    _startTime = osg::Timer::instance()->tick();

    for( ImageLayerVector::const_iterator i = _mapf.imageLayers().begin(); i != _mapf.imageLayers().end(); ++i )
    {
        TaskRequest* r = new TileImageRequest( _key, _mapf, i->get()->getUID(), _factory.get() );
        std::stringstream buf;
        buf << _key.str() << ":" << i->get()->getUID();
        r->setName(buf.str());
        r->setProgressCallback( progress );
        r->setPriority( -(float)_key.getLevelOfDetail() );
        _requests.push_back( r );
#ifdef TEST_RUN_SYNC
        (*r)(0L);
        r->setState( TaskRequest::STATE_COMPLETED );
#else       
        _service->add( r );
#endif
    }
    
    for( ElevationLayerVector::const_iterator i = _mapf.elevationLayers().begin(); i != _mapf.elevationLayers().end(); ++i )
    {
        TaskRequest* r = new TileHeightFieldRequest( _key, _mapf, i->get()->getUID(), _factory.get() );
        std::stringstream buf;
        buf << _key.str() << ":" << i->get()->getUID();
        r->setName(buf.str());
        r->setProgressCallback( progress );
        r->setPriority( -(float)_key.getLevelOfDetail() );
        _requests.push_back( r );
#ifdef TEST_RUN_SYNC
        (*r)(0L);
        r->setState( TaskRequest::STATE_COMPLETED );
#else
        _service->add( r );
#endif
    }
}

double
TileJob::runTime() const
{
    osg::Timer_t endTime = _startTime;
    for( TaskRequestList::const_iterator i = _requests.begin(); i != _requests.end(); ++i )
        if ( i->get()->endTime() > endTime )
            endTime = i->get()->endTime();
    return osg::Timer::instance()->delta_s(_startTime, endTime);
}

bool
TileJob::isCompleted() const
{
    for( TaskRequestList::const_iterator i = _requests.begin(); i != _requests.end(); ++i )
        if ( i->get()->isCompleted() == false )
            return false;
    return true;
}

bool
TileJob::isSuccessful() const
{
    for( TaskRequestList::const_iterator i = _requests.begin(); i != _requests.end(); ++i )
        if ( i->get()->wasCanceled() )
            return false;
    return true;
}

void
TileJob::populateTile( CustomTile* tile )
{
    for( TaskRequestList::const_iterator i = _requests.begin(); i != _requests.end(); ++i )
    {
        if ( dynamic_cast<TileImageRequest*>( i->get() ) )
        {
            CustomColorLayerRef* layerRef = static_cast<CustomColorLayerRef*>( i->get()->getResult() );
            if ( layerRef )
                tile->setCustomColorLayer( layerRef->_layer );
        }
        else //if ( dynamic_cast<TileLoadElevationRequest*>( i->get() ) )
        {
            osgTerrain::HeightFieldLayer* layer = static_cast<osgTerrain::HeightFieldLayer*>( i->get()->getResult() );
            tile->setElevationLayer( layer );
        }
    }

    if ( tile->getElevationLayer() == 0L )
    {
        osgTerrain::HeightFieldLayer* hflayer = new osgTerrain::HeightFieldLayer(
            _factory->createEmptyHeightField( tile->getKey(), 8, 8 ));
        hflayer->setLocator( GeoLocator::createForKey(tile->getKey(), _mapf.getMapInfo()) );
        tile->setElevationLayer( hflayer );
    }
}

//------------------------------------------------------------------------

TileGroupJob::TileGroupJob(const TileKey& parentKey, const Map* map,
                           OSGTileFactory* factory, TaskService* service ) :
_mapf(map)
{
    for( unsigned i=0; i<4; ++i )
    {
        TileKey childKey = parentKey.createChildKey( i );
        _tileJobs[i] = new TileJob( childKey, _mapf, factory, service );
    }
}

void
TileGroupJob::start()
{
    _startTime = osg::Timer::instance()->tick();

    // make a progress that will signal our event when a request completes.
    ProgressCallback* progress = new SignalingProgressCallback( &_completionEvent );

    for( unsigned i=0; i<4; ++i )
        _tileJobs[i]->start( progress );
}

double
TileGroupJob::runTime() const
{
    double t = 0.0;
    for( unsigned i=0; i<4; ++i )
    {
        double jobRunTime = _tileJobs[i]->runTime();
        if ( jobRunTime > t )
            t = jobRunTime;
    }
    return t;
}

bool
TileGroupJob::isCompleted() const
{
    for( unsigned i=0; i<4; ++i )
    {
        if ( !_tileJobs[i]->isCompleted() )
            return false;
    }
    return true;
}

bool
TileGroupJob::isSuccessful() const
{
    for( unsigned i=0; i<4; ++i )
    {
        if ( !_tileJobs[i]->isSuccessful() )
            return false;
    }
    return true;
}

//------------------------------------------------------------------------

TileDataLoader::TileDataLoader( const Map* map, CustomTerrain* terrain ) :
_map( map ),
_terrain( terrain ),
_stamp( 0 )
{
    _service = new TaskService( "TileDataLoader", 16 );
}

static int s_count = 0;
static double s_time = 0.0;

osgDB::ReaderWriter::ReadResult
TileDataLoader::loadSubTileGroup( const TileKey& key )
{
    // fist, check to see if there's already a request in place for this tile.
    TileLoaderTicket* ticket =0L;
    {
        Threading::ScopedReadLock lock( _ticketsMutex );
        TileLoaderTickets::iterator i = _tickets.find( key );
        if ( i != _tickets.end() )
        {
            ticket = &i->second;
        }
    }

    if ( ticket )
    {
        osg::ref_ptr<TileGroupJob> job = ticket->_job.get();

        // sit around and wait for the entire job to complete
        while( !job->isCompleted() )
        {
            job->_completionEvent.waitAndReset();
        }

        // remove its ticket from the lookup table.
        {
            Threading::ScopedWriteLock exclusiveLock( _ticketsMutex );
            _tickets.erase( key );
        }

        // the request is complete. But that doesn't mean it succeeded, so check
        // the status of that too.
        if ( job->isSuccessful() )
        {        
            MapInfo mapInfo(_map);

            osg::Group* result = new osg::Group();

            for( unsigned i=0; i<4; ++i )
            {
                TileJob* tileJob = job->_tileJobs[i].get();

                GeoLocator* locator = GeoLocator::createForKey( tileJob->_key, mapInfo );
                CustomTile* tile = new CustomTile( tileJob->_key, locator, _terrain->getQuickReleaseGLObjects() );
                tileJob->populateTile( tile );
                osg::Node* preparedTile = _terrain->getTileFactory()->prepareTile( tile, _terrain, mapInfo, true );
                result->addChild( preparedTile );
            }

            s_time += job->runTime();
            s_count++;
            OE_INFO << LC << "Average tile time = " << s_time/(double)s_count << " s." << std::endl;

            return osgDB::ReaderWriter::ReadResult( result );
        }
        else
        {
            return osgDB::ReaderWriter::ReadResult::FILE_NOT_FOUND;
        }
    }

    // no request exists, so make a new one and return a status saying that
    // the request is in progress.
    else
    {
        TileLoaderTicket ticket;
        ticket._job = new TileGroupJob( key, _map, _terrain->getTileFactory(), _service.get() );
        ticket._timestamp = DBL_MAX;
        {
            Threading::ScopedWriteLock exclusiveLock( _ticketsMutex );
            _tickets[key] = ticket;
            //OE_INFO << LC << "Queued request for key " << key.str() << std::endl;
        }
        ticket._job->start();
        
        return loadSubTileGroup(key);
    }
}

//------------------------------------------------------------------------
