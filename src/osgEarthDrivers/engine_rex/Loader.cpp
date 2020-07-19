/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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
#include "Loader"
#include "RexTerrainEngineNode"

#include <osgEarth/Registry>
#include <osgEarth/Utils>
#include <osgEarth/NodeUtils>
#include <osgEarth/Metrics>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>

#include <string>


#define REPORT_ACTIVITY true

using namespace osgEarth::REX;


Loader::Request::Request() :
    _delay_s(0.0),
    _readyTick(0.0),
    _delayCount(0),
    _mutex("Request(OE)")
{
    _uid = osgEarth::Registry::instance()->createUID();
    sprintf(_filename, "%u.osgearth_rex_loader", _uid);
    setState(IDLE);
}

void
Loader::Request::setState(Loader::Request::State value)
{
    _state = value;
    _stateTick = osg::Timer::instance()->tick();

    if ( _state == IDLE )
    {
        _loadCount = 0;
        _priority = 0;
        _lastFrameSubmitted = 0;
        _lastTick = 0;
    }
}

void
Loader::Request::setDelay(double seconds)
{
    osg::Timer_t now = osg::Timer::instance()->tick();
    _readyTick = now +
        (osg::Timer_t)(seconds / osg::Timer::instance()->getSecondsPerTick());

    ++_delayCount;

    //OE_WARN << _key.str() << "setDelay(" << osg::Timer::instance()->delta_s(now, _readyTick) << ")" << std::endl;
}

namespace osgEarth { namespace REX
{
    /**
     * Custom progress callback that checks for both request 
     * timeout (via request::isIdle) and OSG database pager
     * shutdown (via getDone)
     */
    struct RequestProgressCallback : public DatabasePagerProgressCallback
    {
        osg::ref_ptr<Loader> _loader;
        Loader::Request* _request;

        RequestProgressCallback(Loader::Request* req, Loader* loader) :
            DatabasePagerProgressCallback(),
            _request(req),
            _loader(loader)
        {
            //NOP
        }

        virtual bool shouldCancel() const
        {
            bool should = 
                (!_loader->isValid(_request)) ||
                (!_request->isRunning()) ||
                (DatabasePagerProgressCallback::shouldCancel());

            if (should)
            {
                OE_DEBUG << "REX: canceling load on thread " << std::this_thread::get_id() << std::endl;
            }

            return should;
        }
    };
} }

//...............................................

#undef  LC
#define LC "[SimpleLoader] "

SimpleLoader::SimpleLoader()
{
}

bool
SimpleLoader::load(Loader::Request* request, float priority, osg::NodeVisitor& nv)
{
    if ( request )
    {
        // take a reference, which will cause an unref after load.
        osg::ref_ptr<Request> r = request;

        r->setState(Request::RUNNING);

        //OE_INFO << LC << "Request invoke : UID = " << request->getUID() << "\n";
        request->run(0L);
        
        //OE_INFO << LC << "Request apply : UID = " << request->getUID() << "\n";
        if (r->isRunning())
        {
            // no re-queue possible. Testing only.
            request->setState(Request::MERGING);
            request->merge();
        }

        r->setState(Request::FINISHED);
    }
    return request != 0L;
}

void
SimpleLoader::clear()
{
    // nop.
}

//...............................................

#undef  LC
#define LC "[PagerLoader.FileLocationCallback] "

namespace
{ 
    class FileLocationCallback : public osgDB::FileLocationCallback
    {
    public:
        FileLocationCallback() { }

        /** dtor */
        virtual ~FileLocationCallback() { }

        Location fileLocation(const std::string& filename, const osgDB::Options* dboptions)
        {
            Location result = REMOTE_FILE;

            if (dboptions)
            {
                osgEarth::UID requestUID;

                const RexTerrainEngineNode* engine = dynamic_cast<const RexTerrainEngineNode*>(
                    osg::getUserObject(dboptions, "osgEarth.RexTerrainEngineNode"));

                sscanf(filename.c_str(), "%d", &requestUID);

                if ( engine )
                {
                    PagerLoader* loader = dynamic_cast<PagerLoader*>( engine->getLoader() );
                    if ( loader )
                    {
                        TileKey key = loader->getTileKeyForRequest(requestUID);

                        const Map* map = engine->getMap();
                        if (map)
                        {
                            LayerVector layers;
                            map->getLayers(layers);
                            if (map->isFast(key, layers))
                            {
                                result = LOCAL_FILE;
                            }
                        }
                    }

                    //OE_NOTICE << "key=" << key.str() << " : " << (result==LOCAL_FILE?"local":"remote") << "\n";
                }
            }

            return result;
        }

        bool useFileCache() const { return false; }
    };
}

//...............................................

#undef  LC
#define LC "[PagerLoader] "

namespace
{
    class RequestResultNode : public osg::Node
    {
    public:
        RequestResultNode(Loader::Request* request)
            : _request(request)
        {
            // Do this so the pager/ICO can find and pre-compile GL objects that are
            // attached to the stateset.
            if ( _request.valid() )
            {
                // TODO: for some reason pre-compiling is causing texture flashing issues 
                // with things like classification maps when using --ico. Figure out why.
                setStateSet( _request->createStateSet() );
            }
        }

        Loader::Request* getRequest() const { return _request.get(); }

        osg::ref_ptr<Loader::Request> _request;
    };
}


PagerLoader::PagerLoader(TerrainEngineNode* engine) :
_checkpoint    (0.0),
_mergesPerFrame( 0 ),
_frameLastUpdated( 0u ),
_numLODs       ( 20u ),
_requests(OE_MUTEX_NAME)
{
    _myNodePath.push_back( this );

    _dboptions = new osgDB::Options();

    _dboptions->setFileLocationCallback( new FileLocationCallback() );

    OptionsData<PagerLoader>::set(_dboptions.get(), "osgEarth.PagerLoader", this);

    // initialize the LOD priority scales and offsets
    for (unsigned i = 0; i < 64; ++i)
    {
        _priorityScales[i] = 1.0f;
        _priorityOffsets[i] = 0.0f;
    }
}

void
PagerLoader::setNumLODs(unsigned lods)
{
    _numLODs = osg::maximum(lods, 1u);
}

void
PagerLoader::setMergesPerFrame(int value)
{
    _mergesPerFrame = osg::maximum(value, 0);
    //ADJUST_EVENT_TRAV_COUNT(this, +1);
    ADJUST_UPDATE_TRAV_COUNT(this, +1);
    OE_DEBUG << LC << "Merges per frame = " << _mergesPerFrame << std::endl;
    
}

void
PagerLoader::setLODPriorityScale(unsigned lod, float priorityScale)
{
    if (lod < 64)
        _priorityScales[lod] = priorityScale;
}

void
PagerLoader::setLODPriorityOffset(unsigned lod, float offset)
{
    if (lod < 64)
        _priorityOffsets[lod] = offset;
}

void
PagerLoader::setOverallPriorityScale(float value)
{
    for(int i=0; i<64; ++i)
    {
        _priorityScales[i] = value;
    }
}

bool
PagerLoader::load(Loader::Request* request, float priority, osg::NodeVisitor& nv)
{
    osg::Timer_t now = osg::Timer::instance()->tick();

    // check that the request is not already completed but unmerged:
    if ( request && !request->isMerging() && !request->isFinished() && nv.getDatabaseRequestHandler() )
    {
        bool addToRequestSet = false;

        // lock the request since multiple cull traversals might hit this function.
        request->lock();
        {
            // remember the last tick at which this request was submitted
            request->_lastTick = _clock->getTime();

            // update the priority, scale and bias it, and then normalize it to [0..1] range.
            unsigned lod = request->getTileKey().getLOD();
            float p = priority * _priorityScales[lod] + _priorityOffsets[lod];
            request->_priority = p / (float)(_numLODs+1);

            // timestamp it
            request->setFrameNumber(_clock->getFrame());

            // increment the load count.
            request->_loadCount++;

            // if this is the first load request since idle, we need to remember this request.
            addToRequestSet = (request->_loadCount == 1);
        }
        request->unlock();

        // is this request eligible to run (based on a possible setDelay call)?
        if (now >= request->_readyTick)
        {
            nv.getDatabaseRequestHandler()->requestNodeFile(
                request->_filename,
                _myNodePath,
                request->_priority,
                nv.getFrameStamp(),
                request->_internalHandle,
                _dboptions.get() );
        }

        // remember the request:
        //if ( addToRequestSet )
        {
            _requests.lock();
            _requests[request->getUID()] = request;
            _requests.unlock();
        }

        return true;
    }
    return false;
}

void
PagerLoader::clear()
{
    // Set a time checkpoint for invalidating old requests.
    _checkpoint = _clock->getTime();
}

bool
PagerLoader::isValid(Request* req) const
{
    return 
        req && 
        req->_lastTick >= _checkpoint;
}

void
PagerLoader::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.UPDATE_VISITOR )
    {
        // prevent UPDATE from running more than once per frame
        unsigned frame = _clock->getFrame();
        bool runUpdate = (_frameLastUpdated < frame);

        if (runUpdate)
        {
            _frameLastUpdated = frame;

            // process pending merges.
            {
                OE_PROFILING_ZONE_NAMED("loader.merge");
                int count;
                for(count=0; count < _mergesPerFrame && !_mergeQueue.empty(); ++count)
                {
                    Request* req = _mergeQueue.begin()->get();
                    if ( req && req->_lastTick >= _checkpoint )
                    {
                        bool merged = req->merge();
                    
                        if (merged)
                        {
                            req->setState(Request::FINISHED);
                            //OE_INFO << LC << req->_key.str() << " finished (delays = " << req->_delayCount << ")" << std::endl;
                        }
                        else
                        {
                            // if apply() returns false, that means the results were invalid
                            // for some reason (probably revision mismatch) and the request
                            // must be requeued.
                            req->setState(Request::IDLE);
                        }
                    }

                    _mergeQueue.erase( _mergeQueue.begin() );
                }
            }

            // cull finished requests.
            {
                OE_PROFILING_ZONE("loader.purge");

                unsigned frame = _clock->getFrame();

                _requests.lock();

                // Purge expired requests.
                for(Requests::iterator i = _requests.begin(); i != _requests.end(); )
                {
                    Request* req = i->second.get();
                    const int frameDiff = (int)frame - (int)req->getLastFrameSubmitted();

                    // Deal with completed requests:
                    if ( req->isFinished() )
                    {
                        //OE_INFO << LC << req->getName() << "(" << i->second->getUID() << ") finished." << std::endl; 
                        if ( REPORT_ACTIVITY )
                            Registry::instance()->endActivity( req->getName() );
                        _requests.erase( i++ );
                    }

                    // Discard requests from tiles that have not pinged the loader in the last couple frames.
                    // This typically means a request was initiated, but then abandoned when the tile
                    // went out of view.
                    else if ( !req->isMerging() && frameDiff > 2 )
                    {
                        OE_DEBUG << LC << req->getName() << "(" << i->second->getUID() << ") was abandoned waiting to be serviced" << std::endl; 
                        req->setState(Request::IDLE);
                        if ( REPORT_ACTIVITY )
                            Registry::instance()->endActivity( req->getName() );
                        _requests.erase( i++ );
                    }

#if 0
                    // Prevent a request from getting stuck in the merge queue:
                    else if ( req->isMerging() && frameDiff > 1800 )
                    {
                        OE_INFO << LC << req->getName() << "(" << i->second->getUID() << ") was abandoned waiting to be merged" << std::endl; 
                        //req->setState( Request::ABANDONED );
                        req->setState(Request::IDLE);
                        if ( REPORT_ACTIVITY )
                            Registry::instance()->endActivity( req->getName() );
                        _requests.erase( i++ );
                    }
#endif

                    else // still valid.
                    {
                        ++i;
                    }
                }

                _requests.unlock();

                //OE_NOTICE << LC << "PagerLoader: requests=" << _requests.size() << "; mergeQueue=" << _mergeQueue.size() << std::endl;
            }
        }
    }

    Loader::traverse( nv );
}


bool
PagerLoader::addChild(osg::Node* node)
{
    osg::ref_ptr<RequestResultNode> result = dynamic_cast<RequestResultNode*>(node);
    if ( result.valid() )
    {
        Request* req = result->getRequest();
        if ( req )
        {
            if (req->_lastTick < _checkpoint)
            {
                // allow it to complete and disappear.
                req->setState(Request::FINISHED);
                if ( REPORT_ACTIVITY )
                    Registry::instance()->endActivity( req->getName() );
            }

            // Make sure the request is both current (newer than the last checkpoint)
            // and running (i.e. has not been canceled along the way)
            else if (req->isRunning())
            {
                if ( _mergesPerFrame > 0 )
                {
                    _mergeQueue.insert( req );
                    req->setState( Request::MERGING );
                }
                else
                {
                    if (req->merge())
                        req->setState( Request::FINISHED );
                    else
                        req->setState( Request::IDLE ); // retry

                    if ( REPORT_ACTIVITY )
                        Registry::instance()->endActivity( req->getName() );
                }
            }                

            else
            {
                OE_WARN << LC << "Request " << req->getName() << " abandoned (in addChild)" << std::endl;
                //GW: allow to requeue (leave idle)
                //req->setState( Request::FINISHED );
                if ( REPORT_ACTIVITY )
                    Registry::instance()->endActivity( req->getName() );
            }
        }
    }

    else
    {
        //OE_WARN << LC << "Internal error: illegal node type in addchild" << std::endl;
    }
    return true;
}

TileKey
PagerLoader::getTileKeyForRequest(UID requestUID) const
{
    TileKey result;

    _requests.lock();
    Requests::const_iterator i = _requests.find( requestUID );
    if ( i != _requests.end() )
    {
        result = i->second->getTileKey();
    }
    _requests.unlock();

    return result;
}

Loader::Request*
PagerLoader::runAndRelease(UID requestUID)
{
    osg::ref_ptr<Request> request;

    _requests.lock();
    Requests::iterator i = _requests.find( requestUID );
    if ( i != _requests.end() )
    {
        request = i->second.get();
    }
    _requests.unlock();

    if ( request.valid() )
    {
        if ( REPORT_ACTIVITY )
            Registry::instance()->startActivity( request->getName() );

        request->setState(Request::RUNNING);

        osg::ref_ptr<ProgressCallback> prog = new RequestProgressCallback(request.get(), this);

        if (request->run(prog.get()) == false)
        {
            request->setState(Request::IDLE);
        }
    }

    else
    {
        // If request is NULL, that means that the Pager dispatched a request that 
        // has already died on the vine.
        //OE_WARN << LC << "Internal: invokeAndRelease (" << requestUID << ") not found." << std::endl;
    }

    return request.release();
}



namespace osgEarth { namespace REX
{
    using namespace osgEarth;

    /** Registry Plugin Agent. */
    struct PagerLoaderAgent : public osgDB::ReaderWriter
    {
        PagerLoaderAgent()
        {
            //nop
        }

        virtual const char* className() const
        {
            return "osgEarth REX Loader Agent";
        }

        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive( extension, "osgearth_rex_loader" );
        }

        ReadResult readNode(const std::string& uri, const osgDB::Options* dboptions) const
        {
            std::string ext = osgDB::getFileExtension(uri);
            if ( acceptsExtension(ext) )
            {
                OE_SCOPED_THREAD_NAME("DBPager", "REX");

                // parse the tile key and engine ID:
                std::string requestdef = osgDB::getNameLessExtension(uri);
                unsigned requestUID;
                sscanf(requestdef.c_str(), "%u", &requestUID);

                osg::ref_ptr<PagerLoader> loader;
                if (OptionsData<PagerLoader>::lock(dboptions, "osgEarth.PagerLoader", loader))
                {
                    osg::ref_ptr<Loader::Request> req = loader->runAndRelease(requestUID);

                    // make sure the request is still running (not canceled)
                    if (req.valid() && req->isRunning())
                        return new RequestResultNode(req.release());
                    else
                        return ReadResult::FILE_LOADED; // fail silenty (cancelation)
                }

                // fail silently - this could happen if the Loader disappears from
                // underneath, if say the terrain is destroyed
                return ReadResult::FILE_LOADED;
            }
            else
            {
                return ReadResult::FILE_NOT_HANDLED;
            }
        };
    };
    REGISTER_OSGPLUGIN(osgearth_rex_loader, PagerLoaderAgent);

} } // namespace osgEarth::REX
