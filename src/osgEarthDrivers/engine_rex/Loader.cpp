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


Loader::Request::Request()
{
    _uid = osgEarth::Registry::instance()->createUID();
    _state = IDLE;
    _loadCount = 0;
    _priority = 0;
    _lastFrameSubmitted = 0;
    _lastTick = 0;
}

void
Loader::Request::addToChangeSet(osg::Node* node)
{
    if ( node )
    {
        _nodesChanged.push_back( node );
    }
}

namespace osgEarth { namespace REX
{
    /**
     * Custom progress callback that checks for both request 
     * timeout (via request::isIdle) and OSG database pager
     * shutdown (via getDone)
     */
    struct RequestProgressCallback : public ProgressCallback
    {
        osgDB::DatabasePager::DatabaseThread* _thread;
        Loader::Request* _request;

        RequestProgressCallback(Loader::Request* req) :
            _request(req)
        {
            // if this is a pager thread, get a handle on it:
            _thread = dynamic_cast<osgDB::DatabasePager::DatabaseThread*>(
                OpenThreads::Thread::CurrentThread());
        }

        virtual bool isCanceled()
        {
            // was the request canceled?
            if (_canceled == false && _request->isIdle())
                _canceled = true;

            // was the pager shut down?
            if (_thread != 0L && _thread->getDone())
                _canceled = true;

            return ProgressCallback::isCanceled();
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
        request->invoke(0L);
        
        //OE_INFO << LC << "Request apply : UID = " << request->getUID() << "\n";
        if (r->isRunning())
        {
            request->apply( nv.getFrameStamp() );
        }

        r->setState(Request::IDLE);
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

                sscanf(filename.c_str(), "%u", &requestUID);

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
_checkpoint    ( (osg::Timer_t)0 ),
_mergesPerFrame( 0 ),
_frameNumber   ( 0 ),
_numLODs       ( 20u )
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
    ADJUST_EVENT_TRAV_COUNT(this, +1);
    OE_INFO << LC << "Merges per frame = " << _mergesPerFrame << std::endl;
    
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

bool
PagerLoader::load(Loader::Request* request, float priority, osg::NodeVisitor& nv)
{
    // check that the request is not already completed but unmerged:
    if ( request && !request->isMerging() && !request->isFinished() && nv.getDatabaseRequestHandler() )
    {
        //OE_INFO << LC << "load (" << request->getTileKey().str() << ")" << std::endl;

        unsigned fn = 0;
        if ( nv.getFrameStamp() )
        {
            fn = nv.getFrameStamp()->getFrameNumber();
            request->setFrameNumber( fn );
        }

        bool addToRequestSet = false;

        // lock the request since multiple cull traversals might hit this function.
        request->lock();
        {
            request->setState(Request::RUNNING);
            
            // remember the last tick at which this request was submitted
            request->_lastTick = osg::Timer::instance()->tick();

            // update the priority, scale and bias it, and then normalize it to [0..1] range.
            unsigned lod = request->getTileKey().getLOD();
            float p = priority * _priorityScales[lod] + _priorityOffsets[lod];            
            request->_priority = p / (float)(_numLODs+1);

            // timestamp it
            request->setFrameNumber( fn );

            // incremenet the load count.
            request->_loadCount++;

            // if this is the first load request since idle, we need to remember this request.
            addToRequestSet = (request->_loadCount == 1);
        }
        request->unlock();

        char filename[64];
        //sprintf(filename, "%u.%u.osgearth_rex_loader", request->_uid, _engineUID);
        sprintf(filename, "%u.osgearth_rex_loader", request->_uid);

        nv.getDatabaseRequestHandler()->requestNodeFile(
            filename,
            _myNodePath,
            request->_priority,
            nv.getFrameStamp(),
            request->_internalHandle,
            _dboptions.get() );

        // remember the request:
        if ( true ) // addToRequestSet // Not sure whether we need to keep doing this in order keep it sorted -- check it out.
        {
            Threading::ScopedMutexLock lock( _requestsMutex );
            _requests[request->getUID()] = request;
        }

        return true;
    }
    return false;
}

void
PagerLoader::clear()
{
    // Set a time checkpoint for invalidating old requests.
    _checkpoint = osg::Timer::instance()->tick();
}

void
PagerLoader::traverse(osg::NodeVisitor& nv)
{
    // only called when _mergesPerFrame > 0
    if ( nv.getVisitorType() == nv.EVENT_VISITOR )
    {
        if ( nv.getFrameStamp() )
        {
            setFrameStamp(nv.getFrameStamp());
        }

        // process pending merges.
        {
            OE_PROFILING_ZONE_NAMED("loader.merge");
            int count;
            for(count=0; count < _mergesPerFrame && !_mergeQueue.empty(); ++count)
            {
                Request* req = _mergeQueue.begin()->get();
                if ( req && req->_lastTick >= _checkpoint )
                {
                    OE_START_TIMER(req_apply);
                    req->apply( getFrameStamp() );
                    double s = OE_STOP_TIMER(req_apply);

                    req->setState(Request::FINISHED);
                }

                _mergeQueue.erase( _mergeQueue.begin() );
            }
        }

        // cull finished requests.
        {
            OE_PROFILING_ZONE("loader.cull");

            Threading::ScopedMutexLock lock( _requestsMutex );

            unsigned fn = 0;
            if ( nv.getFrameStamp() )
                fn = nv.getFrameStamp()->getFrameNumber();

            // Purge expired requests.
            for(Requests::iterator i = _requests.begin(); i != _requests.end(); )
            {
                Request* req = i->second.get();
                const unsigned frameDiff = fn - req->getLastFrameSubmitted();

                // Deal with completed requests:
                if ( req->isFinished() )
                {
                    //OE_INFO << LC << req->getName() << "(" << i->second->getUID() << ") finished." << std::endl; 
                    req->setState( Request::IDLE );
                    if ( REPORT_ACTIVITY )
                        Registry::instance()->endActivity( req->getName() );
                    _requests.erase( i++ );
                }

                // Discard requests that are no longer required:
                else if ( !req->isMerging() && frameDiff > 2 )
                {
                    //OE_INFO << LC << req->getName() << "(" << i->second->getUID() << ") died waiting after " << frameDiff << " frames" << std::endl; 
                    req->setState( Request::IDLE );
                    if ( REPORT_ACTIVITY )
                        Registry::instance()->endActivity( req->getName() );
                    _requests.erase( i++ );
                }

                // Prevent a request from getting stuck in the merge queue:
                else if ( req->isMerging() && frameDiff > 1800 )
                {
                    //OE_INFO << LC << req->getName() << "(" << i->second->getUID() << ") died waiting " << frameDiff << " frames to merge" << std::endl; 
                    req->setState( Request::IDLE );
                    if ( REPORT_ACTIVITY )
                        Registry::instance()->endActivity( req->getName() );
                    _requests.erase( i++ );
                }

                else // still valid.
                {
                    ++i;
                }
            }

            //OE_NOTICE << LC << "PagerLoader: requests=" << _requests.size() << "; mergeQueue=" << _mergeQueue.size() << std::endl;
        }
    }

    LoaderGroup::traverse( nv );
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
            // Make sure the request is both current (newer than the last checkpoint)
            // and running (i.e. has not been canceled along the way)
            if (req->_lastTick >= _checkpoint && req->isRunning())
            {
                if ( _mergesPerFrame > 0 )
                {
                    _mergeQueue.insert( req );
                    req->setState( Request::MERGING );
                }
                else
                {
                    req->apply( getFrameStamp() );
                    req->setState( Request::FINISHED );
                    if ( REPORT_ACTIVITY )
                        Registry::instance()->endActivity( req->getName() );
                }
            }                

            else
            {
                OE_DEBUG << LC << "Request " << req->getName() << " canceled" << std::endl;
                req->setState( Request::FINISHED );
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
    Threading::ScopedMutexLock lock( _requestsMutex );
    Requests::const_iterator i = _requests.find( requestUID );
    if ( i != _requests.end() )
    {
        return i->second->getTileKey();
    }

    return TileKey::INVALID;
}

Loader::Request*
PagerLoader::invokeAndRelease(UID requestUID)
{
    osg::ref_ptr<Request> request;
    {
        Threading::ScopedMutexLock lock( _requestsMutex );
        Requests::iterator i = _requests.find( requestUID );
        if ( i != _requests.end() )
        {
            request = i->second.get();
        }
    }

    if ( request.valid() )
    {
        if ( REPORT_ACTIVITY )
            Registry::instance()->startActivity( request->getName() );

        osg::ref_ptr<ProgressCallback> prog = new RequestProgressCallback(request.get());
        request->invoke(prog.get());
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
            // nop
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
                // parse the tile key and engine ID:
                std::string requestdef = osgDB::getNameLessExtension(uri);
                unsigned requestUID;
                sscanf(requestdef.c_str(), "%u", &requestUID);

                osg::ref_ptr<PagerLoader> loader;
                if (OptionsData<PagerLoader>::lock(dboptions, "osgEarth.PagerLoader", loader))
                {
                    osg::ref_ptr<Loader::Request> req = loader->invokeAndRelease(requestUID);

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
