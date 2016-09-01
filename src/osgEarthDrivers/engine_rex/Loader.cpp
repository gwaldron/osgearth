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
#include "Loader"
#include "RexTerrainEngineNode"

#include <osgEarth/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReaderWriter>

#include <string>

#define REPORT_ACTIVITY true

using namespace osgEarth::Drivers::RexTerrainEngine;


Loader::Request::Request()
{
    _uid = osgEarth::Registry::instance()->createUID();
    _state = IDLE;
    _loadCount = 0;
    _priority = 0;
    _lastFrameSubmitted = 0;
    _lastTick = 0;
}

osg::StateSet*
Loader::Request::getStateSet()
{
    if ( !_stateSet.valid() )
    {
        _stateSet = new osg::StateSet();
        _stateSet->setDataVariance( osg::Object::DYNAMIC );
    }
    return _stateSet.get();
}

void
Loader::Request::addToChangeSet(osg::Node* node)
{
    if ( node )
    {
        _nodesChanged.push_back( node );
    }
}

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

        //OE_INFO << LC << "Request invoke : UID = " << request->getUID() << "\n";
        request->invoke();
        
        //OE_INFO << LC << "Request apply : UID = " << request->getUID() << "\n";
        request->apply( nv.getFrameStamp() );
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

            osgEarth::UID requestUID, engineUID;

            sscanf(filename.c_str(), "%d.%d", &requestUID, &engineUID);

            osg::ref_ptr<RexTerrainEngineNode> engine;
            RexTerrainEngineNode::getEngineByUID( (UID)engineUID, engine );

            if ( engine.valid() )
            {
                PagerLoader* loader = dynamic_cast<PagerLoader*>( engine->getLoader() );
                if ( loader )
                {
                    TileKey key = loader->getTileKeyForRequest(requestUID);

                    MapFrame frame(engine->getMap());
                    if ( frame.isCached(key) )
                    {
                        result = LOCAL_FILE;
                    }
                }

                //OE_NOTICE << "key=" << key.str() << " : " << (result==LOCAL_FILE?"local":"remote") << "\n";
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
    struct RequestResultNode : public osg::Node
    {
        RequestResultNode(Loader::Request* request)
            : _request(request)
        {
            // Do this so the pager/ICO can find and pre-compile GL objects that are
            // attached to the stateset.
            if ( _request.valid() )
            {
                // TODO: for some reason pre-compiling is causing texture flashing issues 
                // with things like classification maps when using --ico. Figure out why.
                setStateSet( _request->getStateSet() );
            }
        }

        Loader::Request* getRequest() const { return _request.get(); }

        osg::ref_ptr<Loader::Request> _request;
    };
}


PagerLoader::PagerLoader(TerrainEngine* engine) :
_engineUID     ( engine->getUID() ),
_checkpoint    ( (osg::Timer_t)0 ),
_mergesPerFrame( 0 ),
_frameNumber   ( 0 )
{
    _myNodePath.push_back( this );

    _dboptions = new osgDB::Options();
    _dboptions->setFileLocationCallback( new FileLocationCallback() );
}

void
PagerLoader::setMergesPerFrame(int value)
{
    _mergesPerFrame = std::max(value, 0);
    this->setNumChildrenRequiringUpdateTraversal( 1 );
}

bool
PagerLoader::load(Loader::Request* request, float priority, osg::NodeVisitor& nv)
{
    // check that the request is not already completed but unmerged:
    //if ( request && !request->isMerging() && nv.getDatabaseRequestHandler() )
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

            // update the priority
            request->_priority = priority;

            // timestamp it
            request->setFrameNumber( fn );

            // incremenet the load count.
            request->_loadCount++;

            // if this is the first load request since idle, we need to remember this request.
            addToRequestSet = (request->_loadCount == 1);
        }
        request->unlock();

        char filename[64];
        sprintf(filename, "%u.%u.osgearth_rex_loader", request->_uid, _engineUID);
        //std::string filename = Stringify() << request->_uid << "." << _engineUID << ".osgearth_rex_loader";

        // scale from LOD to 0..1 range, more or less
        // TODO: need to balance this with normal PagedLOD priority setup
        //float scaledPriority = priority / 20.0f;

        nv.getDatabaseRequestHandler()->requestNodeFile(
            filename,
            _myNodePath,
            priority,
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
    if ( nv.getVisitorType() == nv.UPDATE_VISITOR )
    {
        if ( nv.getFrameStamp() )
        {
            setFrameStamp(nv.getFrameStamp());
        }

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

        // cull finished requests.
        {
            Threading::ScopedMutexLock lock( _requestsMutex );

            unsigned fn = 0;
            if ( nv.getFrameStamp() )
                fn = nv.getFrameStamp()->getFrameNumber();

            // Purge expired requests.
            for(Requests::iterator i = _requests.begin(); i != _requests.end(); )
            {
                Request* req = i->second.get();

                if ( req->isFinished() )
                {
                    //OE_INFO << LC << req->getName() << "(" << i->second->getUID() << ") finished." << std::endl; 
                    req->setState( Request::IDLE );
                    if ( REPORT_ACTIVITY )
                        Registry::instance()->endActivity( req->getName() );
                    _requests.erase( i++ );
                }

                else if ( !req->isMerging() && (fn - req->getLastFrameSubmitted() > 2) )
                {
                    //OE_INFO << LC << req->getName() << "(" << i->second->getUID() << ") died waiting after " << fn-req->getLastFrameSubmitted() << " frames" << std::endl; 
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
            if ( req->_lastTick >= _checkpoint )
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

        request->invoke();
    }

    else
    {
        // If request is NULL, that means that the Pager dispatched a request that 
        // has already died on the vine.
        //OE_WARN << LC << "Internal: invokeAndRelease (" << requestUID << ") not found." << std::endl;
    }

    return request.release();
}



namespace osgEarth { namespace Drivers { namespace RexTerrainEngine
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
                // parse the tile key and engine ID:
                std::string requestdef = osgDB::getNameLessExtension(uri);
                unsigned requestUID, engineUID;
                sscanf(requestdef.c_str(), "%u.%u", &requestUID, &engineUID);

                // find the appropriate engine:
                osg::ref_ptr<RexTerrainEngineNode> engineNode;
                RexTerrainEngineNode::getEngineByUID( (UID)engineUID, engineNode );
                if ( engineNode.valid() )
                {
                    PagerLoader* loader = dynamic_cast<PagerLoader*>(engineNode->getLoader());
                    if ( loader )
                    {
                        Loader::Request* req = loader->invokeAndRelease( requestUID );
                        return new RequestResultNode(req);
                    }
                }
                return ReadResult::FILE_NOT_FOUND;
            }
            else
            {
                return ReadResult::FILE_NOT_HANDLED;
            }
        };
    };
    REGISTER_OSGPLUGIN(osgearth_rex_loader, PagerLoaderAgent);

} } } // namespace osgEarth::Drivers::RexTerrainEngine
