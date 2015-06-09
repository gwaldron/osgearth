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

using namespace osgEarth::Drivers::RexTerrainEngine;


Loader::Request::Request()
{
    _uid = osgEarth::Registry::instance()->createUID();
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
        request->apply();
    }
    return request != 0L;
}

//...............................................

#undef  LC
#define LC "[PagerLoader] "

namespace
{
    struct RequestResultNode : public osg::Node
    {
        RequestResultNode(Loader::Request* request) : _request(request)
        {
            //nop
        }

        Loader::Request* getRequest() const { return _request.get(); }

        osg::ref_ptr<Loader::Request> _request;
    };
}


PagerLoader::PagerLoader(UID engineUID) :
_engineUID( engineUID )
{
    _myNodePath.push_back( this );
}


bool
PagerLoader::load(Loader::Request* request, float priority, osg::NodeVisitor& nv)
{
    if ( request && nv.getDatabaseRequestHandler() )
    {
        osgDB::Options* dboptions = 0L;

        std::string filename = Stringify() << request->_uid << "." << _engineUID << ".osgearth_rex_loader";

        //OE_INFO << LC << "Requesting: " << filename << "\n";

        nv.getDatabaseRequestHandler()->requestNodeFile(
            filename,
            _myNodePath,
            priority,
            nv.getFrameStamp(),
            request->_internalHandle,
            dboptions );

        unsigned fn = 0;
        if ( nv.getFrameStamp() )
        {
            fn = nv.getFrameStamp()->getFrameNumber();
            request->setFrameNumber( fn );
        }

        // remember.
        {
            Threading::ScopedMutexLock lock( _requestsMutex );
            _requests[request->getUID()] = request;

            // Purge expired requests.
            for(Requests::iterator i = _requests.begin(); i != _requests.end(); )
            {
                if ( fn - i->second.get()->getLastFrameSubmitted() > 2 )
                    i = _requests.erase(i);
                else
                    ++i;
            }
            
            OE_DEBUG << LC << "PagerLoader: requests = " << _requests.size() << "\n";
        }

        return true;
    }
    return false;
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
            req->apply();
        }
    }
    return true;
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
            _requests.erase( i );
        }
    }

    if ( request.valid() )
    {
        request->invoke();
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

        virtual const char* className()
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
                sscanf(requestdef.c_str(), "%d.%d", &requestUID, &engineUID);

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