/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarth/Async>
#include <osgEarth/Registry>
#include <osgEarth/Utils>

#include <osg/CullStack>
#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>

using namespace osgEarth;

#define LC "[Async] "

//........................................................................

#define ASYNC_PSEUDOLOADER_EXT "osgearth_async_node"
#define TAG_ASYNC_CALLBACK     "osgearth_async_callback"

//........................................................................

AsyncResult::AsyncResult(osg::Node* node) : 
_node(node), _requestId(0u)
{
    //nop
}

//........................................................................

namespace osgEarth
{
    // internal class
    class AsyncNode : public osg::Referenced
    {
    public:
        AsyncNode()
        {
            _needy = true;
            _id = ++_idgen;
            char buf[64];
            sprintf(buf, "%u." ASYNC_PSEUDOLOADER_EXT, _id);
            _pseudoloaderFilename = buf;
        }

        bool _needy;
        unsigned _id;
        osg::ref_ptr<osg::Node> _node;
        osg::ref_ptr<AsyncFunction> _callback;
        std::string _pseudoloaderFilename;
        double _lastTimeWeMet;
        float _minValue;
        float _maxValue;
        osg::BoundingSphere _bound;
        osg::ref_ptr<osg::Referenced> _internalHandle;
        osg::ref_ptr<osgDB::Options> _options;

        void accept(osg::NodeVisitor& nv) { if (_node) _node->accept(nv); }

        //! Use the node's bound if we have it, or a preset
        //! bound if we don't.
        const osg::BoundingSphere& getBound() const
        {
            if (_node.valid())
                return _node->getBound();
            else
                return _bound;
        }

        static OpenThreads::Atomic _idgen;
    };

    OpenThreads::Atomic AsyncNode::_idgen;
}

//........................................................................

AsyncLOD::AsyncLOD()
{
    _policy = POLICY_ACCUMULATE;
    _mode = MODE_GEOMETRIC_ERROR;
    _nodePath.push_back(this);
}

void
AsyncLOD::setMode(const Mode& value)
{
    _mode = value;
}

const AsyncLOD::Mode&
AsyncLOD::getMode() const
{
    return _mode;
}

void
AsyncLOD::setPolicy(const Policy& value)
{
    _policy = value;
}

const AsyncLOD::Policy&
AsyncLOD::getPolicy() const
{
    return _policy;
}

void
AsyncLOD::setCenter(const osg::Vec3d& center)
{
    _userDefinedBound.set(center, _userDefinedBound.radius());
}

void
AsyncLOD::setRadius(double radius)
{
    _userDefinedBound.set(_userDefinedBound.center(), radius);
}

void
AsyncLOD::addChild(AsyncFunction* callback, float minValue, float maxValue)
{
    _children.push_back(AsyncNode());
    AsyncNode& child = _children.back();
    child._callback = callback;
    child._minValue = minValue;
    child._maxValue = maxValue;
    child._lastTimeWeMet = DBL_MAX;
    child._options = Registry::instance()->cloneOrCreateOptions(_readOptions.get());
    OptionsData<AsyncFunction>::set(child._options.get(), TAG_ASYNC_CALLBACK, callback);
    _lookup[child._id] = &child;
}

void
AsyncLOD::addChild(osg::Node* node, float minValue, float maxValue)
{
    _children.push_back(AsyncNode());
    AsyncNode& child = _children.back();
    child._node = node;
    child._minValue = minValue;
    child._maxValue = maxValue;
    child._lastTimeWeMet = DBL_MAX;
}

void
AsyncLOD::clear()
{
    _children.clear();
    _mutex.lock();
    _lookup.clear();
    _mutex.unlock();
}


// only to be called by the pager
// TODO: move this to a helper object and pass that in the myNodePath instead
// so that this method will not sully our API
bool
AsyncLOD::addChild(osg::Node* node)
{
    AsyncResult* ar = dynamic_cast<AsyncResult*>(node);
    if (ar)
    {
        _mutex.lock();
        std::map<unsigned, AsyncNode*>::iterator i = _lookup.find(ar->_requestId);
        if (i != _lookup.end())
        {
            AsyncNode* async = i->second;

            // if the operation returned a node, install it now
            if (ar->_node.valid())
            {
                async->_node = ar->_node.get();
            }

            // node is no longer needy. Yay!
            async->_needy = false;
        }
        _mutex.unlock();
        return true;
    }
    return false;
}


//! Computes the bounding sphere, using a user-defined one first
//! if it's available

osg::BoundingSphere
AsyncLOD::computeBound() const
{
    if (_userDefinedBound.valid())
    {
        return _userDefinedBound;
    }
    else
    {
        osg::BoundingSphere bs;
        for (std::vector<AsyncNode>::const_iterator i = _children.begin();
            i != _children.end();
            ++i)
        {
            bs.expandBy(i->getBound());
        }
        return bs;
    }
}
void
AsyncLOD::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        // last child that already has a node to display:
        AsyncNode* lastHealthyChild = 0L;

        // last child that still needs a node:
        AsyncNode* lastNeedyChild = 0L;

        // visit each child:
        for (std::vector<AsyncNode>::iterator i = _children.begin(); i != _children.end(); ++i)
        {
            AsyncNode& child = *i;

            // is the child visible?
            if (isVisible(child, nv))
            {
                // if so, update its timestamp
                child._lastTimeWeMet = nv.getFrameStamp()->getReferenceTime();

                // if this child has a live node, traverse it if we're in 
                // accumulate mode (which means everything loaded so far is visible)
                if (child._node.valid())
                {
                    lastHealthyChild = &child;

                    // depending on the policy, traverse the live node now.
                    // In "replace" mode, we have to want and see because we only
                    // want to display the last healthy node.
                    if (_policy == POLICY_ACCUMULATE || _policy == POLICY_INDEPENDENT)
                    {
                        child._node->accept(nv);
                    }
                }

                // if the child does NOT have a node (but wants one)
                // put in a request. If we're in accumulate or refine mode,
                // we'll defer this request until after we have checked on
                // all children.
                else if (child._needy)
                {
                    lastNeedyChild = &child;

                    if (_policy == POLICY_INDEPENDENT)
                    {
                        ask(child, nv);
                    }
                }
            }
        }

        // if we have a needy child, request it's node now. (If we are
        // in "independent" mode we already did this earlier.)
        if (lastNeedyChild != NULL && _policy != POLICY_INDEPENDENT)
        {
            ask(*lastNeedyChild, nv);
        }

        // in replacement mode (only display the last node) we need
        // to traverse the last healthy child now.
        if (lastHealthyChild != NULL && _policy == POLICY_REPLACE)
        {
            lastHealthyChild->accept(nv);
        }
    }

    // Update, ComputeBound, CompileGLObjects, etc.
    else if (nv.getTraversalMode() == nv.TRAVERSE_ALL_CHILDREN)
    {
        for (std::vector<AsyncNode>::iterator i = _children.begin(); i != _children.end(); ++i)
        {
            if (i->_node.valid())
            {
                i->_node->accept(nv);
            }
        }
    }
}


//! Made a request to the pager to schedule the async function call.
void
AsyncLOD::ask(AsyncNode& child, osg::NodeVisitor& nv)
{
    nv.getDatabaseRequestHandler()->requestNodeFile(
        child._pseudoloaderFilename,        // pseudo name that will invoke AsyncNodePseudoLoader
        _nodePath,                          // parent of node to "add" when request completes
        0.5f,                               // priority
        nv.getFrameStamp(),                 // frame stamp
        child._internalHandle,              // associates the request with a unique ID
        child._options.get()                // osgDB plugin options
    );
}

//! Determines whether a child node is visible and therefore
//! should be processed for loading or viewing.
bool
AsyncLOD::isVisible(const AsyncNode& async, osg::NodeVisitor& nv) const
{
#if OSG_VERSION_GREATER_OR_EQUAL(3,5,6)
    osg::CullStack* cullStack = nv.asCullStack();
#else
    osg::CullStack* cullStack = dynamic_cast<osg::CullStack*>(&nv);
#endif

    if (_mode == MODE_GEOMETRIC_ERROR)
    {
        if (cullStack && cullStack->getLODScale() > 0)
        {
            // size of this node. Use setCenter and setRadius to make this work.
            float sizeInMeters = getBound().radius() * 2.0;
            float sizeInPixels = cullStack->clampedPixelSize(getBound()) / cullStack->getLODScale();
            float metersPerPixel = sizeInPixels > 0.0 ? sizeInMeters / sizeInPixels : 0.0f;
            return metersPerPixel < async._maxValue;
        }
    }

    else if (_mode == MODE_PIXEL_SIZE)
    {
        if (cullStack && cullStack->getLODScale() > 0)
        {
            float sizeInPixels = cullStack->clampedPixelSize(getBound()) / cullStack->getLODScale();
            return async._minValue <= sizeInPixels && sizeInPixels < async._maxValue;
        }
    }

    else if (_mode == MODE_RANGE)
    {
        float range = nv.getDistanceToViewPoint(getBound().center(), true);
        return async._minValue <= range && range < async._maxValue;
    }

    return false;
}

namespace osgEarth
{
    struct AsyncNodePseudoLoader : public osgDB::ReaderWriter
    {
        ReadResult readNode(const std::string& location, const osgDB::Options* options) const
        {
            if (osgDB::getFileExtension(location) != ASYNC_PSEUDOLOADER_EXT)
                return ReadResult::FILE_NOT_HANDLED;

            osg::ref_ptr<AsyncFunction> callback = OptionsData<AsyncFunction>::get(options, TAG_ASYNC_CALLBACK);
            if (!callback.valid())
                return ReadResult::FILE_NOT_FOUND;

            unsigned id = atoi(osgDB::getNameLessExtension(location).c_str());
            osgEarth::ReadResult r = (*callback.get())();
            //TODO: record error, use Progress...
            osg::ref_ptr<AsyncResult> result = new AsyncResult(r.releaseNode());
            if (result.valid())result->_requestId = id;
            return ReadResult(result.release());
        }
    };
    REGISTER_OSGPLUGIN(ASYNC_PSEUDOLOADER_EXT, AsyncNodePseudoLoader);
}
