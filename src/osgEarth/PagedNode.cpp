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
#include <osgEarth/PagedNode>
#include <osgEarth/Utils>
#include <osgEarth/GLUtils>
#include <osgEarth/NodeUtils>
#include <osgEarth/Progress>
#include <osgEarth/Registry>

#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

#define LC "[PagedNode] "

using namespace osgEarth;
using namespace osgEarth::Util;

#define PAGEDNODE_ARENA_NAME "oe.nodepager"

PagedNode2::PagedNode2() :
    osg::Group(),
    _pagingManager(nullptr),
    _token(nullptr),
    _loadTriggered(false),
    _compileTriggered(false),
    _mergeTriggered(false),
    _merged(false),
    _failed(false),
    _minRange(0.0f),
    _maxRange(FLT_MAX),
    _minPixels(0.0f),
    _maxPixels(FLT_MAX),
    _useRange(true),
    _priorityScale(1.0f),
    _refinePolicy(REFINE_REPLACE),
    _preCompile(true),
    _autoUnload(true)
{
    _job.setName(typeid(*this).name());
    _job.setArena(PAGEDNODE_ARENA_NAME);
}

PagedNode2::~PagedNode2()
{
    //nop
    // note: do not call reset() from here, we never want to
    // releaseGLObjects in this dtor b/c it could be called
    // from a pager thread at cancelation
}

bool
PagedNode2::isHighestResolution() const
{
    return getNumChildren() == 0;
}

void
PagedNode2::setLoadFunction(const Loader& value)
{
    unload();
    _load = value;
}

void
PagedNode2::traverse(osg::NodeVisitor& nv)
{
    // locate the paging manager if there is one
    if (_pagingManager == nullptr)
    {
        ScopedMutexLock lock(_mutex);
        if (_pagingManager == nullptr) // double check
        {
            osg::ref_ptr<PagingManager> pm;
            if (ObjectStorage::get(&nv, pm))
                _pagingManager = pm.get();
        }
    }

    if (nv.getTraversalMode() == nv.TRAVERSE_ALL_CHILDREN)
    {
        for (auto& child : _children)
            child->accept(nv);
    }

    else if (nv.getTraversalMode() == nv.TRAVERSE_ACTIVE_CHILDREN)
    {
        bool inRange = false;
        float priority = 0.0f;

        if (_useRange) // meters
        {
            float range = std::max(0.0f, nv.getDistanceToViewPoint(getBound().center(), true) - getBound().radius());
            inRange = (range >= _minRange && range <= _maxRange);
            priority = -range * _priorityScale;
        }
        else // pixels
        {
            osg::CullStack* cullStack = nv.asCullStack();
            if (cullStack != nullptr && cullStack->getLODScale() > 0.0f)
            {
                float pixels = cullStack->clampedPixelSize(getBound()) / cullStack->getLODScale();
                inRange = (pixels >= _minPixels && pixels <= _maxPixels);
                priority = pixels * _priorityScale;
            }
        }

        if (inRange)
        {
            load(priority, &nv);

            // finally, traverse children and paged data.
            if (_refinePolicy == REFINE_REPLACE &&
                _merged == true &&
                _compiled.get().valid())
            {
                _compiled.get()->accept(nv);
            }
            else
            {
                for (auto& child : _children)
                    child->accept(nv);
            }

            touch();
        }
        else
        {
            // child out of range; just accept static children
            for (auto& child : _children)
            {
                osg::Node* compiled =
                    _compiled.isAvailable() ? _compiled.get().get() :
                    nullptr;

                if (child.get() != compiled)
                    child->accept(nv);
            }
        }
    }
}

void
PagedNode2::touch()
{
    // tell the paging manager this node is still alive
    // (and should not be removed from the scene graph)
    if (_pagingManager)
    {
        _token = _pagingManager->use(this, _token);
    }
}

bool
PagedNode2::merge(int revision)
{
    // Check the revision, b/c it is possible for a node in the merge queue
    // to be expired before it pops to the front of the merge queue and
    // this method gets invoked.
    if (_revision == revision)
    {
        //static std::set<osg::Node*> nodes;
        //OE_SOFT_ASSERT_AND_RETURN(nodes.count(this) == 0, __func__, false);
        //nodes.insert(this);

        // This is called from PagingManager.
        // We're in the UPDATE traversal.
        OE_SOFT_ASSERT_AND_RETURN(_merged == false, __func__, false);
        OE_SOFT_ASSERT_AND_RETURN(_compiled.isAvailable(), __func__, false);
        OE_SOFT_ASSERT_AND_RETURN(_compiled.get().valid(), __func__, false);

        addChild(_compiled.get());

        if (_callbacks.valid())
            _callbacks->firePostMergeNode(_compiled.get().get());

        _merged = true;
        _failed = false;
    }
    return _merged;
}

osg::BoundingSphere
PagedNode2::computeBound() const
{
    if (_userBS.isSet() && _userBS->radius() >= 0.0f)
    {
        return _userBS.get();
    }

    else
    {
        osg::BoundingSphere bs = osg::Group::computeBound();

        if (_loadTriggered == true &&
            _merged == false &&
            _loaded.isAvailable() &&
            _loaded.get()._node.valid() )
        {
            bs.expandBy(_loaded.get()._node->computeBound());
        }

        return bs;
    }
}

void PagedNode2::load(float priority, const osg::Object* host)
{
    if (_loadTriggered.exchange(true) == false)
    {
        if (_load != nullptr)
        {
            // Load the asynchronous node.
            Loader load(_load);
            osg::ref_ptr<SceneGraphCallbacks> callbacks(_callbacks);
            bool preCompile = _preCompile;

            _job.setPriority(priority);

            _loaded = _job.dispatch<Loaded>(
                [load, callbacks, preCompile](Cancelable* c)
                {
                    Loaded result;

                    osg::ref_ptr<ProgressCallback> progress = new ProgressCallback(c);

                    // invoke the loader function
                    result._node = load(progress.get());

                    // Fire any pre-merge callbacks
                    if (result._node.valid())
                    {
                        if (callbacks.valid())
                            callbacks->firePreMergeNode(result._node.get());

                        if (preCompile)
                        {
                            // Collect the GL objects for later compilation.
                            // Don't waste precious ICO time doing this later
                            GLObjectsCompiler compiler;
                            result._state = compiler.collectState(result._node.get());
                        }
                    }

                    return result;
                }
            );
        }
        else
        {
            // There is no load function so go all the way to the end of the state machine.
            _failed = true;
            _compileTriggered.exchange(true);
            _mergeTriggered.exchange(true);
        }
    }

    else if (
        _loaded.isAvailable() &&
        _compileTriggered.exchange(true) == false)
    {
        if (_loaded.get()._node.valid())
        {
            dirtyBound();

            if (_preCompile)
            {
                // Compile the loaded node.
                GLObjectsCompiler compiler;
                osg::ref_ptr<ProgressCallback> p = new ObserverProgressCallback(this);

                _compiled = compiler.compileAsync(
                    _loaded.get()._node,
                    _loaded.get()._state.get(),
                    host,
                    p.get());
            }
            else
            {
                // resolve immediately
                Promise<osg::ref_ptr<osg::Node>> promise;
                _compiled = promise.getFuture();
                promise.resolve(_loaded.get()._node);
            }
        }
        else
        {
            // The node returned was null, so we need to just go to the end of the state machine
            _failed = true;
            _mergeTriggered.exchange(true);
        }

        // done with it, let it go
        _loaded.abandon();
    }
    else if (
        _compiled.isAvailable() &&
        _pagingManager != nullptr &&
        _mergeTriggered.exchange(true) == false)
    {
        // Submit this node to the paging manager for merging.
        _pagingManager->merge(this);
    }
}

void PagedNode2::unload()
{
    // Note: don't do this. PLOD didn't so neither shall we
    //if (_compiled.isAvailable() && _compiled.get().valid())
    //{
    //    _compiled.get()->releaseGLObjects(nullptr);
    //}
    if (_compiled.isAvailable() && _compiled.get().valid())
    {
        removeChild(_compiled.get());
    }
    _compiled.abandon();
    _loaded.abandon();
    _loadTriggered = false;
    _compileTriggered = false;
    _mergeTriggered = false;
    _merged = false;
    _failed = false;
    _token = nullptr;

    // prevents a node in the PagingManager's merge queue from
    // being merged with old data.
    _revision++;
}

bool PagedNode2::isLoaded() const
{
    return _merged || _failed;
}

PagingManager::PagingManager() :
    _trackerMutex(OE_MUTEX_NAME),
    _mergeMutex(OE_MUTEX_NAME),
    _tracker(),
    _mergesPerFrame(4u),
    _newFrame(false)
{
    setCullingActive(false);
    ADJUST_UPDATE_TRAV_COUNT(this, +1);
    JobArena::get(PAGEDNODE_ARENA_NAME)->setConcurrency(4u);

    // NOTE: this is causing multiple model layers to not appear.
    // Need to debug before using.
    //osg::observer_ptr<PagingManager> pm_ptr(this);
    //_updateFunc = [pm_ptr](Cancelable*) mutable
    //{
    //    osg::ref_ptr<PagingManager> pm(pm_ptr);
    //    if (pm.valid())
    //    {
    //        pm->update();
    //        Job(JobArena::get(JobArena::UPDATE_TRAVERSAL))
    //            .dispatch(pm->_updateFunc);
    //    }
    //};

    //Job(JobArena::get(JobArena::UPDATE_TRAVERSAL))
    //    .dispatch(_updateFunc);
}

void
PagingManager::traverse(osg::NodeVisitor& nv)
{
    ObjectStorage::set(&nv, this);

    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        _newFrame.exchange(true);
    }

    else if (
        nv.getVisitorType() == nv.UPDATE_VISITOR &&
        _newFrame.exchange(false) == true)
    {
        update();
    }

    osg::Group::traverse(nv);
}

void
PagingManager::update()
{
    // Discard expired nodes
    {
        ScopedMutexLock lock(_trackerMutex); // unnecessary?

        _tracker.flush(
            0.0f,
            _mergesPerFrame,
            [](osg::ref_ptr<PagedNode2>& node) -> bool {
                if (node->getAutoUnload())
                {
                    node->unload();
                    return true;
                }
                return false;
            });
    }

    // Handle merges
    if (_mergeQueue.empty() == false)
    {
        ScopedMutexLock lock(_mergeMutex); // unnecessary?

        unsigned count = 0u;
        while (_mergeQueue.empty() == false && count < _mergesPerFrame)
        {
            ToMerge& front = _mergeQueue.front();
            osg::ref_ptr<PagedNode2> next;
            if (front._node.lock(next))
            {
                if (next->merge(front._revision))
                    ++count;
            }
            _mergeQueue.pop();
        }
    }
}
