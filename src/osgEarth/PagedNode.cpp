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

namespace
{
    struct PagedNodePseudoLoader : public osgDB::ReaderWriter
    {
        PagedNodePseudoLoader()
        {
            supportsExtension( "osgearth_pseudo_pagednode", "" );
        }

        const char* className() const
        { // override
            return "PagedNodePseudoLoader";
        }

        ReadResult readNode(const std::string& uri, const Options* options) const
        {
            if ( !acceptsExtension( osgDB::getLowerCaseFileExtension(uri) ) )
                return ReadResult::FILE_NOT_HANDLED;

            osg::ref_ptr<PagedNode> node;
            if (!ObjectStorage::get(options, node))
            {
                OE_WARN << "Internal error - no PagedNode object in ObjectStorage\n";
                return ReadResult::ERROR_IN_READING_FILE;
            }

            return node->loadChild();
        }
    };

    REGISTER_OSGPLUGIN(osgearth_pseudo_pagednode, PagedNodePseudoLoader);
}

PagedNode::PagedNode() :
    _rangeFactor(6.0f),
    _additive(false)
{
    _plod = new osg::PagedLOD;
    addChild(_plod);

    _attachPoint = new osg::Group;

    _plod->addChild( _attachPoint );
}

void PagedNode::setRangeMode(osg::LOD::RangeMode mode)
{
    _plod->setRangeMode(mode);
}

void PagedNode::setNode(osg::Node* node)
{
    if (node)
        _attachPoint->addChild(node);
}

void PagedNode::setupPaging()
{
    osg::BoundingSphere bs = getChildBound();

    _plod->setCenter( bs.center() );
    _plod->setRadius( bs.radius() );

    if ( hasChild() )
    {
        // Now setup a filename on the PagedLOD that will load all of the children of this node.
        _plod->setFileName(1, ".osgearth_pseudo_pagednode");

        // assemble data to pass to the pseudoloader
        osgDB::Options* options = new osgDB::Options();
        ObjectStorage::set(options, this);
        _plod->setDatabaseOptions( options );

        // Setup the min and max ranges.
        float minRange;
        if ( _range.isSet() )
        {
            minRange = _range.get();
        }
        else
        {
            if (_plod->getRangeMode() == _plod->DISTANCE_FROM_EYE_POINT)
            {
                minRange = (float)(bs.radius() * _rangeFactor);
            }
            else
            {
                minRange = 256;
            }
        }

        if (!_additive)
        {
            // Replace mode, the parent is replaced by its children.
            if (_plod->getRangeMode() == _plod->DISTANCE_FROM_EYE_POINT)
            {
                _plod->setRange( 0, minRange, FLT_MAX );
                _plod->setRange( 1, 0, minRange );
            }
            else
            {
                _plod->setRange(0, 0, minRange);
                _plod->setRange(1, minRange, FLT_MAX);
            }
        }
        else
        {
            // Additive, the parent remains and new data is added
            if (_plod->getRangeMode() == _plod->DISTANCE_FROM_EYE_POINT)
            {
                _plod->setRange( 0, 0, FLT_MAX );
                _plod->setRange( 1, 0, minRange );
            }
            else
            {
                _plod->setRange(0, 0, FLT_MAX);
                _plod->setRange(1, minRange, FLT_MAX);
            }
        }
    }
    else
    {
        // no children, so max out the visibility range.
        _plod->setRange( 0, 0, FLT_MAX );
    }
}

osg::BoundingSphere PagedNode::getChildBound() const
{
    return osg::BoundingSphere();
}

bool PagedNode::hasChild() const
{
    return true;
}

//...................................................................

PagedNode2::PagedNode2() :
    osg::Group(),
    _pagingManager(nullptr),
    _token(nullptr),
    _loadTriggered(false),
    _compileTriggered(false),
    _mergeTriggered(false),
    _merged(false),
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
            float range = nv.getDistanceToViewPoint(getBound().center(), true);
            inRange = (range >= 0.0f && range >= _minRange && range <= _maxRange);
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

        // check that range > 0 to avoid trouble from some visitors
        if (inRange)
        {
            load(priority, &nv);

            // finally, traverse children and paged data.
            if (_merged && _refinePolicy == REFINE_REPLACE)
            {
                _compiled.get()->accept(nv);
            }
            else
            {
                for (auto& child : _children)
                    child->accept(nv);
            }

            // tell the paging manager this node is still alive
            // (and should not be removed from the scene graph)
            if (_pagingManager)
            {
                _token = _pagingManager->use(this, _token);
            }
        }
        else
        {
            // child out of range; just accept static children
            for (auto& child : _children)
                if (child.get() != _compiled.get().get())
                    child->accept(nv);
        }
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
        // This is called from PagingManager.
        // We're in the UPDATE traversal.
        OE_SOFT_ASSERT_AND_RETURN(_merged == false, __func__, false);
        OE_SOFT_ASSERT_AND_RETURN(_compiled.isAvailable(), __func__, false);
        OE_SOFT_ASSERT_AND_RETURN(_compiled.get().valid(), __func__, false);

        addChild(_compiled.get());

        if (_callbacks.valid())
            _callbacks->firePostMergeNode(_compiled.get().get());

        _merged = true;
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
    if (_load != nullptr &&
        _loadTriggered.exchange(true) == false)
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

    else if (
        _loaded.isAvailable() &&
        _loaded.get()._node.valid() &&
        _compileTriggered.exchange(true) == false)
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

        // done with it, let it go
        _loaded.abandon();
    }

    else if (
        _compiled.isAvailable() &&
        _compiled.get().valid() &&
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
    if (_merged)
    {
        removeChild(_compiled.get().get());
    }
    _compiled.abandon();
    _loaded.abandon();
    _loadTriggered = false;
    _compileTriggered = false;
    _mergeTriggered = false;
    _merged = false;
    _token = nullptr;

    // prevents a node in the PagingManager's merge queue from
    // being merged with old data.
    _revision++;
}

bool PagedNode2::isLoaded() const
{
    return _merged;
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
}

void
PagingManager::traverse(osg::NodeVisitor& nv)
{
    // Make this object accesible to children
    ObjectStorage::set(&nv, this);

    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        _newFrame.exchange(true);
    }

    else if (
        nv.getVisitorType() == nv.UPDATE_VISITOR &&
        _newFrame.exchange(false)==true)
    {
        // Discard expired nodes
        {
            ScopedMutexLock lock(_trackerMutex); // unnecessary?

            _tracker.flush(
                nv,
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

    osg::Group::traverse(nv);
}