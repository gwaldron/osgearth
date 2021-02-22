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
    osg::Node(),
    _pagingManager(nullptr),
    _token(nullptr),
    _loadTriggered(false),
    _compileTriggered(false),
    _callbackFired(false),
    _minRange(0.0f),
    _maxRange(FLT_MAX),
    _priorityScale(1.0f)
{
    _job.setName("oe.PagedNode");
    _job.setArena(PAGEDNODE_ARENA_NAME);
}

PagedNode2::~PagedNode2()
{
    reset();
}

void
PagedNode2::reset()
{
    releaseGLObjects(nullptr);
    _compiled.abandon();
    _loaded.abandon();
    _token = nullptr;
    _loadTriggered = false;
    _compileTriggered = false;
    _callbackFired = false;
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

    // traverse the placeholder
    if (_placeholder.valid())
    {
        _placeholder->accept(nv);
    }

    if (nv.getTraversalMode() == nv.TRAVERSE_ALL_CHILDREN)
    {
        if (_compiled.isAvailable() && _compiled.get().valid())
        {
            _compiled.get()->accept(nv);
        }
    }

    else if (nv.getTraversalMode() == nv.TRAVERSE_ACTIVE_CHILDREN)
    {
        float range = nv.getDistanceToViewPoint(getBound().center(), true);
        if (range >= _minRange && range <= _maxRange)
        {
            if (_load != nullptr && _loadTriggered.exchange(true) == false)
            {
                // Load the asynchronous node.
                Loader load(_load);
                osg::ref_ptr<SceneGraphCallbacks> callbacks(_callbacks);

                _job.setPriority(-range * _priorityScale);

                _loaded = _job.dispatch<osg::ref_ptr<osg::Node>>(
                    [load, callbacks](Cancelable* c)
                    {
                        osg::ref_ptr<ProgressCallback> progress = new ProgressCallback(c);

                        // invoke the loader function
                        osg::ref_ptr<osg::Node> node = load(progress.get());

                        // Fire any pre-merge callbacks
                        if (node.valid() && callbacks.valid())
                            callbacks->firePreMergeNode(node.get());

                        return node;
                    }
                );
            }

            else if (_loaded.isAvailable() && _loaded.get().valid() && _compileTriggered.exchange(true)==false)
            {
                // Compile the loaded node.
                GLObjectsCompiler compiler;
                osg::ref_ptr<ProgressCallback> p = new ObserverProgressCallback(this);
                _compiled = compiler.compileAsync(_loaded.get(), &nv, p.get());
            }

            else if (_compiled.isAvailable() && _compiled.get().valid())
            {
                // render the compiled node.

                // if we haven't fired the post-merge callback yet, do so now
                if (_callbackFired.exchange(true) == false && _callbacks.valid())
                {
                    _callbacks->firePostMergeNode(_compiled.get().get());
                }

                // update our usage
                if (_pagingManager)
                {
                    _token = _pagingManager->use(this, _token);
                }

                // all is good, traverse it
                _compiled.get()->accept(nv);
            }
        }
    }
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
        osg::BoundingSphere bs;

        if (_placeholder.valid())
        {
            bs = _placeholder->computeBound();
        }

        if (_loadTriggered == true &&
            _loaded.isAvailable() &&
            _loaded.get().valid())
        {
            bs.expandBy(_loaded.get()->computeBound());
        }

        return bs;
    }
}

void
PagedNode2::resizeGLObjectBuffers(unsigned m)
{
    osg::Node::resizeGLObjectBuffers(m);

    if (_placeholder.valid())
        _placeholder->resizeGLObjectBuffers(m);

    if (_compiled.isAvailable() && _compiled.get().valid())
        _compiled.get()->resizeGLObjectBuffers(m);
}

void
PagedNode2::releaseGLObjects(osg::State* state) const
{
    osg::Node::releaseGLObjects(state);

    if (_placeholder.valid())
        _placeholder->releaseGLObjects(state);

    if (_compiled.isAvailable() && _compiled.get().valid())
        _compiled.get()->releaseGLObjects(state);
}


PagingManager::PagingManager() :
    _mutex(OE_MUTEX_NAME),
    _tracker()
{
    ADJUST_UPDATE_TRAV_COUNT(this, +1);
    JobArena::get(PAGEDNODE_ARENA_NAME)->setConcurrency(4u);
}

void
PagingManager::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        ScopedMutexLock lock(_trackerMutex);

        _trash.clear();

        // Collect the expired nodes, pushing them in reverse order
        // so that we can release them in the most efficient manner
        // possible (top-down in the scene graph)

        // You can change this to push_back and put a limiter on it
        // to gradually release stuff instead

        _tracker.collectTrash(nv, 0.0f, ~0u,
            [this](PagedNode2* node) { _trash.push_front(node); });

        //_tracker.collectTrash(nv, 0.0f, 1u,
        //    [this](PagedNode2* node) { _trash.push_back(node); });

        //int count = 0;
        for (auto& node : _trash)
        {
            osg::ref_ptr<PagedNode2> safe(node);
            if (safe.valid())
            {
                safe->reset();
                //count++;
            }
        }
        //if (count > 0)
        //    OE_INFO << LC << "Trashed " << count << "/" << _trash.size() << std::endl;
    }

    osg::Group::traverse(nv);
}