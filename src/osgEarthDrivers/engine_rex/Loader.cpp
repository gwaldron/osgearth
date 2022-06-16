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

#include <osgEarth/Utils>
#include <osgEarth/NodeUtils>
#include <osgEarth/Metrics>
#include <osgEarth/GLUtils>

#include <osgUtil/IncrementalCompileOperation>
#include <osgViewer/View>

#include <string>

using namespace osgEarth;
using namespace osgEarth::REX;

#undef LC
#define LC "[Merger] "

Merger::Merger() :
    _mergesPerFrame(~0)
{
    setCullingActive(false);
    setNumChildrenRequiringUpdateTraversal(+1);
    _mutex.setName(OE_MUTEX_NAME);

    _metrics = JobArena::get(ARENA_LOAD_TILE)->metrics();
}

Merger::~Merger()
{
    //nop
}

void
Merger::setMergesPerFrame(unsigned value)
{
    _mergesPerFrame = value;
}

void
Merger::clear()
{
    ScopedMutexLock lock(_mutex);
    _compileQueue = CompileQueue();
    _mergeQueue = MergeQueue();
}

void
Merger::merge(LoadTileDataOperationPtr data, osg::NodeVisitor& nv)
{
    osg::ref_ptr<osgUtil::IncrementalCompileOperation> ico;
    if (ObjectStorage::get(&nv, ico))
    {
        GLObjectsCompiler glcompiler;

        // create an empty state:
        osg::ref_ptr<osgUtil::StateToCompile> state = glcompiler.collectState(nullptr);
        OE_SOFT_ASSERT_AND_RETURN(state.valid(), void());

        // populate it with the tile model contents:
        bool bindless = GLUtils::useNVGL();
        data->_result.join()->getStateToCompile(*state.get(), bindless);

        ScopedMutexLock lock(_mutex);

        if (!state->empty())
        {
            static osg::ref_ptr<osg::Node> dummyNode = new osg::Node();

            ToCompile toCompile;
            toCompile._data = data;
            toCompile._compiled = glcompiler.compileAsync(
                dummyNode.get(), state.get(), &nv, nullptr);

            _compileQueue.push_back(std::move(toCompile));
        }
        else
        {
            _mergeQueue.push(data);
        }
    }
    else
    {
        ScopedMutexLock lock(_mutex);
        _mergeQueue.push(data);
    }

    if (_metrics)
    {
        _metrics->numJobsRunning++;
    }
}

void
Merger::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.CULL_VISITOR)
    {
        _clock.cull();
    }
    else if (nv.getVisitorType() == nv.UPDATE_VISITOR && _clock.update())
    {
        ScopedMutexLock lock(_mutex);

        // Check the GL compile queue
        // TODO: the ICO will orphan compilesets when a graphics context
        // gets released. We need to way to tell the ICO to release 
        // these compilesets so they don't sit in this queue forever.
        for(auto& next : _compileQueue)
        {
            if (next._compiled.isAvailable())
            {
                // compile finished, put it on the merge queue
                _mergeQueue.emplace(std::move(next._data));

                // note: no change the metrics since we are just moving from
                // one queue to another
            }
            else if (next._compiled.isAbandoned())
            {
                // compile canceled, ditch it
                if (_metrics)
                {
                    _metrics->numJobsRunning--;
                    _metrics->numJobsCanceled++;
                }
            }
            else
            {
                // not ready - requeue
                _tempQueue.push_back(std::move(next));
            }
        }
        _compileQueue.swap(_tempQueue);
        _tempQueue.clear();

        unsigned count = 0u;
        unsigned max_count = _mergesPerFrame;
        if (max_count == 0)
            max_count = INT_MAX;

        while (!_mergeQueue.empty() && count < max_count)
        {
            LoadTileDataOperationPtr next = _mergeQueue.front();

            if (next != nullptr)
            {
                if (next->_result.isAvailable())
                {
                    next->merge();
                }
                else
                {
                    //OE_INFO << LC << "Abandoned " << next->_name << std::endl;
                }
            }

            ++count;

            _mergeQueue.pop();
            if (_metrics)
            {
                _metrics->numJobsRunning--;
            }
        }

        //if (count > 0)
        //{
        //    OE_INFO << LC << "Merged " << count << std::endl;
        //}
    }

    osg::Node::traverse(nv);
}

void
Merger::releaseGLObjects(osg::State* state) const
{
    // TODO
    osg::Node::releaseGLObjects(state);
}
