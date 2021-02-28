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
#if 1
    osg::ref_ptr<osgUtil::IncrementalCompileOperation> ico;
    if (ObjectStorage::get(&nv, ico))
    {
        osgUtil::StateToCompile state(
            osgUtil::GLObjectsVisitor::COMPILE_STATE_ATTRIBUTES |
            osgUtil::GLObjectsVisitor::COMPILE_DISPLAY_LISTS |
            osgUtil::GLObjectsVisitor::CHECK_BLACK_LISTED_MODES,
            nullptr);

        data->_result.get()->getStateToCompile(state);
        
        ScopedMutexLock lock(_mutex);

        if (!state.empty())
        {
            static osg::ref_ptr<osg::Node> unusedNode = new osg::Node();

            GLObjectsCompiler glcompiler;
            ToCompile toCompile;
            toCompile._data = data;
            toCompile._compiled = glcompiler.compileAsync(
                unusedNode, state, &nv, nullptr);

            _compileQueue.push(std::move(toCompile));
        }
        else
        {
            _mergeQueue.push(data);
        }
    }
    else
#endif
    {
        ScopedMutexLock lock(_mutex);
        _mergeQueue.push(data);
    }
}

void
Merger::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        ScopedMutexLock lock(_mutex);

        // First check the GL compile queue
        while (!_compileQueue.empty())
        {
            ToCompile& next = _compileQueue.front();

            if (next._compiled.isAvailable())
            {
                // compile finished, put it on the merge queue
                _mergeQueue.emplace(std::move(next._data));
                _compileQueue.pop();
            }
            else if (next._compiled.isAbandoned())
            {
                // compile canceled, ditch it
                _compileQueue.pop();
            }
            else
            {
                // nothing to do -- bail out
                break;
            }
        }

        unsigned count = 0u;
        unsigned max_count = _mergesPerFrame;
        if (max_count == 0)
            max_count = INT_MAX;

        while (!_mergeQueue.empty() && count < max_count)
        {
            LoadTileDataOperationPtr next = _mergeQueue.front();
            _mergeQueue.pop();

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
        }

        //if (count > 0)
        //{
        //    OE_INFO << LC << "Merged " << count << std::endl;
        //}
    }

    osg::Node::traverse(nv);
}
