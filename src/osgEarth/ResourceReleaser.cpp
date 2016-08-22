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
#include <osgEarth/ResourceReleaser>
#include <osg/Version>

using namespace osgEarth;

#define LC "[ResourceReleaser] "


ResourceReleaser::ResourceReleaser()
{
#if OSG_VERSION_GREATER_OR_EQUAL(3,4,0)
    // ensure this node always gets traversed:
    this->setCullingActive(false);
#endif

    // ensure the draw runs synchronously:
    this->setDataVariance(DYNAMIC);

    // force the draw to run every frame:
    this->setUseDisplayList(false);
}

void
ResourceReleaser::push(osg::Object* node)
{
    Threading::ScopedMutexLock lock(_mutex);
    _toRelease.push_back(node);
}

void
ResourceReleaser::push(const ObjectList& nodes)
{
    Threading::ScopedMutexLock lock(_mutex);
    _toRelease.reserve(_toRelease.size() + nodes.size());
    for (unsigned i = 0; i<nodes.size(); ++i)
        _toRelease.push_back(nodes[i].get());
}

void
ResourceReleaser::drawImplementation(osg::RenderInfo& ri) const
{
    if (!_toRelease.empty())
    {
        Threading::ScopedMutexLock lock(_mutex);
        if (!_toRelease.empty())
        {
            for (ObjectList::const_iterator i = _toRelease.begin(); i != _toRelease.end(); ++i)
            {
                osg::Object* node = i->get();
                node->releaseGLObjects(ri.getState());
            }
            OE_DEBUG << LC << "Released " << _toRelease.size() << " objects\n";
            _toRelease.clear();
        }
    }
}
