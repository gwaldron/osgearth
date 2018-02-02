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
#include <osgEarth/Metrics>
#include <osg/Version>

using namespace osgEarth;

#define LC "[ResourceReleaser] "


ResourceReleaser::ResourceReleaser()
{
    // ensure this node always gets traversed:
    this->setCullingActive(false);

    // ensure the draw runs synchronously:
    this->setDataVariance(DYNAMIC);

    // force the draw to run every frame:
    this->setUseDisplayList(false);
}

void
ResourceReleaser::push(osg::Object* object)
{
    Threading::ScopedMutexLock lock(_mutex);

    _toRelease.push_back(object);

    for (unsigned i = 0; i<_count.size(); ++i)
        _count[i]++;
}

void
ResourceReleaser::push(const ObjectList& objects)
{
    Threading::ScopedMutexLock lock(_mutex);

    for (unsigned i = 0; i<_count.size(); ++i)
        _count[i] += objects.size();

    _toRelease.reserve(_toRelease.size() + objects.size());
    for (unsigned i = 0; i<objects.size(); ++i)
        _toRelease.push_back(objects[i].get());
}

void
ResourceReleaser::drawImplementation(osg::RenderInfo& ri) const
{
    releaseGLObjects(ri.getState());
}

void
ResourceReleaser::releaseGLObjects(osg::State* state) const
{
    osg::Drawable::releaseGLObjects(state);

    if (!_toRelease.empty())
    {
        Threading::ScopedMutexLock lock(_mutex);
        if (!_toRelease.empty())
        {
            METRIC_SCOPED("ResourceReleaser");
            for (ObjectList::const_iterator i = _toRelease.begin(); i != _toRelease.end(); ++i)
            {
                osg::Object* object = i->get();
                object->releaseGLObjects(0L);
            }
            OE_DEBUG << LC << "Released " << _toRelease.size() << " objects\n";
            _toRelease.clear();
        }
    }
    
    if (state && _count[state->getContextID()] > 0)
    {
        osg::Texture::flushAllDeletedTextureObjects(state->getContextID());     
        osg::GLBufferObject::flushAllDeletedBufferObjects(state->getContextID());
        _count[state->getContextID()] = 0;
    }
}
