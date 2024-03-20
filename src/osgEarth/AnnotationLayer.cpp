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
#include <osgEarth/AnnotationLayer>
#include <osgEarth/AnnotationRegistry>
#include <osgEarth/Registry>

using namespace osgEarth;

REGISTER_OSGEARTH_LAYER(annotations, AnnotationLayer);

//...................................................................

Config
AnnotationLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    return conf;
}

void
AnnotationLayer::Options::fromConfig(const Config& conf)
{
    // NOP
}

//...................................................................

void
AnnotationLayer::init()
{
    VisibleLayer::init();

    _root = new osg::Group();

    deserialize();
}

osg::Node*
AnnotationLayer::getNode() const
{
    return _root.get();
}

osg::Group*
AnnotationLayer::getGroup() const
{
    return _root.get();
}

void
AnnotationLayer::addChild(osg::Node* node)
{
    _root->addChild(node);
}

void
AnnotationLayer::deserialize()
{
    // reset:
    _root->removeChildren(0, _root->getNumChildren());

    // deserialize from the options:
    osg::Group* group = 0L;
    AnnotationRegistry::instance()->create(nullptr, options().getConfig(), getReadOptions(), group);
    if (group)
    {
        _root->addChild(group);
    }
}
