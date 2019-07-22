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
#include <osgEarthAnnotation/AnnotationLayer>
#include <osgEarthAnnotation/AnnotationRegistry>

using namespace osgEarth;
using namespace osgEarth::Annotation;

REGISTER_OSGEARTH_LAYER(annotations, AnnotationLayer);


AnnotationLayer::AnnotationLayer() :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

AnnotationLayer::AnnotationLayer(const AnnotationLayerOptions& options) :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

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
AnnotationLayer::addChild(AnnotationNode* node)
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
    AnnotationRegistry::instance()->create(0L, options().getConfig(), getReadOptions(), group);
    if (group)
    {
        _root->addChild(group);
    }
}
