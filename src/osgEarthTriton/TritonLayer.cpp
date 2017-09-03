/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include "TritonLayer"
#include "TritonNode"
#include "TritonContext"
#include <osgEarth/ImageLayer>

#define LC "[TritonLayer] "

using namespace osgEarth::Triton;

/** Register this layer so it can be used in an earth file */
namespace osgEarth { namespace Triton
{
    REGISTER_OSGEARTH_LAYER(triton, TritonLayer);
    REGISTER_OSGEARTH_LAYER(triton_ocean, TritonLayer);
} }


TritonLayer::TritonLayer() :
osgEarth::VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

TritonLayer::TritonLayer(const TritonLayerOptions& options) :
osgEarth::VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

void
TritonLayer::init()
{
    OE_INFO << LC << "Creating a TritonLayer\n";

    osgEarth::VisibleLayer::init();

    this->setName("Triton");
    setRenderType(RENDERTYPE_NONE);

    TritonOptions legacyOptions(options());
    _tritonNode = new TritonNode(legacyOptions);
}

osg::Node*
TritonLayer::getOrCreateNode()
{
    return _tritonNode.get();
}

void
TritonLayer::setMaskLayer(const osgEarth::ImageLayer* maskLayer)
{
    _tritonNode->setMaskLayer(maskLayer);
}

void
TritonLayer::addedToMap(const osgEarth::Map* map)
{    
    if (options().maskLayer().isSet())
    {
        // listen for the mask layer.
        _layerListener.listen(map, options().maskLayer().get(), this, &TritonLayer::setMaskLayer);
    }      
}

void
TritonLayer::removedFromMap(const osgEarth::Map* map)
{
    if (options().maskLayer().isSet())
    {
        _layerListener.clear();
        setMaskLayer(0L);
    }
}

