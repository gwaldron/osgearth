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
#include "BumpMapLayer"
#include "BumpMapTerrainEffect"

#include <osgEarth/TerrainEngineNode>

using namespace osgEarth;

#define LC "[BumpMapLayer] "

REGISTER_OSGEARTH_LAYER(bumpmap, BumpMapLayer);


void
BumpMapLayer::prepareForRendering(TerrainEngine* engine)
{
    if ( !engine )
        return;

    osg::ref_ptr<osg::Image> image = options().imageURI()->getImage(getReadOptions());
    if ( !image.valid() )
    {
        OE_WARN << LC << "Failed; unable to load normal map image from "
            << options().imageURI()->full() << "\n";
        return;
    }

    _effect = new BumpMapTerrainEffect();

    _effect->setBumpMapImage(image.get());

    if (options().intensity().isSet())
        _effect->getIntensityUniform()->set(options().intensity().get());

    if (options().scale().isSet())
        _effect->getScaleUniform()->set(options().scale().get());

    if (options().octaves().isSet())
        _effect->setOctaves(options().octaves().get());

    if (options().baseLOD().isSet())
        _effect->setBaseLOD(options().baseLOD().get());

    engine->addEffect(_effect);

    OE_DEBUG << LC << "Installed.\n";

    onVisibleChanged([&](auto* layer)
        {
            _effect->setActive(layer->getVisible());
        });
}
