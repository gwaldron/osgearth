/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
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
