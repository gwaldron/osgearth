/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/PatchLayer>

using namespace osgEarth;

//............................................................................

Config
PatchLayer::Options::getConfig() const
{
    return VisibleLayer::Options::getConfig();
}

void
PatchLayer::Options::fromConfig(const Config& conf)
{
    //NOP
}

//............................................................................

void
PatchLayer::init()
{
    VisibleLayer::init();    
    setRenderType(RENDERTYPE_TERRAIN_PATCH);
}
