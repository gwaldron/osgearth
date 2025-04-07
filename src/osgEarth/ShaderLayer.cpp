/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "ShaderLayer"

using namespace osgEarth;

#undef LC
#define LC "[Shader] "

//........................................................................

Config
ShaderLayer::Options::getMetadata()
{
    return Config::readJSON(R"(
        {
            "name" : "Shader",
            "properties" : [ ]
        }
    )");
}

Config
ShaderLayer::Options::getConfig() const
{
    Config conf = VisibleLayer::Options::getConfig();
    return conf;
}

void
ShaderLayer::Options::fromConfig(const Config& conf)
{
}

//........................................................................

REGISTER_OSGEARTH_LAYER(shader, ShaderLayer);

void
ShaderLayer::init()
{
    VisibleLayer::init();

    // disable caching
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;

    setRenderType(RENDERTYPE_TERRAIN_SURFACE);
}
