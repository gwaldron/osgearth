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
