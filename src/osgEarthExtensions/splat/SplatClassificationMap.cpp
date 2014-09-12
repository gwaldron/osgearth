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
#include "SplatClassificationMap"
#include <osgEarth/Config>

using namespace osgEarth;
using namespace osgEarth::Extensions::Splat;

#define LC "[SplatClassificationMap] "

//............................................................................

SplatClassificationMap::SplatClassificationMap()
{
    //nop
}

void
SplatClassificationMap::fromConfig(const Config& conf)
{
    //todo
}

Config
SplatClassificationMap::getConfig() const
{
    Config conf;

    OE_WARN << LC << "*** SplatClassificationMap::getConfig is not yet implemented ***\n";
    //todo
    return conf;
}
