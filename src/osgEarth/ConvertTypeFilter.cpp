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
#include <osgEarth/ConvertTypeFilter>

using namespace osgEarth;
using namespace osgEarth::Util;

OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(convert, ConvertTypeFilter)


ConvertTypeFilter::ConvertTypeFilter(const Geometry::Type& toType) :
    _toType(toType)
{
    // NOP
}

ConvertTypeFilter::ConvertTypeFilter(const Config& conf)
{
    if (conf.key() == "convert")
    {
        conf.get("type", "point", _toType, Geometry::TYPE_POINTSET);
        conf.get("type", "line", _toType, Geometry::TYPE_LINESTRING);
        conf.get("type", "polygon", _toType, Geometry::TYPE_POLYGON);
    }
}

Config ConvertTypeFilter::getConfig() const
{
    Config config("convert");
    config.set("type", "point", _toType, Geometry::TYPE_POINTSET);
    config.set("type", "line", _toType, Geometry::TYPE_LINESTRING);
    config.set("type", "polygon", _toType, Geometry::TYPE_POLYGON);
    return config;
}

FilterContext
ConvertTypeFilter::push( FeatureList& input, FilterContext& context )
{
    if ( !isSupported() )
    {
        OE_WARN << "ConvertTypeFilter support not enabled" << std::endl;
        return context;
    }

    if (_toType == Geometry::TYPE_UNKNOWN)
    {
        return context;
    }

    bool ok = true;
    for (auto& feature : input)
    {
        if (feature && feature->getGeometry() && feature->getGeometry()->getComponentType() != _toType.value())
        {
            auto cloned = feature->getGeometry()->cloneAs(_toType.value());
            if (cloned)
            {
                OE_SOFT_ASSERT(cloned->isValid());
                feature->setGeometry(cloned);
            }
        }
    }

    return context;
}
