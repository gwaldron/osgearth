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
#include <osgEarth/SimplifyFilter>
#include <osgEarth/FilterContext>
#include <osgEarth/GeoMath>
#include <osg/io_utils>
#include <list>
#include <cstdlib>

using namespace osgEarth;

OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(simplify, SimplifyFilter);

bool
SimplifyFilter::isSupported()
{
    return true;
}

SimplifyFilter::SimplifyFilter(const Config& conf) :
    _options(conf)
{
    //nop
}

FilterContext
SimplifyFilter::push(FeatureList& input, FilterContext& context)
{
    if (!isSupported())
    {
        OE_WARN << "SimplifyFilter support not enabled" << std::endl;
        return context;
    }

    FeatureList output;
    output.reserve(input.size());

    double t = options().tolerance().value();

    if (options().toleranceIsPercentage() == true && context.extent().isSet())
    {
        // 0.01 = percentage to value
        auto w = context.extent()->width() / 2.0;
        t = w * options().tolerance().value() * 0.01;
    }

    for (auto& feature : input)
    {
        if (feature.valid())
        {
            auto geometry = feature->getGeometry();

            if (options().preserveAllFeatures() == false &&
                !feature->getGeometry()->isPointSet() &&
                feature->getGeometry()->getLength() < t)
            {
                continue;
            }

            auto simplifiedGeometry = geometry->simplify(t, options().preserveTopology().value());
            if (simplifiedGeometry.valid())
            {
                feature->setGeometry(simplifiedGeometry.get());
                output.emplace_back(feature);
            }
        }
    }
    output.swap(input);

    return context;
}
