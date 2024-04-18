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

OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(simplify, SimplifyFilter );

bool
SimplifyFilter::isSupported()
{
    return true;
}

SimplifyFilter::SimplifyFilter() :
SimplifyFilterOptions()
{
    //NOP
}

SimplifyFilter::SimplifyFilter( double tolerance, bool preserveTopology) :
    SimplifyFilterOptions()
{
    _tolerance = tolerance;
    _preserveTopology = preserveTopology;
}

SimplifyFilter::SimplifyFilter( const Config& conf ):
    SimplifyFilterOptions( conf )
{
    //nop
}

FilterContext
SimplifyFilter::push( FeatureList& input, FilterContext& context )
{
    if ( !isSupported() )
    {
        OE_WARN << "SimplifyFilter support not enabled" << std::endl;
        return context;
    }

    FeatureList output;
    output.reserve(input.size());

    for (auto& feature : input)
    {     
        if (feature.valid())
        {
            auto geometry = feature->getGeometry();
            osg::ref_ptr< Geometry > simplifiedGeometry;
            if (geometry->simplify(_tolerance.get(), _preserveTopology.get(), simplifiedGeometry) && simplifiedGeometry.valid())
            {
                feature->setGeometry(simplifiedGeometry);
                output.emplace_back(feature);
            }
        }
    }    
    output.swap(input);

    return context;
}
