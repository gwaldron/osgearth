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
#include <osgEarth/CentroidFilter>
#include <osgEarth/FilterContext>

#define LC "[CentroidFilter] "

using namespace osgEarth;

//------------------------------------------------------------------------

CentroidFilter::CentroidFilter()
{
    //NOP
}

FilterContext
CentroidFilter::push(FeatureList& features, FilterContext& context)
{
    FeatureList output;

    for(auto& feature : features)
    {        
        auto geom = feature->getGeometry();
        if ( !geom )
            continue;

        PointSet* newGeom = new PointSet();
        newGeom->push_back(geom->getBounds().center());

        auto new_feature = new Feature(*feature);
        new_feature->setGeometry(newGeom);
        output.emplace_back(new_feature);
    }

    features.swap(output);
    return context;
}
