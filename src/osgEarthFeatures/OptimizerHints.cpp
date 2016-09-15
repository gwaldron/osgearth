/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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

#include <osgEarthFeatures/OptimizerHints>

using namespace osgEarth::Features;


OptimizerHints::OptimizerHints()
{
    included = 0;
    excluded = 0;
}

OptimizerHints::OptimizerHints( const OptimizerHints& rhs )
{
    included = rhs.included;
    excluded = rhs.excluded;
}

void
OptimizerHints::include( osgUtil::Optimizer::OptimizationOptions value )
{
    included |= (int)value;
}

void
OptimizerHints::exclude( osgUtil::Optimizer::OptimizationOptions value )
{
    excluded |= (int)value;
}

osgUtil::Optimizer::OptimizationOptions
OptimizerHints::getIncludedOptions() const
{
    return (osgUtil::Optimizer::OptimizationOptions)included;
}

osgUtil::Optimizer::OptimizationOptions
OptimizerHints::getExcludedOptions() const
{
    return (osgUtil::Optimizer::OptimizationOptions)excluded;
}
