/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include "TritonIntersections"

using namespace osgEarth;
using namespace osgEarth::Triton;

TritonIntersections::TritonIntersections() :
_maxRange(2.0, Units::KILOMETERS)
{
}

void
TritonIntersections::setAnchor(const GeoPoint& value)
{
    _anchor = value;

    // zero out the other stuff:
    _anchor.z() = 0.0;
    _anchor.altitudeMode() = ALTMODE_ABSOLUTE;
}

void
TritonIntersections::addLocalPoint(const osg::Vec3d& p)
{
    _input.push_back(p);
    _heights.resize(_input.size());
    _normals.resize(_input.size());
}

void
TritonIntersections::setMaxRange(const Distance& range)
{
    _maxRange = range;
}
