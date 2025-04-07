/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
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
