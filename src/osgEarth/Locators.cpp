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
#include <osgEarth/Locators>
#include <osgEarth/GeoData>

using namespace osgEarth;
using namespace osgEarth::Util;

GeoLocator::GeoLocator(const GeoExtent& extent)
{
    setExtent(extent);
}

bool
GeoLocator::setExtent(const GeoExtent& extent)
{
    if (!extent.isValid())
        return false;

    _srs = extent.getSRS();

    double maxX = extent.xMax();
    double maxY = extent.yMax();
    double minX = extent.xMin();
    double minY = extent.yMin();

    if (_srs->isGeographic())
    {
        maxX = osg::DegreesToRadians(maxX);
        maxY = osg::DegreesToRadians(maxY);
        minX = osg::DegreesToRadians(minX);
        minY = osg::DegreesToRadians(minY);
    }

    _transform.set(
        maxX-minX, 0.0,       0.0, 0.0,
        0.0,       maxY-minY, 0.0, 0.0,
        0.0,       0.0,       1.0, 0.0,
        minX,      minY,      0.0, 1.0);

    _inverse.invert(_transform);

    return true;
}

void
GeoLocator::worldToUnit(const osg::Vec3d& model, osg::Vec3d& unit) const
{
    if (_srs->isGeographic())
    {
        double longitude, latitude, height;

        _srs->getEllipsoid()->convertXYZToLatLongHeight(model.x(), model.y(), model.z(),
            latitude, longitude, height);

        unit = osg::Vec3d(longitude, latitude, height) * _inverse;
    }
    else
    {
        unit = model * _inverse;
    }
}

void
GeoLocator::unitToWorld(const osg::Vec3d& unit, osg::Vec3d& model) const
{
    if (_srs->isGeographic())
    {
        osg::Vec3d geographic = unit * _transform;

        _srs->getEllipsoid()->convertLatLongHeightToXYZ(
            geographic.y(), geographic.x(), geographic.z(),
            model.x(), model.y(), model.z());
    }
    else
    {
        model = unit * _transform;
    }
}

void
GeoLocator::mapToUnit(const osg::Vec3d& map, osg::Vec3d& unit) const
{
    if (_srs->isGeographic())
    {
        osg::Vec3d rad(
            osg::DegreesToRadians(map.x()),
            osg::DegreesToRadians(map.y()),
            map.z());

        unit = rad * _inverse;
    }
    else
    {
        unit = map * _inverse;
    }
}

void
GeoLocator::unitToMap(const osg::Vec3d& unit, osg::Vec3d& map) const
{
    map = unit * _transform;

    if (_srs->isGeographic())
    {
        map.x() = osg::RadiansToDegrees(map.x());
        map.y() = osg::RadiansToDegrees(map.y());
    }
}

