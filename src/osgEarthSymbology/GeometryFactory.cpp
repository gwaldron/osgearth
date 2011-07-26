/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthSymbology/GeometryFactory>
#include <osgEarth/GeoMath>

#define LC "[GeometryFactory] "

using namespace osgEarth;
using namespace osgEarth::Symbology;

GeometryFactory::GeometryFactory( const Map* map ) :
_map( map )
{
    //nop
}

Geometry*
GeometryFactory::createCircle(const osg::Vec3d& center,
                              const Linear&     radius,
                              unsigned          numSegments)
{
    Polygon* geom = new Polygon();

    if ( numSegments == 0 )
    {
        // automatically calculate
        double segLen = radius.as(Units::METERS) / 8.0;
        double circumference = 2*osg::PI*radius.as(Units::METERS);
        numSegments = (unsigned)::ceil(circumference / segLen);
    }
    
    bool isGeodetic = _map.valid() && _map->getProfile()->getSRS()->isGeographic();
    double segAngle = (2.0*osg::PI)/(double)numSegments;

    if ( isGeodetic )
    {
        double earthRadius = _map->getProfile()->getSRS()->getEllipsoid()->getRadiusEquator();
        double lat = osg::DegreesToRadians(center.y());
        double lon = osg::DegreesToRadians(center.x());
        double rM  = radius.as(Units::METERS);

        for( unsigned i=0; i<numSegments; ++i )
        {
            double angle = segAngle * (double)i;
            double clat, clon;
            GeoMath::destination( lat, lon, angle, rM, clat, clon, earthRadius );
            geom->push_back( osg::Vec3d(osg::RadiansToDegrees(clon), osg::RadiansToDegrees(clat), center.z()) );
        }
    }

    else
    {
        double rM = radius.as(Units::METERS);

        for( unsigned i=0; i<numSegments; ++i )
        {
            double angle = segAngle * (double)i;
            double x, y;
            x = center.x() + sin(angle)*rM;
            y = center.y() + cos(angle)*rM;
            geom->push_back( osg::Vec3d(x, y, center.z()) );
        }
    }

    return geom;
}
