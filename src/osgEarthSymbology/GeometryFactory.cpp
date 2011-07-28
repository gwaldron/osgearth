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

GeometryFactory::GeometryFactory( const SpatialReference* srs ) :
_srs( srs )
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
    
    double segAngle = (2.0*osg::PI)/(double)numSegments;

    if ( _srs.valid() && _srs->isGeographic() )
    {
        double earthRadius = _srs->getEllipsoid()->getRadiusEquator();
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

Geometry*
GeometryFactory::createArc(const osg::Vec3d& center,
                           const Linear&     radius,
                           const Angular&    start,
                           const Angular&    end,
                           unsigned          numSegments)
{
    Geometry* geom = new LineString();

    if ( numSegments == 0 )
    {
        // automatically calculate
        double segLen = radius.as(Units::METERS) / 8.0;
        double circumference = 2*osg::PI*radius.as(Units::METERS);
        numSegments = (unsigned)::ceil(circumference / segLen);
    }

    double startRad = std::min( start.as(Units::RADIANS), end.as(Units::RADIANS) );
    double endRad   = std::max( start.as(Units::RADIANS), end.as(Units::RADIANS) );
    double span     = endRad - startRad;    
    double step     = span/(double)numSegments;

    if ( _srs.valid() && _srs->isGeographic() )
    {
        double earthRadius = _srs->getEllipsoid()->getRadiusEquator();
        double lat = osg::DegreesToRadians(center.y());
        double lon = osg::DegreesToRadians(center.x());
        double rM  = radius.as(Units::METERS);

        for( unsigned i=0; i<=numSegments; ++i )
        {
            double angle = startRad + step*(double)i;
            double clat, clon;
            GeoMath::destination( lat, lon, angle, rM, clat, clon, earthRadius );
            geom->push_back( osg::Vec3d(osg::RadiansToDegrees(clon), osg::RadiansToDegrees(clat), center.z()) );
        }
    }

    else
    {
        double rM = radius.as(Units::METERS);

        for( unsigned i=0; i<=numSegments; ++i )
        {
            double angle = startRad + step*(double)i;
            double x, y;
            x = center.x() + sin(angle)*rM;
            y = center.y() + cos(angle)*rM;
            geom->push_back( osg::Vec3d(x, y, center.z()) );
        }
    }

    return geom;
}

Geometry*
GeometryFactory::createEllipse(const osg::Vec3d& center,
                               const Linear&     radiusMajor,
                               const Linear&     radiusMinor,
                               const Angular&    rotationAngle,
                               unsigned          numSegments)
{
    Polygon* geom = new Polygon();

    if ( numSegments == 0 )
    {
        // automatically calculate
        double ravgM = 0.5*(radiusMajor.as(Units::METERS) + radiusMinor.as(Units::METERS));
        double segLen = ravgM / 8.0;
        double circumference = 2*osg::PI*ravgM;
        numSegments = (unsigned)::ceil(circumference / segLen);
    }
    
    double segAngle = 2.0*osg::PI/(double)numSegments;

    if ( _srs.valid() && _srs->isGeographic() )
    {
        double earthRadius = _srs->getEllipsoid()->getRadiusEquator();
        double lat = osg::DegreesToRadians(center.y());
        double lon = osg::DegreesToRadians(center.x());
        double a = radiusMajor.as(Units::METERS);
        double b = radiusMinor.as(Units::METERS);
        double g = rotationAngle.as(Units::RADIANS) - osg::PI_2;

        for( unsigned i=0; i<numSegments; ++i )
        {
            double angle = segAngle * (double)i;
            double t = angle - osg::PI_2;
            double clat, clon;

            double rA = (b*b-a*a)*cos(2*t - 2*g) + a*a + b*b;
            double q = sqrt(2.0)*a*b*sqrt(rA);
            double r = q/rA;

            GeoMath::destination( lat, lon, angle, r, clat, clon, earthRadius );
            geom->push_back( osg::Vec3d(osg::RadiansToDegrees(clon), osg::RadiansToDegrees(clat), center.z()) );
        }
    }
    else
    {
        double a = radiusMajor.as(Units::METERS);
        double b = radiusMinor.as(Units::METERS);
        double g = rotationAngle.as(Units::RADIANS) - osg::PI_2;
        double sing = sin(g), cosg = cos(g);

        for( unsigned i=0; i<numSegments; ++i )
        {
            double angle = segAngle * (double)i;
            double t = angle - osg::PI_2;
            double cost = cos(t), sint = sin(t);
            double x = center.x() + a*cost*cosg - b*sint*sing;
            double y = center.y() + a*cost*sing + b*sint*cosg;

            geom->push_back( osg::Vec3d(x, y, center.z()) );
        }
    }

    return geom;
}
