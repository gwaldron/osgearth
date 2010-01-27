/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osgEarth/Cube>
#include <osg/Math>
#include <osg/Notify>
#include <sstream>
#include <algorithm>

using namespace osg;
using namespace osgEarth;

#define LAT 0
#define LON 1

/* Computes face coordinates from latitude and longitude
* param LatLon input coordinates, latitude and longitude (degrees)
* param Coord a 2D vector of calculated face coordinates
* param Face the globe face calculated for vertex
* returns FALSE if coordinates are out of range, else TRUE
*/
bool CubeGridUtils::LatLonToFaceCoord(const Vec2d& LatLon, osg::Vec2d& Coord, int& Face)
{
    Vec2d latlon((LatLon[LAT]+90.0)/180.0, (LatLon[LON]+180.0)/360.0);

    // normalized latitude and longitude
    if(latlon[LAT] < 0 || latlon[LAT] > 1)
        return false; // out of range latitude coordinate
    if(latlon[LON] < 0 || latlon[LAT] > 1)
        return false; // out of range longitude coordinate

    int origFace = Face;


    int face_x = (int)(4 * latlon[LON]);
    int face_y = (int)(2 * latlon[LAT] + 0.5);
    if(face_x == 4)
        face_x = 3;
    if(face_y == 1)
        Face = face_x;
    else
        Face = face_y < 1 ? 5 : 4;
    Coord.x() = 4 * latlon[LON] - face_x;
    Coord.y() = 2 * latlon[LAT] - 0.5;

    //NOTE:  This differs slightly from the GeoMatrix code.  We want -45 to always be considered in Face 5
    if (osg::equivalent(LatLon[LAT],-45))
    {
        Face = 5;
    }

    if(Face < 4) // equatorial calculations done
        return true;
    double tmp;
    if(Face == 4) // north polar face
    {
        Coord.y() = 1.5 - Coord.y();
        Coord.x() = 2 * (Coord.x() - 0.5) * Coord.y() + 0.5;
        switch(face_x)
        {
        case 0: // bottom
            Coord.y() = 0.5 - Coord.y();
            break;
        case 1: // right side, swap and reverse lat
            tmp = Coord.x();
            Coord.x() = 0.5 + Coord.y();
            Coord.y() = tmp;
            break;
        case 2: // top; reverse lat and lon
            Coord.x() = 1 - Coord.x();
            Coord.y() = 0.5 + Coord.y();
            break;
        case 3: // left side; swap and reverse lon
            tmp = Coord.x();
            Coord.x() = 0.5 - Coord.y();
            Coord.y() = 1 - tmp;
            break;
        }
    }
    else // south polar face
    {
        Coord.y() += 0.5;
        Coord.x() = 2 * (Coord.x() - 0.5) * Coord.y() + 0.5;
        switch(face_x)
        {
        case 0: // left
            tmp = Coord.x();
            Coord.x() = 0.5 - Coord.y();
            Coord.y() = tmp;
            break;
        case 1: // top
            Coord.y() = 0.5 + Coord.y();
            break;
        case 2: // right
            tmp = Coord.x();
            Coord.x() = 0.5 + Coord.y();
            Coord.y() = 1 - tmp;
            break;
        case 3: // bottom
            Coord.x() = 1 - Coord.x();
            Coord.y() = 0.5 - Coord.y();
            break;
        }
    }
    return true;
}


/* Converts vertex face coordinates to lat/lon.
* param Coord input a 2D vector of face x,y coordinates
* param Face input the globe face index
* param LatLon output vertex coordinates, latitude and longitude
* returns FALSE if face coordinates are out of range, else TRUE
*/
bool CubeGridUtils::FaceCoordToLatLon(const Vec2d& Coord, const int Face, Vec2d& LatLon)
{
    double offset = 0.0;
    Vec2d s(Coord.x(), Coord.y()); // default
    if(Coord.x() > 1 || Coord.x() < 0)
        return false; // out of range X coordinate
    if(Coord.y() > 1 || Coord.y() < 0)
        return false; // out of range Y coordinate
    if(Face < 4) // equatorial faces
    {
        s.x() = (Coord.x() + Face) * 0.25;
        s.y() = (Coord.y() + 0.5) * 0.5;
    }
    else if(Face == 4) // north polar face
    {
        if(Coord.x() < Coord.y()) // left or top quadrant
        {
            if(Coord.x() + Coord.y() < 1.0) // left quadrant
            {
                s.x() = 1.0 - Coord.y();
                s.y() = Coord.x();
                offset += 3;
            }
            else // top quadrant
            {
                s.y() = 1.0 - Coord.y();
                s.x() = 1.0 - Coord.x();
                offset += 2;
            }
        }
        else if(Coord.x() + Coord.y() >= 1.0) // right quadrant
        {
            s.x() = Coord.y();
            s.y() = 1.0 - Coord.x();
            offset += 1.0;
        }
        s.x() -= s.y();
        if(s.y() != 0.5)
            s.x() *= 0.5 / (0.5 - s.y());
        s.x() = (s.x() + offset) * 0.25;
        s.y() = (s.y() + 1.5) * 0.5;
    }
    else if(Face == 5) // south polar face
    {
        offset = 1.0;
        if(Coord.x() > Coord.y()) // right or bottom quadrant
        {
            if(Coord.x() + Coord.y() >= 1.0) // right quadrant
            {
                s.x() = 1.0 - Coord.y();
                s.y() = Coord.x() - 0.5;
                offset += 1.0;
            }
            else // bottom quadrant
            {
                s.x() = 1.0 - Coord.x();
                s.y() = 0.5 - Coord.y();
                offset += 2;
            }
        }
        else // left or top quadrant
        {
            if(Coord.x() + Coord.y() < 1.0) // left quadrant
            {
                s.x() = Coord.y();
                s.y() = 0.5 - Coord.x();
                offset -= 1.0;
            }
            else // top quadrant
                s.y() = Coord.y() - 0.5;
        }
        if(s.y() != 0)
            s.x() = (s.x() - 0.5) * 0.5 / s.y() + 0.5;
        s.x() = (s.x() + offset) * 0.25;
        s.y() *= 0.5;
    }
    else return false; // invalid face specification
    LatLon[LON] = s.x() * 360 - 180; // convert to degrees
    LatLon[LAT] = s.y() * 180 - 90;
    return true;
}

/********************************************************************************************/

CubeFaceLocator::CubeFaceLocator(unsigned int face):
_face(face)
{
    //NOP
}


bool
CubeFaceLocator::convertLocalToModel( const osg::Vec3d& local, osg::Vec3d& world ) const
{
#if ((OPENSCENEGRAPH_MAJOR_VERSION <= 2) && (OPENSCENEGRAPH_MINOR_VERSION < 8))
    // OSG 2.7 bug workaround: bug fix in Locator submitted by GW
    const_cast<CubeFaceLocator*>(this)->_inverse.invert( _transform );
#endif

    if ( _coordinateSystemType == GEOCENTRIC )
    {
        //Convert the NDC coordinate into face space
        osg::Vec3d faceCoord = local * _transform;

        osg::Vec2d latLon;
        CubeGridUtils::FaceCoordToLatLon(osg::Vec2d(faceCoord.x(), faceCoord.y()), _face, latLon);

        //osg::notify(osg::NOTICE) << "LatLon=" << latLon <<  std::endl;

        // convert to geocentric:
        _ellipsoidModel->convertLatLongHeightToXYZ(
            osg::DegreesToRadians( latLon.x() ),
            osg::DegreesToRadians( latLon.y() ),
            local.z(),
            world.x(), world.y(), world.z() );

        return true;
    }    
    return true;
}


bool
CubeFaceLocator::convertModelToLocal(const osg::Vec3d& world, osg::Vec3d& local) const
{
#if ((OPENSCENEGRAPH_MAJOR_VERSION <= 2) && (OPENSCENEGRAPH_MINOR_VERSION < 8))
    // OSG 2.7 bug workaround: bug fix in Locator submitted by GW
    const_cast<CubeFaceLocator*>(this)->_inverse.invert( _transform );
#endif

    switch(_coordinateSystemType)
    {
    case(GEOCENTRIC):
        {         
            double longitude, latitude, height;

            _ellipsoidModel->convertXYZToLatLongHeight(world.x(), world.y(), world.z(), latitude, longitude, height );
            int face=-1;
            osg::Vec2d coord;

            /*coord.x() = world.x();
            coord.y() = world.y();
            height = world.z();*/

            double latDeg = osg::RadiansToDegrees(latitude);
            double lonDeg = osg::RadiansToDegrees(longitude);

            bool success = CubeGridUtils::LatLonToFaceCoord(osg::Vec2d(latDeg, lonDeg), coord, face);
            if (!success)
            {
                osg::notify(osg::NOTICE) << "[osgEarth::Cube] Couldn't convert to face coords " << std::endl;
            }
            if (face != _face)
            {
                osg::notify(osg::NOTICE) << "[osgEarth::Cube] Face should be " << _face << " but is " << face << std::endl;
            }

            local = osg::Vec3d(coord.x(), coord.y(), height) * _inverse;
            return true;
        }


    case(GEOGRAPHIC):
    case(PROJECTED):
        // Neither of these is supported for this locator..
        {        
            local = world * _inverse;
            return true;      
        }
    }    

    return false;
}


/********************************************************************************************/


CubeFaceSpatialReference::CubeFaceSpatialReference(void *handle, unsigned int face) :
SpatialReference(handle),
_face(face)
{

}

GeoLocator*
CubeFaceSpatialReference::createLocator(double xmin, double ymin, double xmax, double ymax,
                                        bool plate_carre) const
{
    GeoLocator* result = new CubeFaceLocator( _face );

    osg::Matrixd transform;
    transform.set(
        xmax-xmin, 0.0,       0.0, 0.0,
        0.0,       ymax-ymin, 0.0, 0.0,
        0.0,       0.0,       1.0, 0.0,
        xmin,      ymin,      0.0, 1.0); 
    result->setTransform( transform );

    return result;
}

bool
CubeFaceSpatialReference::preTransform(double &x, double &y) const
{
    //Convert the incoming points from face coordinates to lat/lon
    osg::Vec2d latLon;
    bool success = CubeGridUtils::FaceCoordToLatLon(osg::Vec2d(x,y), _face, latLon);
    if (!success)
    {
        osg::notify(osg::WARN) << "[osgEarth::CubeFaceSpatialReference] could not transform face coordinates to lat lon" << std::endl;
        return false;
    }
    x = latLon.y();
    y = latLon.x();
    return true;
}

bool
CubeFaceSpatialReference::postTransform(double &x, double &y) const
{
    //Convert the incoming points from lat/lon to face coordinates
    int face;
    osg::Vec2d coord;
    bool success = CubeGridUtils::LatLonToFaceCoord(osg::Vec2d(y,x), coord, face);
    if (!success)
    {
        osg::notify(osg::WARN) << "[osgEarth::CubeFaceSpatialReference] could not transform face coordinates to lat lon" << std::endl;
        return false;
    }

    //Make sure the face is the same as the computed face
    if (_face != face)
    {
        osg::notify(osg::WARN) << "[osgEarth::CubeFaceSpatialReference] lat lon " << y << ", " << x << " outside bounds for Cube face " << _face << std::endl;
        return false;
    }
    
    x = coord.x();
    y = coord.y();

    return true;
}

#define LL 0
#define LR 1
#define UR 2
#define UL 3
#define SMALLEST( W,X,Y,Z ) osg::minimum(W, osg::minimum( X, osg::minimum( Y, Z ) ) )
#define LARGEST( W,X,Y,Z ) osg::maximum(W, osg::maximum( X, osg::maximum( Y, Z ) ) )

bool
CubeFaceSpatialReference::transformExtent(const SpatialReference* to_srs,
                                          double& in_out_xmin,
                                          double& in_out_ymin,
                                          double& in_out_xmax,
                                          double& in_out_ymax) const
{
    if ( _face < 4 )
        return SpatialReference::transformExtent( to_srs, in_out_xmin, in_out_ymin, in_out_xmax, in_out_ymax );

    bool ok = true;

    double x[4] = { in_out_xmin, in_out_xmax, in_out_xmax, in_out_xmin };
    double y[4] = { in_out_ymin, in_out_ymin, in_out_ymax, in_out_ymax };

    bool north = _face == 4;
    bool crosses_pole = x[LL] < 0.5 && x[UR] > 0.5 && y[LL] < 0.5 && y[UR] > 0.5;
    bool crosses_date_line = x[UL]+(1-y[UL]) < 1.0 && (1-x[LR])+y[LR] < 1.0 && x[LL]+y[LL] < 1.0;

    if ( crosses_pole ) // full x extent.
    {
        to_srs->getGeographicSRS()->transform( -180.0, north? 45.0 : -90.0, to_srs, in_out_xmin, in_out_ymin );
        to_srs->getGeographicSRS()->transform( 180.0, north? 90.0 : -45.0, to_srs, in_out_xmax, in_out_ymax );
    }

    else if ( transformPoints( to_srs, x, y, 4 ) )
    {
        in_out_ymin = SMALLEST( y[0], y[1], y[2], y[3] );
        in_out_ymax = LARGEST( y[0], y[1], y[2], y[3] );

        // check to see whether the extent crosses the date line boundary. If so,
        // make the UL corner the southwest and the LR corner the east.
        if ( crosses_date_line )
        {
            in_out_xmin = x[UL];
            in_out_xmax = x[LR];
        }
        else
        {
            in_out_xmin = SMALLEST( x[0], x[1], x[2], x[3] );
            in_out_xmax = LARGEST( x[0], x[1], x[2], x[3] );
        }
    }
    else
    {
        // point xform failed
        ok = false;
    }
    return ok;
}
