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

using namespace osgEarth;

#define LC "[osgEarth::Cube] "

// --------------------------------------------------------------------------

bool
CubeUtils::latLonToFaceCoords(double lat_deg, double lon_deg,
                              double& out_x, double& out_y, int& out_face,
                              int faceHint )
{
    // normalized latitude and longitude
    double nlat = (lat_deg+90.0)/180.0;
    double nlon = (lon_deg+180.0)/360.0;

    // check for out-of-range:
    if ( nlat < 0 || nlat > 1 || nlon < 0 || nlon > 1 )
        return false;

    int face_x;

    if ( faceHint >= 0 )
    {
        out_face = faceHint;
        if ( faceHint < 4 )
        {
            face_x = faceHint;
        }
        else
        {
            face_x = (int)(4 * nlon);
            if ( face_x == 4 ) 
                face_x = 3;
        }        
    }
    else
    {
        int face_x = (int)(4 * nlon);
        if ( face_x == 4 )
            face_x = 3;

        int face_y = (int)(2 * nlat + 0.5);
        if ( face_y == 1 )
            out_face = face_x;
        else
            out_face = face_y < 1 ? 5 : 4;

        if ( osg::equivalent( lat_deg, -45 ) )
            out_face = 5;
    }

    out_x = 4 * nlon - face_x;
    out_y = 2 * nlat - 0.5;

    if(out_face < 4) // equatorial calculations done
        return true;

    double tmp;
    if(out_face == 4) // north polar face
    {
        out_y = 1.5 - out_y;
        out_x = 2 * (out_x - 0.5) * out_y + 0.5;
        switch(face_x)
        {
        case 0: // bottom
            out_y = 0.5 - out_y;
            break;
        case 1: // right side, swap and reverse lat
            tmp = out_x;
            out_x = 0.5 + out_y;
            out_y = tmp;
            break;
        case 2: // top; reverse lat and lon
            out_x = 1 - out_x;
            out_y = 0.5 + out_y;
            break;
        case 3: // left side; swap and reverse lon
            tmp = out_x;
            out_x = 0.5 - out_y;
            out_y = 1 - tmp;
            break;
        }
    }
    else // south polar face
    {
        out_y += 0.5;
        out_x = 2 * (out_x - 0.5) * out_y + 0.5;
        switch(face_x)
        {
        case 0: // left
            tmp = out_x;
            out_x = 0.5 - out_y;
            out_y = tmp;
            break;
        case 1: // top
            out_y = 0.5 + out_y;
            break;
        case 2: // right
            tmp = out_x;
            out_x = 0.5 + out_y;
            out_y = 1 - tmp;
            break;
        case 3: // bottom
            out_x = 1 - out_x;
            out_y = 0.5 - out_y;
            break;
        }
    }
    return true;
}

bool
CubeUtils::faceCoordsToLatLon( double x, double y, int face, double& out_lat_deg, double& out_lon_deg )
{
    double offset = 0.0;
    osg::Vec2d s( x, y );

    // validate coordinate range:
    if ( x < 0 || x > 1 || y < 0 || y > 1 )
    {
        OE_WARN << LC << "faceCoordToLatLon: input out of range" << std::endl;
        return false;
    }

    if ( face < 4 ) // equatorial faces
    {
        s.x() = (x + face) * 0.25;
        s.y() = (y + 0.5) * 0.5;
    }
    else if( face == 4 ) // north polar face
    {
        if ( x < y ) // left or top quadrant
        {
            if(x + y < 1.0) // left quadrant
            {
                s.x() = 1.0 - y;
                s.y() = x;
                offset += 3;
            }
            else // top quadrant
            {
                s.y() = 1.0 - y;
                s.x() = 1.0 - x;
                offset += 2;
            }
        }
        else if( x + y >= 1.0 ) // right quadrant
        {
            s.x() = y;
            s.y() = 1.0 - x;
            offset += 1.0;
        }
        s.x() -= s.y();
        if(s.y() != 0.5)
            s.x() *= 0.5 / (0.5 - s.y());

        s.x() = (s.x() + offset) * 0.25;
        s.y() = (s.y() + 1.5) * 0.5;
    }
    else if ( face == 5 ) // south polar face
    {
        offset = 1.0;
        if ( x > y ) // right or bottom quadrant
        {
            if( x + y >= 1.0) // right quadrant
            {
                s.x() = 1.0 - y;
                s.y() = x - 0.5;
                offset += 1.0;
            }
            else // bottom quadrant
            {
                s.x() = 1.0 - x;
                s.y() = 0.5 - y;
                offset += 2;
            }
        }
        else // left or top quadrant
        {
            if(x + y < 1.0) // left quadrant
            {
                s.x() = y;
                s.y() = 0.5 - x;
                offset -= 1.0;
            }
            else // top quadrant
                s.y() = y - 0.5;
        }
        if(s.y() != 0)
            s.x() = (s.x() - 0.5) * 0.5 / s.y() + 0.5;
        s.x() = (s.x() + offset) * 0.25;
        s.y() *= 0.5;
    }
    else 
    {
        return false; // invalid face specification
    }

    // convert to degrees
    out_lon_deg = s.x() * 360 - 180;
    out_lat_deg = s.y() * 180 - 90;

    return true;
}

// --------------------------------------------------------------------------

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

        double lat_deg, lon_deg;
        CubeUtils::faceCoordsToLatLon( faceCoord.x(), faceCoord.y(), _face, lat_deg, lon_deg );

        //OE_NOTICE << "LatLon=" << latLon <<  std::endl;

        // convert to geocentric:
        _ellipsoidModel->convertLatLongHeightToXYZ(
            osg::DegreesToRadians( lat_deg ),
            osg::DegreesToRadians( lon_deg ),
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
            double x, y;

            double lat_deg = osg::RadiansToDegrees(latitude);
            double lon_deg = osg::RadiansToDegrees(longitude);

            bool success = CubeUtils::latLonToFaceCoords( lat_deg, lon_deg, x, y, face, _face );

            if (!success)
            {
                OE_NOTICE << LC << "Couldn't convert to face coords " << std::endl;
            }
            if (face != _face)
            {
                OE_NOTICE << LC
                    << "Face should be " << _face << " but is " << face
                    << ", lat = " << lat_deg
                    << ", lon = " << lon_deg
                    << std::endl;
            }

            local = osg::Vec3d( x, y, height ) * _inverse;
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

// --------------------------------------------------------------------------

CubeSpatialReference::CubeSpatialReference( void* handle ) :
SpatialReference( handle, "OSGEARTH", "cube", "Cube" )
{
    //nop
}

GeoLocator*
CubeSpatialReference::createLocator(double xmin, double ymin, double xmax, double ymax,
                                    bool plate_carre) const
{
    int face;
    toFace( xmin, ymin, xmax, ymax, face );

    GeoLocator* result = new CubeFaceLocator( face );

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
CubeSpatialReference::preTransform(double& x, double& y, void* context) const
{
    // Convert the incoming points from cube => face => lat/long.
    int face;
    if ( !toFace( x, y, face ) )
    {
        OE_WARN << LC << "Failed to convert (" << x << "," << y << ") into face coordinates." << std::endl;
        return false;
    }

    double lat_deg, lon_deg;
    bool success = CubeUtils::faceCoordsToLatLon( x, y, face, lat_deg, lon_deg );
    if (!success)
    {
        OE_WARN << LC << "Could not transform face coordinates to lat lon" << std::endl;
        return false;
    }
    x = lon_deg;
    y = lat_deg;
    return true;
}

bool
CubeSpatialReference::postTransform(double& x, double& y, void* context) const
{
    //Convert the incoming points from lat/lon back to face coordinates
    int face;
    double out_x, out_y;

    // convert from lat/long to x/y/face
    bool success = CubeUtils::latLonToFaceCoords( y, x, out_x, out_y, face );
    if (!success)
    {
        OE_WARN << LC << "Could not transform face coordinates to lat lon" << std::endl;
        return false;
    }

    //TODO: what to do about boundary points?

    if ( !fromFace( out_x, out_y, face ) )
    {
        OE_WARN << LC << "fromFace(" << out_x << "," << out_y << "," << face << ") failed" << std::endl;
        return false;
    }
    
    x = out_x;
    y = out_y;

    return true;
}

bool
CubeSpatialReference::toFace( double& in_out_x, double& in_out_y, int& out_face ) const
{
    // convert from unicube space (0,0=>6,1) to face space (0,0=>1,1 + face#)
    // too tired to compute a formula right now
    out_face = 
        in_out_x <= 1.0 ? 0 :
        in_out_x <= 2.0 ? 1 :
        in_out_x <= 3.0 ? 2 :
        in_out_x <= 4.0 ? 3 :
        in_out_x <= 5.0 ? 4 : 5;

    in_out_x = in_out_x - (double)out_face;
    // y unchanged
    return true;
}

bool
CubeSpatialReference::toFace(double& in_out_xmin, double& in_out_ymin,
                             double& in_out_xmax, double& in_out_ymax,
                             int& out_face) const
{
    int min_face = 
        in_out_xmin < 1.0 ? 0 :
        in_out_xmin < 2.0 ? 1 :
        in_out_xmin < 3.0 ? 2 :
        in_out_xmin < 4.0 ? 3 :
        in_out_xmin < 5.0 ? 4 : 5;

    int max_face =
        in_out_xmax <= 1.0 ? 0 :
        in_out_xmax <= 2.0 ? 1 :
        in_out_xmax <= 3.0 ? 2 :
        in_out_xmax <= 4.0 ? 3 :
        in_out_xmax <= 5.0 ? 4 : 5;

    if ( min_face != max_face )
    {
        OE_WARN << LC << "Min face <> Max face!" << std::endl;
        return false;
    }

    out_face = min_face;

    in_out_xmin -= (double)out_face;
    in_out_xmax -= (double)out_face;

    // y values are unchanged
    return true;
}

bool
CubeSpatialReference::fromFace( double& in_out_x, double& in_out_y, int face ) const
{
    // convert from face space (0,0=>1,1 + face#) to unicube space (0,0=>6,1)
    in_out_x = (double)face + in_out_x;
    // y unchanged
    return true;
}

#define LL 0
#define LR 1
#define UR 2
#define UL 3
#define SMALLEST( W,X,Y,Z ) osg::minimum(W, osg::minimum( X, osg::minimum( Y, Z ) ) )
#define LARGEST( W,X,Y,Z ) osg::maximum(W, osg::maximum( X, osg::maximum( Y, Z ) ) )

bool
CubeSpatialReference::transformExtent(const SpatialReference* to_srs,
                                      double& in_out_xmin,
                                      double& in_out_ymin,
                                      double& in_out_xmax,
                                      double& in_out_ymax,
                                      void* context ) const
{
    bool ok = true;

    // figure out which face we are in (find the face of the center point):
    double tempX = 0.5*(in_out_xmin+in_out_xmax);
    double tempY = 0.5*(in_out_ymin+in_out_ymax);
    int face;
    toFace( tempX, tempY, face );

    // for equatorial faces, the normal transformation process will suffice (since it will call into
    // pre/postTransform).
    if ( face < 4 )
    {
        return SpatialReference::transformExtent( to_srs, in_out_xmin, in_out_ymin, in_out_xmax, in_out_ymax );
    }

    double x[4] = { in_out_xmin, in_out_xmax, in_out_xmax, in_out_xmin };
    double y[4] = { in_out_ymin, in_out_ymin, in_out_ymax, in_out_ymax };

    bool north = face == 4; // else south
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
