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
    Vec2d latlon((LatLon[LAT]+90)/180, (LatLon[LON]+180)/360);

    // normalized latitude and longitude
    if(latlon[LAT] < 0 || latlon[LAT] > 1)
        return false; // out of range latitude coordinate
    if(latlon[LON] < 0 || latlon[LAT] > 1)
        return false; // out of range longitude coordinate

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
            Coord.x() = 0.5 + Coord.y();
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

SquarePolarLocator::SquarePolarLocator()
//const osgTerrain::Locator& prototype ) :
//osgTerrain::Locator( prototype )
{
    // assumption: incoming key and image extent are square polar.
}

void
SquarePolarLocator::setFaceExtent( const GeoExtent& face_extent )
{
    _face_extent = face_extent;
}

bool
SquarePolarLocator::convertLocalToModel( const osg::Vec3d& local, osg::Vec3d& world ) const
{
#if ((OPENSCENEGRAPH_MAJOR_VERSION <= 2) && (OPENSCENEGRAPH_MINOR_VERSION < 8))
    // OSG 2.7 bug workaround: bug fix in Locator submitted by GW
    const_cast<SquarePolarLocator*>(this)->_inverse.invert( _transform );
#endif

    if ( _coordinateSystemType == GEOCENTRIC )
    {
        // test: the face extent:
        GeoExtent face_extent(
            NULL, -180.0, 45.0, -90, 90.0 );

        // first expand the local coords into the tile's frame:
        GeoExtent tile_extent(
            NULL,
            osg::RadiansToDegrees(_transform(3,0)),
            osg::RadiansToDegrees(_transform(3,1)),
            osg::RadiansToDegrees(_transform(3,0)+_transform(0,0)),
            osg::RadiansToDegrees(_transform(3,1)+_transform(1,1) ) );

        GeoExtent n_tile_ext(
            NULL,
            (tile_extent.xMin()-face_extent.xMin())/face_extent.width(),
            (tile_extent.yMin()-face_extent.yMin())/face_extent.height(),
            (tile_extent.xMax()-face_extent.xMin())/face_extent.width(),
            (tile_extent.yMax()-face_extent.yMin())/face_extent.height() );

        osg::Vec3d gridPoint;
        gridPoint.x() = n_tile_ext.xMin() + local.x() * n_tile_ext.width();
        gridPoint.y() = n_tile_ext.yMin() + local.y() * n_tile_ext.height();
        gridPoint.z() = local.z();

        //gridPoint = local;
        osg::Vec3d s = gridPoint;
        double offset = 0.0;

        if ( gridPoint.x() < gridPoint.y() )
        {
            if ( gridPoint.x() + gridPoint.y() < 1.0 )
            {
                s.x() = 1.0 - gridPoint.y();
                s.y() = gridPoint.x();
                offset += 3;
            }
            else
            {
                s.y() = 1.0 - gridPoint.y();
                s.x() = 1.0 - gridPoint.x();
                offset += 2;
            }
        }
        else if ( gridPoint.x() + gridPoint.y() >= 1.0 )
        {
            s.x() = gridPoint.y();
            s.y() = 1.0 - gridPoint.x();
            offset += 1.0;
        }

        s.x() -= s.y();
        if ( s.y() != 0.5 )
        {
            s.x() *= 0.5/(0.5 - s.y());
        }

        s.x() = 0.25 * ( s.x() + offset );
        s.y() = 0.5  * ( s.y() + 1.5 );


        //TODO: account for south face...

        // convert to lat/lon:
        osg::Vec3d modelPoint;
        modelPoint.x() = s.x() * 360.0 - 180.0;
        modelPoint.y() = s.y() * 180.0 - 90.0;
        modelPoint.z() = gridPoint.z();

        // convert to geocentric:
        _ellipsoidModel->convertLatLongHeightToXYZ(
            osg::DegreesToRadians( modelPoint.y() ),
            osg::DegreesToRadians( modelPoint.x() ),
            modelPoint.z(),
            world.x(), world.y(), world.z() );

        return true;
    }
    return false;
}


bool
SquarePolarLocator::convertModelToLocal(const osg::Vec3d& world, osg::Vec3d& local) const
{
#if ((OPENSCENEGRAPH_MAJOR_VERSION <= 2) && (OPENSCENEGRAPH_MINOR_VERSION < 8))
    // OSG 2.7 bug workaround: bug fix in Locator submitted by GW
    const_cast<SquarePolarLocator*>(this)->_inverse.invert( _transform );
#endif

    switch(_coordinateSystemType)
    {
    case(GEOCENTRIC):
        {
            //TODO: implement this correctly... this is just stub code from Locator
            double longitude, latitude, height;

            _ellipsoidModel->convertXYZToLatLongHeight(world.x(), world.y(), world.z(),
                latitude, longitude, height );

            local = osg::Vec3d(longitude, latitude, height) * _inverse;
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

