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

#include <osgEarth/ECEF>

using namespace osgEarth;

#define LC "[ECEF] "

// --------------------------------------------------------------------------


osg::Matrixd
ECEF::createLocalToWorld( const osg::Vec3d& input )
{
    double X = input.x(), Y = input.y(), Z = input.z();

    osg::Matrixd localToWorld;
    localToWorld.makeTranslate(X,Y,Z);

    // normalize X,Y,Z
    double inverse_length = 1.0/sqrt(X*X + Y*Y + Z*Z);

    X *= inverse_length;
    Y *= inverse_length;
    Z *= inverse_length;

    double length_XY = sqrt(X*X + Y*Y);
    double inverse_length_XY = 1.0/length_XY;

    // Vx = |(-Y,X,0)|
    localToWorld(0,0) = -Y*inverse_length_XY;
    localToWorld(0,1) = X*inverse_length_XY;
    localToWorld(0,2) = 0.0;

    // Vy = /(-Z*X/(sqrt(X*X+Y*Y), -Z*Y/(sqrt(X*X+Y*Y),sqrt(X*X+Y*Y))| 
    double Vy_x = -Z*X*inverse_length_XY;
    double Vy_y = -Z*Y*inverse_length_XY;
    double Vy_z = length_XY;
    inverse_length = 1.0/sqrt(Vy_x*Vy_x + Vy_y*Vy_y + Vy_z*Vy_z);            
    localToWorld(1,0) = Vy_x*inverse_length;
    localToWorld(1,1) = Vy_y*inverse_length;
    localToWorld(1,2) = Vy_z*inverse_length;

    // Vz = (X,Y,Z)
    localToWorld(2,0) = X;
    localToWorld(2,1) = Y;
    localToWorld(2,2) = Z;

    return localToWorld;
}

void
ECEF::transformAndLocalize(const osg::Vec3d&       input,
                           const SpatialReference* inputSRS,
                           osg::Vec3d&             output,
                           const osg::Matrixd&     world2local)
{
    osg::Vec3d ecef;
    inputSRS->transformToECEF( input, ecef );
    output = ecef * world2local;
}


void
ECEF::transformAndLocalize(const std::vector<osg::Vec3d>& input,
                           const SpatialReference*        inputSRS,
                           osg::Vec3Array*                output,
                           const osg::Matrixd&            world2local )
{
    output->reserve( output->size() + input.size() );
    for( std::vector<osg::Vec3d>::const_iterator i = input.begin(); i != input.end(); ++i )
    {
        osg::Vec3d ecef;
        inputSRS->transformToECEF( *i, ecef );
        output->push_back( ecef * world2local );
    }
}

void
ECEF::transformAndGetRotationMatrix(const osg::Vec3d&       input,
                                    const SpatialReference* inputSRS,
                                    osg::Vec3d&             out_point,
                                    osg::Matrixd&           out_rotation )
{
    osg::Vec3d geod_point;
    if ( !inputSRS->isGeographic() )
        inputSRS->transform( input, inputSRS->getGeographicSRS(), geod_point );
    else
        geod_point = input;

    const osg::EllipsoidModel* em = inputSRS->getEllipsoid();
    
    em->convertLatLongHeightToXYZ(
        osg::DegreesToRadians( geod_point.y() ),
        osg::DegreesToRadians( geod_point.x() ),
        geod_point.z(),
        out_point.x(), out_point.y(), out_point.z() );

    em->computeCoordinateFrame(
        osg::DegreesToRadians( geod_point.y() ),
        osg::DegreesToRadians( geod_point.x() ),
        out_rotation );
}
