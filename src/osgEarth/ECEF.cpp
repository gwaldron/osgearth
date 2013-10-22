/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/Notify>

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
                           const SpatialReference* outputSRS,
                           const osg::Matrixd&     world2local)
{
    osg::Vec3d ecef;
    //inputSRS->transformToECEF( input, ecef );
    inputSRS->transform( input, outputSRS->getECEF(), ecef );
    output = ecef * world2local;
}


void
ECEF::transformAndLocalize(const std::vector<osg::Vec3d>& input,
                           const SpatialReference*        inputSRS,
                           osg::Vec3Array*                output,
                           const SpatialReference*        outputSRS,
                           const osg::Matrixd&            world2local )
{
    const SpatialReference* ecefSRS = outputSRS->getECEF();
    output->reserve( output->size() + input.size() );

    for( std::vector<osg::Vec3d>::const_iterator i = input.begin(); i != input.end(); ++i )
    {
        osg::Vec3d ecef;
        inputSRS->transform( *i, ecefSRS, ecef );
        //inputSRS->transformToECEF( *i, ecef );
        output->push_back( ecef * world2local );
    }
}


void
ECEF::transformAndLocalize(const std::vector<osg::Vec3d>& input,
                           const SpatialReference*        inputSRS,
                           osg::Vec3Array*                out_verts,
                           osg::Vec3Array*                out_normals,
                           const SpatialReference*        outputSRS,
                           const osg::Matrixd&            world2local )
{
    const SpatialReference* ecefSRS = outputSRS->getECEF();
    out_verts->reserve( out_verts->size() + input.size() );

    if ( out_normals )
        out_normals->reserve( out_verts->size() );

    for( std::vector<osg::Vec3d>::const_iterator i = input.begin(); i != input.end(); ++i )
    {
        osg::Vec3d ecef;
        inputSRS->transform( *i, ecefSRS, ecef );
        out_verts->push_back( ecef * world2local );

        if ( out_normals )
        {
            ecef.normalize();
            out_normals->push_back( osg::Matrix::transform3x3(ecef, world2local) );
        }
    }
}

void
ECEF::transformAndGetRotationMatrix(const osg::Vec3d&       input,
                                    const SpatialReference* inputSRS,
                                    osg::Vec3d&             out_point,
                                    const SpatialReference* outputSRS,
                                    osg::Matrixd&           out_rotation )
{
    const SpatialReference* geoSRS  = inputSRS->getGeographicSRS();
    const SpatialReference* ecefSRS = outputSRS->getECEF();

    // first transform the geographic (lat/long):
    osg::Vec3d geoPoint;
    if ( !inputSRS->isGeographic() )
        inputSRS->transform( input, geoSRS, geoPoint );
    else
        geoPoint = input;

    // use that information to calculate a rotation matrix:
    ecefSRS->getEllipsoid()->computeCoordinateFrame(
        osg::DegreesToRadians( geoPoint.y() ),
        osg::DegreesToRadians( geoPoint.x() ),
        out_rotation );

    // then convert that to ECEF.
    geoSRS->transform(geoPoint, ecefSRS, out_point);
}
