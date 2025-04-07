/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */

#include <osgEarth/ECEF>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[ECEF] "

bool
ECEF::transformAndLocalize(const osg::Vec3d&       input,
                           const SpatialReference* inputSRS,
                           osg::Vec3d&             output,
                           const SpatialReference* outputSRS,
                           const osg::Matrixd&     world2local)
{
    if (inputSRS==NULL || outputSRS==NULL)
        return false;

    osg::Vec3d ecef;
    inputSRS->transform( input, outputSRS->getGeocentricSRS(), ecef );
    output = ecef * world2local;
    return true;
}


bool
ECEF::transformAndLocalize(const std::vector<osg::Vec3d>& input,
                           const SpatialReference*        inputSRS,
                           osg::Vec3Array*                output,
                           const SpatialReference*        outputSRS,
                           const osg::Matrixd&            world2local )
{
    if (inputSRS==NULL || outputSRS==NULL)
        return false;

    const SpatialReference* geocentricSRS = outputSRS->getGeocentricSRS();
    output->reserve( output->size() + input.size() );

    for( std::vector<osg::Vec3d>::const_iterator i = input.begin(); i != input.end(); ++i )
    {
        osg::Vec3d geoc;
        inputSRS->transform( *i, geocentricSRS, geoc );
        output->push_back( geoc * world2local );
    }

    return true;
}


bool
ECEF::transformAndLocalize(const std::vector<osg::Vec3d>& input,
                           const SpatialReference*        inputSRS,
                           osg::Vec3Array*                out_verts,
                           osg::Vec3Array*                out_normals,
                           const SpatialReference*        outputSRS,
                           const osg::Matrixd&            world2local )
{
    if (inputSRS==NULL || outputSRS==NULL)
        return false;

    const SpatialReference* ecefSRS = outputSRS->getGeocentricSRS();
    out_verts->reserve( out_verts->size() + input.size() );
    
    for( std::vector<osg::Vec3d>::const_iterator i = input.begin(); i != input.end(); ++i )
    {
        osg::Vec3d ecef;
        inputSRS->transform( *i, ecefSRS, ecef );
        out_verts->push_back( ecef * world2local );
    }

    if ( out_normals )
    {
        out_normals->reserve( out_verts->size() );

        const osg::Vec3f up(0,0,1);
        osg::Vec3f outNormal;
        for(unsigned v=0; v < out_verts->size()-1; ++v)
        {
            osg::Vec3f normal;
            osg::Vec3f out = (*out_verts)[v+1] - (*out_verts)[v];
            osg::Vec3f right = out ^ up;
            outNormal = right ^ out;
            
            if ( v == 0 )
            {
                normal = outNormal;
            }
            else
            {
                osg::Vec3f in = (*out_verts)[v] - (*out_verts)[v-1];
                osg::Vec3f inNormal = right ^ in;
                normal = (inNormal + outNormal) * 0.5;
            }

            normal.normalize();
            out_normals->push_back( normal );
        }

        // final one.
        outNormal.normalize();
        out_normals->push_back( outNormal );
    }
    return true;
}

bool
ECEF::transformAndGetRotationMatrix(const osg::Vec3d&       input,
                                    const SpatialReference* inputSRS,
                                    osg::Vec3d&             out_point,
                                    const SpatialReference* outputSRS,
                                    osg::Matrixd&           out_rotation )
{
    if (inputSRS==NULL || outputSRS==NULL)
        return false;

    const SpatialReference* geoSRS  = inputSRS->getGeographicSRS();
    const SpatialReference* ecefSRS = outputSRS->getGeocentricSRS();

    // first transform the geographic (lat/long):
    osg::Vec3d geoPoint;
    if ( !inputSRS->isGeographic() )
        inputSRS->transform( input, geoSRS, geoPoint );
    else
        geoPoint = input;

    // use that information to calculate a rotation matrix:
    out_rotation = ecefSRS->getEllipsoid().geodeticToCoordFrame(
        osg::Vec3d(geoPoint.x(), geoPoint.y(), geoPoint.z()));
        
        //geoPoint..computeCoordinateFrame(
        //osg::DegreesToRadians( geoPoint.y() ),
        //osg::DegreesToRadians( geoPoint.x() ),
        //out_rotation );

    // then convert that to ECEF.
    geoSRS->transform(geoPoint, ecefSRS, out_point);


    return true;
}
