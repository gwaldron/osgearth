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
#include <osgEarthFeatures/TransformFilter>

using namespace osgEarth;
using namespace osgEarthFeatures;

TransformFilter::TransformFilter(const SpatialReference* outputSRS, bool isGeocentric ) :
_outputSRS( outputSRS ),
_isGeocentric( isGeocentric )
{
    //NOP
}

bool
TransformFilter::push( Feature* input, FilterContext& context )
{
    for( int p=0; p<input->getNumParts(); p++ )
    {
        osg::Vec3dArray* part = input->getPart( p );
        bool success = _outputSRS->transformPoints( context._profile->getSRS(), part, false );
        if ( !success )
            return false;

        if ( _isGeocentric && _outputSRS->isGeographic() )
        {
            const osg::EllipsoidModel* em = context._profile->getSRS()->getEllipsoid();
            for( int i=0; i<part->size(); i++ )
            {
                double x, y, z;
                em->convertLatLongHeightToXYZ(
                    osg::DegreesToRadians( (*part)[i].y() ),
                    osg::DegreesToRadians( (*part)[i].x() ),
                    (*part)[i].z(),
                    x, y, z );
                (*part)[i].set( x, y, z );
            }
        }
    }
    return true;
}

bool
TransformFilter::push( FeatureList& input, FilterContext& context )
{
    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); i++ )
        if ( !push( i->get(), context ) )
            ok = false;
    return true;
}

//osg::Referenced*
//TransformFilter::process( osg::Referenced* inputFeature, FilterContext& context )
//{
//    Feature* input = static_cast<Feature*>( inputFeature );
//
//    for( int p=0; p<input->getNumParts(); p++ )
//    {
//        osg::Vec3dArray* part = input->getPart( p );
//        bool success = _outputSRS->transformPoints( context._profile->getSRS(), part, false );
//        if ( !success )
//            return false;
//
//        if ( _isGeocentric && _outputSRS->isGeographic() )
//        {
//            const osg::EllipsoidModel* em = context._profile->getSRS()->getEllipsoid();
//            for( int i=0; i<part->size(); i++ )
//            {
//                double x, y, z;
//                em->convertLatLongHeightToXYZ(
//                    osg::DegreesToRadians( (*part)[i].y() ),
//                    osg::DegreesToRadians( (*part)[i].x() ),
//                    (*part)[i].z(),
//                    x, y, z );
//                (*part)[i].set( x, y, z );
//            }
//        }
//    }
//
//    if ( !_outputProfile.valid() )
//    {
//        _outputProfile = new FeatureProfile(
//            _outputSRS.get(),
//            context._profile->getGeometryType(),
//            context._profile->getDimensionality(),
//            context._profile->isMultiGeometry() );
//    }
//
//    context._profile = _outputProfile.get();
//    
//    return inputFeature;
//}
//
