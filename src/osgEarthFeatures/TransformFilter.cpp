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
#include <osgEarthFeatures/TransformFilter>
#include <osg/ClusterCullingCallback>

#define LC "[TransformFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//---------------------------------------------------------------------------

namespace 
{
    osg::Matrixd
    createGeocentricInvRefFrame( const osg::Vec3d& input, const SpatialReference* inputSRS )
    {
        // convert to gencentric first:
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
    localizeGeometry( Feature* input, const osg::Matrixd& refFrame )
    {
        if ( input && input->getGeometry() )
        {
            GeometryIterator iter( input->getGeometry() );
            while( iter.hasMore() )
            {
                Geometry* geom = iter.next();
                for( unsigned int i=0; i<geom->size(); i++ )
                {
                    (*geom)[i] = (*geom)[i] * refFrame;
                }
            }
        }
    }
}

//---------------------------------------------------------------------------

TransformFilter::TransformFilter() :
_makeGeocentric( false ),
_localize( false )
{
    // nop
}

TransformFilter::TransformFilter(const SpatialReference* outputSRS,
                                 bool outputGeocentric ) :
_outputSRS( outputSRS ),
_makeGeocentric( outputGeocentric ),
_localize( false )
{
    //NOP
}

bool
TransformFilter::push( Feature* input, const FilterContext& context )
{
    if ( !input || !input->getGeometry() )
        return true;

    bool needsSRSXform =
        _outputSRS.valid() &&
        ( ! context.profile()->getSRS()->isEquivalentTo( _outputSRS.get() ) );

    bool needsMatrixXform = !_mat.isIdentity();

    // optimize: do nothing if nothing needs doing
    if ( !needsSRSXform && !_makeGeocentric && !_localize && !needsMatrixXform )
        return true;

    // iterate over the feature geometry.
    GeometryIterator iter( input->getGeometry() );
    while( iter.hasMore() )
    {
        Geometry* geom = iter.next();

        // pre-transform the point before doing an SRS transformation.
        if ( needsMatrixXform )
        {
            for( unsigned i=0; i < geom->size(); ++i )
                (*geom)[i] = (*geom)[i] * _mat;
        }

        // first transform the geometry to the output SRS:            
        if ( needsSRSXform )
            context.profile()->getSRS()->transformPoints( _outputSRS.get(), geom, false );

        // convert to ECEF if required:
        if ( _makeGeocentric )
            _outputSRS->transformToECEF( geom, false );

        // update the bounding box.
        if ( _localize )
        {
            for( unsigned i=0; i<geom->size(); ++i )
                _bbox.expandBy( (*geom)[i] );
        }
    }

    return true;
}

FilterContext
TransformFilter::push( FeatureList& input, const FilterContext& incx )
{
    _bbox = osg::BoundingBoxd();

    // first transform all the points into the output SRS, collecting a bounding box as we go:
    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); i++ )
        if ( !push( i->get(), incx ) )
            ok = false;

    FilterContext outcx( incx );
    outcx.isGeocentric() = _makeGeocentric;

    if ( _outputSRS.valid() )
    {
        if ( incx.extent()->isValid() )
            outcx.profile() = new FeatureProfile( incx.extent()->transform( _outputSRS.get() ) );
        else
            outcx.profile() = new FeatureProfile( incx.profile()->getExtent().transform( _outputSRS.get() ) );
    }

    // set the reference frame to shift data to the centroid. This will
    // prevent floating point precision errors in the openGL pipeline for
    // properly gridded data.
    if ( _bbox.valid() && _localize )
    {
        // create a suitable reference frame:
        osg::Matrixd localizer;
        if ( _makeGeocentric )
        {
            localizer = createGeocentricInvRefFrame( _bbox.center(), _outputSRS );
            localizer.invert( localizer );
        }
        else
        {
            localizer = osg::Matrixd::translate( -_bbox.center() );
        }

        // localize the geometry relative to the reference frame.
        for( FeatureList::iterator i = input.begin(); i != input.end(); i++ )
        {
            localizeGeometry( i->get(), localizer );
        }
        outcx.setReferenceFrame( localizer );
    }

    return outcx;
}
