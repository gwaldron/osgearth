/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/TransformFilter>
#include <osgEarth/Feature>
#include <osgEarth/FilterContext>

#define LC "[TransformFilter] "

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace 
{
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
_localize( false )
{
    // nop
}

TransformFilter::TransformFilter( const osg::Matrixd& xform ) :
_localize      ( false ),
_mat           ( xform )
{
    //nop
}

TransformFilter::TransformFilter(const SpatialReference* outputSRS ) :
_outputSRS( outputSRS ),
_localize( false )
{
    //NOP
}

bool
TransformFilter::push( Feature* input, FilterContext& context )
{
    if ( !input || !input->getGeometry() )
        return true;

    const SpatialReference* inputSRS =
        input ? input->getSRS() :
        context.profile() ? context.profile()->getSRS() :
        nullptr;

    if (!inputSRS)
        return false;

    bool needsSRSXform = 
        _outputSRS.valid() &&
        !inputSRS->isEquivalentTo(_outputSRS.get());

    bool needsMatrixXform = !_mat.isIdentity();

    // optimize: do nothing if nothing needs doing
    if ( !needsSRSXform && !_localize && !needsMatrixXform )
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
        {
            inputSRS->transform(geom->asVector(), _outputSRS.get());
            //context.profile()->getSRS()->transform( geom->asVector(), _outputSRS.get() );
        }
            //context.profile()->getSRS()->transformPoints( _outputSRS.get(), geom->asVector(), false );

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
TransformFilter::push( FeatureList& input, FilterContext& incx )
{
    _bbox = osg::BoundingBoxd();

    // first transform all the points into the output SRS, collecting a bounding box as we go:
    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); i++ )
        if ( !push( i->get(), incx ) )
            ok = false;

    FilterContext outcx( incx );

    if ( _outputSRS.valid() )
    {
        if ( incx.extent()->isValid() )
            outcx.setProfile( new FeatureProfile( incx.extent()->transform( _outputSRS.get()) ) );
        else
            outcx.setProfile( new FeatureProfile( incx.profile()->getExtent().transform( _outputSRS.get()) ) );
    }

    // set the reference frame to shift data to the centroid. This will
    // prevent floating point precision errors in the openGL pipeline for
    // properly gridded data.
    if ( _bbox.valid() && _localize )
    {
        // create a suitable reference frame:
        osg::Matrixd localizer;
        localizer = osg::Matrixd::translate( -_bbox.center() );

        // localize the geometry relative to the reference frame.
        for( FeatureList::iterator i = input.begin(); i != input.end(); i++ )
        {
            localizeGeometry( i->get(), localizer );
        }
        //outcx.setReferenceFrame( localizer );
    }

    return outcx;
}
