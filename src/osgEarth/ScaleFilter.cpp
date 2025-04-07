/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/ScaleFilter>
#include <osgEarth/FilterContext>
#include <osgEarth/GeoData>

using namespace osgEarth;

ScaleFilter::ScaleFilter() :
_scale( 0.0 )
{
    //NOP
}

ScaleFilter::ScaleFilter( double scale ) :
_scale( scale )
{
    //NOP
}

FilterContext
ScaleFilter::push( FeatureList& input, FilterContext& cx )
{
    for( FeatureList::iterator i = input.begin(); i != input.end(); ++i )
    {
        Feature* input = i->get();
        if ( input && input->getGeometry() )
        {
            Bounds envelope = input->getGeometry()->getBounds();

            // now scale and shift everything
            GeometryIterator scale_iter( input->getGeometry() );
            while( scale_iter.hasMore() )
            {
                Geometry* geom = scale_iter.next();
                for( osg::Vec3dArray::iterator v = geom->begin(); v != geom->end(); v++ )
                {
                    double xr = (v->x() - envelope.xMin()) / width(envelope);
                    v->x() += (xr - 0.5) * _scale;

                    double yr = (v->y() - envelope.yMin()) / height(envelope);
                    v->y() += (yr - 0.5) * _scale;
                }
            }
        }
    }

    return cx;
}

