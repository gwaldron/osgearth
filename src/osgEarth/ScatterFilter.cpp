/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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
#include <osgEarth/ScatterFilter>
#include <osgEarth/FilterContext>
#include <osgEarth/GeoMath>
#include <stdlib.h>

#define LC "[ScatterFilter] "

using namespace osgEarth;

ScatterFilter::ScatterFilter() :
_density   ( 10.0f ),
_random    ( true ),
_randomSeed( 1 )
{
    //NOP
}

void
ScatterFilter::polyScatter(const Geometry*         input,
                           const SpatialReference* inputSRS,
                           const FilterContext&    context,
                           PointSet*               output )
{
    Bounds bounds;
    double areaSqKm = 0.0;

    ConstGeometryIterator iter( input, false );
    while( iter.hasMore() )
    {
        const Polygon* polygon = dynamic_cast<const Polygon*>( iter.next() );
        if ( !polygon )
            continue;

        if ( /*context.isGeocentric() ||*/ context.profile()->getSRS()->isGeographic() )
        {
            bounds = polygon->getBounds();

            double avglat = bounds.yMin() + 0.5*height(bounds);
            double h = height(bounds) * 111.32;
            double w = width(bounds) * 111.32 * sin( 1.57079633 + osg::DegreesToRadians(avglat) );

            areaSqKm = w * h;
        }

        else if ( context.profile()->getSRS()->isProjected() )
        {
            bounds = polygon->getBounds();
            areaSqKm = (0.001*width(bounds)) * (0.001*height(bounds));
        }

        double zMin = 0.0;
        unsigned numInstancesInBoundingRect = (unsigned)(areaSqKm * (double)osg::clampAbove( 0.1f, _density ));
        if ( numInstancesInBoundingRect == 0 )
            continue;

        if ( _random )
        {
            // Random scattering. Note, we try to place as many instances as would
            // fit in the bounding rectangle; The real placed number will be less since
            // we only place models inside the actual polygon. But the density will 
            // be correct.
            for( unsigned j=0; j<numInstancesInBoundingRect; ++j )
            {
                double x = bounds.xMin() + _prng.next() * width(bounds);
                double y = bounds.yMin() + _prng.next() * height(bounds);

                bool include = true;

                if ( include && polygon->contains2D( x, y ) )
                    output->push_back( osg::Vec3d(x, y, zMin) );
            }
        }

        else
        {
            // regular interval scattering:
            double numInst1D = sqrt((double)numInstancesInBoundingRect);
            double ar = width(bounds) / height(bounds);
            unsigned cols = (unsigned)( numInst1D * ar );
            unsigned rows = (unsigned)( numInst1D / ar );
            double colInterval = width(bounds) / (double)(cols-1);
            double rowInterval = height(bounds) / (double)(rows-1);
            double interval = 0.5*(colInterval+rowInterval);

            for( double cy=bounds.yMin(); cy<=bounds.yMax(); cy += interval )
            {
                for( double cx = bounds.xMin(); cx <= bounds.xMax(); cx += interval )
                {
                    bool include = true;

                    if ( include && polygon->contains2D( cx, cy ) )
                        output->push_back( osg::Vec3d(cx, cy, zMin) );
                }
            }
        }
    }
}

void
ScatterFilter::lineScatter(const Geometry*         input,
                           const SpatialReference* inputSRS,
                           const FilterContext&    context,
                           PointSet*               output )
{
    // calculate the number of instances per linear km.
    float instPerKm = sqrt( osg::clampAbove( 0.1f, _density ) );

    bool isGeo = inputSRS->isGeographic();

    ConstGeometryIterator iter( input );
    while( iter.hasMore() )
    {
        const Geometry* part = iter.next();

        // see whether it's a ring, because rings have to connect the last two points.
        bool isRing = part->getType() == Geometry::TYPE_RING;
        
        for( unsigned i=0; i<part->size(); ++i )
        {
            // done if we're not traversing a ring.
            if ( !isRing && i+1 == part->size() )
                break;

            // extract the segment:
            const osg::Vec3d& p0 = (*part)[i];
            const osg::Vec3d& p1 = isRing && i+1 == part->size() ? (*part)[0] : (*part)[i+1];

            // figure out the segment length in meters (assumed geodetic coords)
            double seglen_m;
            double seglen_native = (p1-p0).length();
            
            if ( isGeo )
            {
                seglen_m = GeoMath::distance(
                    osg::DegreesToRadians( p0.y() ), osg::DegreesToRadians( p0.x() ),
                    osg::DegreesToRadians( p1.y() ), osg::DegreesToRadians( p1.x() ) );
            }
            else // projected
            {
                seglen_m = seglen_native;
            }

            unsigned numInstances = (unsigned)((seglen_m*0.001) * instPerKm);
            if ( numInstances > 0 )
            {            
                // a unit vector for scattering points along the segment
                osg::Vec3d unit = p1-p0;
                unit.normalize();

                for( unsigned n=0; n<numInstances; ++n )
                {
                    double offset = _prng.next() * seglen_native;
                    output->push_back( p0 + unit*offset );
                }
            }
        }
    }
}

FilterContext
ScatterFilter::push(FeatureList& features, FilterContext& context )
{
    if ( !isSupported() ) {
        OE_WARN << LC << "support for this filter is not enabled" << std::endl;
        return context;
    }

    // seed the random number generator so the randomness is the same each time
    _prng = Random( _randomSeed, Random::METHOD_FAST );

    for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
    {
        Feature* f = i->get();
        
        Geometry* geom = f->getGeometry();
        if ( !geom )
            continue;

        const SpatialReference* geomSRS = context.profile()->getSRS();

        osg::ref_ptr< PointSet > points = new PointSet();

        if ( geom->getComponentType() == Geometry::TYPE_POLYGON )
        {
            polyScatter( geom, geomSRS, context, points.get() );
        }
        else if (
            geom->getComponentType() == Geometry::TYPE_LINESTRING ||
            geom->getComponentType() == Geometry::TYPE_RING )            
        {
            lineScatter( geom, geomSRS, context, points.get() );
        }
        else {            
            points = static_cast< PointSet*>(geom->cloneAs(Geometry::TYPE_POINTSET));
        }

        // replace the source geometry with the scattered points.
        f->setGeometry( points.get() );
    }

    return context;
}
