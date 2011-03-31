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
#include <osgEarthFeatures/ScatterFilter>
#include <stdlib.h>

#define LC "[ScatterFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

ScatterFilter::ScatterFilter() :
_density( 10.0f ),
_random( true ),
_randomSeed( 0 )
{
    //NOP
}

void
ScatterFilter::polyScatter(Geometry*            geom,
                           const FilterContext& context,
                           PointSet*            output ) const
{
    Bounds bounds;
    double areaSqKm = 0.0;

    GeometryIterator iter( geom );
    iter.traversePolygonHoles() = false;

    while( iter.hasMore() )
    {
        Polygon* polygon = dynamic_cast<Polygon*>( iter.next() );
        if ( !polygon )
            continue;

        if ( context.isGeocentric() )
        {
            bounds = polygon->getBounds();

            double avglat = bounds.yMin() + 0.5*bounds.height();
            double h = bounds.height() * 111.32;
            double w = bounds.width() * 111.32 * sin( 1.57079633 + osg::DegreesToRadians(avglat) );

            areaSqKm = w * h;
        }

        else if ( context.profile()->getSRS()->isProjected() )
        {
            bounds = polygon->getBounds();
            areaSqKm = (0.001*bounds.width()) * (0.001*bounds.height());
        }

        double zMin = 0.0;
        unsigned numInstances = areaSqKm * (double)osg::clampAbove( 0.1f, _density );
        if ( numInstances == 0 )
            continue;

        if ( _random )
        {
            // random scattering:
            for( unsigned j=0; j<numInstances; ++j )
            {
                double rx = ((double)::rand()) / (double)RAND_MAX;
                double ry = ((double)::rand()) / (double)RAND_MAX;

                double x = bounds.xMin() + rx * bounds.width();
                double y = bounds.yMin() + ry * bounds.height();

                if ( polygon->contains2D( x, y ) )
                    output->push_back( osg::Vec3d( x, y, zMin ) );
                else
                    --j;
            }
        }

        else
        {
            // regular interval scattering:
            double numInst1D = sqrt((double)numInstances);
            double ar = bounds.width() / bounds.height();
            unsigned cols = (unsigned)( numInst1D * ar );
            unsigned rows = (unsigned)( numInst1D / ar );
            double colInterval = bounds.width() / (double)(cols-1);
            double rowInterval = bounds.height() / (double)(rows-1);
            double interval = 0.5*(colInterval+rowInterval);

            for( double cy=bounds.yMin(); cy<=bounds.yMax(); cy += interval )
            {
                for( double cx = bounds.xMin(); cx <= bounds.xMax(); cx += interval )
                {
                    if ( polygon->contains2D( cx, cy ) )
                        output->push_back( osg::Vec3d(cx, cy, zMin) );
                }
            }
        }
    }
}

void
ScatterFilter::lineScatter(Geometry*            geom,
                           const FilterContext& context,
                           PointSet*            output ) const
{
    //TODO.
    OE_WARN << LC << "LINE Scattering NOT YET IMPLEMENTED ***" << std::endl;
}

FilterContext
ScatterFilter::push(FeatureList& features, const FilterContext& context )
{
    if ( !isSupported() ) {
        OE_WARN << LC << "support for this filter is not enabled" << std::endl;
        return context;
    }

    // seed the random number generator so the randomness is the same each time
    // todo: control this seeding based on the feature source name, perhaps?
    ::srand( _randomSeed );

    for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
    {
        Feature* f = i->get();
        
        Geometry* geom = f->getGeometry();
        if ( !geom )
            continue;

        //TODO:
        // only works properly for geocentric data OR projected data in meters.
        // does not work for a "plate carre" projection -gw

        // first, undo the localization frame if there is one.
        context.toWorld( geom );

        // convert to geodetic if necessary, and compute the approximate area in sq km
        if ( context.isGeocentric() )
        {
            GeometryIterator gi( geom );
            while( gi.hasMore() )
                context.profile()->getSRS()->getGeographicSRS()->transformFromECEF( gi.next(), true );
        }

        PointSet* points = new PointSet();

        if ( geom->getComponentType() == Geometry::TYPE_POLYGON )
        {
            polyScatter( geom, context, points );
        }

        // convert back to geocentric if necessary.
        if ( context.isGeocentric() )
            context.profile()->getSRS()->getGeographicSRS()->transformToECEF( points, true );

        // re-apply the localization frame.
        context.toLocal( points );

        // replace the source geometry with the scattered points.
        f->setGeometry( points );
    }

    return context;
}
