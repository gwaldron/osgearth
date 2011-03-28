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
_randomSeed( 0 )
{
    //NOP
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
        
        // must contain an aereal geometry:
        Ring* ring = dynamic_cast<Ring*>( f->getGeometry() );
        if ( !ring )
            continue;

        //TODO:
        // only works properly for geocentric data OR projected data in meters.
        // does not work for a "plate carre" projection -gw

        Bounds bounds;
        double areaSqKm = 0.0;
        
        if ( context.hasReferenceFrame() )
            for( Ring::iterator i = ring->begin(); i != ring->end(); ++i )
                *i = context.toWorld( *i );

        if ( context.isGeocentric() )
        {
            context.profile()->getSRS()->getGeographicSRS()->transformFromECEF( ring, true );

            bounds = ring->getBounds();

            double avglat = bounds.yMin() + 0.5*bounds.height();
            double h = bounds.height() * 111.32;
            double w = bounds.width() * 111.32 * sin( 1.57079633 + osg::DegreesToRadians(avglat) );

            areaSqKm = w * h;
        }

        else if ( context.profile()->getSRS()->isProjected() )
        {
            bounds = ring->getBounds();
            areaSqKm = (0.001*bounds.width()) * (0.001*bounds.height());
        }

        double zMin = 0.0;
        unsigned numInstances = areaSqKm * (double)osg::clampAbove( 0.1f, _density );

        if ( numInstances > 0 )
        {
            PointSet* points = new PointSet();

            for( unsigned j=0; j<numInstances; ++j )
            {
                double rx = ((double)::rand()) / (double)RAND_MAX;
                double ry = ((double)::rand()) / (double)RAND_MAX;

                double x = bounds.xMin() + rx * bounds.width();
                double y = bounds.yMin() + ry * bounds.height();

                if ( ring->contains2D( x, y ) )
                    points->push_back( osg::Vec3d( x, y, zMin ) );
                else
                    --j;
            }

            if ( context.isGeocentric() )
            {
                context.profile()->getSRS()->getGeographicSRS()->transformToECEF( points, true );
            }

            if ( context.hasReferenceFrame() )
                for( PointSet::iterator i = points->begin(); i != points->end(); ++i )
                    *i = context.toLocal( *i );

            // replace the source geometry with the scattered points.
            f->setGeometry( points );
        }
    }

    return context;
}
