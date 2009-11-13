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
#include <osgEarthFeatures/FeatureOpFilter>

using namespace osgEarthFeatures;

#ifdef OSGEARTH_HAVE_GEOS

#include <geos/geom/GeometryFactory.h>
#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/MultiPoint.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/LineString.h>
#include <geos/geom/LinearRing.h>

static
void Vec3dArray2CoordVector( osg::Vec3dArray* input, std::vector<geos::geom::Coordinate>& output )
{
    for( osg::Vec3dArray::const_iterator i = input->begin(); i != input->end(); ++i )
    {
        output.push_back( geos::geom::Coordinate( i->x(), i->y(), i->z() ) );
    }
}

geos::geom::Geometry* 
FeatureOpFilter::encode( const FeatureGeometry& input, const FilterContext& context )
{
    geos::geom::Geometry* output = 0L;

    geos::geom::GeometryFactory factory;

    switch( context.profile()->getGeometryType() )
    {
    case FeatureProfile::GEOM_POINT:
        {
        }
        break;

    case FeatureProfile::GEOM_LINE:
        break;

    case FeatureProfile::GEOM_POLYGON:
        {
            geos::geom::Polygon* poly = factory.createPolygon();

            // treat the first part as the solid; the remainder as holes.
            geos::geom::LinearRing* ring = 0L;
            std::vector<geos::geom::LinearRing*> holes;

            for( FeatureGeometry::const_iterator part_i = input.begin(); part_i != input.end(); ++part_i )
            {
                osg::Vec3dArray* part = part_i->get();
                std::vector<geos::geom::Coordinate> coords( part->size() );
                Vec3dArray2CoordVector( part, coords );
                if ( !ring )
                {
                    //ring = factory.createLinearRing(
                    //ring->
                }
            }
        }
        break;
    }

    return output;
}

void
FeatureOpFilter::decode( geos::geom::Geometry* input, FeatureGeometry& output, const FilterContext& context )
{
}


#endif // OSGEARTH_HAVE_GEOS
