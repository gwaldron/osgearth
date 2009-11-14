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

using namespace geos;

#include <geos/geom/GeometryFactory.h>
#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/CoordinateArraySequenceFactory.h>
#include <geos/geom/MultiPoint.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/LineString.h>
#include <geos/geom/LinearRing.h>

static
geom::CoordinateSequence*
Vec3dArray2CoordSeq( osg::Vec3dArray* input )
{   
    std::vector<geos::geom::Coordinate>* coords = new std::vector<geom::Coordinate>( input->size() );
    for( osg::Vec3dArray::const_iterator i = input->begin(); i != input->end(); ++i )
    {
        coords->push_back( geom::Coordinate( i->x(), i->y(), i->z() ) );
    }
    return geom::CoordinateArraySequenceFactory::instance()->create( coords );
}

geos::geom::Geometry* 
FeatureOpFilter::encode( const FeatureGeometry& input, const FilterContext& context )
{
    geom::Geometry* output = 0L;
    const geom::GeometryFactory* factory = geom::GeometryFactory::getDefaultInstance();

    switch( context.profile()->getGeometryType() )
    {
    case FeatureProfile::GEOM_POINT:
        {
            output = 0L;
        }
        break;

    case FeatureProfile::GEOM_LINE:
        {
            output = 0L;
        }
        break;

    case FeatureProfile::GEOM_POLYGON:
        {
            // treat the first part as the solid; the remainder as holes.
            geom::LinearRing* shell = 0L;
            std::vector<geom::Geometry*>* holes = new std::vector<geom::Geometry*>();

            for( FeatureGeometry::const_iterator part_i = input.begin(); part_i != input.end(); ++part_i )
            {
                osg::Vec3dArray* part = part_i->get();

                geom::CoordinateSequence* seq = Vec3dArray2CoordSeq( part );
                if ( part_i == input.begin() )
                    shell = factory->createLinearRing( seq );
                else
                    holes->push_back( factory->createLinearRing( seq ) );
            }

            geom::Polygon* poly = factory->createPolygon( shell, holes );
            output = poly;
        }
        break;
    }

    return output;
}

bool
FeatureOpFilter::decode( geom::Geometry* input, FeatureGeometry& output, const FilterContext& context )
{
    if ( dynamic_cast<geom::MultiPoint*>( input ) )
    {
    }
    else if ( dynamic_cast<geom::LineString*>( input ) )
    {
    }
    else if ( dynamic_cast<geom::Polygon*>( input ) )
    {
        geom::Polygon* poly = static_cast<geom::Polygon*>( input );

        output.clear();
        const geom::LineString* outerRing = poly->getExteriorRing();
        if ( outerRing )
        {
            geom::CoordinateSequence* s = outerRing->getCoordinates();
            osg::Vec3dArray* part = new osg::Vec3dArray( s->getSize() );
            for( int i=0; i<s->getSize(); i++ ) 
            {
                const geom::Coordinate& c = s->getAt( i );
                (*part)[i].set( c.x, c.y, c.z );                
            }
            output.push_back( part );
        }
    }
    return true;
}


#endif // OSGEARTH_HAVE_GEOS
