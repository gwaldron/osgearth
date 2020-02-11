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
#ifdef OSGEARTH_HAVE_GEOS

#include <osgEarth/GEOS>
#include <osg/Notify>

#include <geos/geom/PrecisionModel.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/CoordinateArraySequenceFactory.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/Point.h>
#include <geos/geom/MultiPoint.h>
#include <geos/geom/Polygon.h>
#include <geos/geom/MultiPolygon.h>
#include <geos/geom/LineString.h>
#include <geos/geom/MultiLineString.h>
#include <geos/geom/LinearRing.h>
#include <geos/operation/valid/IsValidOp.h>
#include <geos/util/IllegalArgumentException.h>

using namespace osgEarth;
using namespace geos;
using namespace geos::operation;

#define LC "[GEOS] "

#define GEOS_VERSION_AT_LEAST(MAJOR, MINOR) \
    ((GEOS_VERSION_MAJOR>MAJOR) || (GEOS_VERSION_MAJOR==MAJOR && GEOS_VERSION_MINOR>=MINOR))

namespace
{
    geom::CoordinateSequence*
    vec3dArray2CoordSeq( const Geometry* input, bool close, const geom::CoordinateSequenceFactory* factory )
    {   
        bool needToClose = close && input->size() > 2 && input->front() != input->back();

        std::vector<geos::geom::Coordinate>* coords = new std::vector<geom::Coordinate>();
        coords->reserve( input->size() + (needToClose ? 1 : 0) );
        for( osg::Vec3dArray::const_iterator i = input->begin(); i != input->end(); ++i )
        {
            coords->push_back( geom::Coordinate( i->x(), i->y(), i->z() ));
        }
        if ( needToClose )
        {
            coords->push_back( coords->front() );
        }
        geom::CoordinateSequence* seq = factory->create( coords );

        return seq;
    }

    geom::Geometry*
    import( const Geometry* input, const geom::GeometryFactory* f )
    {
        geom::Geometry* output = 0L;

        if ( input->getType() == Geometry::TYPE_UNKNOWN )
        {
            output = 0L;
        }
        else if ( input->getType() == Geometry::TYPE_MULTI )
        {
            const MultiGeometry* multi = static_cast<const MultiGeometry*>( input );

            Geometry::Type compType = multi->getComponentType();

            std::vector<geom::Geometry*>* children = new std::vector<geom::Geometry*>();
            for( GeometryCollection::const_iterator i = multi->getComponents().begin(); i != multi->getComponents().end(); ++i ) 
            {
                geom::Geometry* child = import( i->get(), f );
                if ( child )
                    children->push_back( child );
            }
            if ( children->size() > 0 )
            {
                if ( compType == Geometry::TYPE_POLYGON )
                    output = f->createMultiPolygon( children );
                else if ( compType == Geometry::TYPE_LINESTRING )
                    output = f->createMultiLineString( children );
                else if ( compType == Geometry::TYPE_POINT || compType == Geometry::TYPE_POINTSET )
                    output = f->createMultiPoint( children );
                else
                    output = f->createGeometryCollection( children );
            }
            else
                delete children;
        }
        else
        {
            // any other type will at least contain points:
            geom::CoordinateSequence* seq = 0L;
            try
            {
                switch( input->getType() )
                {
                case Geometry::TYPE_UNKNOWN: 
                    break;
                case Geometry::TYPE_MULTI: break;

                case Geometry::TYPE_POINT:
                case Geometry::TYPE_POINTSET:
                    seq = vec3dArray2CoordSeq( input, false, f->getCoordinateSequenceFactory() );
                    if ( seq ) output = f->createPoint( seq );
                    break;

                case Geometry::TYPE_LINESTRING:
                    seq = vec3dArray2CoordSeq( input, false, f->getCoordinateSequenceFactory() );
                    if ( seq ) output = f->createLineString( seq );
                    break;

                case Geometry::TYPE_RING:
                    seq = vec3dArray2CoordSeq( input, true, f->getCoordinateSequenceFactory() );
                    if ( seq ) output = f->createLinearRing( seq );
                    break;

                case Geometry::TYPE_POLYGON:
                    seq = vec3dArray2CoordSeq( input, true, f->getCoordinateSequenceFactory() );
                    geom::LinearRing* shell = 0L;
                    if ( seq )
                        shell = f->createLinearRing( seq );

                    if ( shell )
                    {
                        const Polygon* poly = static_cast<const Polygon*>(input);
                        std::vector<geom::Geometry*>* holes = poly->getHoles().size() > 0 ? new std::vector<geom::Geometry*>() : 0L;
                        if (holes)
                        {
                            for( RingCollection::const_iterator r = poly->getHoles().begin(); r != poly->getHoles().end(); ++r )
                            {
                                geom::Geometry* hole = import( r->get(), f );
                                if (hole)
                                {
                                    if (hole->getGeometryTypeId() == geos::geom::GEOS_LINEARRING)
                                    {
                                        holes->push_back(hole);
                                    }
                                    else
                                    {
                                        delete hole;
                                    }
                                }
                            }
                            if (holes->size() == 0)
                            {
                                delete holes;
                                holes = 0L;
                            }
                        }
                        output = f->createPolygon( shell, holes );
                    }
                
                    break;
                }
            }
            catch( util::IllegalArgumentException )
            {
                // catch GEOS exceptions..
                //if ( seq )
                //    delete seq;

                OE_DEBUG << "GEOS::import: Removed degenerate geometry" << std::endl;
            }
        }

        return output;
    }

    Geometry*
    exportPolygon( const geom::Polygon* input )
    {
        Polygon* output = 0L;

        const geom::LineString* outerRing = input->getExteriorRing();
        if ( outerRing )
        {
            const geom::CoordinateSequence* s = outerRing->getCoordinatesRO();

            output = new Polygon( s->getSize() );

            for( unsigned int j=0; j<s->getSize(); j++ ) 
            {
                const geom::Coordinate& c = s->getAt( j );
                output->push_back( osg::Vec3d( c.x, c.y, !osg::isNaN(c.z)? c.z : 0.0) );
                //OE_NOTICE << "c.z = " << c.z << "\n";
            }
            output->rewind( Ring::ORIENTATION_CCW );

            for( unsigned k=0; k < input->getNumInteriorRing(); k++ )
            {
                const geom::LineString* inner = input->getInteriorRingN( k );
                const geom::CoordinateSequence* s = inner->getCoordinatesRO();
                Ring* hole = new Ring( s->getSize() );
                for( unsigned int m = 0; m<s->getSize(); m++ )
                {
                    const geom::Coordinate& c = s->getAt( m );
                    hole->push_back( osg::Vec3d( c.x, c.y, !osg::isNaN(c.z)? c.z : 0.0) );
                }
                hole->rewind( Ring::ORIENTATION_CW );
                output->getHoles().push_back( hole );
            }
        }
        return output;
    }
}


GEOSContext::GEOSContext()
{
    // double-precison:
    geos::geom::PrecisionModel* pm = new geos::geom::PrecisionModel(geom::PrecisionModel::FLOATING);

    // Factory will clone the PM
#if GEOS_VERSION_AT_LEAST(3,6)
    _factory = geos::geom::GeometryFactory::create( pm );
#else
    _factory = new geos::geom::GeometryFactory( pm );
#endif

    // Delete the template.
    delete pm;
}

GEOSContext::~GEOSContext()
{
#if !GEOS_VERSION_AT_LEAST(3,6)
    delete _factory;
#endif
}

geom::Geometry*
GEOSContext::importGeometry(const Geometry* input)
{
    geom::Geometry* output = 0L;
    if ( input && input->isValid() )
    {
#if GEOS_VERSION_AT_LEAST(3,6)
        output = import( input, _factory.get() );
#else
        output = import( input, _factory );

        // if output is ok, it will have a pointer to f. this is probably a leak.
        // TODO: Check whether this is a leak!! -gw
        //if ( !output )
        //    delete f;
#endif
    }
    return output;
}



Geometry*
GEOSContext::exportGeometry(const geom::Geometry* input)
{
    GeometryCollection parts;

    if ( dynamic_cast<const geom::Point*>( input ) )
    {
        const geom::Point* point = dynamic_cast< const geom::Point* >(input);
        Point* part = new Point();
        const geom::Coordinate* c = point->getCoordinate();
        part->set(osg::Vec3d(c->x, c->y, c->z));
        return part;
    }
    else if ( dynamic_cast<const geom::MultiPoint*>( input ) )
    {
        const geom::MultiPoint* mp = dynamic_cast<const geom::MultiPoint*>( input );
        PointSet* part = new PointSet( mp->getNumPoints() );
        for( unsigned int i=0; i < mp->getNumPoints(); i++ )
        {
            const geom::Geometry* g = mp->getGeometryN(i);
            if ( g )
            {
                const geom::Coordinate* c = mp->getCoordinate();
                if ( c )
                {
                    part->push_back( osg::Vec3d( c->x, c->y, c->z ) ); //p->getX(), p->getY(), 0 ) );
                }
            }
        }
        parts.push_back( part );
    }
    else if ( dynamic_cast<const geom::LineString*>( input ) )
    {
        const geom::LineString* line = dynamic_cast<const geom::LineString*>( input );
        LineString* part = new LineString( line->getNumPoints() );
        for( unsigned int i=0; i<line->getNumPoints(); i++ )
        {
            const geom::Coordinate& c = line->getCoordinateN(i);
            part->push_back( osg::Vec3d( c.x, c.y, c.z ) ); //0 ) );
        }
        parts.push_back( part );
    }
    else if ( dynamic_cast<const geom::MultiLineString*>( input ) )
    {
        const geom::MultiLineString* m = dynamic_cast<const geom::MultiLineString*>( input );
        for( unsigned int i=0; i<m->getNumGeometries(); i++ ) 
        {
            Geometry* part = exportGeometry( m->getGeometryN(i) );
            if ( part ) parts.push_back( part );
        }
    }
    else if ( dynamic_cast<const geom::Polygon*>( input ) )
    {
        const geom::Polygon* poly = dynamic_cast<const geom::Polygon*>( input );
        Geometry* part = exportPolygon( poly );
        if ( part ) parts.push_back( part );
    }
    else if ( dynamic_cast<const geom::MultiPolygon*>( input ) )
    {
        //OE_NOTICE << "Multipolygon" << std::endl;
        const geom::MultiPolygon* mpoly = dynamic_cast<const geom::MultiPolygon*>( input );
        for( unsigned int i=0; i<mpoly->getNumGeometries(); i++ )
        {
            Geometry* part = exportPolygon( dynamic_cast<const geom::Polygon*>( mpoly->getGeometryN(i) ) );
            if ( part ) parts.push_back( part );
        }        
    }

    if ( parts.size() == 1 )
    {
        osg::ref_ptr<Geometry> part = parts.front().get();
        parts.clear();
        return part.release();
    }
    else if ( parts.size() > 1 )
    {
        return new MultiGeometry( parts );
    }
    else
    {
        return 0L;
    }
}


void
GEOSContext::disposeGeometry(geom::Geometry* input)
{
    if (input)
    {
#if GEOS_VERSION_AT_LEAST(3,6)
        _factory->destroyGeometry(input);
#else
        geom::GeometryFactory* f = const_cast<geom::GeometryFactory*>(input->getFactory());
        if ( f != _factory )
            delete f;
#endif
    }
}

#endif // OSGEARTH_HAVE_GEOS

