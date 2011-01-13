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
#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/GEOS>
#include <algorithm>
#include <iterator>

using namespace osgEarth;
using namespace osgEarth::Symbology;

#ifdef OSGEARTH_HAVE_GEOS
#  include <geos/geom/Geometry.h>
#  include <geos/geom/GeometryFactory.h>
#  include <geos/operation/buffer/BufferOp.h>
#  include <geos/operation/buffer/BufferBuilder.h> 
#  include <geos/operation/overlay/OverlayOp.h>
using namespace geos;
using namespace geos::operation;
#endif

#define LC "[Geometry] "


Geometry::Geometry( const Geometry& rhs, const osg::CopyOp& op ) :
osg::Vec3dArray( rhs, op )
{
}

Geometry::Geometry( int capacity )
{
    if ( capacity > 0 )
        reserve( capacity );
}

Geometry::Geometry( const osg::Vec3dArray* data )
{
    reserve( data->size() );
    insert( begin(), data->begin(), data->end() );
}

int
Geometry::getTotalPointCount() const
{
    return size();
}

Bounds
Geometry::getBounds() const
{
    Bounds bounds;
    for( const_iterator i = begin(); i != end(); ++i )
        bounds.expandBy( i->x(), i->y(), i->z() );
    return bounds;
}

Geometry*
Geometry::cloneAs( const Geometry::Type& newType ) const
{
    if ( newType == getType() )
        return static_cast<Geometry*>( clone() );
    
    switch( newType )
    {
    case TYPE_POINTSET:
        return new PointSet( (const osg::Vec3dArray*)this );
    case TYPE_LINESTRING:
        return new LineString( (const osg::Vec3dArray*)this );
    case TYPE_RING:
        return new Ring( (const osg::Vec3dArray*)this );
    case TYPE_POLYGON:
        return new Polygon( (const osg::Vec3dArray*)this );
    default:
        break;
    }
    return 0L;
}

osg::Vec3Array*
Geometry::toVec3Array() const 
{
    osg::Vec3Array* result = new osg::Vec3Array( this->size() );
    std::copy( begin(), end(), result->begin() );
    return result;
}

Geometry*
Geometry::create( Type type, const osg::Vec3dArray* toCopy )
{
    Geometry* output = 0L;
    switch( type ) {
        case TYPE_POINTSET:
            output = new PointSet( toCopy ); break;
        case TYPE_LINESTRING:
            output = new LineString( toCopy ); break;
        case TYPE_RING:
            output = new Ring( toCopy ); break;
        case TYPE_POLYGON:
            output = new Polygon( toCopy ); break;
        default:
            break;
    }
    return output;
}

bool
Geometry::hasBufferOperation()
{
#ifdef OSGEARTH_HAVE_GEOS
    return true;
#else
    return false;
#endif
}

bool
Geometry::buffer(double distance,
                 osg::ref_ptr<Geometry>& output,
                 const BufferParameters& params ) const
{
#ifdef OSGEARTH_HAVE_GEOS   

    geom::Geometry* inGeom = GEOSUtils::importGeometry( this );
    if ( inGeom )
    {
        buffer::BufferParameters::EndCapStyle geosEndCap =
            params._capStyle == BufferParameters::CAP_ROUND  ? buffer::BufferParameters::CAP_ROUND :
            params._capStyle == BufferParameters::CAP_SQUARE ? buffer::BufferParameters::CAP_SQUARE :
            params._capStyle == BufferParameters::CAP_FLAT   ? buffer::BufferParameters::CAP_FLAT :
            buffer::BufferParameters::CAP_SQUARE;

        //JB:  Referencing buffer::BufferParameters::DEFAULT_QUADRANT_SEGMENTS causes link errors b/c it is defined as a static in the header of BufferParameters.h and not defined in the cpp anywhere.
        //     This seems to only effect the Linux build, Windows works fine
        int geosQuadSegs = params._cornerSegs > 0 
            ? params._cornerSegs
            : 8;//buffer::BufferParameters::DEFAULT_QUADRANT_SEGMENTS;

        geom::Geometry* outGeom = NULL;

        buffer::BufferParameters geosBufferParams;
        geosBufferParams.setQuadrantSegments( geosQuadSegs );
        geosBufferParams.setEndCapStyle( geosEndCap );
        buffer::BufferBuilder bufBuilder( geosBufferParams );

        if (params._singleSided)
        {
            outGeom = bufBuilder.bufferLineSingleSided(inGeom, distance, params._leftSide );
        }
        else
        {
            outGeom = bufBuilder.buffer(inGeom, distance );
        }

        if ( outGeom )
        {
            output = GEOSUtils::exportGeometry( outGeom );
            outGeom->getFactory()->destroyGeometry( outGeom );
        }
        else
        {
            OE_INFO << LC << "Buffer: no output geometry" << std::endl;
        }

        inGeom->getFactory()->destroyGeometry( inGeom );
    }
    return output.valid();

#else // OSGEARTH_HAVE_GEOS

    OE_WARN << LC << "Buffer failed - GEOS not available" << std::endl;
    return false;

#endif // OSGEARTH_HAVE_GEOS
}

bool
Geometry::crop( const Polygon* cropPoly, osg::ref_ptr<Geometry>& output ) const
{
#ifdef OSGEARTH_HAVE_GEOS

    geom::GeometryFactory* f = new geom::GeometryFactory();

    //Create the GEOS Geometries
    geom::Geometry* inGeom = GEOSUtils::importGeometry( this );
    geom::Geometry* cropGeom = GEOSUtils::importGeometry( cropPoly );

    if ( inGeom )
    {    
        geom::Geometry* outGeom = 0L;
        try {
            outGeom = overlay::OverlayOp::overlayOp(
                inGeom, cropGeom,
                overlay::OverlayOp::opINTERSECTION );
        }
        catch( ... ) {
            outGeom = 0L;
            OE_NOTICE << LC << "::crop, GEOS overlay op exception, skipping feature" << std::endl;
        }

        if ( outGeom )
        {
            output = GEOSUtils::exportGeometry( outGeom );
            f->destroyGeometry( outGeom );
            if ( output.valid() && !output->isValid() )
            {
                output = 0L;
            }
        }
    }

    //Destroy the geometry
    f->destroyGeometry( cropGeom );
    f->destroyGeometry( inGeom );

    delete f;
    return output.valid();

#else // OSGEARTH_HAVE_GEOS

    OE_WARN << LC << "Crop failed - GEOS not available" << std::endl;
    return false;

#endif // OSGEARTH_HAVE_GEOS
}


//----------------------------------------------------------------------------

PointSet::PointSet( const PointSet& rhs, const osg::CopyOp& op ) :
Geometry( rhs, op )
{
    //nop
}

//----------------------------------------------------------------------------

LineString::LineString( const LineString& rhs, const osg::CopyOp& op ) :
Geometry( rhs, op )
{
    //nop
}

LineString::LineString( const osg::Vec3dArray* data ) :
Geometry( data )
{
    //nop
}

double
LineString::getLength() const
{
    double length = 0;
    for (unsigned int i = 0; i < size()-1; ++i)
    {
        osg::Vec3d current = (*this)[i];
        osg::Vec3d next    = (*this)[i+1];
        length += (next - current).length();
    }
    return length;
}

bool
LineString::getSegment(double length, osg::Vec3d& start, osg::Vec3d& end)
{
    double pos = 0;
    for (unsigned int i = 0; i < size()-1; ++i)
    {
        osg::Vec3d current = (*this)[i];
        osg::Vec3d next    = (*this)[i+1];
        pos += (next - current).length();
        if (pos > length)
        {
            start = current;
            end = next;
            return true;
        }
    }
    return false;
}

//----------------------------------------------------------------------------

Ring::Ring( const Ring& rhs, const osg::CopyOp& op ) :
Geometry( rhs, op )
{
    //nop
}

Ring::Ring( const osg::Vec3dArray* data ) :
Geometry( data )
{
    open();
}

Geometry*
Ring::cloneAs( const Geometry::Type& newType ) const
{
    if ( newType == TYPE_LINESTRING )
    {
        LineString* line = new LineString( (osg::Vec3dArray*)this );
        if ( line->size() > 1 && line->front() != line->back() )
            line->push_back( front() );
        return line;
    }
    else return Geometry::cloneAs( newType );
}

Ring::Orientation 
Ring::getOrientation() const
{
    // adjust for a non-open ring:
    int n = size();
    while( n > 0 && front() == back() )
        n--;

    if ( n < 3 )
        return Ring::ORIENTATION_DEGENERATE;

    // copy the open vec:
    std::vector<osg::Vec3d> v;
    v.reserve( n );
    std::copy( begin(), begin()+n, std::back_inserter(v) );

    int rmin = 0;
    double xmin = v[0].x();
    double ymin = v[0].y();
    v[0].z() = 0;
    for( int i=1; i<n; ++i ) {
        double x = v[i].x();
        double y = v[i].y();
        v[i].z() = 0;
        if ( y > ymin )
            continue;
        if ( y == ymin ) {
            if (x  < xmin )
                continue;
        }
        rmin = i;
        xmin = x;
        ymin = y;
    }

    int rmin_less_1 = rmin-1 >= 0 ? rmin-1 : n-1;
    int rmin_plus_1 = rmin+1 < n ? rmin+1 : 0;

    osg::Vec3 in = v[rmin] - v[rmin_less_1]; in.normalize();
    osg::Vec3 out = v[rmin_plus_1] - v[rmin]; out.normalize();
    osg::Vec3 cross = in ^ out;

    return
        cross.z() < 0.0 ? Ring::ORIENTATION_CW :
        cross.z() > 0.0 ? Ring::ORIENTATION_CCW :
        Ring::ORIENTATION_DEGENERATE;
}

// ensures that the first and last points are not idential.
void 
Ring::open()
{            
    while( size() > 2 && front() == back() )
        erase( end()-1 );
}

// opens and rewinds the polygon to the specified orientation.
void 
Ring::rewind( Orientation orientation )
{
    open();
    Orientation current = getOrientation();
    if ( current != orientation && current != ORIENTATION_DEGENERATE && orientation != ORIENTATION_DEGENERATE )
    {
        std::reverse( begin(), end() );
    }
}

//----------------------------------------------------------------------------

Polygon::Polygon( const Polygon& rhs, const osg::CopyOp& op ) :
Ring( rhs, op )
{
    //_boundary = rhs._boundary.valid() ? osg::clone<Ring>( rhs._boundary.get(), op ) : 0L;
    for( RingCollection::const_iterator r = rhs._holes.begin(); r != rhs._holes.end(); ++r )
        _holes.push_back( osg::clone<Ring>( r->get(), op ) );
}

Polygon::Polygon( const osg::Vec3dArray* data ) :
Ring( data )
{
    //nop
}

int
Polygon::getTotalPointCount() const
{
    int total = Ring::getTotalPointCount(); //_boundary.valid() ? _boundary->size() : 0;
    for( RingCollection::const_iterator i = _holes.begin(); i != _holes.end(); ++i )
        total += i->get()->getTotalPointCount();
    return total;
}

//----------------------------------------------------------------------------

MultiGeometry::MultiGeometry( const MultiGeometry& rhs, const osg::CopyOp& op ) :
Geometry( rhs, op )
{
    for( GeometryCollection::const_iterator i = rhs._parts.begin(); i != rhs._parts.end(); ++i )
        _parts.push_back( osg::clone<Geometry>( i->get() ) );
}

MultiGeometry::MultiGeometry( const GeometryCollection& parts ) :
_parts( parts )
{
    //nop
}

Geometry::Type
MultiGeometry::getComponentType() const
{
    // dicey.
    return _parts.size() > 0 ? _parts.front()->getType() : TYPE_UNKNOWN;
}

int
MultiGeometry::getTotalPointCount() const
{
    int total = 0;
    for( GeometryCollection::const_iterator i = _parts.begin(); i != _parts.end(); ++i )
        total += i->get()->getTotalPointCount();
    return total;
}

Bounds
MultiGeometry::getBounds() const
{
    Bounds bounds;
    for( GeometryCollection::const_iterator i = _parts.begin(); i != _parts.end(); ++i )
    {
        bounds.expandBy( i->get()->getBounds() );
    }
    return bounds;
}

Geometry*
MultiGeometry::cloneAs( const Geometry::Type& newType ) const
{
    MultiGeometry* multi = new MultiGeometry();
    for( GeometryCollection::const_iterator i = _parts.begin(); i != _parts.end(); ++i )
    {
        Geometry* part = i->get()->cloneAs( newType );
        if ( part ) multi->getComponents().push_back( part );
    }
    return multi;
}

bool
MultiGeometry::isValid() const
{
    if ( _parts.size() == 0 )
        return false;

    bool valid = true;
    for( GeometryCollection::const_iterator i = _parts.begin(); i != _parts.end() && valid; ++i )
    {
        if ( !i->get()->isValid() )
            valid = false;
    }
    return valid;
}

//----------------------------------------------------------------------------

GeometryIterator::GeometryIterator( Geometry* geom ) :
_next( 0L ),
_traverseMulti( true ),
_traversePolyHoles( true )
{
    if ( geom )
    {
        _stack.push( geom );
        fetchNext();
    }
}

bool
GeometryIterator::hasMore() const
{
    return _next != 0L;
}

Geometry*
GeometryIterator::next()
{
    Geometry* n = _next;
    fetchNext();
    return n;
}

void
GeometryIterator::fetchNext()
{
    _next = 0L;
    if ( _stack.size() == 0 )
        return;

    Geometry* current = _stack.top();
    _stack.pop();

    if ( current->getType() == Geometry::TYPE_MULTI && _traverseMulti )
    {
        MultiGeometry* m = static_cast<MultiGeometry*>(current);
        for( GeometryCollection::const_iterator i = m->getComponents().begin(); i != m->getComponents().end(); ++i )
            _stack.push( i->get() );
        fetchNext();
    }
    else if ( current->getType() == Geometry::TYPE_POLYGON && _traversePolyHoles )
    {
        Polygon* p = static_cast<Polygon*>(current);
        for( RingCollection::const_iterator i = p->getHoles().begin(); i != p->getHoles().end(); ++i )
            _stack.push( i->get() );
        _next = current;
    }
    else
    {
        _next = current;
    }    
}

//----------------------------------------------------------------------------

ConstGeometryIterator::ConstGeometryIterator( const Geometry* geom ) :
_next( 0L ),
_traverseMulti( true ),
_traversePolyHoles( true )
{
    if ( geom )
    {
        _stack.push( geom );
        fetchNext();
    }
}

bool
ConstGeometryIterator::hasMore() const
{
    return _next != 0L;
}

const Geometry*
ConstGeometryIterator::next()
{
    const Geometry* n = _next;
    fetchNext();
    return n;
}

void
ConstGeometryIterator::fetchNext()
{
    _next = 0L;
    if ( _stack.size() == 0 )
        return;

    const Geometry* current = _stack.top();
    _stack.pop();

    if ( current->getType() == Geometry::TYPE_MULTI && _traverseMulti )
    {
        const MultiGeometry* m = static_cast<const MultiGeometry*>(current);
        for( GeometryCollection::const_iterator i = m->getComponents().begin(); i != m->getComponents().end(); ++i )
            _stack.push( i->get() );
        fetchNext();
    }
    else if ( current->getType() == Geometry::TYPE_POLYGON && _traversePolyHoles )
    {
        const Polygon* p = static_cast<const Polygon*>(current);
        for( RingCollection::const_iterator i = p->getHoles().begin(); i != p->getHoles().end(); ++i )
            _stack.push( i->get() );
        _next = current;
    }
    else
    {
        _next = current;
    }    
}

