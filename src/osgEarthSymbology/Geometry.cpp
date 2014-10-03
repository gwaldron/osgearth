/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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


Geometry::Geometry( const Geometry& rhs ) :
osgEarth::MixinVector<osg::Vec3d,osg::Referenced>( rhs )
{
    //nop
}

Geometry::Geometry( int capacity )
{
    if ( capacity > 0 )
        reserve( capacity );
}

Geometry::Geometry( const Vec3dVector* data )
{
    reserve( data->size() );
    insert( begin(), data->begin(), data->end() );
}

Geometry::~Geometry()
{
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
    //if ( newType == getType() )        
    //    return static_cast<Geometry*>( clone() );
    
    switch( newType )
    {
    case TYPE_POINTSET:
        return new PointSet( &this->asVector() );
    case TYPE_LINESTRING:
        return new LineString( &this->asVector() );
    case TYPE_RING:
        return new Ring( &this->asVector() );
    case TYPE_POLYGON:
        if ( dynamic_cast<const Polygon*>(this) )
            return new Polygon( *static_cast<const Polygon*>(this) );
        else
            return new Polygon( &this->asVector() );
    case TYPE_UNKNOWN:
        return new Geometry( &this->asVector() );
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

osg::Vec3dArray*
Geometry::toVec3dArray() const 
{
    osg::Vec3dArray* result = new osg::Vec3dArray( this->size() );
    std::copy( begin(), end(), result->begin() );
    return result;
}

Geometry*
Geometry::create( Type type, const Vec3dVector* toCopy )
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

    GEOSContext gc;

    geom::Geometry* inGeom = gc.importGeometry( this );
    if ( inGeom )
    {
        buffer::BufferParameters::EndCapStyle geosEndCap =
            params._capStyle == BufferParameters::CAP_ROUND  ? buffer::BufferParameters::CAP_ROUND :
            params._capStyle == BufferParameters::CAP_SQUARE ? buffer::BufferParameters::CAP_SQUARE :
            params._capStyle == BufferParameters::CAP_FLAT   ? buffer::BufferParameters::CAP_FLAT :
            buffer::BufferParameters::CAP_SQUARE;

        buffer::BufferParameters::JoinStyle geosJoinStyle =
            params._joinStyle == BufferParameters::JOIN_ROUND ? buffer::BufferParameters::JOIN_ROUND :
            params._joinStyle == BufferParameters::JOIN_MITRE ? buffer::BufferParameters::JOIN_MITRE :
            params._joinStyle == BufferParameters::JOIN_BEVEL ? buffer::BufferParameters::JOIN_BEVEL :
            buffer::BufferParameters::JOIN_ROUND;

        //JB:  Referencing buffer::BufferParameters::DEFAULT_QUADRANT_SEGMENTS causes link errors b/c it is defined as a static in the header of BufferParameters.h and not defined in the cpp anywhere.
        //     This seems to only effect the Linux build, Windows works fine
        int geosQuadSegs = params._cornerSegs > 0 
            ? params._cornerSegs
            : 8; //buffer::BufferParameters::DEFAULT_QUADRANT_SEGMENTS;

        geom::Geometry* outGeom = NULL;

        buffer::BufferParameters geosBufferParams;
        geosBufferParams.setQuadrantSegments( geosQuadSegs );
        geosBufferParams.setEndCapStyle( geosEndCap );
        geosBufferParams.setJoinStyle( geosJoinStyle );
        buffer::BufferBuilder bufBuilder( geosBufferParams );

        try
        {
            if (params._singleSided)
            {
                outGeom = bufBuilder.bufferLineSingleSided(inGeom, distance, params._leftSide);
            }
            else
            {
                outGeom = bufBuilder.buffer(inGeom, distance);
            }
        }
        catch(const geos::util::GEOSException& ex)
        {
            OE_NOTICE << LC << "buffer(GEOS): "
                << (ex.what()? ex.what() : " no error message")
                << std::endl;
            outGeom = 0L;
        }

        bool sharedFactory = 
            inGeom && outGeom &&
            inGeom->getFactory() == outGeom->getFactory();

        if ( outGeom )
        {
            output = gc.exportGeometry( outGeom );
            gc.disposeGeometry( outGeom );
        }

        gc.disposeGeometry( inGeom );
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
    bool success = false;
    output = 0L;

    GEOSContext gc;

    //Create the GEOS Geometries
    geom::Geometry* inGeom   = gc.importGeometry( this );
    geom::Geometry* cropGeom = gc.importGeometry( cropPoly );

    if ( inGeom )
    {    
        geom::Geometry* outGeom = 0L;
        try {
            outGeom = overlay::OverlayOp::overlayOp(
                inGeom,
                cropGeom,
                overlay::OverlayOp::opINTERSECTION );
        }
        catch(const geos::util::GEOSException& ex) {
            OE_NOTICE << LC << "Crop(GEOS): "
                << (ex.what()? ex.what() : " no error message")
                << std::endl;
            outGeom = 0L;
        }

        if ( outGeom )
        {
            output = gc.exportGeometry( outGeom );

            if ( output.valid())
            {
                if ( output->isValid() )
                {
                    success = true;
                }
                else
                {
                    // GEOS result is invalid
                    output = 0L;
                }
            }
            else
            {
                // set output to empty geometry to indicate the (valid) empty case,
                // still returning false but allows for check.
                if (outGeom->getNumPoints() == 0)
                {
                    output = new osgEarth::Symbology::Geometry();
                }
            }

            gc.disposeGeometry( outGeom );
        }
    }

    //Destroy the geometry
    gc.disposeGeometry( cropGeom );
    gc.disposeGeometry( inGeom );

    return success;

#else // OSGEARTH_HAVE_GEOS

    OE_WARN << LC << "Crop failed - GEOS not available" << std::endl;
    return false;

#endif // OSGEARTH_HAVE_GEOS
}

bool
Geometry::difference( const Polygon* diffPolygon, osg::ref_ptr<Geometry>& output ) const
{
#ifdef OSGEARTH_HAVE_GEOS

    GEOSContext gc;

    //Create the GEOS Geometries
    geom::Geometry* inGeom   = gc.importGeometry( this );
    geom::Geometry* diffGeom = gc.importGeometry( diffPolygon );

    if ( inGeom )
    {    
        geom::Geometry* outGeom = 0L;
        try {
            outGeom = overlay::OverlayOp::overlayOp(
                inGeom,
                diffGeom,
                overlay::OverlayOp::opDIFFERENCE );
        }
        catch(const geos::util::GEOSException& ex) {
            OE_NOTICE << LC << "Diff(GEOS): "
                << (ex.what()? ex.what() : " no error message")
                << std::endl;
            outGeom = 0L;
        }

        if ( outGeom )
        {
            output = gc.exportGeometry( outGeom );
            gc.disposeGeometry( outGeom );

            if ( output.valid() && !output->isValid() )
            {
                output = 0L;
            }
        }
    }

    //Destroy the geometry
    gc.disposeGeometry( diffGeom );
    gc.disposeGeometry( inGeom );

    return output.valid();

#else // OSGEARTH_HAVE_GEOS

    OE_WARN << LC << "Difference failed - GEOS not available" << std::endl;
    return false;

#endif // OSGEARTH_HAVE_GEOS
}

osg::Vec3d
Geometry::localize()
{
    osg::Vec3d offset;

    Bounds bounds = getBounds();
    if ( bounds.isValid() )
    {      
        osg::Vec2d center = bounds.center2d();
        offset.set( center.x(), center.y(), 0 );

        GeometryIterator i( this );
        while( i.hasMore() )
        {
            Geometry* part = i.next();
            for( Geometry::iterator j = part->begin(); j != part->end(); ++j )
            {
                *j = *j - offset;
            }
        }
    }

    return offset;
}

void
Geometry::delocalize( const osg::Vec3d& offset )
{
    GeometryIterator i( this );
    while( i.hasMore() )
    {
        Geometry* part = i.next();
        for( Geometry::iterator j = part->begin(); j != part->end(); ++j )
        {
            *j = *j + offset;
        }
    }
}

void 
Geometry::rewind( Orientation orientation )
{
    Orientation current = getOrientation();
    if ( current != orientation && current != ORIENTATION_DEGENERATE && orientation != ORIENTATION_DEGENERATE )
    {
        std::reverse( begin(), end() );
    }
}

Geometry::Orientation 
Geometry::getOrientation() const
{
    // adjust for a non-open ring:
    int n = size();
    while( n > 0 && front() == back() )
        n--;

    if ( n < 3 )
        return Geometry::ORIENTATION_DEGENERATE;

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
        cross.z() < 0.0 ? Geometry::ORIENTATION_CW :
        cross.z() > 0.0 ? Geometry::ORIENTATION_CCW :
        Geometry::ORIENTATION_DEGENERATE;
}

//----------------------------------------------------------------------------

PointSet::PointSet( const PointSet& rhs ) :
Geometry( rhs )
{
    //nop
}

PointSet::~PointSet()
{
}

//----------------------------------------------------------------------------

LineString::LineString( const LineString& rhs ) :
Geometry( rhs )
{
    //nop
}

LineString::LineString( const Vec3dVector* data ) :
Geometry( data )
{
    //nop
}

LineString::~LineString()
{
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

Ring::Ring( const Ring& rhs ) :
Geometry( rhs )
{
    //nop
}

Ring::Ring( const Vec3dVector* data ) :
Geometry( data )
{
    open();
}

Ring::~Ring()
{
}

Geometry*
Ring::cloneAs( const Geometry::Type& newType ) const
{
    if ( newType == TYPE_LINESTRING )
    {
        LineString* line = new LineString( &this->asVector() );
        if ( line->size() > 1 && line->front() != line->back() )
            line->push_back( front() );
        return line;
    }
    else return Geometry::cloneAs( newType );
}

// ensures that the first and last points are not idential.
void 
Ring::open()
{            
    while( size() > 2 && front() == back() )
        erase( end()-1 );
}

// ensures that the first and last points are idential.
void 
Ring::close()
{
    if ( size() > 0 && front() != back() )
        push_back( front() );
}

// gets the signed area.
double
Ring::getSignedArea2D() const
{
    const_cast<Ring*>(this)->open();

    double sum = 0.0;

    for( unsigned i=0; i<size(); ++i )
    {
        const osg::Vec3d& p0 = (*this)[0];
        const osg::Vec3d& p1 = i+1 < size() ? (*this)[i+1] : (*this)[0];
        sum += p0.x()*p1.y() - p1.x()*p0.y();
    }
    return .5*sum;
}

// opens and rewinds the polygon to the specified orientation.
void 
Ring::rewind( Orientation orientation )
{
    open();
    Geometry::rewind( orientation );
}

// point-in-polygon test
bool
Ring::contains2D( double x, double y ) const
{
    bool result = false;
    const Ring& poly = *this;
    for( unsigned i=0, j=size()-1; i<size(); j = i++ )
    {
        if ((((poly[i].y() <= y) && (y < poly[j].y())) ||
            ((poly[j].y() <= y) && (y < poly[i].y()))) &&
            (x < (poly[j].x()-poly[i].x()) * (y-poly[i].y())/(poly[j].y()-poly[i].y())+poly[i].x()))
        {
            result = !result;
        }
    }
    return result;
}

//----------------------------------------------------------------------------

Polygon::Polygon( const Polygon& rhs ) :
Ring( rhs )
{
    for( RingCollection::const_iterator r = rhs._holes.begin(); r != rhs._holes.end(); ++r )
        _holes.push_back( new Ring(*r->get()) );
}

Polygon::Polygon( const Vec3dVector* data ) :
Ring( data )
{
    //nop
}

Polygon::~Polygon()
{
}

int
Polygon::getTotalPointCount() const
{
    int total = Ring::getTotalPointCount();
    for( RingCollection::const_iterator i = _holes.begin(); i != _holes.end(); ++i )
        total += i->get()->getTotalPointCount();
    return total;
}

bool
Polygon::contains2D( double x, double y ) const
{
    // first check the outer ring
    if ( !Ring::contains2D(x, y) )
        return false;

    // then check each inner ring (holes). Point has to be inside the outer ring, 
    // but NOT inside any of the holes
    for( RingCollection::const_iterator i = _holes.begin(); i != _holes.end(); ++i )
    {
        if ( i->get()->contains2D(x, y) )
            return false;
    }

    return true;
}

void
Polygon::open() 
{
    Ring::open();
    for( RingCollection::const_iterator i = _holes.begin(); i != _holes.end(); ++i )
        (*i)->open();
}

void
Polygon::close() 
{
    Ring::close();
    for( RingCollection::const_iterator i = _holes.begin(); i != _holes.end(); ++i )
        (*i)->close();
}

//----------------------------------------------------------------------------

MultiGeometry::MultiGeometry( const MultiGeometry& rhs ) :
Geometry( rhs )
{
    for( GeometryCollection::const_iterator i = rhs._parts.begin(); i != rhs._parts.end(); ++i )
        _parts.push_back( i->get()->clone() ); //i->clone() ); //osg::clone<Geometry>( i->get() ) );
}

MultiGeometry::MultiGeometry( const GeometryCollection& parts ) :
_parts( parts )
{
    //nop
}

MultiGeometry::~MultiGeometry()
{
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

unsigned
MultiGeometry::getNumGeometries() const
{
    unsigned total = 0;
    for( GeometryCollection::const_iterator i = _parts.begin(); i != _parts.end(); ++i )
        total += i->get()->getNumGeometries();
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
        Geometry* part = i->get()->cloneAs( i->get()->getType() );
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

// opens and rewinds the polygon to the specified orientation.
void 
MultiGeometry::rewind( Orientation orientation )
{
    for( GeometryCollection::const_iterator i = _parts.begin(); i != _parts.end(); ++i )
    {
        i->get()->rewind( orientation );
    }
}

//----------------------------------------------------------------------------

GeometryIterator::GeometryIterator( Geometry* geom, bool holes ) :
_next( 0L ),
_traverseMulti( true ),
_traversePolyHoles( holes )
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

ConstGeometryIterator::ConstGeometryIterator( const Geometry* geom, bool holes ) :
_next( 0L ),
_traverseMulti( true ),
_traversePolyHoles( holes )
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

//----------------------------------------------------------------------------

ConstSegmentIterator::ConstSegmentIterator( const Geometry* verts, bool forceClosedLoop ) :
_verts(&verts->asVector()),
_closeLoop(forceClosedLoop),
_iter(verts->begin())
{
    _done = _verts->size() < 2;

    if ( !forceClosedLoop )
    {
        _closeLoop = dynamic_cast<const Ring*>(verts) != 0L;
    }
}

Segment
ConstSegmentIterator::next()
{
    osg::Vec3d p0 = *_iter++;
    if ( _iter == _verts->end() ) 
    {
        _iter = _verts->begin();
        _done = true;
    }
    else if ( _iter+1 == _verts->end() && !_closeLoop )
    {
        _done = true;
    }

    return Segment( p0, *_iter );
}
