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
#include <osgEarthFeatures/Geometry>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Features;

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

    Geometry* result = 0L;
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
    }
    return output;
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
Ring::getOrientation() const {
    return getSignedArea() >= 0.0 ? Ring::ORIENTATION_CCW : Ring::ORIENTATION_CW;
}

double 
Ring::getSignedArea() const
{
    const_cast<Ring*>(this)->open();
    double sum = 0.0;
    for( const_iterator i = begin(); i != end(); ++i )
    {
        const osg::Vec3d& p0 = *i;
        const osg::Vec3d& p1 = i != end()-1? *(i+1) : front(); //*begin();
        sum += p0.x()*p1.y() - p1.x()*p0.y();
    }
    return 0.5 * sum;
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
    if ( getOrientation() != orientation )
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
_traverseMulti( true ),
_traversePolyHoles( true ),
_next( 0L )
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
