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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarth/Geometry>
#include <osgEarth/GEOS>
#include <osgEarth/Math>
#include <algorithm>
#include <iterator>
#include <cstdarg>

using namespace osgEarth;

#define GEOS_OUT OE_DEBUG

#define LC "[Geometry] "

namespace
{
    static void OSGEARTH_GEOSErrorHandler(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);
        char buffer[512];
        vsprintf(buffer, fmt, args);
        OE_DEBUG << " [GEOS Error] " << buffer << std::endl;
        va_end(args);
    }

    static void OSGEARTH_WarningHandler(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);
        char buffer[512];
        vsprintf(buffer, fmt, args);
        OE_DEBUG << " [GEOS Warning] " << buffer << std::endl;
        va_end(args);
    }

    static bool checkGEOSResult(const char result)
    {
        // GEOS functions return 0 for false, 1 for true and 2 for error.
        // Convert those to a bool with an error resulting in false.
        if (result == 0)
        {
            return false;
        }
        else if (result == 1)
        {
            return true;
        }
        else
        {
            OE_WARN << "GEOS encountered an exception" << std::endl;
            return false;
        }
    }
}

Geometry::Geometry( const Geometry& rhs ) :
osgEarth::InlineVector<osg::Vec3d,osg::Referenced>( rhs )
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
    case TYPE_POINT:
        return new Point( &this->asVector() );
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
Geometry::createVec3Array() const
{
    osg::Vec3Array* result = new osg::Vec3Array( this->size() );
    std::copy( begin(), end(), result->begin() );
    return result;
}

osg::Vec3dArray*
Geometry::createVec3dArray() const
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
        case TYPE_POINT:
            output = new Point( toCopy ); break;
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

    GEOSContextHandle_t handle = initGEOS_r(OSGEARTH_WarningHandler, OSGEARTH_GEOSErrorHandler);

    GEOSGeometry* inGeom = GEOS::importGeometry(handle, this);
    if (inGeom)
    {
        int  geosEndCap =
            params._capStyle == BufferParameters::CAP_ROUND ? GEOSBufCapStyles::GEOSBUF_CAP_ROUND :
            params._capStyle == BufferParameters::CAP_SQUARE ? GEOSBufCapStyles::GEOSBUF_CAP_SQUARE :
            params._capStyle == BufferParameters::CAP_FLAT ? GEOSBufCapStyles::GEOSBUF_CAP_FLAT :
            GEOSBufCapStyles::GEOSBUF_CAP_SQUARE;

        int  geosJoinStyle =
            params._joinStyle == BufferParameters::JOIN_ROUND ? GEOSBufJoinStyles::GEOSBUF_JOIN_ROUND :
            params._joinStyle == BufferParameters::JOIN_MITRE ? GEOSBufJoinStyles::GEOSBUF_JOIN_MITRE :
            params._joinStyle == BufferParameters::JOIN_BEVEL ? GEOSBufJoinStyles::GEOSBUF_JOIN_BEVEL :
            GEOSBufJoinStyles::GEOSBUF_JOIN_ROUND;

        int geosQuadSegs = params._cornerSegs > 0
            ? params._cornerSegs
            : 8;

        GEOSGeometry* outGeom = NULL;

        GEOSBufferParams* geosBufferParams = GEOSBufferParams_create_r(handle);
        GEOSBufferParams_setEndCapStyle_r(handle, geosBufferParams, geosEndCap);
        GEOSBufferParams_setJoinStyle_r(handle, geosBufferParams, geosJoinStyle);
        GEOSBufferParams_setQuadrantSegments_r(handle, geosBufferParams, geosQuadSegs);
        GEOSBufferParams_setSingleSided_r(handle, geosBufferParams, params._singleSided);

        outGeom = GEOSBufferWithParams_r(handle, inGeom, geosBufferParams, distance);
        if (outGeom)
        {
            output = GEOS::exportGeometry(handle, outGeom);
            GEOSGeom_destroy_r(handle, outGeom);
        }

        GEOSGeom_destroy_r(handle, inGeom);
    }

    finishGEOS_r(handle);

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
    GEOSContextHandle_t handle = initGEOS_r(OSGEARTH_WarningHandler, OSGEARTH_GEOSErrorHandler);

    bool success = false;
    output = 0L;

    //Create the GEOS Geometries
    GEOSGeometry* inGeom = GEOS::importGeometry(handle, this);
    GEOSGeometry* cropGeom = GEOS::importGeometry(handle, cropPoly);

    if ( inGeom && cropGeom)
    {
        GEOSGeometry* outGeom = GEOSIntersection_r(handle, inGeom, cropGeom);
        if ( outGeom )
        {
            output = GEOS::exportGeometry(handle, outGeom );

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
                if (GEOSGeomGetNumPoints_r(handle, outGeom ) == 0)
                {
                    output = new osgEarth::Geometry();
                }
            }

            GEOSGeom_destroy_r(handle, outGeom);
        }
    }

    //Destroy the geometry
    GEOSGeom_destroy_r(handle, cropGeom);
    GEOSGeom_destroy_r(handle, inGeom);

    finishGEOS_r(handle);

    return success;

#else // OSGEARTH_HAVE_GEOS

    OE_WARN << LC << "Crop failed - GEOS not available" << std::endl;
    return false;

#endif // OSGEARTH_HAVE_GEOS
}

bool
Geometry::crop( const Bounds& bounds, osg::ref_ptr<Geometry>& output ) const
{
    osg::ref_ptr<Polygon> poly = new Polygon;
    poly->resize( 4 );
    (*poly)[0].set(bounds.xMin(), bounds.yMin(), 0);
    (*poly)[1].set(bounds.xMax(), bounds.yMin(), 0);
    (*poly)[2].set(bounds.xMax(), bounds.yMax(), 0);
    (*poly)[3].set(bounds.xMin(), bounds.yMax(), 0);
    return crop(poly.get(), output);
}

bool
Geometry::geounion( const Geometry* other, osg::ref_ptr<Geometry>& output ) const
{
#ifdef OSGEARTH_HAVE_GEOS
    bool success = false;
    output = 0L;

    GEOSContextHandle_t handle = initGEOS_r(OSGEARTH_WarningHandler, OSGEARTH_GEOSErrorHandler);

    //Create the GEOS Geometries
    GEOSGeometry* inGeom = GEOS::importGeometry(handle, this);
    GEOSGeometry* otherGeom = GEOS::importGeometry(handle, other);
    GEOSGeometry* outGeom = GEOSUnion_r(handle, inGeom, otherGeom);

    if (outGeom)
    {
        output = GEOS::exportGeometry(handle, outGeom);

        if (output.valid())
        {
            if (output->isValid())
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
            if (GEOSGeomGetNumPoints_r(handle, outGeom) == 0)
            {
                output = new osgEarth::Geometry();
            }
        }

        GEOSGeom_destroy_r(handle, outGeom);
    }

    //Destroy the geometry
    GEOSGeom_destroy_r(handle, otherGeom );
    GEOSGeom_destroy_r(handle, inGeom );

    finishGEOS_r(handle);

    return success;

#else // OSGEARTH_HAVE_GEOS

    OE_WARN << LC << "Union failed - GEOS not available" << std::endl;
    return false;

#endif // OSGEARTH_HAVE_GEOS
}

bool
Geometry::difference( const Polygon* diffPolygon, osg::ref_ptr<Geometry>& output ) const
{
#ifdef OSGEARTH_HAVE_GEOS

    GEOSContextHandle_t handle = initGEOS_r(OSGEARTH_WarningHandler, OSGEARTH_GEOSErrorHandler);

    //Create the GEOS Geometries
    GEOSGeometry* inGeom = GEOS::importGeometry(handle, this);
    GEOSGeometry* diffGeom = GEOS::importGeometry(handle, diffPolygon);

    if ( inGeom )
    {
        GEOSGeometry* outGeom = GEOSDifference_r(handle, inGeom, diffGeom);
        if ( outGeom )
        {
            output = GEOS::exportGeometry(handle, outGeom);
            GEOSGeom_destroy_r(handle, outGeom);

            if ( output.valid() && !output->isValid() )
            {
                output = 0L;
            }
        }
    }

    //Destroy the geometry
    GEOSGeom_destroy_r(handle, diffGeom);
    GEOSGeom_destroy_r(handle, inGeom);

    finishGEOS_r(handle);

    return output.valid();

#else // OSGEARTH_HAVE_GEOS

    OE_WARN << LC << "Difference failed - GEOS not available" << std::endl;
    return false;

#endif // OSGEARTH_HAVE_GEOS
}

bool
Geometry::intersects(
            const class Geometry* other
            ) const
{
#ifdef OSGEARTH_HAVE_GEOS

    GEOSContextHandle_t handle = initGEOS_r(OSGEARTH_WarningHandler, OSGEARTH_GEOSErrorHandler);

    //Create the GEOS Geometries
    GEOSGeometry* inGeom = GEOS::importGeometry(handle, this);
    GEOSGeometry* otherGeom = GEOS::importGeometry(handle, other);

    bool intersects = false;
    if (inGeom && otherGeom) intersects = checkGEOSResult(GEOSIntersects_r(handle, inGeom, otherGeom));

    //Destroy the geometry
    GEOSGeom_destroy_r(handle, inGeom);
    GEOSGeom_destroy_r(handle, otherGeom);

    finishGEOS_r(handle);

    return intersects;

#else // OSGEARTH_HAVE_GEOS

    OE_WARN << LC << "Intersects failed - GEOS not available" << std::endl;
    return false;

#endif // OSGEARTH_HAVE_GEOS
}

double
Geometry::getSignedDistance2D(
    const osg::Vec3d& point) const
{
    // simple point check.
    double r2 = DBL_MAX;
    double d2, x, y;
    for (const auto& v : *this)
    {
        x = v.x() - point.x(), y = v.y() - point.y();
        d2 = x * x + y * y;
        if (d2 < r2) r2 = d2;
    }
    return sqrt(r2);
}

osg::Vec3d
Geometry::localize()
{
    osg::Vec3d offset;

    Bounds bounds = getBounds();
    if ( bounds.valid() )
    {
        osg::Vec3d center = bounds.center();
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

void Geometry::removeDuplicates()
{
    if (size() > 1)
    {
        osg::Vec3d v = front();
        for (Geometry::iterator itr = begin(); itr != end(); )
        {
            if (itr != begin() && v == *itr)
            {
                itr = erase(itr);
            }
            else
            {
                v = *itr;
                itr++;
            }
        }
    }
}

void
Geometry::removeColinearPoints()
{
    if ( size() >= 3 )
    {
        std::vector<unsigned> ind;

        for(unsigned i=0; i<size()-2; ++i)
        {
            osg::Vec3d v0( at(i+1) - at(i) );
            v0.normalize();
            osg::Vec3d v1( at(i+2) - at(i) );
            v1.normalize();
            if ( osg::equivalent(v0*v1, 1.0) )
                ind.push_back(i+1);
        }

        for(std::vector<unsigned>::reverse_iterator r = ind.rbegin(); r != ind.rend(); ++r)
        {
            erase( begin() + (*r) );
        }
    }
}

Geometry::Orientation
Geometry::getOrientation() const
{
    // adjust for a non-open ring:
    int n = size();
    if ( n > 0 && front() == back() )
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

double
Geometry::getLength() const
{
    if (empty())
        return 0.0;

    double length = 0;
    for (unsigned int i = 0; i < size()-1; ++i)
    {
        osg::Vec3d current = (*this)[i];
        osg::Vec3d next    = (*this)[i+1];
        length += (next - current).length();
    }
    return length;
}

// ensures that the first and last points are idential.
void
Geometry::close()
{
    if ( size() > 0 && front() != back() )
        push_back( front() );
}

void
Geometry::forEachPart(bool includePolygonHoles, const std::function<void(Geometry*)>& func)
{
    GeometryIterator i(this, includePolygonHoles);
    i.forEach(func);
}

void
Geometry::forEachPart(bool includePolygonHoles, const std::function<void(const Geometry*)>& func) const
{
    ConstGeometryIterator i(this, includePolygonHoles);
    i.forEach(func);
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

void
PointSet::close()
{
    //NOP. Don't close point sets..
}

//----------------------------------------------------------------------------

Point::Point(const Point& rhs) :
    PointSet(rhs)
{
    //nop
}

Point::~Point()
{
}

void
Point::set(const osg::Vec3d& value)
{
    clear();
    push_back(value);
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

void
LineString::close()
{
    //NOP - dont' close line strings.
}

double
LineString::getSignedDistance2D(
    const osg::Vec3d& a) const
{
    double r = DBL_MAX;
    Segment2d seg;

    for(int i=0; i<size()-1; ++i)
    {
        seg._a.set((*this)[i].x(), (*this)[i].y(), 0);
        seg._b.set((*this)[i + 1].x(), (*this)[i + 1].y(), 0);
        r = std::min(r, seg.squaredDistanceTo(a));
    }

    return sqrt(r);
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

double
Ring::getLength() const
{
    if (empty())
        return 0.0;

    double length = Geometry::getLength();
    if ( isOpen() )
    {
        length += (front()-back()).length();
    }
    return length;
}

// ensures that the first and last points are not idential.
void
Ring::open()
{
    while( size() > 2 && front() == back() )
        erase( end()-1 );
}

void
Ring::close()
{
    Geometry::close();
}

// whether the ring is open.
bool
Ring::isOpen() const
{
    return size() > 1 && front() != back();
}

// gets the signed area.
double
Ring::getSignedArea2D() const
{
    // Computes area based on the surveyors formula
    const_cast<Ring*>(this)->open();

    unsigned int n = size();
    double area = 0.0;
    int j = n - 1;
    for (int i = 0; i < n; i++)
    {
        area += ((*this)[j].x() + (*this)[i].x()) * ((*this)[j].y() - (*this)[i].y());
        j = i;
    }
    return area / 2.0;
}

double
Ring::getSignedDistance2D(
    const osg::Vec3d& a) const
{
    Segment2d seg;
    double r = DBL_MAX;

    unsigned i = isOpen() ? 0 : 1;
    unsigned j = isOpen() ? size() - 1 : 0;

    for (; i < size(); j = i++)
    {
        seg._a.set((*this)[i].x(), (*this)[i].y(), 0.0);
        seg._b.set((*this)[j].x(), (*this)[j].y(), 0.0);
        r = std::min(r, seg.squaredDistanceTo(a));
    }

    r = sqrt(r);
    return contains2D(a.x(), a.y()) ? -r : r;
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
    bool is_open = isOpen();
    unsigned i = is_open ? 0 : 1;
    unsigned j = is_open ? size() - 1 : 0;
    for( ; i<size(); j = i++ )
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
    for (auto& hole : rhs._holes)
        _holes.push_back(new Ring(&hole->asVector()));
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

void
Polygon::removeDuplicates()
{
    Ring::removeDuplicates();
    for( RingCollection::const_iterator i = _holes.begin(); i != _holes.end(); ++i )
        (*i)->removeDuplicates();
}

void
Polygon::removeColinearPoints()
{
    Ring::removeColinearPoints();
    for( RingCollection::const_iterator i = _holes.begin(); i != _holes.end(); ++i )
        (*i)->removeColinearPoints();
}

double
Polygon::getSignedDistance2D(
    const osg::Vec3d& a) const
{
    Segment2d seg;
    double r = DBL_MAX;

    r = Ring::getSignedDistance2D(a);

    for (const auto& hole : _holes)
        r = std::min(r, hole->getSignedDistance2D(a));

    return r;
}

//----------------------------------------------------------------------------

MultiGeometry::MultiGeometry( const MultiGeometry& rhs ) :
Geometry( rhs )
{
    for( GeometryCollection::const_iterator i = rhs._parts.begin(); i != rhs._parts.end(); ++i )
        _parts.push_back( i->get()->clone() );
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
    if (_parts.size() == 0)
        return TYPE_UNKNOWN;

    if (_parts.front()->getType() == TYPE_MULTI)
        return _parts.front()->getComponentType();

    return _parts.front()->getType();
}

int
MultiGeometry::getTotalPointCount() const
{
    int total = 0;
    for( GeometryCollection::const_iterator i = _parts.begin(); i != _parts.end(); ++i )
        total += i->get()->getTotalPointCount();
    return total;
}

double
MultiGeometry::getLength() const
{
    double total = 0.0;
    for( GeometryCollection::const_iterator i = _parts.begin(); i != _parts.end(); ++i )
        total += i->get()->getLength();
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

void
MultiGeometry::close()
{
    for( GeometryCollection::const_iterator i = _parts.begin(); i != _parts.end(); ++i )
    {
        i->get()->close();
    }
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

void
MultiGeometry::removeColinearPoints()
{
    for( GeometryCollection::const_iterator i = _parts.begin(); i != _parts.end(); ++i )
    {
        i->get()->removeColinearPoints();
    }
}

double
MultiGeometry::getSignedDistance2D(
    const osg::Vec3d& a) const
{
    double r = DBL_MAX;
    for (const auto& part : _parts)
    {
        r = std::min(part->getSignedDistance2D(a), r);
    }
    return r;
}

bool
MultiGeometry::contains2D(double x, double y) const
{
    for (const auto& part : _parts)
    {
        if (part->contains2D(x, y))
            return true;
    }
    return false;
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

    Geometry* current = _stack.front();
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
