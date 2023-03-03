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

    static Geometry* makeValid_GEOS(const Geometry* geom)
    {
        static Mutex m;
        ScopedMutexLock lock(m);

#ifdef OSGEARTH_HAVE_GEOS
        Geometry* output = nullptr;
        GEOSContextHandle_t handle = initGEOS_r(OSGEARTH_WarningHandler, OSGEARTH_GEOSErrorHandler);
        GEOSGeometry* inGeom = GEOS::importGeometry(handle, geom);
        if (inGeom)
        {
            GEOSMakeValidParams* params = GEOSMakeValidParams_create_r(handle);
            GEOSGeometry* outGeom = GEOSMakeValidWithParams_r(handle, inGeom, params);
            if (outGeom)
            {
                output = GEOS::exportGeometry(handle, outGeom);
                GEOSGeom_destroy_r(handle, outGeom);
            }
            GEOSMakeValidParams_destroy_r(handle, params);
        }
        GEOSGeom_destroy_r(handle, inGeom);
        finishGEOS_r(handle);
        return output;
#else
        output = geom->clone();
        output->normalize();
        return output;
#endif
    }
}

Geometry* Geometry::makeValid(const Geometry* in)
{
    return makeValid_GEOS(in);
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

        GEOSBufferParams_destroy_r(handle, geosBufferParams);
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

void Geometry::removeDuplicates()
{
    if (size() > 1)
    {
        Vec3dVector temp;
        temp.reserve(size());
        temp.push_back(front());
        for (unsigned i = 1, j = 0; i < size(); j = i++)
        {
            if ((*this)[i] != (*this)[j])
                temp.push_back((*this)[i]);
        }
        this->swap(temp);
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

PointSet::PointSet(const PointSet& rhs) :
    Geometry(rhs)
{
    //nop
}


//----------------------------------------------------------------------------

Point::Point(const Point& rhs) :
    PointSet(rhs)
{
    //nop
}

void
Point::set(const osg::Vec3d& value)
{
    clear();
    push_back(value);
}

//----------------------------------------------------------------------------

LineString::LineString(const LineString& rhs) :
    Geometry(rhs)
{
    //nop
}

LineString::LineString(const Vec3dVector* data) :
    Geometry(data)
{
    //nop
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

Ring::Ring(const Ring& rhs) :
    Geometry(rhs)
{
    //nop
}

Ring::Ring(const Vec3dVector* data) :
    Geometry(data)
{
    // rings must be closed (last point == first point)
    close();
}

void
Ring::rewind(Orientation orientation)
{
    Orientation current = getOrientation();
    if (current != orientation && current != ORIENTATION_DEGENERATE && orientation != ORIENTATION_DEGENERATE)
    {
        std::reverse(begin(), end());
    }
}

void
Ring::normalize()
{
    close();
    rewind(Orientation::ORIENTATION_CCW);
}

Geometry*
Ring::cloneAs(const Geometry::Type& newType) const
{
    if (newType == TYPE_LINESTRING)
    {
        LineString* line = new LineString(&this->asVector());
        if (line->size() > 1 && line->front() != line->back())
            line->push_back(front());
        return line;
    }
    else return Geometry::cloneAs(newType);
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

// ensures that the first and last points are identical.
void
Ring::close()
{
    if (size() > 0 && front() != back())
        push_back(front());
}

// whether the ring is open.
bool
Ring::isOpen() const
{
    return size() > 1 && front() != back();
}

Geometry::Orientation
Ring::getOrientation() const
{
    return
        size() < (isOpen() ? 3 : 4) ? ORIENTATION_DEGENERATE :
        getSignedArea2D() >= 0.0 ? ORIENTATION_CCW :
        ORIENTATION_CW;
}

// gets the signed area.
double
Ring::getSignedArea2D() const
{
    // Computes area based on the surveyors formula
    double area = 0.0;
    if (isOpen())
    {
        for (int i = size() - 1, j = 0; j < size(); i = j++)
            area += ((*this)[i].x() * (*this)[j].y()) - ((*this)[i].y() * (*this)[j].x());
    }
    else
    {
        for (int i = 0; i < size() - 1; ++i)
            area += ((*this)[i].x() * (*this)[i + 1].y()) - ((*this)[i].y() * (*this)[i + 1].x());
    }
    return 0.5 * area;
}

double
Ring::getSignedDistance2D(const osg::Vec3d& a) const
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

// point-in-polygon test
bool
Ring::contains2D(double x, double y) const
{
    bool result = false;
    const Ring& poly = *this;
    bool is_open = size() > 0 && front() != back();
    unsigned i = is_open ? 0 : 1;
    unsigned j = is_open ? size() - 1 : 0;
    for (; i < size(); j = i++)
    {
        if ((((poly[i].y() <= y) && (y < poly[j].y())) ||
            ((poly[j].y() <= y) && (y < poly[i].y()))) &&
            (x < (poly[j].x() - poly[i].x()) * (y - poly[i].y()) / (poly[j].y() - poly[i].y()) + poly[i].x()))
        {
            result = !result;
        }
    }
    return result;
}

//----------------------------------------------------------------------------

Polygon::Polygon(const Polygon& rhs) :
    Ring(rhs)
{
    for (auto& hole : rhs._holes)
        _holes.push_back(new Ring(&hole->asVector()));
}

Polygon::Polygon(const Vec3dVector* data) :
    Ring(data)
{
    normalize();
}

void
Polygon::normalize()
{
    removeDuplicates();

    // ensure outer and all inner rings are closed
    close();

    // rewind outer to CCW
    rewind(Orientation::ORIENTATION_CCW);

    // wind inner holds to CW
    for (auto& hole : _holes)
    {
        hole->rewind(Orientation::ORIENTATION_CW);
    }
}

int
Polygon::getTotalPointCount() const
{
    int total = Ring::getTotalPointCount();
    for (auto& hole : _holes)
        total += hole->getTotalPointCount();
    return total;
}

bool
Polygon::contains2D(double x, double y) const
{
    // first check the outer ring
    if ( !Ring::contains2D(x, y) )
        return false;

    // then check each inner ring (holes). Point has to be inside the outer ring,
    // but NOT inside any of the holes
    for (auto& hole : _holes)
        if (hole->contains2D(x, y))
            return false;

    return true;
}

void
Polygon::close()
{
    Ring::close();
    for (auto& hole : _holes)
        hole->close();
}

void
Polygon::removeDuplicates()
{
    Ring::removeDuplicates();
    for (auto& hole : _holes)
        hole->removeDuplicates();
}

void
Polygon::removeColinearPoints()
{
    Ring::removeColinearPoints();
    for (auto& hole : _holes)
        hole->removeColinearPoints();
}

double
Polygon::getSignedDistance2D(const osg::Vec3d& a) const
{
    Segment2d seg;
    double r = DBL_MAX;

    r = Ring::getSignedDistance2D(a);

    for (const auto& hole : _holes)
        r = std::min(r, hole->getSignedDistance2D(a));

    return r;
}

//----------------------------------------------------------------------------

MultiGeometry::MultiGeometry(const MultiGeometry& rhs) :
    Geometry(rhs)
{
    for (auto& part : _parts)
        _parts.push_back(part->clone());
}

MultiGeometry::MultiGeometry(const GeometryCollection& parts) :
    _parts(parts)
{
    //nop
}

void
MultiGeometry::normalize()
{
    for (auto& part : _parts)
        part->normalize();
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
    for (auto& part : _parts)
        total += part->getTotalPointCount();
    return total;
}

double
MultiGeometry::getLength() const
{
    double total = 0.0;
    for (auto& part : _parts)
        total += part->getLength();
    return total;
}

unsigned
MultiGeometry::getNumGeometries() const
{
    unsigned total = 0;
    for (auto& part : _parts)
        total += part->getNumGeometries();
    return total;
}

Bounds
MultiGeometry::getBounds() const
{
    Bounds bounds = Geometry::getBounds();

    for (auto& part : _parts)
    {
        bounds.expandBy(part->getBounds());
    }
    return bounds;
}

Geometry*
MultiGeometry::cloneAs(const Geometry::Type& newType) const
{
    MultiGeometry* multi = new MultiGeometry();
    for (auto& part : _parts)
    {
        Geometry* new_part = part->cloneAs(part->getType());
        if (new_part)
            multi->getComponents().push_back(new_part);
    }
    return multi;
}

bool
MultiGeometry::isValid() const
{
    if ( _parts.size() == 0 )
        return false;

    bool valid = true;
    for (auto& part : _parts)
        if (!part->isValid())
            valid = false;

    return valid;
}

void
MultiGeometry::close()
{
    for (auto& part : _parts)
        part->close();
}

void
MultiGeometry::removeDuplicates()
{
    for (auto& part : _parts)
        part->removeDuplicates();
}

void
MultiGeometry::removeColinearPoints()
{
    for (auto& part : _parts)
        part->removeColinearPoints();
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

GeometryIterator::GeometryIterator(Geometry* geom, bool holes) :
    _next(0L),
    _traverseMulti(true),
    _traversePolyHoles(holes)
{
    if (geom)
    {
        _stack.push(geom);
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
