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
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarthSymbology/LineFunctor>
#include <osgEarth/GeoMath>
#include <osg/TriangleFunctor>
#include <climits>
#include <queue>
#include <map>

#define LC "[MeshSubdivider] "

using namespace osgEarth;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

namespace
{
    // convert geocenric coords to spherical geodetic coords in radians.
    void
    geocentricToGeodetic( const osg::Vec3d& g, osg::Vec2d& out_geod )
    {
        double r = g.length();
        out_geod.set( atan2(g.y(),g.x()), acos(g.z()/r) );
    }

    // calculate the lat/long midpoint, taking care to use the shortest
    // global distance.
    void
    geodeticMidpoint( const osg::Vec2d& g0, const osg::Vec2d& g1, osg::Vec2d& out_mid )
    {
        if ( fabs(g0.x()-g1.x()) < osg::PI )
            out_mid.set( 0.5*(g0.x()+g1.x()), 0.5*(g0.y()+g1.y()) );
        else if ( g1.x() > g0.x() )
            out_mid.set( 0.5*((g0.x()+2*osg::PI)+g1.x()), 0.5*(g0.y()+g1.y()) );
        else
           out_mid.set( 0.5*(g0.x()+(g1.x()+2*osg::PI)), 0.5*(g0.y()+g1.y()) );
    }

    // finds the midpoint between two geocentric coordinates. We have to convert
    // back to geographic in order to get the correct interpolation. Spherical
    // conversion is good enough
    osg::Vec3d
    geocentricMidpoint( const osg::Vec3d& v0, const osg::Vec3d& v1 )
    {
        // geocentric to spherical:
        osg::Vec2d g0, g1;
        geocentricToGeodetic(v0, g0);
        geocentricToGeodetic(v1, g1);

        osg::Vec2d mid;
        geodeticMidpoint(g0, g1, mid);

        double size = 0.5*(v0.length() + v1.length());

        // spherical to geocentric:
        double sin_lat = sin(mid.y());
        return osg::Vec3d( cos(mid.x())*sin_lat, sin(mid.x())*sin_lat, cos(mid.y()) ) * size;
    }

    // the approximate surface-distance between two geocentric points (spherical)
    double
    geocentricSurfaceDistance( const osg::Vec3d& v0, const osg::Vec3d& v1 )
    {
        osg::Vec2d g0, g1;
        geocentricToGeodetic(v0, g0);
        geocentricToGeodetic(v1, g1);
        return GeoMath::distance( v0.y(), v0.x(), v1.y(), v1.x() );
    }    

    // returns the geocentric bisection vector
    osg::Vec3d
    bisector( const osg::Vec3d& v0, const osg::Vec3d& v1 )
    {
        osg::Vec3d f = (v0+v1)*0.5;
        f.normalize();
        return f * 0.5*(v0.length() + v1.length());
    }    

    // the angle between two 3d vectors
    double
    angleBetween( const osg::Vec3d& v0, const osg::Vec3d& v1 )
    {
        osg::Vec3d v0n = v0; v0n.normalize();
        osg::Vec3d v1n = v1; v1n.normalize();
        return fabs( acos( v0n * v1n ) );
    }

    //--------------------------------------------------------------------

    template<typename ITYPE>
    struct Triangle {
        Triangle() { }
        Triangle(ITYPE i0, ITYPE i1, ITYPE i2) : _i0(i0), _i1(i1), _i2(i2) { }
        ITYPE _i0, _i1, _i2;
    };

    template<typename ITYPE> struct TriangleQueue : public std::queue<Triangle<ITYPE> > { };

    template<typename ITYPE> struct TriangleVector : public std::vector<Triangle<ITYPE> > { };

    template<typename ITYPE>
    struct TriangleData
    {
        typedef std::map<osg::Vec3,ITYPE> VertMap;
        VertMap _vertMap;
        osg::Vec3Array* _verts;
        TriangleQueue<ITYPE> _tris;
        
        TriangleData()
        {
            _verts = new osg::Vec3Array();
        }

        ITYPE record( const osg::Vec3& v )
        {
            typename VertMap::iterator i = _vertMap.find(v);
            if ( i == _vertMap.end() )
            {
                ITYPE index = _verts->size();
                _verts->push_back(v);
                _vertMap[v] = index;
                return index;
            }
            else
            {
                return i->second;
            }
        }
        
        void operator()( const osg::Vec3& v0, const osg::Vec3& v1, const osg::Vec3& v2, bool temp )
        {
            _tris.push( Triangle<ITYPE>(record(v0), record(v1), record(v2)) );
        }
    };      

    template<typename ITYPE> 
    struct Edge
    {
        Edge() { }
        Edge( ITYPE i0, ITYPE i1 ) : _i0(i0), _i1(i1) { }
        ITYPE _i0, _i1;
        bool operator < (const Edge& rhs) const {
            return
                _i0 < rhs._i0 ? true :
                _i0 > rhs._i0 ? false :
                _i1 < rhs._i1 ? true :
                _i1 > rhs._i1 ? false :
                false;
        }
        bool operator == (const Edge& rhs) const { return _i0==rhs._i0 && _i1==rhs._i1; }
    };

    template<typename ITYPE>
    struct EdgeMap : public std::map<Edge<ITYPE>,ITYPE> { };          
    
    /**
     * Populates the geometry object with a collection of index elements primitives.
     */
    template<typename ETYPE, typename ITYPE, typename VTYPE>
    void populateTriangles( osg::Geometry& geom, const TriangleVector<ITYPE>& tris, unsigned int maxElementsPerEBO )
    {
        unsigned int totalTris = tris.size();
        unsigned int totalTrisWritten = 0;
        unsigned int numElementsInCurrentEBO = maxElementsPerEBO;

        ETYPE* ebo = 0L;

        for( typename TriangleVector<ITYPE>::const_iterator i = tris.begin(); i != tris.end(); ++i )
        {
            if ( numElementsInCurrentEBO+2 >= maxElementsPerEBO )
            {
                if ( ebo )
                {
                    geom.addPrimitiveSet( ebo );
                }

                ebo = new ETYPE( GL_TRIANGLES );

                unsigned int trisRemaining = totalTris - totalTrisWritten;
                ebo->reserve( osg::minimum( trisRemaining*3, maxElementsPerEBO ) );

                numElementsInCurrentEBO = 0;
            }
            ebo->push_back( static_cast<VTYPE>( i->_i0 ) );
            ebo->push_back( static_cast<VTYPE>( i->_i1 ) );
            ebo->push_back( static_cast<VTYPE>( i->_i2 ) );

            numElementsInCurrentEBO += 3;
            ++totalTrisWritten;
        }

        if ( ebo && ebo->size() > 0 )
        {
            geom.addPrimitiveSet( ebo );
        }
    }

    //----------------------------------------------------------------------

    template<typename ITYPE>
    struct Line {
        Line() { }
        Line(ITYPE i0, ITYPE i1) : _i0(i0), _i1(i1) { }
        ITYPE _i0, _i1;
    };

    template<typename ITYPE> struct LineQueue : public std::queue<Line<ITYPE> > { };

    template<typename ITYPE> struct LineVector : public std::vector<Line<ITYPE> > { };

    template<typename ITYPE>
    struct LineData
    {
        typedef std::map<osg::Vec3,ITYPE> VertMap;
        VertMap _vertMap;
        osg::Vec3Array* _verts;
        LineQueue<ITYPE> _lines;
        
        LineData()
        {
            _verts = new osg::Vec3Array();
        }

        ITYPE record( const osg::Vec3& v )
        {
            typename VertMap::iterator i = _vertMap.find(v);
            if ( i == _vertMap.end() )
            {
                ITYPE index = _verts->size();
                _verts->push_back(v);
                _vertMap[v] = index;
                return index;
            }
            else
            {
                return i->second;
            }
        }
        
        void operator()( const osg::Vec3& v0, const osg::Vec3& v1, bool temp )
        {
            _lines.push( Line<ITYPE>( record(v0), record(v1) ) );
        }
    };       
    
    /**
     * Populates the geometry object with a collection of index elements primitives.
     */
    template<typename ETYPE, typename ITYPE, typename VTYPE>
    void populateLines( osg::Geometry& geom, const LineVector<ITYPE>& lines, unsigned int maxElementsPerEBO )
    {
        unsigned int totalLines = lines.size();
        unsigned int totalLinesWritten = 0;
        unsigned int numElementsInCurrentEBO = maxElementsPerEBO;

        ETYPE* ebo = 0L;

        for( typename LineVector<ITYPE>::const_iterator i = lines.begin(); i != lines.end(); ++i )
        {
            if ( numElementsInCurrentEBO+2 >= maxElementsPerEBO )
            {
                if ( ebo )
                {
                    geom.addPrimitiveSet( ebo );
                }

                ebo = new ETYPE( GL_LINES );

                unsigned int linesRemaining = totalLines - totalLinesWritten;
                ebo->reserve( osg::minimum( linesRemaining*2, maxElementsPerEBO ) );

                numElementsInCurrentEBO = 0;
            }
            ebo->push_back( static_cast<VTYPE>( i->_i0 ) );
            ebo->push_back( static_cast<VTYPE>( i->_i1 ) );

            numElementsInCurrentEBO += 3;
            ++totalLinesWritten;
        }

        if ( ebo && ebo->size() > 0 )
        {
            geom.addPrimitiveSet( ebo );
        }
    }

    static const osg::Vec3d s_pole(0,0,1);
    static const double s_maxLatAdjustment(0.75);

    /**
     * Collects all the line segments from the geometry, coalesces them into a single
     * line set, subdivides it according to the granularity threshold, and replaces
     * the data in the Geometry object with the new vertex and primitive data.
     */
    template<typename ITYPE>
    void subdivideLines(
        double granularity,
        osg::Geometry& geom,
        const osg::Matrixd& W2L, // world=>local xform
        const osg::Matrixd& L2W, // local=>world xform
        unsigned int maxElementsPerEBO )
    {
        // collect all the line segments in the geometry.
        LineFunctor<LineData<ITYPE> > data;
        geom.accept( data );
    
        int numLinesIn = data._lines.size();

        LineVector<ITYPE> done;
        done.reserve( 2 * data._lines.size() );

        // Subdivide lines until we run out.
        while( data._lines.size() > 0 )
        {
            Line<ITYPE> line = data._lines.front();
            data._lines.pop();

            osg::Vec3d v0_w = (*data._verts)[line._i0] * L2W;
            osg::Vec3d v1_w = (*data._verts)[line._i1] * L2W;

            double g0 = angleBetween(v0_w, v1_w);

            if ( g0 > granularity )
            {
                data._verts->push_back( geocentricMidpoint(v0_w, v1_w) * W2L );
                ITYPE i = data._verts->size()-1;

                data._lines.push( Line<ITYPE>( line._i0, i ) );
                data._lines.push( Line<ITYPE>( i, line._i1 ) );
            }
            else
            {
                // line is small enough- put it on the "done" list.
                done.push_back( line );
            }
        }

        if ( done.size() > 0 )
        {
            while( geom.getNumPrimitiveSets() > 0 )
                geom.removePrimitiveSet(0);

            // set the new VBO.
            geom.setVertexArray( data._verts );

            if ( data._verts->size() < 256 )
                populateLines<osg::DrawElementsUByte,ITYPE,GLubyte>( geom, done, maxElementsPerEBO );
            else if ( data._verts->size() < 65536 )
                populateLines<osg::DrawElementsUShort,ITYPE,GLushort>( geom, done, maxElementsPerEBO );
            else
                populateLines<osg::DrawElementsUInt,ITYPE,GLuint>( geom, done, maxElementsPerEBO );
        }
    }


    //----------------------------------------------------------------------

    /**
     * Collects all the triangles from the geometry, coalesces them into a single
     * triangle set, subdivides them according to the granularity threshold, and
     * replaces the data in the Geometry object with the new vertex and primitive
     * data.
     *
     * The subdivision algorithm is adapted from http://bit.ly/dTIagq
     * (c) Copyright 2010 Patrick Cozzi and Deron Ohlarik, MIT License.
     */
    template<typename ITYPE>
    void subdivideTriangles(
        double granularity,
        osg::Geometry& geom,
        const osg::Matrixd& W2L, // world=>local xform
        const osg::Matrixd& L2W, // local=>world xform
        unsigned int maxElementsPerEBO )
    {
        // collect all the triangled in the geometry.
        osg::TriangleFunctor<TriangleData<ITYPE> > data;
        geom.accept( data );

        int numTrisIn = data._tris.size();

        TriangleVector<ITYPE> done;
        done.reserve(2.0 * data._tris.size());

        // Used to make sure shared edges are not split more than once.
        EdgeMap<ITYPE> edges;

        // Subdivide triangles until we run out
        while( data._tris.size() > 0 )
        {
            Triangle<ITYPE> tri = data._tris.front();
            data._tris.pop();

            osg::Vec3d v0_w = (*data._verts)[tri._i0] * L2W;
            osg::Vec3d v1_w = (*data._verts)[tri._i1] * L2W;
            osg::Vec3d v2_w = (*data._verts)[tri._i2] * L2W;

            double g0 = angleBetween(v0_w, v1_w);
            double g1 = angleBetween(v1_w, v2_w);
            double g2 = angleBetween(v2_w, v0_w);
            double max = osg::maximum( g0, osg::maximum(g1, g2) );

            if ( max > granularity )
            {
                if ( g0 == max )
                {
                    Edge<ITYPE> edge( osg::minimum(tri._i0, tri._i1), osg::maximum(tri._i0, tri._i1) );
                    
                    typename EdgeMap<ITYPE>::iterator ei = edges.find(edge);
                    ITYPE i;
                    if ( ei == edges.end() )
                    {
                        data._verts->push_back( geocentricMidpoint(v0_w, v1_w) * W2L );
                        i = data._verts->size() - 1;
                        edges[edge] = i;
                    }
                    else
                    {
                        i = ei->second;
                    }

                    data._tris.push( Triangle<ITYPE>(tri._i0, i, tri._i2) );
                    data._tris.push( Triangle<ITYPE>(i, tri._i1, tri._i2) );
                }
                else if ( g1 == max )
                {
                    Edge<ITYPE> edge( osg::minimum(tri._i1, tri._i2), osg::maximum(tri._i1,tri._i2) );

                    typename EdgeMap<ITYPE>::iterator ei = edges.find(edge);
                    ITYPE i;
                    if ( ei == edges.end() )
                    {
                        data._verts->push_back( geocentricMidpoint(v1_w, v2_w) * W2L );
                        i = data._verts->size() - 1;
                        edges[edge] = i;
                    }
                    else
                    {
                        i = ei->second;
                    }

                    data._tris.push( Triangle<ITYPE>(tri._i1, i, tri._i0) );
                    data._tris.push( Triangle<ITYPE>(i, tri._i2, tri._i0) );
                }
                else if ( g2 == max )
                {
                    Edge<ITYPE> edge( osg::minimum(tri._i2, tri._i0), osg::maximum(tri._i2,tri._i0) );

                    typename EdgeMap<ITYPE>::iterator ei = edges.find(edge);
                    ITYPE i;
                    if ( ei == edges.end() )
                    {
                        data._verts->push_back( geocentricMidpoint(v2_w, v0_w) * W2L );
                        i = data._verts->size() - 1;
                        edges[edge] = i;
                    }
                    else
                    {
                        i = ei->second;
                    }

                    data._tris.push( Triangle<ITYPE>(tri._i2, i, tri._i1) );
                    data._tris.push( Triangle<ITYPE>(i, tri._i0, tri._i1) );
                }
            }
            else
            {
                // triangle is small enough- put it on the "done" list.
                done.push_back(tri);
            }
        }

        if ( done.size() > 0 )
        {
            // first, remove the old primitive sets.
            while( geom.getNumPrimitiveSets() > 0 )
                geom.removePrimitiveSet( 0 );

            // set the new VBO.
            geom.setVertexArray( data._verts );

            if ( data._verts->size() < 256 )
                populateTriangles<osg::DrawElementsUByte,ITYPE,GLubyte>( geom, done, maxElementsPerEBO );
            else if ( data._verts->size() < 65536 )
                populateTriangles<osg::DrawElementsUShort,ITYPE,GLushort>( geom, done, maxElementsPerEBO );
            else
                populateTriangles<osg::DrawElementsUInt,ITYPE,GLuint>( geom, done, maxElementsPerEBO );
        }
    }

    template<typename ITYPE>
    void subdivide(
        double granularity,
        osg::Geometry& geom,
        const osg::Matrixd& W2L, // world=>local xform
        const osg::Matrixd& L2W, // local=>world xform
        unsigned int maxElementsPerEBO )
    {
        //double maxEdgeLen = (2.0 * 6378100.0 * sin(0.5 * granularity));

        GLenum mode = geom.getPrimitiveSet(0)->getMode();

        if ( mode == GL_POINTS )
        {
            return;
        }
        else if ( mode == GL_LINES || mode == GL_LINE_STRIP || mode == GL_LINE_LOOP )
        {
            subdivideLines<ITYPE>( granularity, geom, W2L, L2W, maxElementsPerEBO );
        }
        else
        {
            subdivideTriangles<ITYPE>( granularity, geom, W2L, L2W, maxElementsPerEBO );
        }
    }
}

//------------------------------------------------------------------------

MeshSubdivider::MeshSubdivider(const osg::Matrixd& world2local,
                               const osg::Matrixd& local2world ) :
_local2world(local2world),
_world2local(world2local),
_maxElementsPerEBO( INT_MAX )
{
    if ( !_world2local.isIdentity() && _local2world.isIdentity() )
        _local2world = osg::Matrixd::inverse(_world2local);
    else if ( _world2local.isIdentity() && !_local2world.isIdentity() )
        _world2local = osg::Matrixd::inverse(_local2world);
}

void
MeshSubdivider::run(double granularity, osg::Geometry& geom)
{
    if ( geom.getNumPrimitiveSets() < 1 )
        return;

    subdivide<GLuint>( granularity, geom, _world2local, _local2world, _maxElementsPerEBO );
}
