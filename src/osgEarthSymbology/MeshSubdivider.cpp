/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/LineFunctor>
#include <osgEarth/GeoMath>
#include <osg/TriangleFunctor>
#include <osg/TriangleIndexFunctor>
#include <climits>
#include <queue>
#include <map>

#define LC "[MeshSubdivider] "

using namespace osgEarth;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

namespace
{
    // returns the geocentric bisection vector
    osg::Vec3d
    bisector( const osg::Vec3d& v0, const osg::Vec3d& v1 )
    {
        osg::Vec3d f = (v0+v1)*0.5;
        f.normalize();
        return f * 0.5*(v0.length() + v1.length());
    }

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
    geocentricMidpoint( const osg::Vec3d& v0, const osg::Vec3d& v1, GeoInterpolation interp )
    {
        if ( interp == GEOINTERP_GREAT_CIRCLE )
        {
            return bisector(v0, v1);
        }
        else
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

    // the angle between two 3d vectors
    double
    angleBetween( const osg::Vec3d& v0, const osg::Vec3d& v1 )
    {
        osg::Vec3d v0n = v0; v0n.normalize();
        osg::Vec3d v1n = v1; v1n.normalize();
        return fabs( acos( v0n * v1n ) );
    }

    //--------------------------------------------------------------------

    struct Triangle
    {
        Triangle() { }
        Triangle(GLuint i0, GLuint i1, GLuint i2) : 
        _i0(i0), _i1(i1), _i2(i2){}                 
        GLuint _i0, _i1, _i2;
    };

    typedef std::queue<Triangle> TriangleQueue;
    typedef std::vector<Triangle> TriangleVector;
    

    struct TriangleData
    {
        typedef std::map<osg::Vec3,GLuint> VertMap;        
        VertMap _vertMap;
        osg::Vec3Array* _sourceVerts;
        osg::Vec4Array* _sourceColors;
        osg::Vec2Array* _sourceTexCoords;
        osg::Vec3Array* _sourceNormals;
        osg::ref_ptr<osg::Vec3Array> _verts;
        osg::ref_ptr<osg::Vec4Array> _colors;
        osg::ref_ptr<osg::Vec2Array> _texcoords;
        osg::ref_ptr<osg::Vec3Array> _normals;
        TriangleQueue _tris;
        
        TriangleData()
        {            
            _verts           = new osg::Vec3Array();             
            _sourceVerts     = 0;
            _sourceColors    = 0;
            _sourceTexCoords = 0;
            _sourceNormals   = 0;
        }       

        void setSourceVerts(osg::Vec3Array* sourceVerts )
        {
            _sourceVerts = sourceVerts;
        }

        void setSourceColors(osg::Vec4Array* sourceColors)
        {
            if ( sourceColors )
            {
                _sourceColors = sourceColors;
                _colors       = new osg::Vec4Array();
            }
        }

        void setSourceTexCoords(osg::Vec2Array* sourceTexCoords)
        {
            if ( sourceTexCoords )
            {
                _sourceTexCoords = sourceTexCoords;
                _texcoords       = new osg::Vec2Array();
            }
        }

        void setSourceNormals(osg::Vec3Array* sourceNormals)
        {
            if ( sourceNormals )
            {
                _sourceNormals = sourceNormals;
                _normals       = new osg::Vec3Array();
            }
        }

        GLuint record( const osg::Vec3& v, const osg::Vec2f& t, const osg::Vec4f& c, const osg::Vec3& n )
        {
            VertMap::iterator i = _vertMap.find(v);
            if ( i == _vertMap.end() )
            {
                GLuint index = _verts->size();
                _verts->push_back(v);                
                _vertMap[v] = index;
                //Only push back the texture coordinate if it's valid
                if (_texcoords)
                {
                    _texcoords->push_back( t );
                }
                if (_colors)
                {
                    _colors->push_back( c );
                }
                if ( _normals )
                {
                    _normals->push_back( n );
                }
                return index;
            }
            else
            {
                return i->second;
            }
        }
       

        void operator() (unsigned int p1, unsigned int p2, unsigned int p3)
        {
            const osg::Vec3 v0 = (*_sourceVerts)[p1];
            const osg::Vec3 v1 = (*_sourceVerts)[p2];
            const osg::Vec3 v2 = (*_sourceVerts)[p3];   

            osg::Vec2 t0, t1, t2;
            if (_sourceTexCoords)
            {
                t0 = (*_sourceTexCoords)[p1];
                t1 = (*_sourceTexCoords)[p2];
                t2 = (*_sourceTexCoords)[p3];
            }
            
            osg::Vec4 c0, c1, c2;
            if (_sourceColors)
            {
                c0 = (*_sourceColors)[p1];
                c1 = (*_sourceColors)[p2];
                c2 = (*_sourceColors)[p3];
            }

            osg::Vec3 n0, n1, n2;
            if (_sourceNormals)
            {
                n0 = (*_sourceNormals)[p1];
                n1 = (*_sourceNormals)[p2];
                n2 = (*_sourceNormals)[p3];
            }

            _tris.push( Triangle(record(v0, t0, c0, n0), record(v1, t1, c1, n1), record(v2, t2, c2, n2)) );
        }
    };      

    struct Edge
    {
        Edge() { }
        Edge( GLuint i0, GLuint i1 ) : _i0(i0), _i1(i1) { }
        GLuint _i0, _i1;
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

    typedef std::map<Edge,GLuint> EdgeMap;
    
    /**
     * Populates the geometry object with a collection of index elements primitives.
     */
    template<typename ETYPE, typename VTYPE>
    void populateTriangles( osg::Geometry& geom, const TriangleVector& tris, unsigned int maxElementsPerEBO )
    {
        unsigned int totalTris = tris.size();
        unsigned int totalTrisWritten = 0;
        unsigned int numElementsInCurrentEBO = maxElementsPerEBO;

        ETYPE* ebo = 0L;

        for( TriangleVector::const_iterator i = tris.begin(); i != tris.end(); ++i )
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

    struct Line
    {
        Line() { }
        Line(GLuint i0, GLuint i1) : _i0(i0), _i1(i1) { }
        GLuint _i0, _i1;
    };

    typedef std::queue<Line>  LineQueue;
    typedef std::vector<Line> LineVector;

    struct LineData
    {
        typedef std::map<osg::Vec3,GLuint> VertMap;        
        VertMap _vertMap;
        osg::Vec3Array* _sourceVerts;
        osg::Vec4Array* _sourceColors;
        osg::Vec2Array* _sourceTexCoords;
        osg::ref_ptr<osg::Vec3Array> _verts;        
        osg::ref_ptr<osg::Vec4Array> _colors;
        osg::ref_ptr<osg::Vec2Array> _texcoords;
        LineQueue _lines;
        
        LineData()
        {            
            _verts           = new osg::Vec3Array();      
            _colors          = 0;
            _texcoords       = 0;
            _sourceVerts     = 0;
            _sourceColors    = 0;
            _sourceTexCoords = 0;
        }       

        void setSourceVerts(osg::Vec3Array* sourceVerts )
        {
            _sourceVerts = sourceVerts;
        }

        void setSourceColors(osg::Vec4Array* sourceColors )
        {
            if ( sourceColors )
            {
                _sourceColors = sourceColors;
                _colors       = new osg::Vec4Array();
            }
        }

        void setSourceTexCoords(osg::Vec2Array* sourceTexCoords)
        {
            if ( sourceTexCoords )
            {
                _sourceTexCoords = sourceTexCoords;
                _texcoords       = new osg::Vec2Array();
            }
        }

        GLuint record( const osg::Vec3& v, const osg::Vec2f& t, const osg::Vec4f& c )
        {
            VertMap::iterator i = _vertMap.find(v);
            if ( i == _vertMap.end() )
            {
                GLuint index = _verts->size();
                _verts->push_back(v);                
                _vertMap[v] = index;
                if (_texcoords)
                {
                    _texcoords->push_back( t );
                }
                if (_colors)
                {
                    _colors->push_back( c );
                }
                return index;
            }
            else
            {
                return i->second;
            }
        }
       

        void line(unsigned p0, unsigned p1)
        {
            const osg::Vec3 v0 = (*_sourceVerts)[p0];
            const osg::Vec3 v1 = (*_sourceVerts)[p1];   

            osg::Vec2 t0, t1;
            if (_sourceTexCoords)
            {
                t0 = (*_sourceTexCoords)[p0];
                t1 = (*_sourceTexCoords)[p1];
            }
            
            osg::Vec4 c0, c1;
            if (_sourceColors)
            {
                c0 = (*_sourceColors)[p0];
                c1 = (*_sourceColors)[p1];
            }

            _lines.push( Line(record(v0,t0,c0), record(v1,t1,c1)) );
        }
    };         
    
    /**
     * Populates the geometry object with a collection of index elements primitives.
     */
    template<typename ETYPE, typename VTYPE>
    void populateLines( osg::Geometry& geom, const LineVector& lines, unsigned int maxElementsPerEBO )
    {
        unsigned int totalLines = lines.size();
        unsigned int totalLinesWritten = 0;
        unsigned int numElementsInCurrentEBO = maxElementsPerEBO;

        ETYPE* ebo = 0L;

        for( LineVector::const_iterator i = lines.begin(); i != lines.end(); ++i )
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
    void subdivideLines(
        double               granularity,
        GeoInterpolation     interp,
        osg::Geometry&       geom,
        const osg::Matrixd&  W2L, // world=>local xform
        const osg::Matrixd&  L2W, // local=>world xform
        unsigned int         maxElementsPerEBO )
    {
        // collect all the line segments in the geometry.
        LineIndexFunctor<LineData> data;
        data.setSourceVerts( static_cast<osg::Vec3Array*>(geom.getVertexArray()) );
        if ( geom.getColorBinding() == osg::Geometry::BIND_PER_VERTEX )
            data.setSourceColors( static_cast<osg::Vec4Array*>(geom.getColorArray()) );
        //LineFunctor<LineData> data;
        geom.accept( data );
    
        int numLinesIn = data._lines.size();

        LineVector done;
        done.reserve( 2 * data._lines.size() );

        // Subdivide lines until we run out.
        while( data._lines.size() > 0 )
        {
            Line line = data._lines.front();
            data._lines.pop();

            osg::Vec3d v0_w = (*data._verts)[line._i0] * L2W;
            osg::Vec3d v1_w = (*data._verts)[line._i1] * L2W;

            double g0 = angleBetween(v0_w, v1_w);

            if ( g0 > granularity )
            {
                data._verts->push_back( geocentricMidpoint(v0_w, v1_w, interp) * W2L );

                if ( data._colors )
                {
                    const osg::Vec4f& c0 = (*data._colors)[line._i0];
                    const osg::Vec4f& c1 = (*data._colors)[line._i1];
                    data._colors->push_back( (c0 + c1) / 2.0 );
                }

                if ( data._texcoords )
                {
                    const osg::Vec2& t0 = (*data._texcoords)[line._i0];
                    const osg::Vec2& t1 = (*data._texcoords)[line._i1];
                    data._texcoords->push_back( (t0 + t1) / 2.0 );
                }

                GLuint i = data._verts->size()-1;

                data._lines.push( Line( line._i0, i ) );
                data._lines.push( Line( i, line._i1 ) );
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
            if ( geom.getVertexArray()->getVertexBufferObject() && data._verts->getVertexBufferObject() )
            {
                data._verts->getVertexBufferObject()->setUsage( geom.getVertexArray()->getVertexBufferObject()->getUsage() );
            }

            if ( data._colors )
                geom.setColorArray( data._colors );

            if ( data._verts->size() < 256 )
                populateLines<osg::DrawElementsUByte,GLubyte>( geom, done, maxElementsPerEBO );
            else if ( data._verts->size() < 65536 )
                populateLines<osg::DrawElementsUShort,GLushort>( geom, done, maxElementsPerEBO );
            else
                populateLines<osg::DrawElementsUInt,GLuint>( geom, done, maxElementsPerEBO );
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
    void subdivideTriangles(
        double               granularity,
        GeoInterpolation     interp,
        osg::Geometry&       geom,
        const osg::Matrixd&  W2L, // world=>local xform
        const osg::Matrixd&  L2W, // local=>world xform
        unsigned int         maxElementsPerEBO )
    {
        
        // collect all the triangled in the geometry.
        osg::TriangleIndexFunctor<TriangleData> data;;
        data.setSourceVerts(dynamic_cast<osg::Vec3Array*>(geom.getVertexArray()));
        data.setSourceTexCoords(dynamic_cast<osg::Vec2Array*>(geom.getTexCoordArray(0)));
        if ( geom.getColorBinding() == osg::Geometry::BIND_PER_VERTEX )
            data.setSourceColors(dynamic_cast<osg::Vec4Array*>(geom.getColorArray()));
        if ( geom.getNormalBinding() == osg::Geometry::BIND_PER_VERTEX )
            data.setSourceNormals(dynamic_cast<osg::Vec3Array*>(geom.getNormalArray()));

        //TODO normals
        geom.accept( data );
        
        int numTrisIn = data._tris.size();        

        TriangleVector done;
        done.reserve(2.0 * data._tris.size());

        // Used to make sure shared edges are not split more than once.
        EdgeMap edges;

        // Subdivide triangles until we run out
        while( data._tris.size() > 0 )
        {
            Triangle tri = data._tris.front();
            data._tris.pop();

            osg::Vec3d v0_w = (*data._verts)[tri._i0] * L2W;
            osg::Vec3d v1_w = (*data._verts)[tri._i1] * L2W;
            osg::Vec3d v2_w = (*data._verts)[tri._i2] * L2W;

            osg::Vec2 t0,t1,t2;
            if ( data._texcoords.valid() )
            {
                t0 = (*data._texcoords)[tri._i0];
                t1 = (*data._texcoords)[tri._i1];
                t2 = (*data._texcoords)[tri._i2];
            }

            osg::Vec4f c0,c1,c2;
            if ( data._colors.valid() )
            {
                c0 = (*data._colors)[tri._i0];
                c1 = (*data._colors)[tri._i1];
                c2 = (*data._colors)[tri._i2];
            }

            osg::Vec3 n0,n1,n2;
            if ( data._normals.valid() )
            {
                n0 = (*data._normals)[tri._i0];
                n1 = (*data._normals)[tri._i1];
                n2 = (*data._normals)[tri._i2];
            }

            double g0 = angleBetween(v0_w, v1_w);
            double g1 = angleBetween(v1_w, v2_w);
            double g2 = angleBetween(v2_w, v0_w);
            double max = osg::maximum( g0, osg::maximum(g1, g2) );

            if ( max > granularity )
            {
                if ( g0 == max )
                {
                    Edge edge( osg::minimum(tri._i0, tri._i1), osg::maximum(tri._i0, tri._i1) );
                    
                    EdgeMap::iterator ei = edges.find(edge);
                    GLuint i;
                    if ( ei == edges.end() )
                    {
                        data._verts->push_back( geocentricMidpoint(v0_w, v1_w, interp) * W2L );
                        if ( data._colors.valid() )
                            data._colors->push_back( (c0 + c1) / 2.0f );
                        if ( data._texcoords.valid() )
                            data._texcoords->push_back( (t0 + t1) / 2.0f );
                        if ( data._normals.valid() )
                            data._normals->push_back( (n0 + n1) / 2.0f );
                        i = data._verts->size() - 1;
                        edges[edge] = i;
                    }
                    else
                    {
                        i = ei->second;
                    }

                    data._tris.push( Triangle(tri._i0, i, tri._i2) );
                    data._tris.push( Triangle(i, tri._i1, tri._i2) );
                }
                else if ( g1 == max )
                {
                    Edge edge( osg::minimum(tri._i1, tri._i2), osg::maximum(tri._i1,tri._i2) );

                    EdgeMap::iterator ei = edges.find(edge);
                    GLuint i;
                    if ( ei == edges.end() )
                    {
                        data._verts->push_back( geocentricMidpoint(v1_w, v2_w, interp) * W2L );
                        if ( data._colors.valid() )
                            data._colors->push_back( (c1 + c2) / 2.0f );
                        if ( data._texcoords.valid() )
                            data._texcoords->push_back( (t1 + t2) / 2.0f );
                        if ( data._normals.valid() )
                            data._normals->push_back( (n1 + n2) / 2.0f );
                        i = data._verts->size() - 1;
                        edges[edge] = i;
                    }
                    else
                    {
                        i = ei->second;
                    }

                    data._tris.push( Triangle(tri._i1, i, tri._i0) );
                    data._tris.push( Triangle(i, tri._i2, tri._i0) );
                }
                else if ( g2 == max )
                {
                    Edge edge( osg::minimum(tri._i2, tri._i0), osg::maximum(tri._i2,tri._i0) );

                    EdgeMap::iterator ei = edges.find(edge);
                    GLuint i;
                    if ( ei == edges.end() )
                    {
                        data._verts->push_back( geocentricMidpoint(v2_w, v0_w, interp) * W2L );
                        if ( data._colors.valid() )
                            data._colors->push_back( (c2 + c0) / 2.0f );
                        if ( data._texcoords.valid() )
                            data._texcoords->push_back( (t2 + t0) / 2.0f );
                        if ( data._normals.valid() )
                            data._normals->push_back( (n2 + n0) / 2.0f );
                        i = data._verts->size() - 1;
                        edges[edge] = i;
                    }
                    else
                    {
                        i = ei->second;
                    }

                    data._tris.push( Triangle(tri._i2, i, tri._i1) );
                    data._tris.push( Triangle(i, tri._i0, tri._i1) );
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
            geom.setVertexArray( data._verts.get() );
            if ( geom.getVertexArray()->getVertexBufferObject() && data._verts->getVertexBufferObject() )
            {
                data._verts->getVertexBufferObject()->setUsage( geom.getVertexArray()->getVertexBufferObject()->getUsage() );
            }

            if ( data._colors.valid() )
                geom.setColorArray( data._colors.get() );
            if ( data._texcoords.valid() )
                geom.setTexCoordArray(0, data._texcoords.get() );
            if ( data._normals.valid() )
                geom.setNormalArray( data._normals.get() );

            if ( data._verts->size() < 256 )
                populateTriangles<osg::DrawElementsUByte,GLubyte>( geom, done, maxElementsPerEBO );
            else if ( data._verts->size() < 65536 )
                populateTriangles<osg::DrawElementsUShort,GLushort>( geom, done, maxElementsPerEBO );
            else
                populateTriangles<osg::DrawElementsUInt,GLuint>( geom, done, maxElementsPerEBO );
        }
    }

    void subdivide(
        double               granularity,
        GeoInterpolation     interp,
        osg::Geometry&       geom,
        const osg::Matrixd&  W2L, // world=>local xform
        const osg::Matrixd&  L2W, // local=>world xform
        unsigned int         maxElementsPerEBO )
    {
        GLenum mode = geom.getPrimitiveSet(0)->getMode();

        if ( mode == GL_POINTS )
            return;

        if ( mode == GL_LINES || mode == GL_LINE_STRIP || mode == GL_LINE_LOOP )
        {
            subdivideLines( granularity, interp, geom, W2L, L2W, maxElementsPerEBO );
        }
        else
        {
            subdivideTriangles( granularity, interp, geom, W2L, L2W, maxElementsPerEBO );

            //osgUtil::VertexCacheVisitor cacheOptimizer;
            //cacheOptimizer.optimizeVertices( geom );
        }
        
        //osgUtil::VertexAccessOrderVisitor orderOptimizer;
        //orderOptimizer.optimizeOrder( geom );
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
MeshSubdivider::run(osg::Geometry& geom, double granularity, GeoInterpolation interp)
{
    if ( geom.getNumPrimitiveSets() < 1 )
        return;

    // unsupported for now. NYI.
    if ( geom.getVertexAttribArrayList().size() > 0 )
        return;

    subdivide( granularity, interp, geom, _world2local, _local2world, _maxElementsPerEBO );
}
