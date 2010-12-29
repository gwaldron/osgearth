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
#include <queue>
#include <map>

using namespace osgEarth;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

#define ITYPE unsigned int

namespace
{
    struct Triangle {
        Triangle() { }
        Triangle(ITYPE i0, ITYPE i1, ITYPE i2) : _i0(i0), _i1(i1), _i2(i2) { }
        ITYPE _i0, _i1, _i2;
    };

    struct Edge {
        Edge() { }
        Edge( ITYPE i0, ITYPE i1 ) : _i0(i0), _i1(i1) { }
        ITYPE _i0, _i1;
        bool operator < (const Edge& rhs) const { return _i0+_i1 < rhs._i0+rhs._i1; }
    };

    typedef std::queue<Triangle> TriangleQueue;
    typedef std::vector<Triangle> TriangleVector;

    typedef osg::VectorGLuint UIntVector;

    typedef std::map<Edge,ITYPE> EdgeMap;

    double s_angleBetween( const osg::Vec3& v0, const osg::Vec3& v1 )
    {
        osg::Vec3 v0n = v0; v0n.normalize();
        osg::Vec3 v1n = v1; v1n.normalize();
        return acos( v0n * v1n );
    }
}

//------------------------------------------------------------------------

MeshSubdivider::MeshSubdivider(const osg::Matrixd& refMatrix,
                               const osg::Matrixd& invRefMatrix ) :
_refMatrix(refMatrix),
_invRefMatrix(invRefMatrix)                               
{
    if ( !_refMatrix.isIdentity() && _invRefMatrix.isIdentity() )
        _invRefMatrix = osg::Matrixd::inverse(_refMatrix);
    else if ( _refMatrix.isIdentity() && !_invRefMatrix.isIdentity() )
        _refMatrix = osg::Matrixd::inverse(_invRefMatrix);

    _useRefs = !_refMatrix.isIdentity();
}

void
MeshSubdivider::run(double granularity, osg::Geometry* geom)
{
    osg::ref_ptr<osg::Vec3Array> verts = static_cast<osg::Vec3Array*>( geom->getVertexArray() );

    for( int i=0; i<geom->getNumPrimitiveSets(); ++i )
    {
        osg::ref_ptr<osg::DrawElementsUInt> elements = dynamic_cast<osg::DrawElementsUInt*>( geom->getPrimitiveSet(i) );
        if ( elements )
        {
            run( granularity, verts, elements );
            geom->setVertexArray( verts.get() );
            geom->setPrimitiveSet( i, elements.get() );
        }
    }
}

void
MeshSubdivider::run(double granularity,
                    osg::ref_ptr<osg::Vec3Array> in_out_verts,
                    osg::ref_ptr<osg::DrawElementsUInt> in_out_indicies)
{
    // Use two queues:  one for triangles that need (or might need) to be 
    // subdivided and other for triangles that are fully subdivided.
    TriangleQueue triangles; //(); // in_out_indicies->size() / 3 );
    TriangleVector done( 2.0 * (in_out_indicies->size()/3) );

    for( unsigned int i=0; i<in_out_indicies->size(); ++i )
    {
        triangles.push( Triangle( (*in_out_indicies)[i], (*in_out_indicies)[i+1], (*in_out_indicies)[i+2] ) );
    }

    // New positions due to edge splits are appended to the positions list.
    osg::Vec3Array* subdividedPositions = new osg::Vec3Array();
    subdividedPositions->reserve( in_out_verts->size() );
    for( int i=0; i<in_out_verts->size(); ++i )
    {
        if ( _useRefs )
            subdividedPositions->push_back( (*in_out_verts.get())[i] * _invRefMatrix );
        else
            subdividedPositions->push_back( (*in_out_verts.get())[i] );
    }

    // Used to make sure shared edges are not split more than once.
    EdgeMap edges;

    // Subdivide triangles until we run out
    while( triangles.size() != 0 )
    {
        Triangle triangle = triangles.front();
        triangles.pop();

        osg::Vec3 v0 = (*subdividedPositions)[triangle._i0];
        osg::Vec3 v1 = (*subdividedPositions)[triangle._i1];
        osg::Vec3 v2 = (*subdividedPositions)[triangle._i2];

        double g0 = s_angleBetween( v0, v1 );
        double g1 = s_angleBetween( v1, v2 );
        double g2 = s_angleBetween( v2, v0 );
        
        double max = osg::maximum( g0, osg::maximum( g1, g2 ) );

        if ( max > granularity )
        {
            if ( g0 == max )
            {
                Edge edge( osg::minimum(triangle._i0, triangle._i1), osg::maximum(triangle._i0, triangle._i1) );
                
                EdgeMap::iterator ei = edges.find(edge);
                ITYPE i;
                if ( ei == edges.end() )
                {
                    subdividedPositions->push_back( (v0+v1)*0.5 );
                    i = subdividedPositions->size() - 1;
                    edges[edge] = i;
                }
                else
                {
                    i = ei->second;
                }

                triangles.push( Triangle(triangle._i0, i, triangle._i2) );
                triangles.push( Triangle(i, triangle._i1, triangle._i2) );
            }
            else if ( g1 == max )
            {
                Edge edge( osg::minimum(triangle._i1, triangle._i2), osg::maximum(triangle._i1,triangle._i2) );

                EdgeMap::iterator ei = edges.find(edge);
                ITYPE i;
                if ( ei == edges.end() )
                {
                    subdividedPositions->push_back( (v1+v2)*0.5 );
                    i = subdividedPositions->size() - 1;
                    edges[edge] = i;
                }
                else
                {
                    i = ei->second;
                }

                triangles.push( Triangle(triangle._i1, i, triangle._i0) );
                triangles.push( Triangle(i, triangle._i2, triangle._i0) );
            }
            else if ( g2 == max )
            {
                Edge edge( osg::minimum(triangle._i2, triangle._i0), osg::maximum(triangle._i2, triangle._i0) );

                EdgeMap::iterator ei = edges.find(edge);
                ITYPE i;
                if ( ei == edges.end() )
                {
                    subdividedPositions->push_back( (v2+v0)*0.5 );
                    i = subdividedPositions->size() - 1;
                    edges[edge] = i;
                }

                triangles.push( Triangle(triangle._i2, i, triangle._i1) );
                triangles.push( Triangle(i, triangle._i0, triangle._i1) );
            }
            else
            {
                done.push_back(triangle);
            }
        }

        // New indices
        osg::DrawElementsUInt* newIndicies = new osg::DrawElementsUInt( done.size() * 3 );

        for( TriangleVector::iterator i = done.begin(); i != done.end(); ++i )
        {
            newIndicies->push_back( i->_i0 );
            newIndicies->push_back( i->_i1 );
            newIndicies->push_back( i->_i2 );
        }

        if ( _useRefs )
        {
            in_out_verts->clear();
            in_out_verts->reserve( subdividedPositions->size() );
            for( int i=0; i<subdividedPositions->size(); ++i )
                in_out_verts->push_back( (*subdividedPositions)[i] * _refMatrix );
        }
        else
        {
            in_out_verts = subdividedPositions;
        }

        in_out_indicies = newIndicies;
    }
}
