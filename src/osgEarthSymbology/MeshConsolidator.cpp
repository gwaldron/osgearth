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
#include <osgEarthSymbology/MeshConsolidator>
#include <osgEarthSymbology/LineFunctor>
#include <osg/TriangleFunctor>
#include <osg/TriangleIndexFunctor>
#include <limits>
#include <map>

#define LC "[MeshConsolidator] "

using namespace osgEarth;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

namespace
{
    template<typename T>
    struct Collector
    {
        osg::Geometry::PrimitiveSetList* _newPrimSets;
        unsigned _maxSize;
        T* _current;

        Collector() : _current(0L) { }

        void operator()( unsigned i0, unsigned i1, unsigned i2 )
        {
            if ( _current == 0L || _current->size() > _maxSize-3 )
            {
                _current = new T (GL_TRIANGLES);
                _newPrimSets->push_back( _current );
            }

            _current->push_back( i0 );
            _current->push_back( i1 );
            _current->push_back( i2 );
        }
    };
}

//------------------------------------------------------------------------

void
MeshConsolidator::run( osg::Geometry& geom )
{
    osg::Array* vertexArray = geom.getVertexArray();
    if ( !vertexArray )
        return;

    //TODO: support POLYGON, QUAD, QUADSTRIP, and LINE types.

    osg::Geometry::PrimitiveSetList& primSets = geom.getPrimitiveSetList();
    osg::Geometry::PrimitiveSetList  triSets, nonTriSets;

    for( osg::Geometry::PrimitiveSetList::iterator i = primSets.begin(); i != primSets.end(); ++i )
    {
        osg::PrimitiveSet* pset = i->get();
        switch( pset->getMode() )
        {
        case osg::PrimitiveSet::TRIANGLES:
        case osg::PrimitiveSet::TRIANGLE_STRIP:
        case osg::PrimitiveSet::TRIANGLE_FAN:
        case osg::PrimitiveSet::QUADS:
        case osg::PrimitiveSet::QUAD_STRIP:
        case osg::PrimitiveSet::POLYGON:
            triSets.push_back( pset );
            break;

        default:
            nonTriSets.push_back( pset );
        }
    }

    unsigned numVerts = vertexArray->getNumElements();
    osg::Geometry::PrimitiveSetList newPrimSets;

    if ( numVerts < 0x100 )
    {
        osg::TriangleIndexFunctor< Collector<osg::DrawElementsUByte> > collector;
        collector._newPrimSets = &newPrimSets;
        collector._maxSize = 0xFF;
        geom.accept( collector );
    }
    else if ( numVerts < 0x10000 )
    {
        osg::TriangleIndexFunctor< Collector<osg::DrawElementsUShort> > collector;
        collector._newPrimSets = &newPrimSets;
        collector._maxSize = 0xFFFF;
        geom.accept( collector );
    }
    else
    {
        osg::TriangleIndexFunctor< Collector<osg::DrawElementsUInt> > collector;
        collector._newPrimSets = &newPrimSets;
        collector._maxSize = 0xFFFFFFFF;
        geom.accept( collector );
    }

    for( osg::Geometry::PrimitiveSetList::iterator i = newPrimSets.begin(); i != newPrimSets.end(); ++i )
        nonTriSets.push_back( i->get() );

    geom.setPrimitiveSetList( nonTriSets );
}

namespace
{
    template<typename FROM, typename TO>
    osg::PrimitiveSet* copy( FROM* src, unsigned offset )
    {
        TO* newDE = new TO( GL_TRIANGLES );
        newDE->reserve( src->size() );
        for( typename FROM::const_iterator i = src->begin(); i != src->end(); ++i )
            newDE->push_back( (*i) + offset );
        return newDE;
    }

    template<typename FROM>
    osg::PrimitiveSet* remake( FROM* src, unsigned numVerts, unsigned offset )
    {
        if ( numVerts < 0x100 )
            return copy<FROM,osg::DrawElementsUByte>( src, offset );
        else if ( numVerts < 0x10000 )
            return copy<FROM,osg::DrawElementsUShort>( src, offset );
        else
            return copy<FROM,osg::DrawElementsUInt>( src, offset );
    }
}

void
MeshConsolidator::run( osg::Geode& geode )
{
    unsigned numVerts = 0;
    unsigned numColors = 0;
    unsigned numNormals = 0;
    unsigned numTexCoordArrays = 0;
    unsigned numVertAttribArrays = 0;

    osg::Geometry::AttributeBinding newColorsBinding;
    osg::Geometry::AttributeBinding newNormalsBinding;

    // first, triangulate all the geometries:
    for( unsigned i=0; i<geode.getNumDrawables(); ++i )
    {
        osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
        if ( geom )
        {
            // optimize it into triangles first:
            run( *geom );

            osg::Array* verts = geom->getVertexArray();
            if ( verts )
                numVerts += verts->getNumElements();

            osg::Array* colors = geom->getColorArray();
            if ( colors )
                numColors += colors->getNumElements();

            osg::Array* normals = geom->getNormalArray();
            if ( normals )
                numNormals += normals->getNumElements();

            numTexCoordArrays += geom->getNumTexCoordArrays();
            numVertAttribArrays += geom->getNumVertexAttribArrays();
        }
    }

    // bail if there are unsupported items in there.
    if (geode.getNumDrawables() < 2 ||
        numTexCoordArrays       > 0 ||
        numVertAttribArrays     > 0 )
    {
        return;
    }


    osg::Vec3Array* newVerts = new osg::Vec3Array();
    newVerts->reserve( numVerts );
    osg::Vec3Array::iterator newVertsIter = newVerts->begin();

    osg::Vec4Array* newColors =0L;
    osg::Vec4Array::iterator newColorsIter;
    if ( numColors > 0 )
    {
        newColors = new osg::Vec4Array();
        newColors->reserve( numColors==numVerts? numColors : 1 );
        newColorsBinding = numColors==numVerts? osg::Geometry::BIND_PER_VERTEX : osg::Geometry::BIND_OVERALL;
        newColorsIter = newColors->begin();
    }

    osg::Vec3Array* newNormals =0L;
    osg::Vec3Array::iterator newNormalsIter;
    if ( numNormals > 0 )
    {
        newNormals = new osg::Vec3Array();
        newNormals->reserve( numNormals==numVerts? numNormals : 1 );
        newNormalsBinding = numNormals==numVerts? osg::Geometry::BIND_PER_VERTEX : osg::Geometry::BIND_OVERALL;
        newNormalsIter = newNormals->begin();
    }

    osg::Geometry::PrimitiveSetList newPrimSets;

    for( unsigned i=0; i<geode.getNumDrawables(); ++i )
    {
        osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
        if ( geom )
        {
            // copy over the verts:
            osg::Vec3Array* geomVerts = dynamic_cast<osg::Vec3Array*>( geom->getVertexArray() );
            if ( geomVerts )
            {
                std::copy( geomVerts->begin(), geomVerts->end(), newVertsIter );
                newVertsIter = newVerts->end();

                if ( newColors )
                {
                    osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>( geom->getColorArray() );
                    if ( colors )
                    {
                        if ( newColorsBinding == osg::Geometry::BIND_PER_VERTEX )
                        {
                            std::copy( colors->begin(), colors->end(), newColorsIter );
                            newColorsIter = newColors->end();
                        }
                        else if ( i == 0 ) // overall
                        {
                            (*newColors)[0] = (*colors)[0];
                        }
                    }
                }

                if ( newNormals )
                {
                    osg::Vec3Array* normals = dynamic_cast<osg::Vec3Array*>( geom->getNormalArray() );
                    if ( normals )
                    {
                        if ( newNormalsBinding == osg::Geometry::BIND_PER_VERTEX )
                        {
                            std::copy( normals->begin(), normals->end(), newNormalsIter );
                            newNormalsIter = newNormals->end();
                        }
                        else if ( i == 0 ) // overall
                        {
                            (*newNormals)[0] = (*normals)[0];
                        }
                    }
                }

                unsigned offset = (unsigned)(newVertsIter - geomVerts->begin());

                for( unsigned j=0; j < geom->getNumPrimitiveSets(); ++j )
                {
                    osg::PrimitiveSet* pset = geom->getPrimitiveSet(j);
                    osg::PrimitiveSet* newpset = 0L;
                    
                    if ( dynamic_cast<osg::DrawElementsUByte*>(pset) )
                        newpset = remake( static_cast<osg::DrawElementsUByte*>(pset), numVerts, offset );
                    else if ( dynamic_cast<osg::DrawElementsUShort*>(pset) )
                        newpset = remake( static_cast<osg::DrawElementsUShort*>(pset), numVerts, offset );
                    else
                        newpset = remake( static_cast<osg::DrawElementsUInt*>(pset), numVerts, offset );

                    newPrimSets.push_back( newpset );
                }
            }
        }
    }

    // assemble the new geometry.
    osg::Geometry* newGeom = new osg::Geometry();

    newGeom->setVertexArray( newVerts );
    
    if ( newColors )
    {
        newGeom->setColorArray( newColors );
        newGeom->setColorBinding( newColorsBinding );
    }

    if ( newNormals )
    {
        newGeom->setNormalArray( newNormals );
        newGeom->setNormalBinding( newNormalsBinding );
    }

    newGeom->setPrimitiveSetList( newPrimSets );

    // replace the geode's drawables
    geode.removeDrawables( 0, geode.getNumDrawables() );
    geode.addDrawable( newGeom );
}
