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
#include <iterator>

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

    template<typename FROM, typename TO>
    osg::PrimitiveSet* copy( FROM* src, unsigned offset )
    {
        TO* newDE = new TO( src->getMode() );
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

    bool canOptimize( osg::Geometry& geom )
    {
        osg::Array* vertexArray = geom.getVertexArray();
        if ( !vertexArray )
            return false;

        // check that everything is bound per-vertex

        if ( geom.getColorArray() != 0L && geom.getColorBinding() != osg::Geometry::BIND_PER_VERTEX )
            return false;

        if ( geom.getNormalArray() != 0L && geom.getNormalBinding() != osg::Geometry::BIND_PER_VERTEX )
            return false;

        if ( geom.getSecondaryColorArray() != 0L && geom.getSecondaryColorBinding() != osg::Geometry::BIND_PER_VERTEX )
            return false;

        if ( geom.getVertexAttribArrayList().size() > 0 )
        {
            unsigned n = geom.getVertexAttribArrayList().size();
            for( unsigned i=0; i<n; ++i ) 
            {
                if ( geom.getVertexAttribBinding( i ) != osg::Geometry::BIND_PER_VERTEX )
                    return false;
            }
        }

        return true;
    }
}

//------------------------------------------------------------------------

void
MeshConsolidator::run( osg::Geometry& geom )
{
    if ( !canOptimize(geom) )
        return;

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

    if ( triSets.size() > 0 )
    {
        osg::Array* vertexArray = geom.getVertexArray();
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
    }

    geom.setPrimitiveSetList( nonTriSets );
}

void
MeshConsolidator::run( osg::Geode& geode )
{
    unsigned numVerts = 0;
    unsigned numColors = 0;
    unsigned numNormals = 0;
    unsigned numTexCoordArrays = 0;
    unsigned numVertAttribArrays = 0;
    std::vector<unsigned> texCoordArrayUnits;

    osg::Geometry::AttributeBinding newColorsBinding;
    osg::Geometry::AttributeBinding newNormalsBinding;

    // first, triangulate all the geometries and count all the components:
    for( unsigned i=0; i<geode.getNumDrawables(); ++i )
    {
        osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
        if ( geom )
        {
            if ( !canOptimize(*geom) )
                continue;

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

            // NOTE!! tex/attrib array counts much already be equal.
            if ( texCoordArrayUnits.size() == 0 )
            {
                for( unsigned u=0; u<32; ++u ) {
                    if ( geom->getTexCoordArray(u) != 0L )
                        texCoordArrayUnits.push_back( u );
                }
            }

            numVertAttribArrays += geom->getNumVertexAttribArrays();
        }
    }

    // bail if there are unsupported items in there.
    if (geode.getNumDrawables() < 2 ||
        //numTexCoordArrays       > 0 ||
        numVertAttribArrays     > 0 )
    {
        return;
    }


    osg::Vec3Array* newVerts = new osg::Vec3Array();
    newVerts->reserve( numVerts );

    osg::Vec4Array* newColors =0L;
    if ( numColors > 0 )
    {
        newColors = new osg::Vec4Array();
        newColors->reserve( numVerts );
        newColorsBinding = osg::Geometry::BIND_PER_VERTEX;
        //newColors->reserve( numColors==numVerts? numColors : 1 );
        //newColorsBinding = numColors==numVerts? osg::Geometry::BIND_PER_VERTEX : osg::Geometry::BIND_OVERALL;
    }

    osg::Vec3Array* newNormals =0L;
    if ( numNormals > 0 )
    {
        newNormals = new osg::Vec3Array();
        newNormals->reserve( numVerts );
        newNormalsBinding = osg::Geometry::BIND_PER_VERTEX;
        //newNormals->reserve( numNormals==numVerts? numNormals : 1 );
        //newNormalsBinding = numNormals==numVerts? osg::Geometry::BIND_PER_VERTEX : osg::Geometry::BIND_OVERALL;
    }

    std::vector<osg::Vec2Array*> newTexCoordsArrays;
    for( unsigned i=0; i<texCoordArrayUnits.size(); ++i )
    {
        osg::Vec2Array* newTexCoords = new osg::Vec2Array();
        newTexCoords->reserve( numVerts );
        newTexCoordsArrays.push_back( newTexCoords );
    }

    unsigned offset = 0;
    osg::Geometry::PrimitiveSetList newPrimSets;

    std::vector<osg::ref_ptr<osg::Geometry> > nonOptimizedGeoms;

    osg::StateSet* unifiedStateSet = 0L;

    for( unsigned i=0; i<geode.getNumDrawables(); ++i )
    {
        osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
        if ( geom )
        {
            if ( !canOptimize(*geom) )
            {
                nonOptimizedGeoms.push_back(geom);
                continue;
            }

            // merge in the stateset:
            if ( unifiedStateSet == 0L )
                unifiedStateSet = geom->getStateSet();
            else if ( geom->getStateSet() )
                unifiedStateSet->merge( *geom->getStateSet() );                

            // copy over the verts:
            osg::Vec3Array* geomVerts = dynamic_cast<osg::Vec3Array*>( geom->getVertexArray() );
            if ( geomVerts )
            {
                std::copy( geomVerts->begin(), geomVerts->end(), std::back_inserter(*newVerts) );

                if ( newColors )
                {
                    osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>( geom->getColorArray() );
                    if ( colors )
                    {
                        if ( newColorsBinding == osg::Geometry::BIND_PER_VERTEX )
                        {
                            std::copy( colors->begin(), colors->end(), std::back_inserter(*newColors) );
                        }
                        else if ( i == 0 ) // overall
                        {
                            newColors->push_back( (*colors)[0] );
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
                            std::copy( normals->begin(), normals->end(), std::back_inserter(*newNormals) );
                        }
                        else if ( i == 0 ) // overall
                        {
                            newNormals->push_back( (*normals)[0] );
                        }
                    }
                }

                if ( newTexCoordsArrays.size() > 0 )
                {
                    for( unsigned a=0; a<texCoordArrayUnits.size(); ++a )
                    {
                        unsigned unit = texCoordArrayUnits[a];
                        osg::Vec2Array* texCoords = dynamic_cast<osg::Vec2Array*>( geom->getTexCoordArray(unit) );
                        if ( texCoords )
                        {
                            osg::Vec2Array* newTexCoords = newTexCoordsArrays[a];
                            std::copy( texCoords->begin(), texCoords->end(), std::back_inserter(*newTexCoords) );
                        }
                    }
                }

                for( unsigned j=0; j < geom->getNumPrimitiveSets(); ++j )
                {
                    osg::PrimitiveSet* pset = geom->getPrimitiveSet(j);
                    osg::PrimitiveSet* newpset = 0L;
                    
                    if ( dynamic_cast<osg::DrawElementsUByte*>(pset) )
                        newpset = remake( static_cast<osg::DrawElementsUByte*>(pset), numVerts, offset );
                    else if ( dynamic_cast<osg::DrawElementsUShort*>(pset) )
                        newpset = remake( static_cast<osg::DrawElementsUShort*>(pset), numVerts, offset );
                    else if ( dynamic_cast<osg::DrawElementsUInt*>(pset) )
                        newpset = remake( static_cast<osg::DrawElementsUInt*>(pset), numVerts, offset );
                    else if ( dynamic_cast<osg::DrawArrays*>(pset) )
                        newpset = new osg::DrawArrays( pset->getMode(), offset, geomVerts->size() );

                    if ( newpset )
                        newPrimSets.push_back( newpset );
                }

                offset += geomVerts->size();
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

    if ( newTexCoordsArrays.size() > 0 )
    {
        for( unsigned a=0; a<texCoordArrayUnits.size(); ++a )
        {
            unsigned unit = texCoordArrayUnits[a];
            newGeom->setTexCoordArray( unit, newTexCoordsArrays[a] );
        }
    }

    newGeom->setPrimitiveSetList( newPrimSets );
    newGeom->setStateSet( unifiedStateSet );

    // replace the geode's drawables
    geode.removeDrawables( 0, geode.getNumDrawables() );
    geode.addDrawable( newGeom );
    for( std::vector<osg::ref_ptr<osg::Geometry> >::iterator i = nonOptimizedGeoms.begin(); i != nonOptimizedGeoms.end(); ++i )
        geode.addDrawable( i->get() );
}
