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

    unsigned numVerts = geom.getVertexArray()->getNumElements();
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
