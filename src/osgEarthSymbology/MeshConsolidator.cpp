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

#include <osgEarthSymbology/MeshConsolidator>
#include <osgEarth/StringUtils>
#include <osg/TriangleFunctor>
#include <osg/TriangleIndexFunctor>
#include <osgDB/WriteFile>
#include <osgUtil/MeshOptimizers>
#include <limits>
#include <map>
#include <iterator>

using namespace osgEarth::Symbology;

#define LC "[MeshConsolidator] "

using namespace osgEarth;

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

    osg::PrimitiveSet* convertDAtoDE( osg::DrawArrays* da, unsigned numVerts, unsigned offset )
    {
        osg::DrawElements* de = 0L;
        if ( numVerts < 0x100 )
            de = new osg::DrawElementsUByte( da->getMode() );
        else if ( numVerts < 0x10000 )
            de = new osg::DrawElementsUShort( da->getMode() );
        else
            de = new osg::DrawElementsUInt( da->getMode() );

        de->reserveElements( da->getCount() );
        for( GLsizei i=0; i<da->getCount(); ++i )
            de->addElement( offset + da->getFirst() + i );

        return de;
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

        // just for now.... TODO: allow thi later
        if ( geom.getVertexAttribArrayList().size() > 0 )
            return false;

        // check that all primitive sets share the same user data
        osg::Geometry::PrimitiveSetList& pslist = geom.getPrimitiveSetList();
        osg::Referenced* lastUserData = 0L;
        for( osg::Geometry::PrimitiveSetList::const_iterator i = pslist.begin(); i != pslist.end(); ++i )
        {
            osg::Referenced* userData = i->get()->getUserData();
            if ( i == pslist.begin() || userData == lastUserData )
            {
                lastUserData = userData;
            }
            else
            {
                OE_WARN << LC << "Differing user data in a primset list!" << std::endl;
                return false;
            }
        }

        return true;
    }
}

//------------------------------------------------------------------------

void
MeshConsolidator::convertToTriangles( osg::Geometry& geom, bool force )
{
    if ( !force && !canOptimize(geom) )
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
        // we are assuming at this point that all the primitive sets in a single geometry
        // share a single user data structure.
        osg::Referenced* sharedUserData = triSets[0]->getUserData();

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
#ifdef OSG_GLES2_AVAILABLE
            // GLES only supports UShort, not UInt
            osg::TriangleIndexFunctor< Collector<osg::DrawElementsUShort> > collector;
            collector._newPrimSets = &newPrimSets;
            collector._maxSize = 0xFFFF;
            geom.accept( collector );
#else
            osg::TriangleIndexFunctor< Collector<osg::DrawElementsUInt> > collector;
            collector._newPrimSets = &newPrimSets;
            collector._maxSize = 0xFFFFFFFF;
            geom.accept( collector );
#endif
        }

        for( osg::Geometry::PrimitiveSetList::iterator i = newPrimSets.begin(); i != newPrimSets.end(); ++i )
        {
            i->get()->setUserData( sharedUserData );
            nonTriSets.push_back( i->get() );
        }
    }

    geom.setPrimitiveSetList( nonTriSets );
}

typedef std::vector<osg::ref_ptr<osg::Drawable> > DrawableList;

namespace
{
    void merge( 
        DrawableList::iterator&       start, 
        DrawableList::iterator&       end,
        unsigned                      numVerts,
        unsigned                      numColors,
        unsigned                      numNormals,
        const std::vector<unsigned>&  texCoordArrayUnits,
        bool                          useVBOs,
        DrawableList&                 results )
    {
        osg::Geometry::AttributeBinding newColorsBinding, newNormalsBinding;

        osg::Vec3Array* newVerts = new osg::Vec3Array();
        newVerts->reserve( numVerts );

        // Determine if we need to use 3D texture coordinates or not.
        bool use3DTextureCoords = false;
        for( DrawableList::iterator i = start; i != end; ++i )
        {
            for( unsigned a=0; a<texCoordArrayUnits.size(); ++a )
            {
                unsigned unit = texCoordArrayUnits[a];

                osg::Vec3Array* texCoords = dynamic_cast<osg::Vec3Array*>(i->get()->asGeometry()->getTexCoordArray( unit ));
                if (texCoords)
                {
                    use3DTextureCoords = true;
                    break;
                }
            }

            if (use3DTextureCoords) break;
        }


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

        std::vector<osg::Array*> newTexCoordsArrays;
        for( unsigned i=0; i<texCoordArrayUnits.size(); ++i )
        {
            osg::Array* newTexCoords;
            if (use3DTextureCoords)
            {
                osg::Vec3Array* texCoords3D = new osg::Vec3Array;
                texCoords3D->reserve( numVerts );
                newTexCoords = texCoords3D;                                    
            }
            else
            {
                osg::Vec2Array* texCoords2D = new osg::Vec2Array;
                texCoords2D->reserve( numVerts );
                newTexCoords = texCoords2D;                                    
            }                        
            newTexCoordsArrays.push_back( newTexCoords );
        }

        unsigned offset = 0;
        osg::Geometry::PrimitiveSetList newPrimSets;

        std::vector<osg::ref_ptr<osg::Geometry> > nonOptimizedGeoms;

        osg::StateSet* unifiedStateSet = 0L;

        for( DrawableList::iterator i = start; i != end; ++i )
        {
            osg::Geometry* geom = i->get()->asGeometry(); //geode.getDrawable(i)->asGeometry();

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
                        else if ( i == start ) //i == 0 ) // overall
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
                        else if ( i == start ) //0 ) // overall
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

                        // We are just using 2D texture coordinates so only check for the 2D case
                        if (!use3DTextureCoords)
                        {
                            osg::Vec2Array* texCoords2D = dynamic_cast<osg::Vec2Array*>( geom->getTexCoordArray(unit) );
                            if (texCoords2D)
                            {
                                osg::Vec2Array* newTexCoords = static_cast< osg::Vec2Array*>( newTexCoordsArrays[a] );
                                std::copy( texCoords2D->begin(), texCoords2D->end(), std::back_inserter(*newTexCoords) );
                            }                                                           
                        }
                        else
                        {
                            // We are using 3D coordinates, so check for both 2D and 3D coordinates.  We will consolidate them all into 3D coordinates.
                            osg::Vec2Array* texCoords2D = dynamic_cast<osg::Vec2Array*>( geom->getTexCoordArray(unit) );
                            osg::Vec3Array* texCoords3D = dynamic_cast<osg::Vec3Array*>( geom->getTexCoordArray(unit) );
                            if (texCoords2D || texCoords3D)
                            {
                                osg::Vec3Array* newTexCoords = static_cast< osg::Vec3Array*>( newTexCoordsArrays[a] );
                                if (texCoords3D)
                                {
                                    std::copy( texCoords3D->begin(), texCoords3D->end(), std::back_inserter(*newTexCoords) );
                                }
                                else
                                {
                                    // Convert 2D to 3D coords
                                    for (osg::Vec2Array::iterator itr = texCoords2D->begin(); itr != texCoords2D->end(); ++itr)
                                    {                                        
                                        newTexCoords->push_back( osg::Vec3(itr->x(), itr->y(), 0) );
                                    }
                                }
                            }
                        }                                                
                    }
                }

                osg::ref_ptr<osg::Referenced> sharedUserData;

                for( unsigned j=0; j < geom->getNumPrimitiveSets(); ++j )
                {
                    osg::PrimitiveSet* pset = geom->getPrimitiveSet(j);
                    osg::PrimitiveSet* newpset = 0L;

                    // all primsets have the same user data (or else we would not have made it this far
                    // since canOptimize would be false)
                    if ( !sharedUserData.valid() )
                        sharedUserData = pset->getUserData();

                    if ( dynamic_cast<osg::DrawElementsUByte*>(pset) )
                        newpset = remake( static_cast<osg::DrawElementsUByte*>(pset), numVerts, offset );
                    else if ( dynamic_cast<osg::DrawElementsUShort*>(pset) )
                        newpset = remake( static_cast<osg::DrawElementsUShort*>(pset), numVerts, offset );
                    else if ( dynamic_cast<osg::DrawElementsUInt*>(pset) )
                        newpset = remake( static_cast<osg::DrawElementsUInt*>(pset), numVerts, offset );
                    else if ( dynamic_cast<osg::DrawArrays*>(pset) )
                        newpset = convertDAtoDE( static_cast<osg::DrawArrays*>(pset), numVerts, offset );

                    if ( newpset )
                    {
                        newpset->setUserData( sharedUserData.get() );

                        newPrimSets.push_back( newpset );
                    }
                }

                offset += geomVerts->size();
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

        newGeom->setUseVertexBufferObjects( useVBOs );
        newGeom->setUseDisplayList( !useVBOs );

        results.push_back( newGeom );

        //GeometryValidator().apply( *newGeom );
    }
}


void
MeshConsolidator::run( osg::Geode& geode )
{
    bool useVBOs = false;
    
    // NOTE: we'd rather use the IndexMeshVisitor instead of our own code here,
    // but the IMV does not preserve the user data attached to the primitive sets.
    // We need that since it holds the feature index information.
    //osgUtil::IndexMeshVisitor mesher;
    //geode.accept(mesher);

    // trivial bailout:
    if ( geode.getNumDrawables() <= 1 )
        return;

    // list of geometries to consolidate and not to consolidate.
    DrawableList consolidate, dontConsolidate;

    // list of texture coordinate array image units in use
    std::vector<unsigned> texCoordArrayUnits;
    texCoordArrayUnits.reserve(32);

    // sort the drawables:
    for( unsigned i=0; i<geode.getNumDrawables(); ++i )
    {
        osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
        if ( geom )
        {
            if ( canOptimize(*geom) )
            {
                // convert all surface primitives to triangles.
                convertToTriangles( *geom );

                // NOTE!! tex/attrib array counts much already be equal.
                if ( texCoordArrayUnits.size() == 0 )
                {
                    for( unsigned u=0; u<32; ++u ) {
                        if ( geom->getTexCoordArray(u) != 0L )
                            texCoordArrayUnits.push_back( u );
                    }

                    if ( geom->getUseVertexBufferObjects() )
                        useVBOs = true;
                }

                consolidate.push_back(geom);
            }
            else
            {
                dontConsolidate.push_back(geom);
            }
        }
    }

    // start consolidating the geometries.
    unsigned targetNumVertsPerGeom = 100000; //TODO: configurable?
    DrawableList results;

    unsigned numVerts = 0, numColors = 0, numNormals = 0;
    DrawableList::iterator start = consolidate.begin();

    for( DrawableList::iterator end = consolidate.begin(); end != consolidate.end(); )
    {
        osg::Geometry* geom = end->get()->asGeometry(); // already type-checked this earlier.
        unsigned geomNumVerts = geom->getVertexArray()->getNumElements();

        ++end;

        numVerts += geomNumVerts;
        if ( geom->getColorArray() )
            numColors += geom->getColorArray()->getNumElements();
        if ( geom->getNormalArray() )
            numNormals += geom->getNormalArray()->getNumElements();

        if ( numVerts > targetNumVertsPerGeom || end == consolidate.end() )
        {
            OE_DEBUG << LC << "Merging " << ((unsigned)(end-start)) << " geoms with " << numVerts << " verts." << std::endl;

            merge( start, end, numVerts, numColors, numNormals, texCoordArrayUnits, useVBOs, results );

            start = end;
            numVerts = 0, numColors = 0, numNormals = 0;
        }
    }

    // re-build the geode:
    geode.removeDrawables( 0, geode.getNumDrawables() );

    for( DrawableList::iterator i = results.begin(); i != results.end(); ++i )
        geode.addDrawable( i->get() );

    for( DrawableList::iterator i = dontConsolidate.begin(); i != dontConsolidate.end(); ++i )
        geode.addDrawable( i->get() );
}
