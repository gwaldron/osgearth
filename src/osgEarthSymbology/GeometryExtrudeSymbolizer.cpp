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

#include <osgEarthSymbology/GeometryExtrudeSymbolizer>
#include <osgEarthSymbology/ExtrudedSymbol>
#include <osgUtil/Tessellator>
#include <osg/Geometry>
#include <osg/Point>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Geode>

using namespace osgEarth::Symbology;

GeometryExtrudeSymbolizer::GeometryExtrudeSymbolizer()
{
}


void GeometryExtrudeSymbolizer::tessellate( osg::Geometry* geom )
{
    osgUtil::Tessellator tess;
    tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
    tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD );
//    tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
    tess.retessellatePolygons( *geom );
}

bool 
GeometryExtrudeSymbolizer::compile(State<GeometryContent>* state,
                                   osg::Group* attachPoint)
{
    if ( !state || !attachPoint || !state->getContent() || !state->getStyle() )
        return false;

    osg::ref_ptr<osg::Group> newSymbolized = new osg::Group;

    const GeometryList& geometryList = state->getContent()->getGeometryList();
    for (GeometryList::const_iterator it = geometryList.begin(); it != geometryList.end(); ++it)
    {
        Geometry* geometry = *it;
        if (!geometry)
            continue;

        GeometryIterator geomIterator( geometry );
        geomIterator.traverseMultiGeometry() = true;
        geomIterator.traversePolygonHoles() = true;
        while( geomIterator.hasMore() )
        {
            Geometry* part = geomIterator.next();
            if (!part)
                continue;

            osg::Vec4 color = osg::Vec4(1.0, 0.0, 1.0, 1.);
            float height = 1.0;
            float offset = 1.0;

            switch( part->getType())
            {
            case Geometry::TYPE_LINESTRING:
            case Geometry::TYPE_RING:
            {
                const ExtrudedLineSymbol* line = state->getStyle()->getSymbol<ExtrudedLineSymbol>();
                if (line) 
                {
                    color = line->stroke()->color();
                    height = line->extrude()->height();
                    offset = line->extrude()->offset();
                }
            }
            break;

            case Geometry::TYPE_POLYGON:
            {
                const ExtrudedPolygonSymbol* polygon = state->getStyle()->getSymbol<ExtrudedPolygonSymbol>();
                if (polygon)
                {
                    color  = polygon->fill()->color();
                    height = polygon->extrude()->height();
                    offset = polygon->extrude()->offset();
                }
            }
            break;
            default:
                continue;
                break;
            }

            osg::Geode* geode = extrude(part, offset, height, state->getContext() );
            if (geode && geode->getNumDrawables()) 
            {
                osg::Material* material = new osg::Material;
                material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
                geode->getOrCreateStateSet()->setAttributeAndModes(material);
                newSymbolized->addChild(geode);
            }
        }
    }

    if (newSymbolized->getNumChildren())
    {
        attachPoint->removeChildren(0, attachPoint->getNumChildren());
        attachPoint->addChild(newSymbolized.get());
        return true;
    }

    return false;
}

osg::Geode* GeometryExtrudeSymbolizer::extrude(Geometry* geom, double offset, double height, SymbolizerContext* context )
{
    if ( !geom ) return 0L;

    int numRings = 0;

    // start by offsetting the input data.
    {
        GeometryIterator i( geom );
        i.traverseMultiGeometry() = true;
        i.traversePolygonHoles() = true;
        while( i.hasMore() )
        {
            Geometry* part = i.next();
            if (offset != 0.0)
            {
                for( osg::Vec3dArray::iterator j = part->begin(); j != part->end(); j++ )
                {
                    {
                        (*j).z() += offset;
                    }
                }
            }

            // in the meantime, count the # of closed geoms. We will need to know this in 
            // order to pre-allocate the proper # of verts.
            if ( part->getType() == Geometry::TYPE_POLYGON || part->getType() == Geometry::TYPE_RING )
            {
                numRings++;
            }
        }
    }

    // now, go thru and remove any coplanar segments from the geometry. The tesselator will
    // not work include a vert connecting two colinear segments in the tesselation, and this
    // will break the stenciling logic.
#define PARALLEL_EPSILON 0.01
    GeometryIterator i( geom );
    i.traverseMultiGeometry() = true;
    i.traversePolygonHoles() = true;
    while( i.hasMore() )
    {
        Geometry* part = i.next();
        if ( part->size() >= 3 )
        {
            osg::Vec3d prevVec = part->front() - part->back();
            prevVec.normalize();

            for( osg::Vec3dArray::iterator j = part->begin(); part->size() >= 3 && j != part->end(); )
            {
                osg::Vec3d& p0 = *j;
                osg::Vec3d& p1 = j+1 != part->end() ? *(j+1) : part->front();
                osg::Vec3d vec = p1-p0; vec.normalize();

                // if the vectors are essentially parallel, remove the extraneous vertex.
                if ( (prevVec ^ vec).length() < PARALLEL_EPSILON )
                {
                    j = part->erase( j );
                    //OE_NOTICE << "removed colinear segment" << std::endl;
                }
                else
                {
                    ++j;
                    prevVec = vec;
                }
            }
        }
    }


    bool made_geom = true;

    // total up all the points so we can pre-allocate the vertex arrays.
    int num_cap_verts = geom->getTotalPointCount();
    int num_wall_verts = 2 * (num_cap_verts + numRings); // add in numRings b/c we need to close each wall

    osg::Geometry* walls = new osg::Geometry();
    osg::Vec3Array* verts = new osg::Vec3Array( num_wall_verts );
    walls->setVertexArray( verts );

    osg::Geometry* top_cap = new osg::Geometry();
    osg::Vec3Array* top_verts = new osg::Vec3Array( num_cap_verts );
    top_cap->setVertexArray( top_verts );

    osg::Geometry* bottom_cap = new osg::Geometry();
    osg::Vec3Array* bottom_verts = new osg::Vec3Array( num_cap_verts );
    bottom_cap->setVertexArray( bottom_verts );

    int wall_vert_ptr = 0;
    int top_vert_ptr = 0;
    int bottom_vert_ptr = 0;

    //double target_len = height;

    // now generate the extruded geometry.
    GeometryIterator k( geom );
    while( k.hasMore() )
    {
        Geometry* part = k.next();

        unsigned int wall_part_ptr = wall_vert_ptr;
        unsigned int top_part_ptr = top_vert_ptr;
        unsigned int bottom_part_ptr = bottom_vert_ptr;
        double part_len = 0.0;

        GLenum prim_type = part->getType() == Geometry::TYPE_POINTSET ? GL_LINES : GL_TRIANGLE_STRIP;

        for( osg::Vec3dArray::const_iterator m = part->begin(); m != part->end(); ++m )
        {
            osg::Vec3d extrude_vec;
            {
                extrude_vec.set( m->x(), m->y(), height );
            }

            (*top_verts)[top_vert_ptr++] = extrude_vec;
            (*bottom_verts)[bottom_vert_ptr++] = *m;
             
            part_len += wall_vert_ptr > wall_part_ptr?
                (extrude_vec - (*verts)[wall_vert_ptr-2]).length() :
                0.0;

            int p;

            p = wall_vert_ptr++;
            (*verts)[p] = extrude_vec;

            p = wall_vert_ptr++;
            (*verts)[p] = *m;
        }

        // close the wall if it's a ring/poly:
        if ( part->getType() == Geometry::TYPE_RING || part->getType() == Geometry::TYPE_POLYGON )
        {
            part_len += wall_vert_ptr > wall_part_ptr?
                ((*verts)[wall_part_ptr] - (*verts)[wall_vert_ptr-2]).length() :
                0.0;

            int p;

            p = wall_vert_ptr++;
            (*verts)[p] = (*verts)[wall_part_ptr];

            p = wall_vert_ptr++;
            (*verts)[p] = (*verts)[wall_part_ptr+1];
        }

        walls->addPrimitiveSet( new osg::DrawArrays(
                                    prim_type,
                                    wall_part_ptr, wall_vert_ptr - wall_part_ptr ) );

        top_cap->addPrimitiveSet( new osg::DrawArrays(
                                      osg::PrimitiveSet::LINE_LOOP,
                                      top_part_ptr, top_vert_ptr - top_part_ptr ) );

        // reverse the bottom verts so the front face is down:
        std::reverse( bottom_verts->begin()+bottom_part_ptr, bottom_verts->begin()+bottom_vert_ptr );

        bottom_cap->addPrimitiveSet( new osg::DrawArrays(
                                         osg::PrimitiveSet::LINE_LOOP,
                                         bottom_part_ptr, bottom_vert_ptr - bottom_part_ptr ) );
    }

    // build solid surfaces for the caps:
    tessellate( top_cap );
    tessellate( bottom_cap );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( walls );
    geode->addDrawable( top_cap );
    geode->addDrawable( bottom_cap );

    return geode;
}
