/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthFeatures/ExtrudeGeometryFilter>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/PolygonOffset>
#include <osgUtil/Tessellator>

using namespace osgEarth;
using namespace osgEarthFeatures;
using namespace osgEarthFeatures::Styling;

ExtrudeGeometryFilter::ExtrudeGeometryFilter() :
_offset( 0 ),
_distance( 100 )
{
    //NOP
}

ExtrudeGeometryFilter::ExtrudeGeometryFilter( const ExtrudeGeometryFilter& rhs ) :
_offset( rhs._offset ),
_distance( rhs._distance ),
_styleClass( rhs._styleClass ),
_geode( rhs._geode.get() )
{
    //NOP
}

ExtrudeGeometryFilter::ExtrudeGeometryFilter( double offset, double distance ) :
_offset( offset ),
_distance( distance )
{
    //NOP
}

ExtrudeGeometryFilter::ExtrudeGeometryFilter( double distance ) :
_offset( 0 ),
_distance( distance )
{
    //NOP
}

static
bool extrudeWallsUp(const FeatureGeometry&     parts,
                    double                     height,
                    bool                       uniform_height,
                    osg::Geometry*             walls,
                    osg::Geometry*             top_cap,
                    osg::Geometry*             bottom_cap,
                    optional<osg::Vec4ub>&     color,
                    bool                       isGeocentric,
                    bool                       makeNormals,
                    bool                       makeTexCoords,
                    FilterContext&             context )
{
    bool made_geom = true;
    const SpatialReference* srs = context.profile()->getSRS();

    //double tex_width_m = skin? skin->getTextureWidthMeters() : 1.0;
    //double tex_height_m = skin? skin->getTextureHeightMeters() : 1.0;
    double tex_width_m = 1.0;
    double tex_height_m = 1.0;

    //Adjust the texture height so it is a multiple of the extrusion height
    //bool   tex_repeats_y = skin? skin->getRepeatsVertically() : true;
    bool tex_repeats_y = true;


    int point_count = parts.getTotalPointCount();
    int num_verts = 2 * point_count;
    if ( context.profile()->getGeometryType() == FeatureProfile::GEOM_POLYGON )
        num_verts += 2 * parts.size();

    osg::Vec3Array* verts = new osg::Vec3Array( num_verts );
    walls->setVertexArray( verts );

    osg::Vec2Array* texcoords = 0L;
    if ( makeTexCoords )
    {
        texcoords = new osg::Vec2Array( num_verts );
        walls->setTexCoordArray( 0, texcoords );
    }

    osg::Vec4ubArray* colors = 0L;
    if ( color.isSet() )
    {
        colors = new osg::Vec4ubArray( num_verts );
        walls->setColorArray( colors );
        walls->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    }
    
    osg::Vec3Array* normals = 0L;
    if ( makeNormals )
    {
        normals = new osg::Vec3Array( num_verts );
        walls->setNormalArray( normals );
        walls->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
    }

    osg::Vec3Array* top_verts = NULL;
    osg::Vec4ubArray* top_colors = NULL;
    if ( top_cap )
    {
        top_verts = new osg::Vec3Array( point_count );
        top_cap->setVertexArray( top_verts );

        if ( color.isSet() )
        {
            top_colors = new osg::Vec4ubArray( point_count );
            top_cap->setColorArray( top_colors );
            top_cap->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
        }
    }

    osg::Vec3Array* bottom_verts = NULL;
    if ( bottom_cap )
    {
        bottom_verts = new osg::Vec3Array( point_count );
        bottom_cap->setVertexArray( bottom_verts );
    }

    int wall_vert_ptr = 0;
    int top_vert_ptr = 0;
    int bottom_vert_ptr = 0;

    GLenum prim_type =
        context.profile()->getGeometryType() == FeatureProfile::GEOM_POINT ? GL_LINES :
        GL_TRIANGLE_STRIP;

    double target_len = -DBL_MAX;
    osg::Vec3d min_loc(DBL_MAX, DBL_MAX, DBL_MAX);

    // first calculate the minimum Z across all parts:
    for( FeatureGeometry::const_iterator k = parts.begin(); k != parts.end(); k++ )
    {
        const osg::Vec3dArray* part = k->get();

        for( osg::Vec3dArray::const_iterator m = part->begin(); m != part->end(); m++ )
        {
            osg::Vec3d m_world = *m * context.inverseReferenceFrame();
            if ( isGeocentric )
            {
                osg::Vec3d p_vec = m_world;
                osg::Vec3d e_vec = p_vec;
                e_vec.normalize();
                p_vec = p_vec + (e_vec * height);

                if (m_world.length() < min_loc.length())
                {
                    min_loc = m_world;
                }

                double p_ex_len = p_vec.length();
                if ( p_ex_len > target_len )
                {
                    target_len = p_ex_len;
                }
            }
            else
            {
                if ( m_world.z() + height > target_len )
                {
                    target_len = m_world.z() + height;
                }

                if (m_world.z() < min_loc.z())
                {
                    min_loc = m_world;
                }                     
            }
        }
    }

    // now generate the extruded geometry.
    for( FeatureGeometry::const_iterator k = parts.begin(); k != parts.end(); ++k )
    {
        double tex_height_m_adj = tex_height_m;

        const osg::Vec3dArray* part = k->get();
        unsigned int wall_part_ptr = wall_vert_ptr;
        unsigned int top_part_ptr = top_vert_ptr;
        unsigned int bottom_part_ptr = bottom_vert_ptr;
        double part_len = 0.0;

        double max_height = 0;
        if ( isGeocentric )
        {
            max_height = target_len - min_loc.length();
        }
        else
        {
            max_height = target_len - min_loc.z();
        }

        //Adjust the texture height so it is a multiple of the maximum height
        double div = osg::round(max_height / tex_height_m);
        //Prevent divide by zero
        if (div == 0) div = 1;
        tex_height_m_adj = max_height / div;

        for( osg::Vec3dArray::const_iterator m = part->begin(); m != part->end(); ++m )
        {
            osg::Vec3d extrude_vec;

            if ( srs )
            {
                osg::Vec3d m_world = *m * context.inverseReferenceFrame(); //srs->getInverseReferenceFrame();
                if ( isGeocentric )
                {
                    osg::Vec3d p_vec = m_world;
                    
                    if ( uniform_height )
                    {
                        double p_len = p_vec.length();
                        double ratio = target_len/p_len;
                        p_vec *= ratio;
                    }
                    else
                    {
                        osg::Vec3d unit_vec = p_vec; 
                        unit_vec.normalize();
                        p_vec = p_vec + unit_vec*height;
                    }

                    extrude_vec = p_vec * context.referenceFrame(); //srs->getReferenceFrame();
                }
                else
                {
                    if ( uniform_height )
                    {
                        extrude_vec.set( m_world.x(), m_world.y(), target_len );
                    }
                    else
                    {
                        extrude_vec.set( m_world.x(), m_world.y(), m_world.z() + height );
                    }
                    extrude_vec = extrude_vec * context.referenceFrame(); //srs->getReferenceFrame();
                }
            }
            else
            {
                extrude_vec.set( m->x(), m->y(), target_len );
            }

            if ( top_cap )
            {
                if ( color.isSet() )
                    (*top_colors)[top_vert_ptr] = color.get();
                (*top_verts)[top_vert_ptr++] = extrude_vec;
            }
            if ( bottom_cap )
            {
                (*bottom_verts)[bottom_vert_ptr++] = *m;
            }
             
            part_len += wall_vert_ptr > wall_part_ptr?
                (extrude_vec - (*verts)[wall_vert_ptr-2]).length() :
                0.0;

            double h;
            if ( tex_repeats_y ) {
                h = -(extrude_vec - *m).length();
            }
            else {
                h = -tex_height_m_adj;
            }
            int p;

            p = wall_vert_ptr++;
            (*verts)[p] = extrude_vec;
            if ( color.isSet() )
                (*colors)[p] = color.get();
            if ( makeTexCoords )
                (*texcoords)[p].set( part_len/tex_width_m, 0.0f );

            p = wall_vert_ptr++;
            (*verts)[p] = *m;
            if ( color.isSet() )
                (*colors)[p] = color.get();
            if ( makeTexCoords )
                (*texcoords)[p].set( part_len/tex_width_m, h/tex_height_m_adj );
        }

        // close the wall if it's a poly:
        if ( context.profile()->getGeometryType() == FeatureProfile::GEOM_POLYGON && !parts.isClosed(part) )
        {
            part_len += wall_vert_ptr > wall_part_ptr?
                ((*verts)[wall_part_ptr] - (*verts)[wall_vert_ptr-2]).length() :
                0.0;

            int p;

            p = wall_vert_ptr++;
            (*verts)[p] = (*verts)[wall_part_ptr];
            if ( color.isSet() )
                (*colors)[p] = color.get();
            if ( makeTexCoords )
                (*texcoords)[p].set( part_len/tex_width_m, (*texcoords)[wall_part_ptr].y() ); //h/tex_height_m );

            p = wall_vert_ptr++;
            (*verts)[p] = (*verts)[wall_part_ptr+1];
            if ( color.isSet() )
                (*colors)[p] = color.get();
            if ( makeTexCoords )
                (*texcoords)[p].set( part_len/tex_width_m, (*texcoords)[wall_part_ptr+1].y() );
        }

        walls->addPrimitiveSet( new osg::DrawArrays(
            prim_type,
            wall_part_ptr, wall_vert_ptr - wall_part_ptr ) );

        if ( top_cap )
        {
            top_cap->addPrimitiveSet( new osg::DrawArrays(
                osg::PrimitiveSet::LINE_LOOP,
                top_part_ptr, top_vert_ptr - top_part_ptr ) );
        }
        if ( bottom_cap )
        {
            // reverse the bottom verts:
            int len = bottom_vert_ptr - bottom_part_ptr;
            for( int i=bottom_part_ptr; i<len/2; i++ )
                std::swap( (*bottom_verts)[i], (*bottom_verts)[bottom_part_ptr+(len-1)-i] );

            bottom_cap->addPrimitiveSet( new osg::DrawArrays(
                osg::PrimitiveSet::LINE_LOOP,
                bottom_part_ptr, bottom_vert_ptr - bottom_part_ptr ) );
        }
    }

    return made_geom;
}

bool
ExtrudeGeometryFilter::push( Feature* input, FilterContext& context )
{   
    FeatureProfile::GeometryType geomType = context.profile()->getGeometryType();
    GLenum primType =
        geomType == FeatureProfile::GEOM_LINE ? GL_LINE_STRIP :
        geomType == FeatureProfile::GEOM_POINT ? GL_POINTS :
        geomType == FeatureProfile::GEOM_POLYGON ? GL_LINE_LOOP : // for later tessellation
        GL_POINTS;

    osg::Vec4ub color(255,255,255,255);
    
    if ( _styleClass.isSet() )
    {
        if (geomType == FeatureProfile::GEOM_POLYGON)
        {
            color = _styleClass->polygonSymbolizer().fill().color();
            color.a() = (int)(255.0f * _styleClass->polygonSymbolizer().fill().opacity());
        }
        else
        {
            color = _styleClass->lineSymbolizer().stroke().color();
            color.a() = (int)(255.0f * _styleClass->lineSymbolizer().stroke().opacity());
        }
    }

    osg::Geometry* geom = new osg::Geometry();
    osg::Vec4ubArray* colors = new osg::Vec4ubArray(1);
    (*colors)[0] = color;
    geom->setColorArray( colors );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    return true;
}

bool
ExtrudeGeometryFilter::push( FeatureList& input, FilterContext& context )
{
    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); i++ )
        if ( !push( i->get(), context ) )
            ok = false;
    return ok;
}

osg::Node*
ExtrudeGeometryFilter::getOutput( FilterContext& context )
{
    if ( _styleClass.isSet() )
    {
        FeatureProfile::GeometryType geomType = context.profile()->getGeometryType();

        if ( geomType == FeatureProfile::GEOM_POLYGON )
        {
            //NOP
        }
        else if ( geomType == FeatureProfile::GEOM_POINT )
        {
            float size = _styleClass->lineSymbolizer().stroke().width();
            _geode->getOrCreateStateSet()->setAttribute( new osg::Point(size), osg::StateAttribute::ON );
        }
        else // GEOM_LINE
        {
            float width = _styleClass->lineSymbolizer().stroke().width();
            _geode->getOrCreateStateSet()->setAttribute( new osg::LineWidth(width), osg::StateAttribute::ON );
        }
    }

    return _geode.get();
}

osg::Node* 
ExtrudeGeometryFilter::takeOutput( FilterContext& context )
{
    getOutput( context );
    return _geode.release();
}



