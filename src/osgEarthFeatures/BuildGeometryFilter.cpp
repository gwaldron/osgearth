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
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osgEarthSymbology/Text>
#include <osgEarthSymbology/MeshSubdivider>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Depth>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osg/ClusterCullingCallback>
#include <osgText/Text>
#include <osgUtil/Tessellator>
#include <osgUtil/MeshOptimizers>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

namespace
{
    struct CullPlaneCallback : public osg::Drawable::CullCallback
    {
        osg::Vec3d _n;

        CullPlaneCallback( const osg::Vec3d& planeNormal ) : _n(planeNormal) {
            _n.normalize();
        }

        bool cull(osg::NodeVisitor* nv, osg::Drawable* drawable, osg::RenderInfo* renderInfo) const {
            return nv && nv->getEyePoint() * _n <= 0;
        }
    };
}


BuildGeometryFilter::BuildGeometryFilter() :
_style( new Style() ),
_geomTypeOverride( Symbology::Geometry::TYPE_UNKNOWN ),
_maxAngle_deg( 10.0 )
{
    reset();
}

void
BuildGeometryFilter::reset()
{
    _geode = new osg::Geode();
    _hasLines = false;
    _hasPoints = false;
}

bool
BuildGeometryFilter::pushTextAnnotation( TextAnnotation* anno, const FilterContext& context )
{
    // find the centroid
    osg::Vec3d centroid = anno->getGeometry()->getBounds().center();

    osgText::Text* t = new osgText::Text();
    t->setText( anno->text() );
    t->setFont( "fonts/arial.ttf" );
    t->setAutoRotateToScreen( true );
    t->setCharacterSizeMode( osgText::TextBase::SCREEN_COORDS );
    t->setCharacterSize( 32.0f );
    //t->setCharacterSizeMode( osgText::TextBase::OBJECT_COORDS_WITH_MAXIMUM_SCREEN_SIZE_CAPPED_BY_FONT_HEIGHT );
    //t->setCharacterSize( 300000.0f );
    t->setPosition( centroid );
    t->setAlignment( osgText::TextBase::CENTER_CENTER );
    t->getOrCreateStateSet()->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS), osg::StateAttribute::ON );
    t->getOrCreateStateSet()->setRenderBinDetails( 99999, "RenderBin" );

    // apply styling as appropriate:
    osg::Vec4f textColor(1,1,1,1);
    osg::Vec4f haloColor(0,0,0,1);

    const TextSymbol* textSymbolizer = getStyle()->getSymbol<TextSymbol>();
    if ( textSymbolizer )
    {
        textColor = textSymbolizer->fill()->color();
        if ( textSymbolizer->halo().isSet() )
        {
            haloColor = textSymbolizer->halo()->color();
        }
    }

    t->setColor( textColor );
    t->setBackdropColor( haloColor );
    t->setBackdropType( osgText::Text::OUTLINE );

    if ( context.isGeocentric() )
    {
        // install a cluster culler: note that the CCC control point and normal must be
        // in world coordinates
        const osg::EllipsoidModel* ellip = context.profile()->getSRS()->getEllipsoid();
        osg::Vec3d cp = centroid * context.inverseReferenceFrame();
        osg::Vec3d normal = ellip->computeLocalUpVector( cp.x(), cp.y(), cp.z() );
        osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback( cp, normal, 0.0f );
        t->setCullCallback( ccc );
    }

    _geode->addDrawable( t );

    return true;    
}

bool
BuildGeometryFilter::pushRegularFeature( Feature* input, const FilterContext& context )
{
    GeometryIterator parts( input->getGeometry() );
    parts.traversePolygonHoles() = false;
    while( parts.hasMore() )
    {
        Geometry* part = parts.next();
        
        osg::PrimitiveSet::Mode primMode = osg::PrimitiveSet::POINTS;

        Geometry::Type renderType = _geomTypeOverride.isSet() ? _geomTypeOverride.get() : part->getType();

        //OE_NOTICE
        //    << "BuildGeomFilter: part type = "
        //    << Geometry::toString( part->getType() ) << ", renderType = "
        //    << Geometry::toString( renderType ) << std::endl;

        const Style* myStyle = input->style().isSet() ? input->style()->get() : _style.get();

        osg::Vec4f color = osg::Vec4(1,1,1,1);
        bool tessellatePolys = true;

        bool setWidth = input->style().isSet(); // otherwise it will be set globally, we assume
        float width = 1.0f;

        switch( renderType )
        {
        case Geometry::TYPE_POINTSET:
            {
                _hasPoints = true;
                primMode = osg::PrimitiveSet::POINTS;
                const PointSymbol* point = myStyle->getSymbol<PointSymbol>();
                if (point)
                {
                    color = point->fill()->color();
                }
            }
            break;

        case Geometry::TYPE_LINESTRING:
            {
                _hasLines = true;
                primMode = osg::PrimitiveSet::LINE_STRIP;
                const LineSymbol* lineSymbol = myStyle->getSymbol<LineSymbol>();
                if (lineSymbol)
                {
                    color = lineSymbol->stroke()->color();
                    width = lineSymbol->stroke()->width().isSet() ? *lineSymbol->stroke()->width() : 1.0f;
                }
            }
            break;

        case Geometry::TYPE_RING:
            {
                _hasLines = true;
                primMode = osg::PrimitiveSet::LINE_LOOP;
                const LineSymbol* lineSymbol = myStyle->getSymbol<LineSymbol>();
                if (lineSymbol)
                {
                    color = lineSymbol->stroke()->color();
                    width = lineSymbol->stroke()->width().isSet() ? *lineSymbol->stroke()->width() : 1.0f;
                }
            }
            break;

        case Geometry::TYPE_POLYGON:
            {
                primMode = osg::PrimitiveSet::LINE_LOOP; // loop will tessellate into polys
                const PolygonSymbol* poly = myStyle->getSymbol<PolygonSymbol>();
                if (poly)
                {
                    color = poly->fill()->color();
                }
                else
                {
                    // if we have a line symbol and no polygon symbol, draw as an outline.
                    const LineSymbol* line = myStyle->getSymbol<LineSymbol>();
                    if ( line )
                    {
                        color = line->stroke()->color();
                        width = line->stroke()->width().isSet() ? *line->stroke()->width() : 1.0f;
                        tessellatePolys = false;
                    }
                }
            }
            break;
        case Geometry::TYPE_MULTI:
		case Geometry::TYPE_UNKNOWN:
			break;
        }
        
        osg::Geometry* osgGeom = new osg::Geometry();

        osgGeom->setUseVertexBufferObjects( true );
        osgGeom->setUseDisplayList( false );

        if ( setWidth && width != 1.0f )
        {
            osgGeom->getOrCreateStateSet()->setAttributeAndModes(
                new osg::LineWidth( width ), osg::StateAttribute::ON );
        }
        
        if ( renderType == Geometry::TYPE_POLYGON && part->getType() == Geometry::TYPE_POLYGON && static_cast<Polygon*>(part)->getHoles().size() > 0 )
        {
            Polygon* poly = static_cast<Polygon*>(part);
            int totalPoints = poly->getTotalPointCount();
            osg::Vec3Array* allPoints = new osg::Vec3Array( totalPoints );

            std::copy( part->begin(), part->end(), allPoints->begin() );
            osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, 0, part->size() ) );

            int offset = part->size();

            for( RingCollection::const_iterator h = poly->getHoles().begin(); h != poly->getHoles().end(); ++h )
            {
                Geometry* hole = h->get();
                std::copy( hole->begin(), hole->end(), allPoints->begin() + offset );
                osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, offset, hole->size() ) );
                offset += hole->size();
            }
            osgGeom->setVertexArray( allPoints );
        }
        else
        {
            osgGeom->setVertexArray( part->toVec3Array() );
            osgGeom->addPrimitiveSet( new osg::DrawArrays( primMode, 0, part->size() ) );
        }

        // tessellate all polygon geometries. Tessellating each geometry separately
        // with TESS_TYPE_GEOMETRY is much faster than doing the whole bunch together
        // using TESS_TYPE_DRAWABLE.

        if ( renderType == Geometry::TYPE_POLYGON && tessellatePolys )
        {
            osgUtil::Tessellator tess;
            tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
            tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
            tess.retessellatePolygons( *osgGeom );

            // apply the triangle subdivision if necessary:
            if ( context.isGeocentric() )
            {
                double threshold = osg::DegreesToRadians( *_maxAngle_deg );

                MeshSubdivider ms( context.referenceFrame(), context.inverseReferenceFrame() );
                //ms.setMaxElementsPerEBO( INT_MAX );
                ms.run( threshold, *osgGeom );
            }
        }

        // set the color array. We have to do this last, otherwise it screws up any modifications
        // make by the MeshSubdivider. No idea why. gw
        osg::Vec4Array* colors = new osg::Vec4Array(1);
        (*colors)[0] = color;
        osgGeom->setColorArray( colors );
        osgGeom->setColorBinding( osg::Geometry::BIND_OVERALL );

        // add the part to the geode.
        _geode->addDrawable( osgGeom );
    }

    return true;
}

bool
BuildGeometryFilter::push( Feature* input, const FilterContext& context )
{
    if ( !input || !input->getGeometry() )
        return true;
    else if ( dynamic_cast<TextAnnotation*>(input) )
        return pushTextAnnotation( static_cast<TextAnnotation*>(input), context );
    else
        return pushRegularFeature( input, context );
}

FilterContext
BuildGeometryFilter::push( FeatureList& input, osg::ref_ptr<osg::Node>& output, const FilterContext& context )
{
    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); i++ )
        if ( !push( i->get(), context ) )
            ok = false;

    if ( ok )
    {
        if ( _style.valid() && _geode.valid() )
        {
            // could optimize this to only happen is lines or points were created ..
            const LineSymbol* lineSymbol = _style->getSymbol<LineSymbol>();
            float size = 1.0;
            if (lineSymbol)
                size = lineSymbol->stroke()->width().value();
            _geode->getOrCreateStateSet()->setAttribute( new osg::Point(size), osg::StateAttribute::ON );
            _geode->getOrCreateStateSet()->setAttribute( new osg::LineWidth(size), osg::StateAttribute::ON );
        }

        output = _geode.release();

        if ( context.hasReferenceFrame() )
        {
            osg::MatrixTransform* delocalizer = new osg::MatrixTransform(
                context.inverseReferenceFrame() );
            delocalizer->addChild( output.get() );
            output = delocalizer;
        }
    }
    else
    {
        output = 0L;
    }

    FilterContext outCx( context );
    outCx.setReferenceFrame( osg::Matrixd::identity() ); // clear the ref frame.
    return outCx;
}



static
void tessellate( osg::Geometry* geom )
{
    osgUtil::Tessellator tess;
    tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
    tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD );
//    tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
    tess.retessellatePolygons( *geom );
}

osg::Geode*
osgEarth::Features::createVolume(Geometry*            geom,
                                 double               offset,
                                 double               height,
                                 const FilterContext& context )
{
    if ( !geom ) return 0L;

    int numRings = 0;

    // start by offsetting the input data and counting the number of rings
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
                    if ( context.isGeocentric() )
                    {
                        osg::Vec3d world = context.toWorld( *j );
                        // TODO: get the proper up vector; this is spherical.. or does it really matter for
                        // stencil volumes?
                        osg::Vec3d offset_vec = world;
                        offset_vec.normalize();
                        *j = context.toLocal( world + offset_vec * offset ); //(*j) += offset_vec * offset;
                    }
                    else
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


    //bool made_geom = true;
    const SpatialReference* srs = context.profile()->getSRS();

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

            if ( srs )
            {
                osg::Vec3d m_world = context.toWorld( *m ); //*m * context.inverseReferenceFrame();
                if ( context.isGeocentric() )
                {
                    osg::Vec3d p_vec = m_world; // todo: not exactly right; spherical

                    osg::Vec3d unit_vec = p_vec; 
                    unit_vec.normalize();
                    p_vec = p_vec + unit_vec*height;

                    extrude_vec = context.toLocal( p_vec ); //p_vec * context.referenceFrame();
                }
                else
                {
                    extrude_vec.set( m_world.x(), m_world.y(), height );
                    extrude_vec = context.toLocal( extrude_vec ); //extrude_vec * context.referenceFrame();
                }
            }
            else
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
