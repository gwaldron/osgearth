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
#include <osgEarthFeatures/StencilVolumeNode>
#include <osg/Stencil>
#include <osg/StencilTwoSided>
#include <osg/Depth>
#include <osg/Drawable>
#include <osg/CopyOp>
#include <osg/CullFace>
#include <osg/MatrixTransform>
#include <osg/Projection>
#include <osg/GLExtensions>
#include <osg/Geode>
#include <osg/Notify>
#include <osgUtil/Tessellator>
#include <algorithm>

#define ON_AND_PROTECTED  osg::StateAttribute::ON | osg::StateAttribute::PROTECTED
#define OFF_AND_PROTECTED osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED

using namespace osgEarth;
using namespace osgEarth::Features;


StencilVolumeNode::StencilVolumeNode( bool preRenderChildrenToDepthBuffer, bool inverted ) :
osgEarth::MaskNode(),
_preRenderChildrenToDepthBuffer( preRenderChildrenToDepthBuffer ),
_inverted( inverted ),
_stencilGroup1( 0L ),
_stencilGroup2( 0L ),
_depthPass( 0L ),
_renderPass( 0L )
{
    init();
}

StencilVolumeNode::StencilVolumeNode(const StencilVolumeNode& rhs,
                                     const osg::CopyOp& op) :
osgEarth::MaskNode( rhs, op ),
_root( rhs._root.get() ),
_stencilGroup1( rhs._stencilGroup1 ),
_stencilGroup2( rhs._stencilGroup2 ),
_depthPass( rhs._depthPass ),
_renderPass( rhs._renderPass ),
_inverted( rhs._inverted ),
_preRenderChildrenToDepthBuffer( rhs._preRenderChildrenToDepthBuffer )
{
    //nop
}

int
StencilVolumeNode::setBaseRenderBin( int bin )
{
    if ( _depthPass )
    {
        _depthPass->getOrCreateStateSet()->setRenderBinDetails( bin++, "RenderBin" );
    }
    if ( _stencilGroup1 )
    {
        _stencilGroup1->getOrCreateStateSet()->setRenderBinDetails( bin++, "RenderBin" );
    }
    if ( _stencilGroup2 )
    {
        _stencilGroup2->getOrCreateStateSet()->setRenderBinDetails( bin++, "RenderBin" );
    }
    if ( _renderPass )
    {
        _renderPass->getOrCreateStateSet()->setRenderBinDetails( bin++, "RenderBin" );
    }
    return bin;
}

void
StencilVolumeNode::addVolumes( osg::Node* node )
{
    if ( _stencilGroup1 )
        _stencilGroup1->addChild( node );
    if ( _stencilGroup2 )
        _stencilGroup2->addChild( node );
}

/** Override all the osg::Group methods: */

bool 
StencilVolumeNode::addChild( Node *child ) {
    if ( !child ) return false;
    dirtyBound();
    if ( _depthPass )  _depthPass->addChild( child );
    return _renderPass->addChild( child );
}
bool 
StencilVolumeNode::insertChild( unsigned int index, Node *child ) {
    if ( !child ) return false;
    dirtyBound();
    if ( _depthPass ) _depthPass->insertChild( index, child );
    return _renderPass->insertChild( index, child );
}
bool 
StencilVolumeNode::removeChildren(unsigned int pos,unsigned int numChildrenToRemove) {
    dirtyBound();
    if ( _depthPass ) _depthPass->removeChildren( pos, numChildrenToRemove );
    return _renderPass->removeChildren( pos, numChildrenToRemove );
}
bool 
StencilVolumeNode::replaceChild( Node *origChild, Node* newChild ) {
    dirtyBound();
    if ( _depthPass ) _depthPass->replaceChild( origChild, newChild );
    return _renderPass->replaceChild( origChild, newChild );
}
bool 
StencilVolumeNode::setChild( unsigned  int i, Node* node ) {
    dirtyBound();
    if ( _depthPass ) _depthPass->setChild( i, node );
    return _renderPass->setChild( i, node );
}

osg::BoundingSphere
StencilVolumeNode::computeBound() const {
    if ( _depthPass ) return _depthPass->computeBound();
    else return _renderPass->computeBound();
}


void
StencilVolumeNode::traverse( osg::NodeVisitor& nv )
{
    _root->accept( nv );
}

void
StencilVolumeNode::init()
{
    _root = new osg::Group();
    int baseBin = 100;

    // First, if we need to render the children to the depth buffer, create a depth
    // buffer only pass. This is necessary if we are masking out terrain, for example. We have
    // to populate the depth buffer before this algoritm can work.
    if ( _preRenderChildrenToDepthBuffer )
    {
        _depthPass = new osg::Group();
        _depthPass->setName( "StencilVolumeNode::depth_pass" );
        osg::StateSet* ss = _depthPass->getOrCreateStateSet();
        ss->setAttributeAndModes(new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
        //ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), ON_AND_PROTECTED);
        ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS), ON_AND_PROTECTED);
        ss->setRenderBinDetails( baseBin++, "RenderBin" );

        _root->addChild( _depthPass );
    }

    // Create the stenciled geometry pass:

    // For now we are hard-coding these to what are hopefully "safe" values.
    // In the future we hope to rewrite this as a custom StencilVolumeNode that 
    // will automatically detect extension availability at draw time and choose
    // the optimal GL rendering path.
    static bool s_EXT_stencil_wrap     = true; //osg::isGLExtensionSupported(0, "GL_EXT_stencil_wrap");
    static bool s_EXT_stencil_two_side = false; //osg::isGLExtensionSupported(0, "GL_EXT_stencil_two_side");


    // zFail=true if more compute intensive, but lets you get inside the volume.
    // Again, a custom node will give us a better opportunity to choose between zFail and zPass based on
    // the eye location (you only need zFail if you camera is inside the volume).
    bool zFail = true;

    OE_INFO << "Stencil buffer wrap = " << s_EXT_stencil_wrap << std::endl;

    if ( s_EXT_stencil_two_side )
    {
        OE_INFO << "Two-sided stenciling" << std::endl;

        osg::StencilTwoSided::Operation incrOp = s_EXT_stencil_wrap ? osg::StencilTwoSided::INCR_WRAP : osg::StencilTwoSided::INCR;
        osg::StencilTwoSided::Operation decrOp = s_EXT_stencil_wrap ? osg::StencilTwoSided::DECR_WRAP : osg::StencilTwoSided::DECR;
        _stencilGroup1 = new osg::Group();
        //osg::Group* stencil_group = new osg::Group();
        osg::StateSet* ss = _stencilGroup1->getOrCreateStateSet();
        ss->setRenderBinDetails( baseBin++, "RenderBin" );

        if ( zFail )
        {
            osg::StencilTwoSided* stencil = new osg::StencilTwoSided();
            stencil->setFunction(osg::StencilTwoSided::BACK, osg::StencilTwoSided::ALWAYS, 1, ~0u); 
            stencil->setOperation(osg::StencilTwoSided::BACK, osg::StencilTwoSided::KEEP, incrOp, osg::StencilTwoSided::KEEP);

            stencil->setFunction(osg::StencilTwoSided::FRONT, osg::StencilTwoSided::ALWAYS, 1, ~0u); 
            stencil->setOperation(osg::StencilTwoSided::FRONT, osg::StencilTwoSided::KEEP, decrOp, osg::StencilTwoSided::KEEP);
            ss->setAttributeAndModes( stencil, ON_AND_PROTECTED );
        }
        else
        {
            osg::StencilTwoSided* stencil = new osg::StencilTwoSided();
            stencil->setFunction(osg::StencilTwoSided::FRONT, osg::StencilTwoSided::ALWAYS, 1, ~0u); 
            stencil->setOperation(osg::StencilTwoSided::FRONT, osg::StencilTwoSided::KEEP, osg::StencilTwoSided::KEEP, incrOp);

            stencil->setFunction(osg::StencilTwoSided::BACK, osg::StencilTwoSided::ALWAYS, 1, ~0u); 
            stencil->setOperation(osg::StencilTwoSided::BACK, osg::StencilTwoSided::KEEP, osg::StencilTwoSided::KEEP, decrOp);
            ss->setAttributeAndModes( stencil, ON_AND_PROTECTED );
        }

        ss->setAttributeAndModes(new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
        ss->setAttributeAndModes(new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

        _root->addChild( _stencilGroup1 );
    }
    else
    {
        OE_INFO << "One-sided stenciling" << std::endl;

        if ( !zFail )  // Z-Pass
        {
            _stencilGroup1 = new osg::Group();
            osg::StateSet* front_ss = _stencilGroup1->getOrCreateStateSet();
            front_ss->setRenderBinDetails( baseBin++, "RenderBin" );

            // incrementing stencil op for front faces:
            osg::Stencil* front_stencil = new osg::Stencil();
            front_stencil->setFunction( osg::Stencil::ALWAYS, 1, ~0u );
            osg::Stencil::Operation incrOp = s_EXT_stencil_wrap ? osg::Stencil::INCR_WRAP : osg::Stencil::INCR;
            front_stencil->setOperation( osg::Stencil::KEEP, osg::Stencil::KEEP, incrOp );
            front_ss->setAttributeAndModes( front_stencil, ON_AND_PROTECTED );

            front_ss->setAttributeAndModes( new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
            front_ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), ON_AND_PROTECTED);
            front_ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

            _root->addChild( _stencilGroup1 );

            // decrementing stencil op for back faces:
            _stencilGroup2 = new osg::Group();
            osg::StateSet* back_ss = _stencilGroup2->getOrCreateStateSet();
            back_ss->setRenderBinDetails( baseBin++, "RenderBin" );

            osg::Stencil* back_stencil = new osg::Stencil();
            back_stencil->setFunction( osg::Stencil::ALWAYS, 1, ~0u );
            osg::Stencil::Operation decrOp = s_EXT_stencil_wrap ? osg::Stencil::DECR_WRAP : osg::Stencil::DECR;
            back_stencil->setOperation( osg::Stencil::KEEP, osg::Stencil::KEEP, decrOp );
            back_ss->setAttributeAndModes( back_stencil, ON_AND_PROTECTED );

            back_ss->setAttributeAndModes( new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
            back_ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::FRONT), ON_AND_PROTECTED);
            back_ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

            _root->addChild( _stencilGroup2 );    
        }
        else
        {
            // incrementing stencil op for back faces:
            {
                _stencilGroup1 = new osg::Group();
                osg::StateSet* front_ss = _stencilGroup1->getOrCreateStateSet();
                front_ss->setRenderBinDetails( baseBin++, "RenderBin" );

                osg::Stencil* front_stencil = new osg::Stencil();
                front_stencil->setFunction( osg::Stencil::ALWAYS ); //, 1, ~0u );
                osg::Stencil::Operation incrOp = s_EXT_stencil_wrap ? osg::Stencil::INCR_WRAP : osg::Stencil::INCR;
                front_stencil->setOperation( osg::Stencil::KEEP, incrOp, osg::Stencil::KEEP );
                front_ss->setAttributeAndModes( front_stencil, ON_AND_PROTECTED );

                front_ss->setAttributeAndModes( new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
                front_ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::FRONT), ON_AND_PROTECTED);
                front_ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

                _root->addChild( _stencilGroup1 );
            }

            // decrementing stencil buf for front faces
            {
                _stencilGroup2 = new osg::Group();
                osg::StateSet* back_ss = _stencilGroup2->getOrCreateStateSet();
                back_ss->setRenderBinDetails( baseBin++, "RenderBin" );

                osg::Stencil* back_stencil = new osg::Stencil();
                back_stencil->setFunction( osg::Stencil::ALWAYS ); //, 1, ~0u );
                osg::Stencil::Operation decrOp = s_EXT_stencil_wrap ? osg::Stencil::DECR_WRAP : osg::Stencil::DECR;
                back_stencil->setOperation( osg::Stencil::KEEP, decrOp, osg::Stencil::KEEP );
                back_ss->setAttributeAndModes( back_stencil, ON_AND_PROTECTED );

                back_ss->setAttributeAndModes( new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
                back_ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), ON_AND_PROTECTED);
                back_ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

                _root->addChild( _stencilGroup2 );
            }
        }
    }
    

    // NOW build the full-screen quad mask that will paint the volume:

    _renderPass = new osg::Group();
    _renderPass->setName( "StencilVolumeNode::render_pass" );

    osg::Stencil* quad_stencil = new osg::Stencil();
    osg::Stencil::Function func = _inverted ? osg::Stencil::EQUAL : osg::Stencil::NOTEQUAL;
    quad_stencil->setFunction( func, 128, (unsigned int)~0 );
    // this will conveniently re-clear the stencil buffer in prep for the next pass:
    quad_stencil->setOperation( osg::Stencil::REPLACE, osg::Stencil::REPLACE, osg::Stencil::REPLACE );

    osg::StateSet* renderSS = _renderPass->getOrCreateStateSet();
    renderSS->setAttributeAndModes( quad_stencil, ON_AND_PROTECTED );
    renderSS->setRenderBinDetails( baseBin++, "RenderBin" );

    // if we did a pre-render depth pass, adjust the depth buffer function to account for that
    if ( _preRenderChildrenToDepthBuffer )
    {
        renderSS->setAttributeAndModes( new osg::Depth( osg::Depth::LEQUAL ), osg::StateAttribute::ON );
    }

    // testing
    //renderSS->setMode( GL_DEPTH_TEST, OFF_AND_PROTECTED );
    //renderSS->setAttributeAndModes( new osg::Depth( osg::Depth::ALWAYS ), ON_AND_PROTECTED );

    _root->addChild( _renderPass );
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
StencilVolumeFactory::createVolume(Geometry*            geom,
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


    bool made_geom = true;
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

