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
#ifndef OSGEARTH_MODEL_FEATURE_STENCIL_STENCIL_UTILS_H
#define OSGEARTH_MODEL_FEATURE_STENCIL_STENCIL_UTILS_H 1

#include <osgEarthFeatures/Styling>
#include <osg/Node>
#include <osg/Stencil>
#include <osg/StencilTwoSided>
#include <osg/Depth>
#include <osg/Drawable>
#include <osg/CopyOp>
#include <osg/CullFace>
#include <osg/MatrixTransform>
#include <osg/Projection>
#include <osg/GLExtensions>
#include <osg/Notify>

#define ON_AND_PROTECTED  osg::StateAttribute::ON | osg::StateAttribute::PROTECTED
#define OFF_AND_PROTECTED osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED

struct StencilUtils
{
    static
    osg::Node* createGeometryPass( osg::Node* geometry, int& ref_renderBin )
    {
        static bool s_EXT_stencil_wrap     = osg::isGLExtensionSupported(0, "GL_EXT_stencil_wrap");
        static bool s_EXT_stencil_two_side = osg::isGLExtensionSupported(0, "GL_EXT_stencil_two_side");

        // zFail=true if more compute intensive, but lets you get inside the volume.
        bool zFail = true;

        osg::Group* root = new osg::Group();

        osg::notify(osg::INFO) << "[osgEarth] Stencil buffer wrap = " << s_EXT_stencil_wrap << std::endl;

        if ( s_EXT_stencil_two_side )
        {
            osg::notify(osg::INFO) << "[osgEarth] Two-sided stenciling" << std::endl;

            osg::StencilTwoSided::Operation incrOp = s_EXT_stencil_wrap ? osg::StencilTwoSided::INCR_WRAP : osg::StencilTwoSided::INCR;
            osg::StencilTwoSided::Operation decrOp = s_EXT_stencil_wrap ? osg::StencilTwoSided::DECR_WRAP : osg::StencilTwoSided::DECR;
            osg::Group* stencil_group = new osg::Group();
            osg::StateSet* ss = stencil_group->getOrCreateStateSet();
            ss->setRenderBinDetails( ref_renderBin++, "RenderBin" );
            
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
            stencil_group->addChild( geometry );

            root->addChild( stencil_group );
        }
        else
        {
            osg::notify(osg::INFO) << "[osgEarth] One-sided stenciling" << std::endl;
            
            if ( !zFail )  // Z-Pass
            {
                osg::Group* front_group = new osg::Group();
                osg::StateSet* front_ss = front_group->getOrCreateStateSet();
                front_ss->setRenderBinDetails( ref_renderBin++, "RenderBin" );

                // incrementing stencil op for front faces:
                osg::Stencil* front_stencil = new osg::Stencil();
                front_stencil->setFunction( osg::Stencil::ALWAYS, 1, ~0u );
                osg::Stencil::Operation incrOp = s_EXT_stencil_wrap ? osg::Stencil::INCR_WRAP : osg::Stencil::INCR;
                front_stencil->setOperation( osg::Stencil::KEEP, osg::Stencil::KEEP, incrOp );
                front_ss->setAttributeAndModes( front_stencil, ON_AND_PROTECTED );

                front_ss->setAttributeAndModes( new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
                front_ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), ON_AND_PROTECTED);
                front_ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

                front_group->addChild( geometry );
                root->addChild( front_group );

                // decrementing stencil op for back faces:
                osg::Group* back_group = new osg::Group();
                osg::StateSet* back_ss = back_group->getOrCreateStateSet();
                back_ss->setRenderBinDetails( ref_renderBin++, "RenderBin" );

                osg::Stencil* back_stencil = new osg::Stencil();
                back_stencil->setFunction( osg::Stencil::ALWAYS, 1, ~0u );
                osg::Stencil::Operation decrOp = s_EXT_stencil_wrap ? osg::Stencil::DECR_WRAP : osg::Stencil::DECR;
                back_stencil->setOperation( osg::Stencil::KEEP, osg::Stencil::KEEP, decrOp );
                back_ss->setAttributeAndModes( back_stencil, ON_AND_PROTECTED );

                back_ss->setAttributeAndModes( new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
                back_ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::FRONT), ON_AND_PROTECTED);
                back_ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

                back_group->addChild( geometry );
                root->addChild( back_group );       
            }
            else
            {
                // incrementing stencil op for back faces:
                {
                    osg::Group* front_group = new osg::Group();
                    osg::StateSet* front_ss = front_group->getOrCreateStateSet();
                    front_ss->setRenderBinDetails( ref_renderBin++, "RenderBin" );

                    osg::Stencil* front_stencil = new osg::Stencil();
                    front_stencil->setFunction( osg::Stencil::ALWAYS ); //, 1, ~0u );
                    osg::Stencil::Operation incrOp = s_EXT_stencil_wrap ? osg::Stencil::INCR_WRAP : osg::Stencil::INCR;
                    front_stencil->setOperation( osg::Stencil::KEEP, incrOp, osg::Stencil::KEEP );
                    front_ss->setAttributeAndModes( front_stencil, ON_AND_PROTECTED );

                    front_ss->setAttributeAndModes( new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
                    front_ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::FRONT), ON_AND_PROTECTED);
                    front_ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

                    front_group->addChild( geometry );
                    root->addChild( front_group );
                }

                // decrementing stencil buf for front faces
                {
                    osg::Group* back_group = new osg::Group();
                    osg::StateSet* back_ss = back_group->getOrCreateStateSet();
                    back_ss->setRenderBinDetails( ref_renderBin++, "RenderBin" );

                    osg::Stencil* back_stencil = new osg::Stencil();
                    back_stencil->setFunction( osg::Stencil::ALWAYS ); //, 1, ~0u );
                    osg::Stencil::Operation decrOp = s_EXT_stencil_wrap ? osg::Stencil::DECR_WRAP : osg::Stencil::DECR;
                    back_stencil->setOperation( osg::Stencil::KEEP, decrOp, osg::Stencil::KEEP );
                    back_ss->setAttributeAndModes( back_stencil, ON_AND_PROTECTED );

                    back_ss->setAttributeAndModes( new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
                    back_ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), ON_AND_PROTECTED);
                    back_ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

                    back_group->addChild( geometry );
                    root->addChild( back_group );
                }
            }
        }

        return root;
    }

    static
    osg::Node* createMaskPass( const osg::Vec4ub& color, int& ref_renderBin )
    {
        osg::Group* result = new osg::Group();

        // make a full screen quad:
        osg::Geometry* quad = new osg::Geometry();
        osg::Vec3Array* verts = new osg::Vec3Array(4);
        (*verts)[0].set( 0, 1, 0 );
        (*verts)[1].set( 0, 0, 0 );
        (*verts)[2].set( 1, 0, 0 );
        (*verts)[3].set( 1, 1, 0 );
        quad->setVertexArray( verts );
        quad->addPrimitiveSet( new osg::DrawArrays( osg::PrimitiveSet::QUADS, 0, 4 ) );
        osg::Vec4ubArray* colors = new osg::Vec4ubArray(1);
        (*colors)[0] = color;
        quad->setColorArray( colors );
        quad->setColorBinding( osg::Geometry::BIND_OVERALL );
        osg::Geode* quad_geode = new osg::Geode();
        quad_geode->addDrawable( quad );

        osg::StateSet* quad_ss = quad->getOrCreateStateSet();
        quad_ss->setRenderBinDetails( ref_renderBin++, "RenderBin" );
        quad_ss->setMode( GL_CULL_FACE, OFF_AND_PROTECTED );
        quad_ss->setMode( GL_DEPTH_TEST, OFF_AND_PROTECTED );
        quad_ss->setMode( GL_LIGHTING, OFF_AND_PROTECTED );

        osg::Stencil* quad_stencil = new osg::Stencil();
        quad_stencil->setFunction( osg::Stencil::NOTEQUAL, 128, (unsigned int)~0 );
        //quad_stencil->setOperation( osg::Stencil::KEEP, osg::Stencil::KEEP, osg::Stencil::KEEP );
        quad_stencil->setOperation( osg::Stencil::REPLACE, osg::Stencil::REPLACE, osg::Stencil::REPLACE );
        quad_ss->setAttributeAndModes( quad_stencil, ON_AND_PROTECTED );

        osg::MatrixTransform* abs = new osg::MatrixTransform();
        abs->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
        abs->setMatrix( osg::Matrix::identity() );
        abs->addChild( quad_geode );

        osg::Projection* proj = new osg::Projection();
        proj->setMatrix( osg::Matrix::ortho(0, 1, 0, 1, 0, -1) );
        proj->addChild( abs );

        result->addChild( proj );
        result->getOrCreateStateSet()->setMode( GL_BLEND, 1 );
        return result;
    }
};

#endif // OSGEARTH_MODEL_FEATURE_STENCIL_STENCIL_UTILS_H
