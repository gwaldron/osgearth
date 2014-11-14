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
#include <osgEarthUtil/LogarithmicDepthBuffer>
#include <osgEarth/CullingUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgUtil/CullVisitor>
#include <osg/Uniform>
#include <osg/buffered_value>
#include <cmath>

#define LC "[LogarithmicDepthBuffer] "

#define DEFAULT_NEAR_PLANE     0.1
#define NEAR_RES_COEFF      0.0005  // a.k.a. "C"
#define NEAR_RES_COEFF_STR "0.0005"
#define LOG2(X) (::log((double)(X))/::log(2.0))

using namespace osgEarth;
using namespace osgEarth::Util;

//------------------------------------------------------------------------

namespace
{
    struct LogDepthCullCallback : public osg::NodeCallback
    {
        void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
            osg::Camera* camera = cv->getCurrentCamera();
            if ( camera )
            {
                // find (or create) a stateset
                osg::StateSet* stateset = 0L;
                osg::ref_ptr<osg::StateSet> refStateSet;

                osg::GraphicsContext* gc = camera->getGraphicsContext();
                if ( gc )
                {
                    // faster method of re-using statesets when a GC is present
                    unsigned id = gc->getState()->getContextID();
                    refStateSet = _stateSets[id];
                    if ( !refStateSet.valid() )
                        refStateSet = new osg::StateSet();
                    stateset = refStateSet.get();
                }
                else
                {
                    // no GC is present (e.g., RTT camera) so just make a fresh one
                    refStateSet = new osg::StateSet();
                    stateset = refStateSet.get();
                }

                // the uniform conveying the far clip plane:
                //osg::Uniform* u = stateset->getOrCreateUniform("oe_ldb_far", osg::Uniform::FLOAT);
                osg::Uniform* u = stateset->getOrCreateUniform("oe_ldb_FC", osg::Uniform::FLOAT);

                // calculate the far plane based on the camera location:
                osg::Vec3d E, A, U;
                camera->getViewMatrixAsLookAt(E, A, U);                
                double farplane = E.length() + 1e6;
                
                // set for culling purposes:
                double L, R, B, T, N, F;
                camera->getProjectionMatrixAsFrustum(L, R, B, T, N, F);                
                camera->setProjectionMatrixAsFrustum(L, R, B, T, DEFAULT_NEAR_PLANE, farplane);

                // communicate to the shader:
                u->set( (float)(2.0/LOG2(farplane*NEAR_RES_COEFF + 1.0)) );

                // and continue traversal of the camera's subgraph.
                cv->pushStateSet( stateset );
                traverse(node, nv);
                cv->popStateSet();
            }
            else
            {                    
                traverse(node, nv);
            }
        }

        // context-specific stateset collection
        osg::buffered_value<osg::ref_ptr<osg::StateSet> > _stateSets;
    };

    const char* vertSource =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform float oe_ldb_FC; \n"
        "varying float oe_ldb_logz; \n"
        "void oe_ldb_vert(inout vec4 clip) \n"
        "{ \n"
        "    const float C = " NEAR_RES_COEFF_STR ";\n"
        "    oe_ldb_logz = max(1e-6, clip.w*C + 1.0); \n"
        "    clip.z = log2(oe_ldb_logz)*oe_ldb_FC - 1.0; \n"
        "} \n";

    const char* fragSource =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform float oe_ldb_FC; \n"
        "varying float oe_ldb_logz; \n"
        "void oe_ldb_frag(inout vec4 color) \n"
        "{\n"
        "    gl_FragDepth = log2(oe_ldb_logz)*0.5*oe_ldb_FC; \n"
        "}\n";

    // This variant does not require using gl_FragDepth, but it less tolerant
    // of low-res tessellations near the camera.
    const char* vertOnlySource =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform float oe_ldb_FC; \n"
        "void oe_ldb_vert(inout vec4 clip) \n"
        "{ \n"
        "    const float C = " NEAR_RES_COEFF_STR ";\n"
        //"    clip.z = (2.0*log2(C*clip.w+1.0)/log2(C*oe_logdepth_farplane+1.0)-1.0)*clip.w; \n"
        "    clip.z = (log2(C*clip.w+1.0)*oe_ldb_FC - 1.0) * clip.w;\n"
        "} \n";
}

//------------------------------------------------------------------------

LogarithmicDepthBuffer::LogarithmicDepthBuffer() :
_useFragDepth(true)
{
    _supported = Registry::capabilities().supportsGLSL();
    if ( _supported )
    {
        _cullCallback = new LogDepthCullCallback();
    }
    else
    {
        OE_WARN << LC << "Not supported on this platform (no GLSL)" << std::endl;
    }
}

void
LogarithmicDepthBuffer::setUseFragDepth(bool value)
{
    _useFragDepth = value;
}

void
LogarithmicDepthBuffer::install(osg::Camera* camera)
{
    if ( camera && _supported )
    {
        // install the shader component:
        osg::StateSet* stateset = camera->getOrCreateStateSet();
        
        VirtualProgram* vp = VirtualProgram::getOrCreate( stateset );

        if ( _useFragDepth )
        {
            vp->setFunction( "oe_ldb_vert", vertSource, ShaderComp::LOCATION_VERTEX_CLIP, FLT_MAX );        
            vp->setFunction( "oe_ldb_frag", fragSource, ShaderComp::LOCATION_FRAGMENT_LIGHTING, FLT_MAX );        
        }
        else
        {
            vp->setFunction( "oe_ldb_vert", vertOnlySource, ShaderComp::LOCATION_VERTEX_CLIP, FLT_MAX );  
        }

        // configure the camera:
        camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
        double fovy, ar, zn, zf;
        camera->getProjectionMatrixAsPerspective(fovy, ar, zn ,zf);
        camera->setProjectionMatrixAsPerspective(fovy, ar, DEFAULT_NEAR_PLANE, zf);

        // install a cull callback to control the far plane:
        camera->addCullCallback( _cullCallback.get() );
    }
}

void
LogarithmicDepthBuffer::uninstall(osg::Camera* camera)
{
    if ( camera && _supported )
    {
        camera->removeCullCallback( _cullCallback.get() );

        osg::StateSet* stateset = camera->getStateSet();
        if ( stateset )
        {
            VirtualProgram* vp = VirtualProgram::get( camera->getStateSet() );
            if ( vp )
            {
                vp->removeShader( "oe_ldb_vert" );
                vp->removeShader( "oe_ldb_frag" );
            }

            stateset->removeUniform( "oe_ldb_far" );
        }
    }
}
