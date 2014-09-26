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

#define LC "[LogarithmicDepthBuffer] "

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
                osg::Uniform* u = stateset->getOrCreateUniform("oe_ldb_far", osg::Uniform::FLOAT);

                // calculate the far plane based on the camera location:
                osg::Vec3d E, C, U;
                camera->getViewMatrixAsLookAt(E, C, U);                
                double farplane = E.length() + 1e6;
                const double nearplane = 1.0;
                
                // set for culling purposes:
                double L, R, B, T, N, F;
                camera->getProjectionMatrixAsFrustum(L, R, B, T, N, F);                
                camera->setProjectionMatrixAsFrustum(L, R, B, T, N, farplane);

                // communicate to the shader:
                u->set( (float)farplane );

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
        "uniform float oe_ldb_far; \n"
        "varying float logz; \n"
        "void oe_ldb_vert(inout vec4 clip) \n"
        "{ \n"
        "    const float C = 0.0005; \n"
        "    float FC = 1.0/log2(oe_ldb_far*C + 1.0); \n"
        "    logz = log2(clip.w*C + 1.0)*FC; \n"
        "    clip.z = (2.0*logz - 1.0)*clip.w; \n"
        "} \n";

    const char* fragSource =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "varying float logz; \n"
        "void oe_ldb_frag(inout vec4 clip) \n"
        "{\n"
        "    gl_FragDepth = logz; \n"
        "}\n";
}

//------------------------------------------------------------------------

LogarithmicDepthBuffer::LogarithmicDepthBuffer()
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
LogarithmicDepthBuffer::install(osg::Camera* camera)
{
    if ( camera && _supported )
    {
        // install the shader component:
        osg::StateSet* stateset = camera->getOrCreateStateSet();
        
        VirtualProgram* vp = VirtualProgram::getOrCreate( stateset );
        vp->setFunction( "oe_ldb_vert", vertSource, ShaderComp::LOCATION_VERTEX_CLIP, FLT_MAX );
        vp->setFunction( "oe_ldb_frag", fragSource, ShaderComp::LOCATION_FRAGMENT_LIGHTING, FLT_MAX );

        // configure the camera:
        camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

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
            }

            stateset->removeUniform( "oe_ldb_far" );
        }
    }
}
