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
#include <osgEarthUtil/Shaders>
#include <osgEarth/CullingUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ShaderUtils>
#include <osgUtil/CullVisitor>
#include <osg/Uniform>
#include <osg/buffered_value>
#include <cmath>

#define LC "[LogarithmicDepthBuffer] "

//#define DEFAULT_NEAR_PLANE     0.1
//#define NEAR_RES_COEFF      1.0 //0.0005  // a.k.a. "C"
#define LOG2(X) (::log((double)(X))/::log(2.0))

//#define C_UNIFORM  "oe_logDepth_C"
#define FC_UNIFORM "oe_logDepth_FC"

using namespace osgEarth;
using namespace osgEarth::Util;

//------------------------------------------------------------------------

namespace
{
    // Callback to set the "far plane" uniform just before drawing.
    struct SetFarPlaneUniformCallback : public osg::Camera::DrawCallback
    {
        osg::ref_ptr<osg::Uniform>              _uniform;
        osg::ref_ptr<osg::Camera::DrawCallback> _next;

        SetFarPlaneUniformCallback(osg::Uniform*              uniform,
                                   osg::Camera::DrawCallback* next)
        {
            _uniform = uniform;
            _next    = next;
        }

        void operator () (osg::RenderInfo& renderInfo) const
        {
            const osg::Matrix& proj = renderInfo.getCurrentCamera()->getProjectionMatrix();

            osg::GL2Extensions* ext = osg::GL2Extensions::Get(renderInfo.getContextID(), true);

            if ( osg::equivalent(proj(3,3), 0.0) ) // perspective
            {
                float vfov, ar, n, f;
                proj.getPerspective(vfov, ar, n, f);
                float fc = (float)(2.0/LOG2(f+1.0));
                _uniform->set( fc );
                //OE_NOTICE << "PERSP: n = " << n << ", f = " << f << "\n";
            }
            else // ortho
            {
                float l, r, b, t, n, f;
                proj.getOrtho(l, r, b, t, n, f);
                float fc = (float)(2.0/LOG2(f+1.0));
                fc = -1.0f;
                _uniform->set( fc );
                //OE_NOTICE << "ORTHO: n = " << n << ", f = " << f << "\n";
            }

            if ( _next.valid() )
            {
                _next->operator()( renderInfo );
            }
        }
    };
}

//------------------------------------------------------------------------

LogarithmicDepthBuffer::LogarithmicDepthBuffer() :
_useFragDepth(false)
{
    _supported = Registry::capabilities().supportsGLSL();
    if ( _supported )
    {
        //_cullCallback = new LogDepthCullCallback();
        _FCUniform = new osg::Uniform(FC_UNIFORM, (float)0.0f);
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

        //stateset->addUniform( new osg::Uniform(C_UNIFORM, (float)NEAR_RES_COEFF) );
        
        VirtualProgram* vp = VirtualProgram::getOrCreate( stateset );
        Shaders pkg;

        if ( _useFragDepth )
        {
            pkg.loadFunction( vp, pkg.LogDepthBuffer_VertFile );
            pkg.loadFunction( vp, pkg.LogDepthBuffer_FragFile );
        }
        else
        {
            pkg.loadFunction( vp, pkg.LogDepthBuffer_VertOnly_VertFile );
        }

        osg::ref_ptr<osg::Camera::DrawCallback> next = camera->getPreDrawCallback();
        if ( dynamic_cast<SetFarPlaneUniformCallback*>(next.get()) )
            next = static_cast<SetFarPlaneUniformCallback*>(next.get())->_next.get();
        
        stateset->addUniform( _FCUniform.get() );
        camera->setPreDrawCallback( new SetFarPlaneUniformCallback(_FCUniform.get(), next.get()) );
    }
}

void
LogarithmicDepthBuffer::uninstall(osg::Camera* camera)
{
    if ( camera && _supported )
    {
        SetFarPlaneUniformCallback* dc = dynamic_cast<SetFarPlaneUniformCallback*>(camera->getPreDrawCallback());
        if ( dc )
        {
            osg::ref_ptr<osg::Camera::DrawCallback> next = dc->_next.get();
            camera->setPreDrawCallback( next.get() );
        }

        osg::StateSet* stateset = camera->getStateSet();
        if ( stateset )
        {
            VirtualProgram* vp = VirtualProgram::get( camera->getStateSet() );
            if ( vp )
            {
                Shaders pkg;
                pkg.unloadFunction( vp, pkg.LogDepthBuffer_FragFile );
                pkg.unloadFunction( vp, pkg.LogDepthBuffer_VertFile );
                pkg.unloadFunction( vp, pkg.LogDepthBuffer_VertOnly_VertFile );
            }

            stateset->removeUniform( FC_UNIFORM );
        }
    }
}
