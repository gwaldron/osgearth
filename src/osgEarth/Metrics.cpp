/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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


#include <osgEarth/Metrics>
#include <osgViewer/ViewerBase>
#include <osgViewer/View>
#include <osgEarth/MemoryUtils>
#include <stdlib.h>

using namespace osgEarth::Util;

#define LC "[Metrics] "

static bool s_metricsEnabled = true;
static bool s_gpuMetricsEnabled = true;
static bool s_gpuMetricsInstalled = false;

#ifdef OSGEARTH_HAVE_TRACY

#ifdef OSGEARTH_GPU_PROFILING
void (GL_APIENTRY * osgEarth::MetricsGL::_glGenQueries)(GLsizei, GLuint*);
void (GL_APIENTRY * osgEarth::MetricsGL::_glGetInteger64v)(GLenum, GLint64*);
void (GL_APIENTRY * osgEarth::MetricsGL::_glGetQueryiv)(GLenum, GLenum, GLint*);
void (GL_APIENTRY * osgEarth::MetricsGL::_glGetQueryObjectiv)(GLint, GLenum, GLint*);
void (GL_APIENTRY * osgEarth::MetricsGL::_glGetQueryObjectui64v)(GLint, GLenum, GLuint64*);
void (GL_APIENTRY * osgEarth::MetricsGL::_glQueryCounter)(GLuint, GLenum);
#endif

namespace
{
#ifdef OSGEARTH_GPU_PROFILING
    struct TracyGPUCallback : public osg::GraphicsContext::SwapCallback
    {
        TracyGPUCallback() : _tracyInit(false) { }
        bool _tracyInit;

        void swapBuffersImplementation(osg::GraphicsContext* gc)
        {
            // Enable and collect tracy gpu stats.  There is a better place for this somewhere else :)
            if (!_tracyInit)
            {
                osg::setGLExtensionFuncPtr(osgEarth::MetricsGL::_glGenQueries, "glGenQueries");
                osg::setGLExtensionFuncPtr(osgEarth::MetricsGL::_glGetInteger64v, "glGetInteger64v");
                osg::setGLExtensionFuncPtr(osgEarth::MetricsGL::_glGetQueryiv, "glGetQueryiv");
                osg::setGLExtensionFuncPtr(osgEarth::MetricsGL::_glGetQueryObjectiv, "glGetQueryObjectiv");
                osg::setGLExtensionFuncPtr(osgEarth::MetricsGL::_glGetQueryObjectui64v, "glGetQueryObjectui64v");
                osg::setGLExtensionFuncPtr(osgEarth::MetricsGL::_glQueryCounter, "glQueryCounter");

                TracyGpuContext;
                _tracyInit = true;
            }

            // Run the default system swapBufferImplementation
            {
                OE_PROFILING_ZONE_NAMED("SwapBuffers");
                TracyGpuZone("SwapBuffers");
                gc->swapBuffersImplementation();
            }

            TracyGpuCollect;
        }
    };
#endif
}
#endif // OSGEARTH_HAVE_TRACY


bool Metrics::enabled()
{
    return s_metricsEnabled;
}

void Metrics::setEnabled(bool enabled)
{
    s_metricsEnabled = enabled;
}

void Metrics::setGPUProfilingEnabled(bool enabled)
{
    s_gpuMetricsEnabled = enabled;
}

void Metrics::frame()
{
    OE_PROFILING_FRAME_MARK;
}

int Metrics::run(osgViewer::ViewerBase& viewer)
{
    if (!viewer.isRealized())
    {
        viewer.realize();
    }

#ifdef OSGEARTH_HAVE_TRACY

    if (s_gpuMetricsEnabled == false &&
        ::getenv("OE_PROFILE_GPU") != NULL)
    {
        s_gpuMetricsEnabled = true;
    }

    // Report memory and fps every 60 frames.
    unsigned int reportEvery = 60;

    osgViewer::ViewerBase::Views views;
    viewer.getViews(views);

#ifdef OSGEARTH_GPU_PROFILING
    if (s_gpuMetricsEnabled)
    {
        osgViewer::ViewerBase::Contexts contexts;
        viewer.getContexts(contexts);
        for(auto& context : contexts)
        {
            context->setSwapCallback(new TracyGPUCallback());
        }
    }
#endif

    while (!viewer.done())
    {
        {
            OE_PROFILING_ZONE_NAMED("advance");
            viewer.advance();
        }

        {
            OE_PROFILING_ZONE_NAMED("eventTraversal");
            viewer.eventTraversal();
        }

        {
            OE_PROFILING_ZONE_NAMED("updateTraversal");
            viewer.updateTraversal();
        }

        {
            OE_PROFILING_ZONE_NAMED("renderingTraversals");
            viewer.renderingTraversals();
        }

        // Report memory and fps periodically. periodically.
        if (views[0]->getFrameStamp()->getFrameNumber() % reportEvery == 0)
        {         
            OE_PROFILING_PLOT("WorkingSet", (float)(Memory::getProcessPhysicalUsage() / 1048576));
            OE_PROFILING_PLOT("PrivateBytes", (float)(Memory::getProcessPrivateUsage() / 1048576));
            OE_PROFILING_PLOT("PeakPrivateBytes", (float)(Memory::getProcessPeakPrivateUsage() / 1048576));                                                                                                 
        }

        frame();
    }

    return 0;

#else

    return viewer.run();

#endif
}
