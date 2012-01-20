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
#include <osgEarthFeatures/DepthAdjustment>
#include <osgEarth/StringUtils>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/LineFunctor>
#include <osgEarth/Registry>

#include <osg/Geode>
#include <osg/Geometry>
#include <sstream>

#define LC "[DepthAdjustment] "

using namespace osgEarth;
using namespace osgEarth::Features;

#define UNIFORM_NAME "osgearth_depthAdjRes"
#define MAX_DEPTH_OFFSET 10000.0

// undef this if you want to adjust in the normal direction (of a geocentric point) instead
#define ADJUST_TOWARDS_EYE 1


//------------------------------------------------------------------------

        
namespace
{
    static char remapFunction[] =
        "float remap( float val, float vmin, float vmax, float r0, float r1 ) \n"
        "{ \n"
        "    float vr = (clamp(val, vmin, vmax)-vmin)/(vmax-vmin); \n"
        "    return r0 + vr * (r1-r0); \n"
        "} \n";

    static char castMat4ToMat3Function[] = 
        "mat3 normalMatrix(in mat4 m) { \n"
        "    return mat3( m[0].xyz, m[1].xyz, m[2].xyz ); \n"
        "} \n";

    static std::string createVertexShader( const std::string& shaderCompName, float fixedMinDepthOffset )
    {
        std::stringstream buf;

        std::string minDepthOffset = fixedMinDepthOffset == FLT_MAX ? 
            std::string(UNIFORM_NAME) :
            Stringify() << "float(" << fixedMinDepthOffset << ")";

        float glslVersion = Registry::instance()->getCapabilities().getGLSLVersion();

        std::string versionString = glslVersion < 1.2f ? "#version 110 \n" : "#version 120 \n";

        std::string castMat4ToMat3 = glslVersion < 1.2f ? "castMat4ToMat3Function" : "mat3";
        

        buf
            << versionString
            << remapFunction;

        if ( glslVersion < 1.2f )
            buf << castMat4ToMat3Function;

        buf
            << "uniform mat4 osg_ViewMatrix; \n"
            << "uniform mat4 osg_ViewMatrixInverse; \n"
            << "varying vec4 adjV; \n"
            << "varying float adjZ; \n";

        // FLT_MAX is a special value that means "use a uniform"
        if ( fixedMinDepthOffset == FLT_MAX )
            buf << "uniform float " << UNIFORM_NAME << "; \n";

        if ( !shaderCompName.empty() )
            buf << "void " << shaderCompName << "() { \n";
        else
            buf << "void main(void) { \n";

        buf <<
            // transform the vertex into eye space:
            "vec4 vertEye  = gl_ModelViewMatrix * gl_Vertex; \n"
            "vec3 vertEye3 = vertEye.xyz/vertEye.w; \n"
            "float range = length(vertEye3); \n"

#ifdef ADJUST_TOWARDS_EYE

            "vec3 adjVecEye3 = normalize(vertEye3); \n"

#else // ADJUST_ALONG_UP_VECTOR

            // calculate the "up" vector, that will be our adjustment vector:
            "vec4 vertWorld = osg_ViewMatrixInverse * vertEye; \n"
            "vec3 adjVecWorld3 = -normalize(vertWorld.xyz/vertWorld.w); \n"
            "vec3 adjVecEye3 = " << castMat4ToMat3 << "(osg_ViewMatrix) * adjVecWorld3; \n"

#endif

            // remap depth offset based on camera distance to vertex. The farther you are away,
            // the more of an offset you need.        
            "float offset = remap( range, 1000.0, 10000000.0, " << minDepthOffset << ", 10000.0); \n"

            // adjust the Z (distance from the eye) by our offset value:
            "vertEye3 -= adjVecEye3 * offset; \n"
            "vertEye.xyz = vertEye3 * vertEye.w; \n"

            // Transform the new adjusted vertex into clip space and pass it to the fragment shader.
            "adjV = gl_ProjectionMatrix * vertEye; \n"
            ;

        if ( shaderCompName.empty() )
        {
            buf << 
                "gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
                //"gl_Position = adjV; \n" // <-- uncomment for debugging
                "gl_FrontColor = gl_Color; \n";
        }

        buf << "} \n";

        std::string result;
        result = buf.str();
        return result;
    }

    static std::string createFragmentShader( const std::string& shaderCompName )
    {
        if ( shaderCompName.empty() )
        {
            // Transform out adjusted Z value (from the vertex shader) from clip space [-1..1]
            // into depth buffer space [0..1] and write it to the z-buffer. Yes, this will
            // deactivate early-Z optimizations; so be it!!
            return 
                "#version 110 \n"
                "varying vec4 adjV; \n"
                "void main(void) { \n"
                "    gl_FragColor = gl_Color; \n"
                "    float z = adjV.z/adjV.w; \n"
                "    gl_FragDepth = 0.5*(1.0+z); \n"
                "} \n";
        }
        else
        {
            return Stringify() <<
                "#version 110 \n"
                "varying float adjZ; \n"
                "void " << shaderCompName << "(inout vec4 color) { \n"
                "    gl_FragDepth = 0.5*(1.0+adjZ); \n"
                "} \n";
        }
    }
}

//------------------------------------------------------------------------

namespace
{
    struct SegmentAnalyzer {
        SegmentAnalyzer() : _maxLen2(0) { }
        void operator()( const osg::Vec3& v0, const osg::Vec3& v1, bool ) {
            double len2 = (v1-v0).length2();
            if ( len2 > _maxLen2 ) _maxLen2 = len2;
        }
        double _maxLen2;
    };

    struct SegmentAnalyzerVisitor : public osg::NodeVisitor {
        SegmentAnalyzerVisitor() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) { }
        void apply( osg::Geode& geode ) {
            for( unsigned i=0; i<geode.getNumDrawables(); ++i ) {
                osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
                if ( geom )
                    geom->accept(_a);
            }
        }
        LineFunctor<SegmentAnalyzer> _a;
    };
}

osg::Uniform*
DepthAdjustment::createUniform( osg::Node* graph )
{
    double minDepthOffset = 0.0;
    if ( graph )
    {
        SegmentAnalyzerVisitor v;
        graph->accept( v );
        double maxLen = sqrt(v._a._maxLen2);
        minDepthOffset = sqrt(maxLen)*19.0;

        OE_DEBUG << LC << std::fixed << std::setprecision(2)
            << "max res = " << maxLen << ", min offset = " << minDepthOffset << std::endl;
    }
    osg::Uniform* u = new osg::Uniform(osg::Uniform::FLOAT, UNIFORM_NAME);
    u->set( float(minDepthOffset) );
    return u;
}

osg::Program*
DepthAdjustment::getOrCreateProgram()
{
    static osg::ref_ptr<osg::Program> s_singleton;
    static Threading::Mutex           s_mutex;

    if ( !s_singleton.valid() )
    {
        Threading::ScopedMutexLock exclusive(s_mutex);
        if ( !s_singleton.valid() )
        {
            s_singleton = new osg::Program();
            s_singleton->addShader( new osg::Shader(osg::Shader::VERTEX, createVertexShader("", FLT_MAX)) );
            s_singleton->addShader( new osg::Shader(osg::Shader::FRAGMENT, createFragmentShader("")) );
        }
    }

    return s_singleton.get();
}

osg::Program*
DepthAdjustment::getOrCreateProgram( float staticDepthOffset )
{
    static std::map<float, osg::ref_ptr<osg::Program> > s_programs;
    static Threading::Mutex                             s_mutex;

    Threading::ScopedMutexLock exclusive(s_mutex);
    std::map<float, osg::ref_ptr<osg::Program> >::iterator i = s_programs.find(staticDepthOffset);
    if ( i != s_programs.end() )
    {
        return i->second.get();
    }
    else
    {
        osg::Program* p = new osg::Program();
        p->addShader( new osg::Shader(osg::Shader::VERTEX, createVertexShader("", staticDepthOffset)));
        p->addShader( new osg::Shader(osg::Shader::FRAGMENT, createFragmentShader("")) );
        s_programs[staticDepthOffset] = p;
        return p;
    }
}


std::string
DepthAdjustment::createVertexFunction( const std::string& funcName )
{
    return createVertexShader( funcName, FLT_MAX );
}

std::string
DepthAdjustment::createFragmentFunction( const std::string& funcName )
{
    return createFragmentShader( funcName );
}
