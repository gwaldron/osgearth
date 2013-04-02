/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/DepthOffset>
#include <osgEarth/StringUtils>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/LineFunctor>
#include <osgEarth/Registry>
#include <osgEarth/NodeUtils>
#include <osgEarth/Capabilities>

#include <osg/Geode>
#include <osg/Geometry>
#include <osgText/Text>
#include <sstream>

#define LC "[DepthOffset] "

using namespace osgEarth;

#define MIN_OFFSET_UNIFORM "osgearth_depthoffset_minoffset"
#define IS_TEXT_UNIFORM    "osgearth_depthoffset_istext"

#define MAX_DEPTH_OFFSET 10000.0

// undef this if you want to adjust in the normal direction (of a geocentric point) instead
#define ADJUST_TOWARDS_EYE 1


//------------------------------------------------------------------------


DepthOffsetOptions::DepthOffsetOptions(const Config& conf) :
_enabled ( true ),
_minBias (      100.0f ),
_maxBias (    10000.0f ),
_minRange(     1000.0f ),
_maxRange( 10000000.0f )
{
    conf.getIfSet( "enabled",   _enabled );
    conf.getIfSet( "min_bias",  _minBias );
    conf.getIfSet( "max_bias",  _maxBias );
    conf.getIfSet( "min_range", _minRange );
    conf.getIfSet( "max_range", _maxRange );
}


Config
DepthOffsetOptions::getConfig() const
{
    Config conf("depth_offset");
    conf.addIfSet( "enabled",   _enabled );
    conf.addIfSet( "min_bias",  _minBias );
    conf.addIfSet( "max_bias",  _maxBias );
    conf.addIfSet( "min_range", _minRange );
    conf.addIfSet( "max_range", _maxRange );
    return conf;
}


//------------------------------------------------------------------------


DepthOffsetOptionsAdapter::DepthOffsetOptionsAdapter(osg::StateSet* stateSet) :
_stateSet( stateSet )
{
    if ( _stateSet.valid() )
    {
        _biasUniform = _stateSet->getOrCreateUniform( "oe_clamp_bias", osg::Uniform::FLOAT_VEC2 );
        _biasUniform->set( osg::Vec2f(*_options.minBias(), *_options.maxBias()) );

        _rangeUniform = _stateSet->getOrCreateUniform( "oe_clamp_range", osg::Uniform::FLOAT_VEC2 );
        _rangeUniform->set( osg::Vec2f(*_options.minRange(), *_options.maxRange()) );
    }
}


void 
DepthOffsetOptionsAdapter::setOptions(const DepthOffsetOptions& options)
{
    _options = options;

    if ( _stateSet.valid() )
    {
        _biasUniform->set( osg::Vec2f(*_options.minBias(), *_options.maxBias()) );
        _rangeUniform->set( osg::Vec2f(*_options.minRange(), *_options.maxRange()) );
    }
}


//------------------------------------------------------------------------


namespace
{
    struct SegmentAnalyzer
    {
        SegmentAnalyzer() : _maxLen2(0), _segmentsAnalyzed(0) { }
        void operator()( const osg::Vec3& v0, const osg::Vec3& v1, bool ) {
            double len2 = (v1-v0).length2();
            if ( len2 > _maxLen2 ) _maxLen2 = len2;
            _segmentsAnalyzed++;
        }
        double _maxLen2;
        int    _segmentsAnalyzed;
    };

    struct GeometryAnalysisVisitor : public osg::NodeVisitor
    {
        GeometryAnalysisVisitor()
            : osg::NodeVisitor     (osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
              _analyzeSegments     (true),
              _applyTextUniforms   (false),
              _maxSegmentsToAnalyze(250) { }

        void apply( osg::Node& node )
        {
            if ( _segmentAnalyzer._segmentsAnalyzed < _maxSegmentsToAnalyze )
            {
                traverse(node);
            }
        }

        void apply( osg::Geode& geode )
        {
            for( unsigned i=0; i<geode.getNumDrawables(); ++i )
            {
                osg::Drawable* d = geode.getDrawable(i);

                if ( _analyzeSegments && d->asGeometry() )
                {
                    d->accept( _segmentAnalyzer );
                }
                else if ( _applyTextUniforms && dynamic_cast<osgText::Text*>(d) )
                {
                    d->getOrCreateStateSet()->addUniform( DepthOffsetUtils::getIsTextUniform() );
                }
            }
            traverse((osg::Node&)geode);
        }

        LineFunctor<SegmentAnalyzer> _segmentAnalyzer;
        int                          _maxSegmentsToAnalyze;
        bool                         _analyzeSegments;
        bool                         _applyTextUniforms;
    };


    //...............................
    // Shader code:


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

    static std::string createVertexShader( const std::string& shaderCompName )
    {

        float glslVersion = Registry::instance()->getCapabilities().getGLSLVersion();

        std::string versionString = glslVersion < 1.2f ? "#version 110 \n" : "#version 120 \n";

        std::string castMat4ToMat3 = glslVersion < 1.2f ? "castMat4ToMat3Function" : "mat3";
        

        std::stringstream buf;
        buf
            << versionString
            << remapFunction;

        if ( glslVersion < 1.2f )
            buf << castMat4ToMat3Function;

        buf
            << "uniform mat4 osg_ViewMatrix; \n"
            << "uniform mat4 osg_ViewMatrixInverse; \n"
            << "uniform float " << MIN_OFFSET_UNIFORM << "; \n"
            << "uniform bool " << IS_TEXT_UNIFORM << "; \n"
            << "varying vec4 adjV; \n"
            << "varying float simRange; \n";
        

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
            "float offset = remap( range, 1000.0, 10000000.0, " << MIN_OFFSET_UNIFORM << ", 10000.0); \n"

            // adjust the Z (distance from the eye) by our offset value:
            "vertEye3 -= adjVecEye3 * offset; \n"
            "vertEye.xyz = vertEye3 * vertEye.w; \n"

            // Transform the new adjusted vertex into clip space and pass it to the fragment shader.
            "adjV = gl_ProjectionMatrix * vertEye; \n"

            // Also pass along the simulated range (eye=>vertex distance). We will need this
            // to detect when the depth offset has pushed the Z value "behind" the camera.
            "simRange = range - offset; \n"
            ;

        if ( shaderCompName.empty() )
        {
            buf << 
                "gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
                //"gl_Position = adjV; \n" // <-- uncomment for debugging
                "gl_FrontColor = gl_Color; \n"
                "gl_TexCoord[0] = gl_MultiTexCoord0; \n";
            
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

                "uniform bool " IS_TEXT_UNIFORM "; \n"
                "uniform sampler2D tex0; \n"
                "varying vec4 adjV; \n"
                "varying float simRange; \n"

                "void main(void) \n"
                "{ \n"
                "    if (" IS_TEXT_UNIFORM ") \n"
                "    { \n"
                "        float alpha = texture2D(tex0,gl_TexCoord[0].st).a; \n"
                "        gl_FragColor = vec4( gl_Color.rgb, gl_Color.a * alpha); \n"
                "    } \n"
                "    else \n"
                "    { \n"
                "        gl_FragColor = gl_Color; \n"
                "    } \n"

                // transform clipspace depth into [0..1] for FragDepth:
                "    float z = 0.5 * (1.0+(adjV.z/adjV.w)); \n"

                // if the offset pushed the Z behind the eye, the projection mapping will
                // result in a z>1. We need to bring these values back down to the 
                // near clip plan (z=0). We need to check simRange too before doing this
                // so we don't draw fragments that are legitimently beyond the far clip plane.
                "    if ( z > 1.0 && simRange < 0.0 ) { z = 0.0; } \n"

                "    gl_FragDepth = max(0.0, z); \n"
                "} \n";
        }
        else
        {
            return Stringify() <<
                "#version 110 \n"
                "varying vec4 adjV; \n"
                "varying float simRange; \n"
                "void " << shaderCompName << "(inout vec4 color) \n"
                "{ \n"
                "    float z = 0.5 * (1.0+(adjV.z/adjV.w)); \n"
                "    if ( z > 1.0 && simRange < 0.0 ) { z = 0.0; } \n"
                "    gl_FragDepth = max(0.0,z); \n"
                "} \n";
        }
    }
}

//---------------------------------------------------------------------------

osg::Uniform*
DepthOffsetUtils::createMinOffsetUniform( osg::Node* graph )
{
    osg::Uniform* u = new osg::Uniform(osg::Uniform::FLOAT, MIN_OFFSET_UNIFORM);
    u->set( graph ? recalculate(graph) : 0.0f );
    return u;
}

float
DepthOffsetUtils::recalculate( const osg::Node* graph )
{
    double minDepthOffset = 0.0;
    if ( graph )
    {
        GeometryAnalysisVisitor v;
        v._analyzeSegments = true;
        const_cast<osg::Node*>(graph)->accept( v );
        double maxLen = std::max(1.0, sqrt(v._segmentAnalyzer._maxLen2));
        minDepthOffset = sqrt(maxLen)*19.0;

        OE_DEBUG << LC << std::fixed << std::setprecision(2)
            << "max res = " << maxLen << ", min offset = " << minDepthOffset << std::endl;
    }
    return float(minDepthOffset);
}

void
DepthOffsetUtils::prepareGraph( osg::Node* graph )
{
    if ( graph )
    {
        GeometryAnalysisVisitor v;
        v._analyzeSegments = false;
        graph->accept( v );
    }
}

namespace
{
    // dubious. refactor static-init stuff away please.
    static osg::ref_ptr<osg::Uniform> s_isTextUniform;
    static Threading::Mutex           s_isTextUniformMutex;
}

osg::Uniform*
DepthOffsetUtils::getIsTextUniform()
{
    if ( !s_isTextUniform.valid() )
    {
        Threading::ScopedMutexLock exclusive(s_isTextUniformMutex);
        if ( !s_isTextUniform.valid() )
        {
            s_isTextUniform = new osg::Uniform(osg::Uniform::BOOL, IS_TEXT_UNIFORM);
            s_isTextUniform->set( true );
        }
    }
    return s_isTextUniform.get();
}

namespace
{    
    // please refactor away the static-init stuff.
    static osg::ref_ptr<osg::Uniform> s_isNotTextUniform;
    static Threading::Mutex           s_isNotTextUniformMutex;
}

osg::Uniform*
DepthOffsetUtils::getIsNotTextUniform()
{
    if ( !s_isNotTextUniform.valid() )
    {
        Threading::ScopedMutexLock exclusive(s_isNotTextUniformMutex);
        if ( !s_isNotTextUniform.valid() )
        {
            s_isNotTextUniform = new osg::Uniform(osg::Uniform::BOOL, IS_TEXT_UNIFORM);
            s_isNotTextUniform->set( false );
        }
    }
    return s_isNotTextUniform.get();
}
 
namespace
{
    // todo: refactor away the static init stuff
    static osg::ref_ptr<osg::Program> s_depthOffsetProgram;
    static Threading::Mutex           s_depthOffsetProgramMutex;
}

osg::Program*
DepthOffsetUtils::getOrCreateProgram()
{
    if ( !s_depthOffsetProgram.valid() )
    {
        Threading::ScopedMutexLock exclusive(s_depthOffsetProgramMutex);
        if ( !s_depthOffsetProgram.valid() )
        {
            s_depthOffsetProgram = new osg::Program();
            s_depthOffsetProgram->setName( "osgEarth::DepthOffset" );
            s_depthOffsetProgram->addShader( new osg::Shader(osg::Shader::VERTEX, createVertexShader("")) );
            s_depthOffsetProgram->addShader( new osg::Shader(osg::Shader::FRAGMENT, createFragmentShader("")) );
        }
    }

    return s_depthOffsetProgram.get();
}

std::string
DepthOffsetUtils::createVertexFunction( const std::string& funcName )
{
    return createVertexShader( funcName );
}

std::string
DepthOffsetUtils::createFragmentFunction( const std::string& funcName )
{
    return createFragmentShader( funcName );
}

//------------------------------------------------------------------------

DepthOffsetGroup::DepthOffsetGroup() :
_auto ( true ),
_dirty( false )
{
    osg::StateSet* s = this->getOrCreateStateSet();

    osg::Program* program = DepthOffsetUtils::getOrCreateProgram();
    s->setAttributeAndModes( program, 1 );

    _minOffsetUniform = DepthOffsetUtils::createMinOffsetUniform();
    s->addUniform( _minOffsetUniform );

    s->addUniform( DepthOffsetUtils::getIsNotTextUniform() );
}

void
DepthOffsetGroup::setMinimumOffset( float value )
{
    _auto = false;
    _minOffsetUniform->set( std::max(0.0f, value) );
}

void
DepthOffsetGroup::setAutoMinimumOffset()
{
    _auto = true;
    dirty();
}

void
DepthOffsetGroup::dirty()
{
    if ( !_dirty )
    {
        _dirty = true;
        ADJUST_UPDATE_TRAV_COUNT(this, 1);
    }
}

osg::BoundingSphere
DepthOffsetGroup::computeBound() const
{
    const_cast<DepthOffsetGroup*>(this)->dirty();
    return osg::Group::computeBound();
}

void
DepthOffsetGroup::traverse(osg::NodeVisitor& nv)
{
    if ( _dirty && nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
        update();
        ADJUST_UPDATE_TRAV_COUNT( this, -1 );
        _dirty = false;
    }
    osg::Group::traverse( nv );
}

void
DepthOffsetGroup::update()
{
    GeometryAnalysisVisitor v;
    v._analyzeSegments = _auto;
    this->accept( v );

    if ( _auto )
    {
        double maxLen = sqrt(v._segmentAnalyzer._maxLen2);
        _minOffsetUniform->set( float(sqrt(maxLen)*19.0) );
    }
}
