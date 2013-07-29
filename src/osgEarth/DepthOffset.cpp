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
#include <osgEarth/VirtualProgram>

#include <osg/Geode>
#include <osg/Geometry>

#define LC "[DepthOffset] "

// development testing; set to OE_NULL for production
#define OE_TEST OE_NULL

using namespace osgEarth;

// undef this if you want to adjust in the normal direction (of a geocentric point) instead
#define ADJUST_TOWARDS_EYE 1



//------------------------------------------------------------------------


namespace
{
    struct SegmentAnalyzer
    {
        SegmentAnalyzer() : _maxLen2(0), _segmentsAnalyzed(0) { }

        void operator()( const osg::Vec3& v0, const osg::Vec3& v1, bool ) {
            float len2 = (v1-v0).length2();
            if ( len2 > _maxLen2 ) _maxLen2 = len2;
            _segmentsAnalyzed++;
        }
        float _maxLen2;
        int   _segmentsAnalyzed;
    };

    struct GeometryAnalysisVisitor : public osg::NodeVisitor
    {
        GeometryAnalysisVisitor()
            : osg::NodeVisitor     (osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
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

                if ( d->asGeometry() )
                {
                    d->accept( _segmentAnalyzer );
                }
            }
            traverse((osg::Node&)geode);
        }

        LineFunctor<SegmentAnalyzer> _segmentAnalyzer;
        int                          _maxSegmentsToAnalyze;
    };


    //...............................
    // Shader code:

    const char* s_vertex =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        // uniforms from ClampableNode:
        "uniform vec2 oe_doff_bias; \n"
        "uniform vec2 oe_doff_range; \n"

        // values to pass to fragment shader:
        "varying vec4 oe_doff_vert; \n"
        "varying float oe_doff_vertRange; \n"

        "void oe_doff_vertex(inout vec4 VertexVIEW) \n"
        "{ \n"
        //   calculate range to target:
        "    vec3 vert3 = VertexVIEW.xyz/VertexVIEW.w; \n"
        "    float range = length(vert3); \n"

        //   calculate the depth offset bias for this range:
        "    float ratio = (clamp(range, oe_doff_range[0], oe_doff_range[1])-oe_doff_range[0])/(oe_doff_range[1]-oe_doff_range[0]);\n"
        "    float bias = oe_doff_bias[0] + ratio * (oe_doff_bias[1]-oe_doff_bias[0]);\n"

        //   calculate the "simulated" vertex, pulled toward the camera:
        "    vec3 pullVec = normalize(vert3); \n"
        "    vec3 simVert3 = vert3 - pullVec*bias; \n"
        "    vec4 simVert = vec4( simVert3 * VertexVIEW.w, VertexVIEW.w ); \n"
        "    oe_doff_vert = gl_ProjectionMatrix * simVert; \n"
        "    oe_doff_vertRange = range - bias; \n"
        "} \n";

    const char* s_fragment =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"

        // values to pass to fragment shader:
        "varying vec4 oe_doff_vert; \n"
        "varying float oe_doff_vertRange; \n"

        "void oe_doff_fragment(inout vec4 color) \n"
        "{ \n"
        //   calculate the new depth value for the zbuffer.
        "    float sim_depth = 0.5 * (1.0+(oe_doff_vert.z/oe_doff_vert.w));\n"

        //   if the offset pushed the Z behind the eye, the projection mapping will
        //   result in a z>1. We need to bring these values back down to the 
        //   near clip plan (z=0). We need to check simRange too before doing this
        //   so we don't draw fragments that are legitimently beyond the far clip plane.
        "    if ( sim_depth > 1.0 && oe_doff_vertRange < 0.0 ) { sim_depth = 0.0; } \n"
        "    gl_FragDepth = max(0.0, sim_depth); \n"
        "} \n";
}

//------------------------------------------------------------------------


DepthOffsetOptions::DepthOffsetOptions(const Config& conf) :
_enabled ( true ),
_minBias (      100.0f ),
_maxBias (    10000.0f ),
_minRange(     1000.0f ),
_maxRange( 10000000.0f ),
_auto    ( true )
{
    conf.getIfSet( "enabled",   _enabled );
    conf.getIfSet( "min_bias",  _minBias );
    conf.getIfSet( "max_bias",  _maxBias );
    conf.getIfSet( "min_range", _minRange );
    conf.getIfSet( "max_range", _maxRange );
    conf.getIfSet( "auto",      _auto );
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
    conf.addIfSet( "auto",      _auto );
    return conf;
}


//------------------------------------------------------------------------

DepthOffsetAdapter::DepthOffsetAdapter() :
_dirty( false )
{
    init();
}

DepthOffsetAdapter::DepthOffsetAdapter(osg::Node* graph) :
_dirty( false )
{
    init();
    setGraph( graph );
}

void
DepthOffsetAdapter::init()
{
    _supported = Registry::capabilities().supportsFragDepthWrite();
    if ( _supported )
    {
        _biasUniform  = new osg::Uniform(osg::Uniform::FLOAT_VEC2, "oe_doff_bias");
        _rangeUniform = new osg::Uniform(osg::Uniform::FLOAT_VEC2, "oe_doff_range");
        updateUniforms();
    }
}

void
DepthOffsetAdapter::setGraph(osg::Node* graph)
{
    if ( !_supported ) return;

    bool graphChanging =
        _graph.get() != graph;

    bool uninstall =
        (_graph.valid() && _graph->getStateSet()) &&
        (graphChanging || (_options.enabled() == false));

    bool install =
        (graph && graphChanging ) || 
        (graph && (_options.enabled() == true));

    if ( uninstall )
    {
        OE_TEST << LC << "Removing depth offset shaders" << std::endl;

        // uninstall uniforms and shaders.
        osg::StateSet* s = _graph->getStateSet();
        s->removeUniform( _biasUniform.get() );
        s->removeUniform( _rangeUniform.get() );
        VirtualProgram* vp = VirtualProgram::get( s );
        if ( vp )
        {
            vp->removeShader( "oe_doff_vertex" );
            vp->removeShader( "oe_doff_fragment" );
        }
    }

    if ( install )
    {
        OE_TEST << LC << "Installing depth offset shaders" << std::endl;

        // install uniforms and shaders.
        osg::StateSet* s = graph->getOrCreateStateSet();
        s->addUniform( _biasUniform.get() );
        s->addUniform( _rangeUniform.get() );

        VirtualProgram* vp = VirtualProgram::getOrCreate( s );
        vp->setFunction( "oe_doff_vertex", s_vertex, ShaderComp::LOCATION_VERTEX_VIEW );
        vp->setFunction( "oe_doff_fragment", s_fragment, ShaderComp::LOCATION_FRAGMENT_COLORING );
        s->setAttributeAndModes( vp, osg::StateAttribute::ON );
    }

    if ( graphChanging )
    {
        _graph = graph;
    }

    // always set Dirty when setGraph is called sine it may be called anytime
    // the subgraph changes (as can be detected by a computeBound)
    _dirty = (_options.automatic() == true);
}

void
DepthOffsetAdapter::updateUniforms()
{
    if ( !_supported ) return;

    _biasUniform->set( osg::Vec2f(*_options.minBias(), *_options.maxBias()) );
    _rangeUniform->set( osg::Vec2f(*_options.minRange(), *_options.maxRange()) );

    if ( _options.enabled() == true )
    {
        OE_TEST << LC << "bias=[" << *_options.minBias() << ", " << *_options.maxBias() << "] ... "
                << "range=[" << *_options.minRange() << ", " << *_options.maxRange() << "]" << std::endl;
    }
}

void 
DepthOffsetAdapter::setDepthOffsetOptions(const DepthOffsetOptions& options)
{
    if ( !_supported ) return;

    // if "enabled" changed, reset the graph.
    bool reinitGraph = ( options.enabled() != _options.enabled() );

    _options = options;

    if ( reinitGraph )
    {
        setGraph( _graph.get() );
    }

    updateUniforms();

    _dirty = (options.automatic() == true);
}

void
DepthOffsetAdapter::recalculate()
{
    if ( _supported && _graph.valid() )
    {
        if ( _options.automatic() == true )
        {
            GeometryAnalysisVisitor v;
            _graph->accept( v );
            float maxLen = std::max(1.0f, sqrtf(v._segmentAnalyzer._maxLen2));
            _options.minRange() = sqrtf(maxLen) * 19.0f;
            _dirty = false;
            OE_TEST << LC << "Recalcluated." << std::endl;
        }
        updateUniforms();
    }
}

//------------------------------------------------------------------------

DepthOffsetGroup::DepthOffsetGroup() :
_updatePending( false )
{
    _adapter.setGraph( this );

    if ( _adapter.isDirty() )
        _adapter.recalculate();
}

void
DepthOffsetGroup::setDepthOffsetOptions(const DepthOffsetOptions& options)
{
    _adapter.setDepthOffsetOptions(options);
    if ( _adapter.isDirty() && !_updatePending )
        scheduleUpdate();
}

const DepthOffsetOptions&
DepthOffsetGroup::getDepthOffsetOptions() const
{
    return _adapter.getDepthOffsetOptions();
}

void
DepthOffsetGroup::scheduleUpdate()
{
    ADJUST_UPDATE_TRAV_COUNT(this, 1);
    _updatePending = true;
}

osg::BoundingSphere
DepthOffsetGroup::computeBound() const
{
    static Threading::Mutex s_mutex;
    {
        Threading::ScopedMutexLock lock(s_mutex);
        const_cast<DepthOffsetGroup*>(this)->scheduleUpdate();
    }
    return osg::Group::computeBound();
}

void
DepthOffsetGroup::traverse(osg::NodeVisitor& nv)
{
    if ( _updatePending && nv.getVisitorType() == nv.UPDATE_VISITOR )
    {
        _adapter.recalculate();
        ADJUST_UPDATE_TRAV_COUNT( this, -1 );
        _updatePending = false;
    }
    osg::Group::traverse( nv );
}
