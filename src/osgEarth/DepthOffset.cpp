/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarth/LineFunctor>
#include <osgEarth/Registry>
#include <osgEarth/NodeUtils>
#include <osgEarth/Capabilities>
#include <osgEarth/Shaders>

#include <osg/Geometry>
#include <osg/Depth>

#define LC "[DepthOffset] "

// development testing; set to OE_NULL for production
#define OE_TEST OE_NULL

using namespace osgEarth;

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

        void apply(osg::Drawable& drawable)
        {
            if (drawable.asGeometry())
            {
                drawable.asGeometry()->accept(_segmentAnalyzer);
            }
            apply(static_cast<osg::Node&>(drawable));
        }

        LineFunctor<SegmentAnalyzer> _segmentAnalyzer;
        int                          _maxSegmentsToAnalyze;
    };
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
    conf.get( "enabled",   _enabled );
    conf.get( "min_bias",  _minBias );
    conf.get( "max_bias",  _maxBias );
    conf.get( "min_range", _minRange );
    conf.get( "max_range", _maxRange );
    conf.get( "auto",      _auto );
}


Config
DepthOffsetOptions::getConfig() const
{
    Config conf("depth_offset");
    conf.set( "enabled",   _enabled );
    conf.set( "min_bias",  _minBias );
    conf.set( "max_bias",  _maxBias );
    conf.set( "min_range", _minRange );
    conf.set( "max_range", _maxRange );
    conf.set( "auto",      _auto );
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
    _supported = Registry::capabilities().supportsGLSL();
    if ( _supported )
    {
        _paramsUniform = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "oe_DepthOffset_params");
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
        (graph && graphChanging && _options.enabled() == true);

    // shader package:
    Shaders shaders;

    if ( uninstall )
    {
        OE_TEST << LC << "Removing depth offset shaders" << std::endl;

        // uninstall uniforms and shaders.
        osg::StateSet* s = _graph->getStateSet();
        s->removeUniform( _paramsUniform.get() );
        
        shaders.unload( VirtualProgram::get(s), shaders.DepthOffsetVertex );

        s->removeAttribute(osg::StateAttribute::DEPTH);
    }

    if ( install )
    {
        OE_TEST << LC << "Installing depth offset shaders" << std::endl;

        // install uniforms and shaders.
        osg::StateSet* s = graph->getOrCreateStateSet();

        // so the stateset doesn't get merged by a state set optimizer
        s->setDataVariance(s->DYNAMIC);

        s->addUniform( _paramsUniform.get() );
        
        VirtualProgram* vp = VirtualProgram::getOrCreate(s);
        vp->setName("DepthOffset");
        shaders.load(vp, shaders.DepthOffsetVertex);    

        // disable depth writes
        s->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 0.0, 1.0, false), 1);
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

    _paramsUniform->set(osg::Vec4f(
        (float)_options.minBias()->as(Units::METERS),
        (float)_options.maxBias()->as(Units::METERS),
        (float)_options.minRange()->as(Units::METERS),
        (float)_options.maxRange()->as(Units::METERS)));
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
            float maxLen = osg::maximum(1.0f, sqrtf(v._segmentAnalyzer._maxLen2));
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
    if ( _adapter.supported() )
    {
        _adapter.setGraph( this );

        if ( _adapter.isDirty() )
            _adapter.recalculate();
    }
}

void
DepthOffsetGroup::setDepthOffsetOptions(const DepthOffsetOptions& options)
{
    _adapter.setDepthOffsetOptions(options);
    if ( _adapter.supported() && _adapter.isDirty() && !_updatePending )
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
    if ( _adapter.supported() )
    {
        ADJUST_UPDATE_TRAV_COUNT(this, 1);
        _updatePending = true;
    }
}

osg::BoundingSphere
DepthOffsetGroup::computeBound() const
{
    if ( _adapter.supported() )
    {
        static Threading::Mutex s_mutex;
        s_mutex.lock();
        const_cast<DepthOffsetGroup*>(this)->scheduleUpdate();
        s_mutex.unlock();
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
