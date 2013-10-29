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
#include "GeometryCompiler"
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osgEarthFeatures/BuildTextFilter>
#include <osgEarthFeatures/AltitudeFilter>
#include <osgEarthFeatures/CentroidFilter>
#include <osgEarthFeatures/ExtrudeGeometryFilter>
#include <osgEarthFeatures/PolygonizeLines>
#include <osgEarthFeatures/ScatterFilter>
#include <osgEarthFeatures/SubstituteModelFilter>
#include <osgEarthFeatures/TessellateOperator>
#include <osgEarth/AutoScale>
#include <osgEarth/CullingUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/ShaderUtils>
#include <osg/MatrixTransform>
#include <osg/Timer>
#include <osgDB/WriteFile>

#define LC "[GeometryCompiler] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//-----------------------------------------------------------------------

namespace
{
    /**
     * Visitor that will exaggerate the bounding box of each Drawable
     * in the scene graph to encompass a local high and low mark. We use this
     * to support GPU-clamping, which will move vertex positions in the 
     * GPU code. Since OSG is not aware of this movement, it may inadvertenly
     * cull geometry which is actually visible.
     */
    struct OverlayGeometryAdjuster : public osg::NodeVisitor
    {
        float _low, _high;

        OverlayGeometryAdjuster(float low, float high)
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), _low(low), _high(high) { }

        void apply(osg::Geode& geode)
        {
            for( unsigned i=0; i<geode.getNumDrawables(); ++i )
            {
                osg::Drawable* d = geode.getDrawable(i);
                osg::BoundingBox bbox = d->getBound();
                if ( bbox.zMin() > _low )
                    bbox.expandBy( osg::Vec3f(bbox.xMin(), bbox.yMin(), _low) );
                if ( bbox.zMax() < _high )
                    bbox.expandBy( osg::Vec3f(bbox.xMax(), bbox.yMax(), _high) );
                d->setInitialBound( bbox );
                d->dirtyBound();
            }
        }
    };
}

//-----------------------------------------------------------------------

GeometryCompilerOptions::GeometryCompilerOptions( const ConfigOptions& conf ) :
ConfigOptions      ( conf ),
_maxGranularity_deg( 1.0 ),
_mergeGeometry     ( false ),
_clustering        ( false ),
_instancing        ( false ),
_ignoreAlt         ( false ),
_useVertexBufferObjects( true ),
_shaderPolicy      ( SHADERPOLICY_GENERATE )
{
    fromConfig(_conf);
    _useVertexBufferObjects = !Registry::capabilities().preferDisplayListsForStaticGeometry();
}

void
GeometryCompilerOptions::fromConfig( const Config& conf )
{
    conf.getIfSet   ( "max_granularity",  _maxGranularity_deg );
    conf.getIfSet   ( "merge_geometry",   _mergeGeometry );
    conf.getIfSet   ( "clustering",       _clustering );
    conf.getIfSet   ( "instancing",       _instancing );
    conf.getObjIfSet( "feature_name",     _featureNameExpr );
    conf.getIfSet   ( "ignore_altitude",  _ignoreAlt );
    conf.getIfSet   ( "geo_interpolation", "great_circle", _geoInterp, GEOINTERP_GREAT_CIRCLE );
    conf.getIfSet   ( "geo_interpolation", "rhumb_line",   _geoInterp, GEOINTERP_RHUMB_LINE );
    conf.getIfSet   ( "use_vbo", _useVertexBufferObjects);

    conf.getIfSet( "shader_policy", "disable",  _shaderPolicy, SHADERPOLICY_DISABLE );
    conf.getIfSet( "shader_policy", "inherit",  _shaderPolicy, SHADERPOLICY_INHERIT );
    conf.getIfSet( "shader_policy", "generate", _shaderPolicy, SHADERPOLICY_GENERATE );
}

Config
GeometryCompilerOptions::getConfig() const
{
    Config conf = ConfigOptions::getConfig();
    conf.addIfSet   ( "max_granularity",  _maxGranularity_deg );
    conf.addIfSet   ( "merge_geometry",   _mergeGeometry );
    conf.addIfSet   ( "clustering",       _clustering );
    conf.addIfSet   ( "instancing",       _instancing );
    conf.addObjIfSet( "feature_name",     _featureNameExpr );
    conf.addIfSet   ( "ignore_altitude",  _ignoreAlt );
    conf.addIfSet   ( "geo_interpolation", "great_circle", _geoInterp, GEOINTERP_GREAT_CIRCLE );
    conf.addIfSet   ( "geo_interpolation", "rhumb_line",   _geoInterp, GEOINTERP_RHUMB_LINE );
    conf.addIfSet   ( "use_vbo", _useVertexBufferObjects);

    conf.addIfSet( "shader_policy", "disable",  _shaderPolicy, SHADERPOLICY_DISABLE );
    conf.addIfSet( "shader_policy", "inherit",  _shaderPolicy, SHADERPOLICY_INHERIT );
    conf.addIfSet( "shader_policy", "generate", _shaderPolicy, SHADERPOLICY_GENERATE );
    return conf;
}

void
GeometryCompilerOptions::mergeConfig( const Config& conf )
{
    ConfigOptions::mergeConfig( conf );
    fromConfig( conf );
}

//-----------------------------------------------------------------------

GeometryCompiler::GeometryCompiler()
{
    //nop
}

GeometryCompiler::GeometryCompiler( const GeometryCompilerOptions& options ) :
_options( options )
{
    //nop
}

osg::Node*
GeometryCompiler::compile(Geometry*             geometry,
                          const Style&          style,
                          const FilterContext&  context)
{
    osg::ref_ptr<Feature> f = new Feature(geometry, 0L); // no SRS!
    return compile(f.get(), style, context);
}

osg::Node*
GeometryCompiler::compile(Geometry*             geometry,
                          const FilterContext&  context)
{
    return compile( geometry, Style(), context );
}

osg::Node*
GeometryCompiler::compile(Feature*              feature,
                          const Style&          style,
                          const FilterContext&  context)
{
    FeatureList workingSet;
    workingSet.push_back(feature);
    return compile(workingSet, style, context);
}

osg::Node*
GeometryCompiler::compile(Feature*              feature,
                          const FilterContext&  context)
{
    return compile(feature, *feature->style(), context);
}

osg::Node*
GeometryCompiler::compile(FeatureCursor*        cursor,
                          const Style&          style,
                          const FilterContext&  context)

{
    // start by making a working copy of the feature set
    FeatureList workingSet;
    cursor->fill( workingSet );

    return compile(workingSet, style, context);
}

osg::Node*
GeometryCompiler::compile(FeatureList&          workingSet,
                          const Style&          style,
                          const FilterContext&  context)
{
#ifdef PROFILING
    osg::Timer_t p_start = osg::Timer::instance()->tick();
    unsigned p_features = workingSet.size();
#endif

    osg::ref_ptr<osg::Group> resultGroup = new osg::Group();

    // create a filter context that will track feature data through the process
    FilterContext sharedCX = context;
    if ( !sharedCX.extent().isSet() && sharedCX.profile() )
    {
        sharedCX.extent() = sharedCX.profile()->getExtent();
    }

    // ref_ptr's to hold defaults in case we need them.
    osg::ref_ptr<PointSymbol>   defaultPoint;
    osg::ref_ptr<LineSymbol>    defaultLine;
    osg::ref_ptr<PolygonSymbol> defaultPolygon;

    // go through the Style and figure out which filters to use.
    const PointSymbol*     point     = style.get<PointSymbol>();
    const LineSymbol*      line      = style.get<LineSymbol>();
    const PolygonSymbol*   polygon   = style.get<PolygonSymbol>();
    const ExtrusionSymbol* extrusion = style.get<ExtrusionSymbol>();
    const AltitudeSymbol*  altitude  = style.get<AltitudeSymbol>();
    const TextSymbol*      text      = style.get<TextSymbol>();
    const MarkerSymbol*    marker    = style.get<MarkerSymbol>();    // to be deprecated
    const IconSymbol*      icon      = style.get<IconSymbol>();
    const ModelSymbol*     model     = style.get<ModelSymbol>();

    // check whether we need tessellation:
    if ( line && line->tessellation().isSet() )
    {
        TemplateFeatureFilter<TessellateOperator> filter;
        filter.setNumPartitions( *line->tessellation() );
        sharedCX = filter.push( workingSet, sharedCX );
    }

    // if the style was empty, use some defaults based on the geometry type of the
    // first feature.
    if ( !point && !line && !polygon && !marker && !extrusion && !text && !model && !icon && workingSet.size() > 0 )
    {
        Feature* first = workingSet.begin()->get();
        Geometry* geom = first->getGeometry();
        if ( geom )
        {
            switch( geom->getComponentType() )
            {
            case Geometry::TYPE_LINESTRING:
            case Geometry::TYPE_RING:
                defaultLine = new LineSymbol();
                line = defaultLine.get();
                break;
            case Geometry::TYPE_POINTSET:
                defaultPoint = new PointSymbol();
                point = defaultPoint.get();
                break;
            case Geometry::TYPE_POLYGON:
                defaultPolygon = new PolygonSymbol();
                polygon = defaultPolygon.get();
                break;
            case Geometry::TYPE_MULTI:
            case Geometry::TYPE_UNKNOWN:
                break;
            }
        }
    }

    // resample the geometry if necessary:
    if (_options.resampleMode().isSet())
    {
        ResampleFilter resample;
        resample.resampleMode() = *_options.resampleMode();        
        if (_options.resampleMaxLength().isSet())
        {
            resample.maxLength() = *_options.resampleMaxLength();
        }                   
        sharedCX = resample.push( workingSet, sharedCX );        
    }    
    
    // check whether we need to do elevation clamping:
    bool altRequired =
        _options.ignoreAltitudeSymbol() != true &&
        altitude && (
            altitude->clamping() != AltitudeSymbol::CLAMP_NONE ||
            altitude->verticalOffset().isSet() ||
            altitude->verticalScale().isSet() );

    // marker substitution -- to be deprecated in favor of model/icon
    if ( marker )
    {
        // use a separate filter context since we'll be munging the data
        FilterContext markerCX = sharedCX;

        if ( marker->placement() == MarkerSymbol::PLACEMENT_RANDOM   ||
             marker->placement() == MarkerSymbol::PLACEMENT_INTERVAL )
        {
            ScatterFilter scatter;
            scatter.setDensity( *marker->density() );
            scatter.setRandom( marker->placement() == MarkerSymbol::PLACEMENT_RANDOM );
            scatter.setRandomSeed( *marker->randomSeed() );
            markerCX = scatter.push( workingSet, markerCX );
        }
        else if ( marker->placement() == MarkerSymbol::PLACEMENT_CENTROID )
        {
            CentroidFilter centroid;
            centroid.push( workingSet, markerCX );
        }

        if ( altRequired )
        {
            AltitudeFilter clamp;
            clamp.setPropertiesFromStyle( style );
            markerCX = clamp.push( workingSet, markerCX );

            // don't set this; we changed the input data.
            //altRequired = false;
        }

        SubstituteModelFilter sub( style );

        sub.setClustering( *_options.clustering() );

        sub.setUseDrawInstanced( *_options.instancing() );

        if ( _options.featureName().isSet() )
            sub.setFeatureNameExpr( *_options.featureName() );

        osg::Node* node = sub.push( workingSet, markerCX );
        if ( node )
        {
            resultGroup->addChild( node );
        }
    }

    // instance substitution (replaces marker)
    else if ( model ) // || icon )
    {
        const InstanceSymbol* instance = model ? (const InstanceSymbol*)model : (const InstanceSymbol*)icon;

        // use a separate filter context since we'll be munging the data
        FilterContext localCX = sharedCX;

        if ( instance->placement() == InstanceSymbol::PLACEMENT_RANDOM   ||
             instance->placement() == InstanceSymbol::PLACEMENT_INTERVAL )
        {
            ScatterFilter scatter;
            scatter.setDensity( *instance->density() );
            scatter.setRandom( instance->placement() == InstanceSymbol::PLACEMENT_RANDOM );
            scatter.setRandomSeed( *instance->randomSeed() );
            localCX = scatter.push( workingSet, localCX );
        }
        else if ( instance->placement() == InstanceSymbol::PLACEMENT_CENTROID )
        {
            CentroidFilter centroid;
            centroid.push( workingSet, localCX );
        }

        if ( altRequired )
        {
            AltitudeFilter clamp;
            clamp.setPropertiesFromStyle( style );
            localCX = clamp.push( workingSet, localCX );

            // don't set this; we changed the input data.
            //altRequired = false;
        }

        SubstituteModelFilter sub( style );

        // activate clustering
        sub.setClustering( *_options.clustering() );

        // activate draw-instancing
        sub.setUseDrawInstanced( *_options.instancing() );

        // activate feature naming
        if ( _options.featureName().isSet() )
            sub.setFeatureNameExpr( *_options.featureName() );

        osg::Node* node = sub.push( workingSet, localCX );
        if ( node )
        {
            resultGroup->addChild( node );

            // enable auto scaling on the group?
            if ( model && model->autoScale() == true )
            {
                resultGroup->getOrCreateStateSet()->setRenderBinDetails(0, osgEarth::AUTO_SCALE_BIN );
            }
        }
    }

    // extruded geometry
    if ( extrusion )
    {
        if ( altRequired )
        {
            AltitudeFilter clamp;
            clamp.setPropertiesFromStyle( style );
            sharedCX = clamp.push( workingSet, sharedCX );
            altRequired = false;
        }

        ExtrudeGeometryFilter extrude;
        extrude.setStyle( style );

        // apply per-feature naming if requested.
        if ( _options.featureName().isSet() )
            extrude.setFeatureNameExpr( *_options.featureName() );
        if ( _options.useVertexBufferObjects().isSet())
            extrude.useVertexBufferObjects() = *_options.useVertexBufferObjects();

        osg::Node* node = extrude.push( workingSet, sharedCX );
        if ( node )
        {
            resultGroup->addChild( node );
        }
    }

    // simple geometry
    else if ( point || line || polygon )
    {
        if ( altRequired )
        {
            AltitudeFilter clamp;
            clamp.setPropertiesFromStyle( style );
            sharedCX = clamp.push( workingSet, sharedCX );
            altRequired = false;
        }

        BuildGeometryFilter filter( style );
        if ( _options.maxGranularity().isSet() )
            filter.maxGranularity() = *_options.maxGranularity();
        if ( _options.geoInterp().isSet() )
            filter.geoInterp() = *_options.geoInterp();
        if ( _options.featureName().isSet() )
            filter.featureName() = *_options.featureName();

        osg::Node* node = filter.push( workingSet, sharedCX );
        if ( node )
        {
            resultGroup->addChild( node );
        }
    }

    if ( text || icon )
    {
        if ( altRequired )
        {
            AltitudeFilter clamp;
            clamp.setPropertiesFromStyle( style );
            sharedCX = clamp.push( workingSet, sharedCX );
            altRequired = false;
        }

        BuildTextFilter filter( style );
        osg::Node* node = filter.push( workingSet, sharedCX );
        if ( node )
        {
            resultGroup->addChild( node );
        }
    }

    // Common state set cache?
    osg::ref_ptr<StateSetCache> sscache;
    if ( sharedCX.getSession() )
        sscache = sharedCX.getSession()->getStateSetCache();
    else 
        sscache = new StateSetCache();

    // Generate shaders, if necessary
    if (Registry::capabilities().supportsGLSL())
    {
        if ( _options.shaderPolicy() == SHADERPOLICY_GENERATE )
        {
            ShaderGenerator gen;  // no ss cache because we will optimize later
            resultGroup->accept( gen );
        }
        else if ( _options.shaderPolicy() == SHADERPOLICY_DISABLE )
        {
            resultGroup->getOrCreateStateSet()->setAttributeAndModes(
                new osg::Program(),
                osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
        }
    }

    // Optimize stateset sharing.
    sscache->optimize( resultGroup.get() );
    
    // todo: this helps a lot, but is currently broken for non-triangle
    // geometries. (gw, 12-17-2012)
    // TODO: See: VertexCacheOptimizer in Utils
    // ..note, the BuildGeometryFilter and ExtrudeGeometryFilter call this now
#if 0
        osgUtil::Optimizer optimizer;
        optimizer.optimize(
            resultGroup.get(),
            osgUtil::Optimizer::VERTEX_PRETRANSFORM );
            osgUtil::Optimizer::VERTEX_POSTTRANSFORM );
#endif

#if 0
    // if necessary, modify the bounding boxes of the underlying Geometry
    // drawables so they will work with clamping.
    if (altitude &&
        (altitude->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN || altitude->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN) &&
        altitude->technique() == AltitudeSymbol::TECHNIQUE_GPU)
    {
        OverlayGeometryAdjuster adjuster( -10000.0f, 10000.0f );
        resultGroup->accept( adjuster );
    }
#endif


    //osgDB::writeNodeFile( *(resultGroup.get()), "out.osg" );

#ifdef PROFILING
    osg::Timer_t p_end = osg::Timer::instance()->tick();
    OE_INFO << LC
        << "features = " << p_features <<
        << " ,time = " << osg::Timer::instance()->delta_s(p_start, p_end) << " s." << std::endl;
#endif

    return resultGroup.release();
}
