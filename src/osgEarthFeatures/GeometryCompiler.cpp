/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include "GeometryCompiler"

#include <osgEarthFeatures/BuildGeometryFilter>
#include <osgEarthFeatures/BuildTextFilter>
#include <osgEarthFeatures/AltitudeFilter>
#include <osgEarthFeatures/CentroidFilter>
#include <osgEarthFeatures/ExtrudeGeometryFilter>
#include <osgEarthFeatures/ScatterFilter>
#include <osgEarthFeatures/SubstituteModelFilter>
#include <osgEarthFeatures/TessellateOperator>
#include <osgEarthFeatures/Session>

#include <osgEarth/Utils>
#include <osgEarth/CullingUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/ShaderUtils>
#include <osgEarth/Utils>

#include <osg/MatrixTransform>
#include <osg/Timer>
#include <osgDB/WriteFile>
#include <osgUtil/Optimizer>

#include <cstdlib>

#define LC "[GeometryCompiler] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//#define PROFILING 1

//-----------------------------------------------------------------------

GeometryCompilerOptions GeometryCompilerOptions::s_defaults(true);

void
GeometryCompilerOptions::setDefaults(const GeometryCompilerOptions& defaults)
{
   s_defaults = defaults;
}

// defaults.
GeometryCompilerOptions::GeometryCompilerOptions(bool stockDefaults) :
_maxGranularity_deg    ( 10.0 ),
_mergeGeometry         ( true ),
_clustering            ( false ),
_instancing            ( true ),
_ignoreAlt             ( false ),
_shaderPolicy          ( SHADERPOLICY_GENERATE ),
_geoInterp             ( GEOINTERP_GREAT_CIRCLE ),
_optimizeStateSharing  ( true ),
_optimize              ( false ),
_optimizeVertexOrdering( true ),
_validate              ( false ),
_maxPolyTilingAngle    ( 45.0f ),
_useGPULines           ( false )
{
    if (::getenv("OSGEARTH_GPU_SCREEN_SPACE_LINES") != 0L)
    {
        _useGPULines.init(true);
    }
}

//-----------------------------------------------------------------------

GeometryCompilerOptions::GeometryCompilerOptions(const ConfigOptions& conf) :
_maxGranularity_deg    ( s_defaults.maxGranularity().value() ),
_mergeGeometry         ( s_defaults.mergeGeometry().value() ),
_clustering            ( s_defaults.clustering().value() ),
_instancing            ( s_defaults.instancing().value() ),
_ignoreAlt             ( s_defaults.ignoreAltitudeSymbol().value() ),
_shaderPolicy          ( s_defaults.shaderPolicy().value() ),
_geoInterp             ( s_defaults.geoInterp().value() ),
_optimizeStateSharing  ( s_defaults.optimizeStateSharing().value() ),
_optimize              ( s_defaults.optimize().value() ),
_optimizeVertexOrdering( s_defaults.optimizeVertexOrdering().value() ),
_validate              ( s_defaults.validate().value() ),
_maxPolyTilingAngle    ( s_defaults.maxPolygonTilingAngle().value() ),
_useGPULines           ( s_defaults.useGPUScreenSpaceLines().value() )
{
    fromConfig(conf.getConfig());
}

void
GeometryCompilerOptions::fromConfig( const Config& conf )
{
    conf.get( "max_granularity",  _maxGranularity_deg );
    conf.get( "merge_geometry",   _mergeGeometry );
    conf.get( "clustering",       _clustering );
    conf.get( "instancing",       _instancing );
    conf.get( "feature_name",     _featureNameExpr );
    conf.get( "ignore_altitude",  _ignoreAlt );
    conf.get( "geo_interpolation", "great_circle", _geoInterp, GEOINTERP_GREAT_CIRCLE );
    conf.get( "geo_interpolation", "rhumb_line",   _geoInterp, GEOINTERP_RHUMB_LINE );
    conf.get( "optimize_state_sharing", _optimizeStateSharing );
    conf.get( "optimize", _optimize );
    conf.get( "optimize_vertex_ordering", _optimizeVertexOrdering);
    conf.get( "validate", _validate );
    conf.get( "max_polygon_tiling_angle", _maxPolyTilingAngle );
    conf.get( "use_gpu_screen_space_lines", _useGPULines );

    conf.get( "shader_policy", "disable",  _shaderPolicy, SHADERPOLICY_DISABLE );
    conf.get( "shader_policy", "inherit",  _shaderPolicy, SHADERPOLICY_INHERIT );
    conf.get( "shader_policy", "generate", _shaderPolicy, SHADERPOLICY_GENERATE );
}

Config
GeometryCompilerOptions::getConfig() const
{
    Config conf;
    conf.set( "max_granularity",  _maxGranularity_deg );
    conf.set( "merge_geometry",   _mergeGeometry );
    conf.set( "clustering",       _clustering );
    conf.set( "instancing",       _instancing );
    conf.set( "feature_name",     _featureNameExpr );
    conf.set( "ignore_altitude",  _ignoreAlt );
    conf.set( "geo_interpolation", "great_circle", _geoInterp, GEOINTERP_GREAT_CIRCLE );
    conf.set( "geo_interpolation", "rhumb_line",   _geoInterp, GEOINTERP_RHUMB_LINE );
    conf.set( "optimize_state_sharing", _optimizeStateSharing );
    conf.set( "optimize", _optimize );
    conf.set( "optimize_vertex_ordering", _optimizeVertexOrdering);
    conf.set( "validate", _validate );
    conf.set( "max_polygon_tiling_angle", _maxPolyTilingAngle );
    conf.set( "use_gpu_screen_space_lines", _useGPULines );

    conf.set( "shader_policy", "disable",  _shaderPolicy, SHADERPOLICY_DISABLE );
    conf.set( "shader_policy", "inherit",  _shaderPolicy, SHADERPOLICY_INHERIT );
    conf.set( "shader_policy", "generate", _shaderPolicy, SHADERPOLICY_GENERATE );
    return conf;
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
                          const Style&          style)
{
    osg::ref_ptr<Feature> f = new Feature(geometry, 0L); // no SRS!
    return compile(f.get(), style, FilterContext(0L) );
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

    // for debugging/validation.
    std::vector<std::string> history;
    bool trackHistory = (_options.validate() == true);

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
    const IconSymbol*      icon      = style.get<IconSymbol>();
    const ModelSymbol*     model     = style.get<ModelSymbol>();
    const RenderSymbol*    render    = style.get<RenderSymbol>();

    // Perform tessellation first.
    if ( line )
    {
        if ( line->tessellation().isSet() )
        {
            TessellateOperator filter;
            filter.setNumPartitions( *line->tessellation() );
            filter.setDefaultGeoInterp( _options.geoInterp().get() );
            sharedCX = filter.push( workingSet, sharedCX );
            if ( trackHistory ) history.push_back( "tessellation" );
        }
        else if ( line->tessellationSize().isSet() )
        {
            TessellateOperator filter;
            filter.setMaxPartitionSize( *line->tessellationSize() );
            filter.setDefaultGeoInterp( _options.geoInterp().get() );
            sharedCX = filter.push( workingSet, sharedCX );
            if ( trackHistory ) history.push_back( "tessellationSize" );
        }
    }

    // if the style was empty, use some defaults based on the geometry type of the
    // first feature.
    if ( !point && !line && !polygon && !extrusion && !text && !model && !icon && workingSet.size() > 0 )
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
        if ( trackHistory ) history.push_back( "resample" );
    }    
    
    // check whether we need to do elevation clamping:
    bool altRequired =
        _options.ignoreAltitudeSymbol() != true &&
        altitude && (
            altitude->clamping() != AltitudeSymbol::CLAMP_NONE ||
            altitude->verticalOffset().isSet() ||
            altitude->verticalScale().isSet() ||
            altitude->script().isSet() );    

    // instance substitution (replaces marker)
    if ( model )
    {
        const InstanceSymbol* instance = (const InstanceSymbol*)model;

        // use a separate filter context since we'll be munging the data
        FilterContext localCX = sharedCX;
        
        if ( trackHistory ) history.push_back( "model");

        if ( instance->placement() == InstanceSymbol::PLACEMENT_RANDOM   ||
             instance->placement() == InstanceSymbol::PLACEMENT_INTERVAL )
        {
            ScatterFilter scatter;
            scatter.setDensity( *instance->density() );
            scatter.setRandom( instance->placement() == InstanceSymbol::PLACEMENT_RANDOM );
            scatter.setRandomSeed( *instance->randomSeed() );
            localCX = scatter.push( workingSet, localCX );
            if ( trackHistory ) history.push_back( "scatter" );
        }
        else if ( instance->placement() == InstanceSymbol::PLACEMENT_CENTROID )
        {
            CentroidFilter centroid;
            localCX = centroid.push( workingSet, localCX );
            if ( trackHistory ) history.push_back( "centroid" );
        }

        if ( altRequired )
        {
            AltitudeFilter clamp;
            clamp.setPropertiesFromStyle( style );
            localCX = clamp.push( workingSet, localCX );
            if ( trackHistory ) history.push_back( "altitude" );
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
            if ( trackHistory ) history.push_back( "substitute" );

            resultGroup->addChild( node );
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
            if ( trackHistory ) history.push_back( "altitude" );
            altRequired = false;
        }

        ExtrudeGeometryFilter extrude;
        extrude.setStyle( style );

        // apply per-feature naming if requested.
        if ( _options.featureName().isSet() )
            extrude.setFeatureNameExpr( *_options.featureName() );

        if ( _options.mergeGeometry().isSet() )
            extrude.setMergeGeometry( *_options.mergeGeometry() );

        osg::Node* node = extrude.push( workingSet, sharedCX );
        if ( node )
        {
            if ( trackHistory ) history.push_back( "extrude" );
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
            if ( trackHistory ) history.push_back( "altitude" );
            altRequired = false;
        }

        BuildGeometryFilter filter( style );

        filter.maxGranularity() = *_options.maxGranularity();
        filter.geoInterp()      = *_options.geoInterp();
        filter.shaderPolicy()   = *_options.shaderPolicy();

        if (_options.maxPolygonTilingAngle().isSet())
            filter.maxPolygonTilingAngle() = *_options.maxPolygonTilingAngle();

        if ( _options.featureName().isSet() )
            filter.featureName() = *_options.featureName();

        if (_options.optimizeVertexOrdering().isSet())
            filter.optimizeVertexOrdering() = *_options.optimizeVertexOrdering();

        if (render && render->maxCreaseAngle().isSet())
            filter.maxCreaseAngle() = render->maxCreaseAngle().get();

        osg::Node* node = filter.push( workingSet, sharedCX );
        if ( node )
        {
            if ( trackHistory ) history.push_back( "geometry" );
            resultGroup->addChild( node );
        }
    }

    if ( text || icon )
    {
        // Only clamp annotation types when the technique is 
        // explicity set to MAP. Otherwise, the annotation subsystem
        // will automatically use SCENE clamping.
        bool altRequiredForAnnotations =
            altRequired &&
            altitude->technique().isSetTo(altitude->TECHNIQUE_MAP);

        if ( altRequiredForAnnotations )
        {
            AltitudeFilter clamp;
            clamp.setPropertiesFromStyle( style );
            sharedCX = clamp.push( workingSet, sharedCX );
            if ( trackHistory ) history.push_back( "altitude" );
            altRequired = false;
        }

        BuildTextFilter filter( style );
        osg::Node* node = filter.push( workingSet, sharedCX );
        if ( node )
        {
            if ( trackHistory ) history.push_back( "text" );
            resultGroup->addChild( node );
        }
    }

    if (Registry::capabilities().supportsGLSL())
    {
        if ( _options.shaderPolicy() == SHADERPOLICY_GENERATE )
        {
            // no ss cache because we will optimize later.
            Registry::shaderGenerator().run( 
                resultGroup.get(),
                "GeometryCompiler shadergen" );
        }
        else if ( _options.shaderPolicy() == SHADERPOLICY_DISABLE )
        {
            resultGroup->getOrCreateStateSet()->setAttributeAndModes(
                new osg::Program(),
                osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
        
            if ( trackHistory ) history.push_back( "no shaders" );
        }
    }

    // Optimize stateset sharing.
    if ( _options.optimizeStateSharing() == true )
    {
        // Common state set cache?
        osg::ref_ptr<StateSetCache> sscache;
        if ( sharedCX.getSession() )
        {
            // with a shared cache, don't combine statesets. They may be
            // in the live graph
            sscache = sharedCX.getSession()->getStateSetCache();
            sscache->consolidateStateAttributes( resultGroup.get() );
        }
        else 
        {
            // isolated: perform full optimization
            sscache = new StateSetCache();
            sscache->optimize( resultGroup.get() );
        }
        
        if ( trackHistory ) history.push_back( "share state" );
    }

    if ( _options.optimize() == true )
    {
        OE_DEBUG << LC << "optimize begin" << std::endl;

        // Run the optimizer on the resulting graph
        int optimizations =
            osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS |
            osgUtil::Optimizer::REMOVE_REDUNDANT_NODES |
            osgUtil::Optimizer::COMBINE_ADJACENT_LODS |
            osgUtil::Optimizer::SHARE_DUPLICATE_STATE |
            //osgUtil::Optimizer::MERGE_GEOMETRY |
            osgUtil::Optimizer::CHECK_GEOMETRY |
            osgUtil::Optimizer::MERGE_GEODES |
            osgUtil::Optimizer::STATIC_OBJECT_DETECTION;

        osgUtil::Optimizer opt;
        opt.optimize(resultGroup.get(), optimizations);

        osgUtil::Optimizer::MergeGeometryVisitor mg;
        mg.setTargetMaximumNumberOfVertices(65536);
        resultGroup->accept(mg);

        OE_DEBUG << LC << "optimize complete" << std::endl;

        if ( trackHistory ) history.push_back( "optimize" );
    }
    

    //test: dump the tile to disk
    //OE_WARN << "Writing GC node file to out.osgt..." << std::endl;
    //osgDB::writeNodeFile( *(resultGroup.get()), "out.osgt" );

#ifdef PROFILING
    static double totalTime = 0.0;
    static Threading::Mutex totalTimeMutex;
    osg::Timer_t p_end = osg::Timer::instance()->tick();
    double t = osg::Timer::instance()->delta_s(p_start, p_end);
    totalTimeMutex.lock();
    totalTime += t;
    totalTimeMutex.unlock();
    OE_INFO << LC
        << "features = " << p_features
        << ", time = " << t << " s.  cummulative = " 
        << totalTime << " s."
        << std::endl;
#endif


    if ( _options.validate() == true )
    {
        OE_NOTICE << LC << "-- Start Debugging --\n";
        std::stringstream buf;
        buf << "HISTORY ";
        for(std::vector<std::string>::iterator h = history.begin(); h != history.end(); ++h)
            buf << ".. " << *h;
        OE_NOTICE << LC << buf.str() << "\n";
        osgEarth::GeometryValidator validator;
        resultGroup->accept(validator);
        OE_NOTICE << LC << "-- End Debugging --\n";
    }

    return resultGroup.release();
}
