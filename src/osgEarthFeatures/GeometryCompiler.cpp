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
#include "GeometryCompiler"
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osgEarthFeatures/BuildTextFilter>
#include <osgEarthFeatures/ClampFilter>
#include <osgEarthFeatures/ExtrudeGeometryFilter>
#include <osgEarthFeatures/ScatterFilter>
#include <osgEarthFeatures/SubstituteModelFilter>
#include <osgEarthFeatures/TransformFilter>
#include <osg/MatrixTransform>
#include <osg/Timer>
#include <osgDB/WriteFile>

#define LC "[GeometryCompiler] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//-----------------------------------------------------------------------

GeometryCompilerOptions::GeometryCompilerOptions( const ConfigOptions& conf ) :
ConfigOptions( conf ),
_maxGranularity_deg( 5.0 ),
_mergeGeometry( false ),
_clustering( true )
{
    fromConfig(_conf);
}

void
GeometryCompilerOptions::fromConfig( const Config& conf )
{
    conf.getIfSet   ( "max_granularity", _maxGranularity_deg );
    conf.getIfSet   ( "merge_geometry",  _mergeGeometry );
    conf.getIfSet   ( "clustering",      _clustering );
    conf.getObjIfSet( "feature_name",    _featureNameExpr );
    conf.getIfSet   ( "geo_interpolation", "great_circle", _geoInterp, GEOINTERP_GREAT_CIRCLE );
    conf.getIfSet   ( "geo_interpolation", "rhumb_line",   _geoInterp, GEOINTERP_RHUMB_LINE );
}

Config
GeometryCompilerOptions::getConfig() const
{
    Config conf = ConfigOptions::getConfig();
    conf.addIfSet   ( "max_granularity", _maxGranularity_deg );
    conf.addIfSet   ( "merge_geometry",  _mergeGeometry );
    conf.addIfSet   ( "clustering",      _clustering );
    conf.addObjIfSet( "feature_name",    _featureNameExpr );
    conf.addIfSet   ( "geo_interpolation", "great_circle", _geoInterp, GEOINTERP_GREAT_CIRCLE );
    conf.addIfSet   ( "geo_interpolation", "rhumb_line",   _geoInterp, GEOINTERP_RHUMB_LINE );
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
GeometryCompiler::compile(Feature*              feature,
                          const Style&          style,
                          const FilterContext&  context)
{
    if ( !context.profile() ) {
        OE_WARN << LC << "Valid feature profile required" << std::endl;
        return 0L;
    }

    if ( style.empty() ) {
        OE_WARN << LC << "Non-empty style required" << std::endl;
        return 0L;
    }

    FeatureList workingSet;
    workingSet.push_back(feature);
    return compile(workingSet, style, context);
}

osg::Node*
GeometryCompiler::compile(FeatureCursor*        cursor,
                          const Style&          style,
                          const FilterContext&  context)

{
    if ( !context.profile() ) {
        OE_WARN << LC << "Valid feature profile required" << std::endl;
        return 0L;
    }

    if ( style.empty() ) {
        OE_WARN << LC << "Non-empty style required" << std::endl;
        return 0L;
    }

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
    if ( !sharedCX.extent().isSet() )
        sharedCX.extent() = sharedCX.profile()->getExtent();

    // only localize coordinates if the map is geocentric AND the extent is
    // less than 180 degrees.
    const MapInfo& mi = sharedCX.getSession()->getMapInfo();
    GeoExtent workingExtent = sharedCX.extent()->transform( sharedCX.profile()->getSRS()->getGeographicSRS() );
    bool localize = mi.isGeocentric() && workingExtent.width() < 180.0;

    // go through the Style and figure out which filters to use.
    const MarkerSymbol*    marker    = style.get<MarkerSymbol>();
    const PointSymbol*     point     = style.get<PointSymbol>();
    const LineSymbol*      line      = style.get<LineSymbol>();
    const PolygonSymbol*   polygon   = style.get<PolygonSymbol>();
    const ExtrusionSymbol* extrusion = style.get<ExtrusionSymbol>();
    const AltitudeSymbol*  altitude  = style.get<AltitudeSymbol>();
    const TextSymbol*      text      = style.get<TextSymbol>();

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
    
    bool clampRequired =
        altitude && altitude->clamping() != AltitudeSymbol::CLAMP_NONE;
    
    // first, apply a vertical offset if called for.
    if ( altitude && altitude->verticalOffset().isSet() && !clampRequired )
    {
        TransformFilter xform( osg::Matrixd::translate(0, 0, *altitude->verticalOffset()) );
        sharedCX = xform.push( workingSet, sharedCX );
    }

    // model substitution
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

        if ( clampRequired )
        {
            ClampFilter clamp;
            clamp.setPropertiesFromStyle( style );
            markerCX = clamp.push( workingSet, markerCX );

            // don't set this; we changed the input data.
            //clampRequired = false;
        }

        SubstituteModelFilter sub( style );
        if ( marker->scale().isSet() )
            sub.setModelMatrix( osg::Matrixd::scale( *marker->scale() ) );

        sub.setClustering( *_options.clustering() );
        if ( _options.featureName().isSet() )
            sub.setFeatureNameExpr( *_options.featureName() );

        osg::Node* node = sub.push( workingSet, markerCX );
        if ( node )
        {
            resultGroup->addChild( node );
        }
    }

    // extruded geometry
    if ( extrusion )
    {
        if ( clampRequired )
        {
            ClampFilter clamp;
            clamp.setPropertiesFromStyle( style );
            sharedCX = clamp.push( workingSet, sharedCX );
            clampRequired = false;
        }

        ExtrudeGeometryFilter extrude;
        extrude.setStyle( style );

        // apply per-feature naming if requested.
        if ( _options.featureName().isSet() )
            extrude.setFeatureNameExpr( *_options.featureName() );

        osg::Node* node = extrude.push( workingSet, sharedCX );
        if ( node )
        {
            resultGroup->addChild( node );
        }
    }

    // simple geometry
    else if ( point || line || polygon )
    {
        if ( clampRequired )
        {
            ClampFilter clamp;
            clamp.setPropertiesFromStyle( style );
            sharedCX = clamp.push( workingSet, sharedCX );
            clampRequired = false;
        }

        BuildGeometryFilter filter( style );
        if ( _options.maxGranularity().isSet() )
            filter.maxGranularity() = *_options.maxGranularity();
        if ( _options.geoInterp().isSet() )
            filter.geoInterp() = *_options.geoInterp();
        if ( _options.mergeGeometry().isSet() )
            filter.mergeGeometry() = *_options.mergeGeometry();
        if ( _options.featureName().isSet() )
            filter.featureName() = *_options.featureName();

        osg::Node* node = filter.push( workingSet, sharedCX );
        if ( node )
        {
            resultGroup->addChild( node );
        }
    }

    if ( text )
    {
        if ( clampRequired )
        {
            ClampFilter clamp;
            clamp.setPropertiesFromStyle( style );
            sharedCX = clamp.push( workingSet, sharedCX );
            clampRequired = false;
        }

        BuildTextFilter filter( style );
        osg::Node* node = filter.push( workingSet, sharedCX );
        if ( node )
        {
            resultGroup->addChild( node );
        }
    }

    resultGroup->getOrCreateStateSet()->setMode( GL_BLEND, 1 );

    //osgDB::writeNodeFile( *(resultGroup.get()), "out.osg" );

#ifdef PROFILING
    osg::Timer_t p_end = osg::Timer::instance()->tick();
    OE_INFO << LC
        << "features = " << p_features <<
        << " ,time = " << osg::Timer::instance()->delta_s(p_start, p_end) << " s." << std::endl;
#endif

    return resultGroup.release();
}
